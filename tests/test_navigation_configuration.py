from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(relative: str):
    return yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))


class NavigationConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.nav = load_yaml("config/nav2_params_go2.yaml")
        cls.mapping = load_yaml("config/rtabmap_params.yaml")
        cls.manifest = load_yaml("robonix_manifest.yaml")
        cls.tree = ET.parse(ROOT / "config" / "navigate.xml")

    def service_config(self, name: str) -> dict:
        return next(row["config"] for row in self.manifest["service"] if row["name"] == name)

    def primitive_config(self, name: str) -> dict:
        return next(row.get("config", {}) for row in self.manifest["primitive"] if row["name"] == name)

    def test_humble_uses_the_deploy_owned_safe_tree(self) -> None:
        params = self.nav["bt_navigator"]["ros__parameters"]
        token = "__ROBONIX_BT_XML__"
        self.assertEqual(params["default_nav_to_pose_bt_xml"], token)
        self.assertEqual(params["default_bt_xml_filename"], token)
        self.assertEqual(params["global_frame"], "map")
        self.assertEqual(params["robot_base_frame"], "base_link")
        self.assertEqual(params["odom_topic"], "__ROBONIX_ODOM_TOPIC__")

    def test_tree_has_only_non_motion_recovery_actions(self) -> None:
        tags = {node.tag for node in self.tree.iter()}
        for forbidden in ("Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"):
            self.assertNotIn(forbidden, tags)
        for required in (
            "ComputePathToPose",
            "FollowPath",
            "ClearEntireCostmap",
            "GoalUpdated",
            "Wait",
        ):
            self.assertIn(required, tags)
        retries = [int(node.attrib["number_of_retries"]) for node in self.tree.iter("RecoveryNode")]
        self.assertTrue(retries)
        self.assertTrue(all(value <= 1 for value in retries))

    def test_known_space_planning_and_live_obstacle_layers(self) -> None:
        planner = self.nav["planner_server"]["ros__parameters"]["GridBased"]
        self.assertFalse(planner["allow_unknown"])

        for costmap_name in ("local_costmap", "global_costmap"):
            params = self.nav[costmap_name][costmap_name]["ros__parameters"]
            self.assertIn("obstacle_layer", params["plugins"])
            self.assertIn("inflation_layer", params["plugins"])
            scan = params["obstacle_layer"]["scan"]
            self.assertEqual(scan["topic"], "__ROBONIX_SCAN_TOPIC__")
            self.assertEqual(scan["data_type"], "LaserScan")
            self.assertTrue(scan["marking"])
            self.assertTrue(scan["clearing"])
            self.assertTrue(scan["inf_is_valid"])
            self.assertEqual(scan["observation_persistence"], 0.0)
            self.assertGreater(scan["expected_update_rate"], 0.0)
            self.assertLessEqual(scan["expected_update_rate"], 1.0)
            self.assertGreater(scan["raytrace_max_range"], scan["obstacle_max_range"])
            self.assertLessEqual(scan["min_obstacle_height"], 0.0)
            self.assertNotIn("qos", scan)

        global_params = self.nav["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertIn("static_layer", global_params["plugins"])
        self.assertEqual(global_params["static_layer"]["map_topic"], "__ROBONIX_MAP_TOPIC__")

    def test_velocity_smoother_is_bounded_and_times_out(self) -> None:
        controller = self.nav["controller_server"]["ros__parameters"]["FollowPath"]
        smoother = self.nav["velocity_smoother"]["ros__parameters"]
        chassis = self.primitive_config("go2_chassis")

        self.assertEqual(smoother["max_velocity"], [0.25, 0.0, 0.40])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.40])
        self.assertLessEqual(smoother["max_velocity"][0], controller["max_vel_x"])
        self.assertLessEqual(smoother["max_velocity"][2], controller["max_vel_theta"])
        self.assertLessEqual(smoother["max_velocity"][0], chassis["max_linear_x_mps"])
        self.assertLessEqual(smoother["max_velocity"][2], chassis["max_angular_z_rps"])
        self.assertLessEqual(smoother["max_accel"][0], chassis["max_linear_accel_mps2"])
        self.assertLessEqual(smoother["max_accel"][2], chassis["max_angular_accel_rps2"])
        self.assertLessEqual(abs(smoother["max_decel"][0]), chassis["max_linear_accel_mps2"])
        self.assertLessEqual(abs(smoother["max_decel"][2]), chassis["max_angular_accel_rps2"])
        self.assertLessEqual(smoother["velocity_timeout"], chassis["command_timeout_s"])
        self.assertEqual(smoother["odom_topic"], "__ROBONIX_ODOM_TOPIC__")

        self.assertLessEqual(abs(controller["decel_lim_x"]), chassis["max_linear_accel_mps2"])
        self.assertLessEqual(abs(controller["decel_lim_theta"]), chassis["max_angular_accel_rps2"])
        self.assertNotIn("RotateToGoal.yaw_goal_tolerance", controller)

        managed = set(self.nav["lifecycle_manager"]["ros__parameters"]["node_names"])
        self.assertTrue({"smoother_server", "velocity_smoother", "bt_navigator"} <= managed)

    def test_provider_roles_are_pinned_to_this_robot(self) -> None:
        nav = self.service_config("nav2")
        self.assertEqual(
            nav["provider_ids"],
            {
                "map": "mapping",
                "odom": "go2_chassis",
                "scan": "go2_sensors",
                "scan_cloud": "go2_sensors",
            },
        )
        self.assertTrue(nav["scan_projection"]["enabled"])
        self.assertEqual(nav["scan_projection"]["target_frame"], "base_link")
        self.assertLess(nav["scan_projection"]["min_height_m"], 0.0)
        self.assertGreater(nav["scan_projection"]["max_height_m"], 0.0)
        self.assertLessEqual(nav["scan_projection"]["self_filter_margin_m"], 0.02)
        self.assertFalse(nav["scan_projection"]["deskewing"])

        mapping = self.service_config("mapping")
        self.assertEqual(
            mapping["sensor_providers"],
            {"lidar3d": "go2_sensors", "imu": "go2_sensors", "odom": "go2_chassis"},
        )
        self.assertEqual(mapping["rtabmap_inputs"], ["lidar", "imu", "odom"])
        self.assertFalse(mapping["deskew_lidar"])

    def test_terminal_guard_matches_goal_window(self) -> None:
        goal = self.nav["controller_server"]["ros__parameters"]["go2_goal_checker"]
        guard = self.service_config("nav2")
        self.assertLessEqual(guard["guard_terminal_xy_m"], goal["xy_goal_tolerance"])
        self.assertGreater(guard["guard_terminal_timeout_s"], 0.0)
        self.assertGreater(guard["guard_no_progress_s"], 0.0)
        self.assertGreater(guard["guard_global_spin_timeout_s"], 0.0)
        self.assertGreater(guard["guard_global_spin_limit_rad"], 0.0)

    def test_rtabmap_profile_is_planar_lidar_only_and_scalar(self) -> None:
        self.assertTrue(all(not isinstance(value, (dict, list)) for value in self.mapping.values()))
        self.assertEqual(self.mapping["Grid/Sensor"], 0)
        self.assertFalse(self.mapping["Grid/FromDepth"])
        self.assertFalse(self.mapping["Grid/3D"])
        self.assertTrue(self.mapping["Reg/Force3DoF"])
        self.assertGreaterEqual(self.mapping["Grid/FootprintLength"], 0.80)
        self.assertGreaterEqual(self.mapping["Grid/FootprintWidth"], 0.50)


if __name__ == "__main__":
    unittest.main()
