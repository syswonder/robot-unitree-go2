from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "go2_robottrack"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Origamii520",
    maintainer_email="Origamii520@users.noreply.github.com",
    description=(
        "D435i RGB to MiniCPM-RobotTrack HTTP bridge with a mutually exclusive "
        "navigation/following command-source mux."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robottrack_node = go2_robottrack.ros_node:main",
            "robottrack_camera_preview = go2_robottrack.camera_preview_node:main",
        ],
    },
)
