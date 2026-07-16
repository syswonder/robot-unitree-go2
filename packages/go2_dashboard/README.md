# Go2 只读态势面板

这是 Robonix Unitree Go2 部署内的正式只读 Service。Provider 固定为
`Service(id="go2_dashboard", namespace="robonix/service/telemetry/dashboard")`，
由生命周期回调拥有一个 Web 面板子进程。面板把 ROS 2 数据转换为有界的浏览器预览，单页显示：

- `/camera/color/image_raw` 相机图像；
- `/scanner/scan` 或 `/scanner/cloud` 雷达俯视图；
- `/map` 占据栅格；
- TF `map -> base_link` 机器人位置和朝向；
- `/odom` 位置与速度；
- `/navigate_to_pose/_action/status` Nav2 动作状态；
- 由语义导航组件注入的任务显示状态。

本包向 Atlas 注册 lifecycle driver 和只读 status 两条 capability，不提供机器人命令接口。Web 子进程只创建 ROS 订阅和 TF 监听；语义任务接口只更新该进程内的 UI 状态，不调用导航服务。

## Robonix 生命周期

`rbnx boot` 中的部署名称必须是 `go2_dashboard`，与 provider id 完全一致：

```yaml
service:
  - name: go2_dashboard
    path: packages/go2_dashboard
    config:
      host: 127.0.0.1
      port: 8092
      camera_topic: /camera/color/image_raw
      scan_topic: /scanner/scan
      cloud_topic: /scanner/cloud
      map_topic: /map
      odom_topic: /odom
      nav_status_topic: /navigate_to_pose/_action/status
      map_frame: map
      base_frame: base_link
```

Provider 启动后先向 Atlas 注册。`Driver(CMD_INIT)` 会严格验证 config，并启动唯一的直接子进程；`CMD_ACTIVATE` 验证子进程仍在线。`CMD_DEACTIVATE`、`CMD_SHUTDOWN` 或 provider 收到终止信号时，只通过保存的 `Popen` 对象终止自身创建的子进程，不按进程名杀死其它程序。

注册能力：

- `robonix/service/telemetry/dashboard/driver`：包内 lifecycle contract；
- `robonix/service/telemetry/dashboard/status`：包内 `GetDashboardStatus.srv`，同时提供 MCP 与 unary gRPC 只读查询，返回 `ok`、`url`、`detail` 和有界 `status_json`。

## 运行要求

- Ubuntu 22.04、ROS 2 Humble；
- `rclpy`、`sensor_msgs`、`nav_msgs`、`action_msgs` 和 `tf2_ros` 来自已安装的 ROS 环境，不从 PyPI 安装；
- Python 3.10 及 `python3-venv`；
- 可用的 `rbnx` CLI 以及 Robonix `robonix_api`；
- FastAPI、Uvicorn、Pillow 以及 `robonix_api` 所需的 gRPC/MCP 运行依赖由 `requirements.txt` 安装到包内 `rbnx-build/venv`；`robonix_api` 本体从 `ROBONIX_SOURCE_PATH/pylib/robonix-api` 加入 Python 路径。

构建命令会先执行不依赖 ROS 的解析与安全测试，再执行 `rbnx codegen -p <包目录> --mcp`，最后创建虚拟环境并验证 provider 与生成类型可导入：

```bash
bash scripts/build.sh
```

若依赖已由离线镜像预装，可设置 `GO2_DASHBOARD_SKIP_PIP=1`，构建仍会验证这些模块可导入。

正常部署由 `rbnx boot` 调用 build/start/lifecycle。单独调试 provider 时，Atlas 必须已经运行；启动脚本只启动 provider，Web 子进程要等 lifecycle INIT：

```bash
bash scripts/start.sh
```

默认 config 下，INIT 成功后访问 `http://127.0.0.1:8092/`。停止 provider：

```bash
bash scripts/stop.sh
```

停止脚本只向经过命令行校验的 provider PID 发送终止信号；子进程由 provider 的 `on_shutdown` 回收。面板自身不实现登录，因此 Provider 配置和直接命令行入口都只接受 `host: 127.0.0.1`。远程访问必须使用经过认证的 SSH 隧道，不得把面板直接暴露到局域网。

## 配置

以下字段来自 `robonix_manifest.yaml` 中该 service 的 `config:`。未知字段会使 INIT 失败，避免拼写错误被静默忽略：

| config 字段 | 默认值 | 作用 |
| --- | --- | --- |
| `camera_topic` | `/camera/color/image_raw` | 彩色图像 |
| `scan_topic` | `/scanner/scan` | 二维扫描 |
| `cloud_topic` | `/scanner/cloud` | 三维点云 |
| `map_topic` | `/map` | 占据栅格 |
| `odom_topic` | `/odom` | 里程计 |
| `nav_status_topic` | `/navigate_to_pose/_action/status` | Nav2 状态 |
| `map_frame` | `map` | 全局地图坐标系 |
| `base_frame` | `base_link` | 机器人基座坐标系 |
| `host` | `127.0.0.1` | 固定的回环 HTTP 监听地址；其他值会被拒绝 |
| `port` | `8092` | HTTP 端口 |
| `public_url` | 空 | status capability 对外报告的 HTTP(S) URL |
| `log_level` | `info` | Web 日志等级 |
| `startup_timeout_s` | `8.0` | 等待本地 health endpoint 的秒数 |
| `stop_timeout_s` | `5.0` | 优雅停止子进程的秒数 |

为兼容现有 Go2 部署清单，`image_topic` 可作为 `camera_topic` 的别名；若两者同时设置但值不同，INIT 会失败。

启动脚本另外读取两个本机环境变量：

| 环境变量 | 默认值 | 作用 |
| --- | --- | --- |
| `ROS_SETUP_FILE` | `/opt/ros/humble/setup.bash` | ROS 环境脚本 |
| `GO2_ROS_OVERLAY_SETUP` | 未设置 | 可选工作区 overlay 环境脚本 |

相机和雷达预览在观察线程内限频到 5 Hz，点云预览最多 1200 点。地图使用缓存 PNG；状态响应不携带二进制图像。页面会把每个输入标记为 `fresh`、`stale`、`missing` 或 `error`，并显示最近帧年龄、`frame_id` 和解析错误。

## HTTP 接口

| 方法与路径 | 内容 |
| --- | --- |
| `GET /` | 单页态势面板 |
| `GET /healthz` | HTTP 与 ROS 观察线程健康摘要 |
| `GET /api/status` | 有界 JSON 状态快照 |
| `GET /api/camera.jpg` | 最近相机预览 |
| `GET /api/map.png` | 最近地图预览 |
| `GET /api/semantic-task` | 当前语义任务显示状态 |
| `POST /api/semantic-task` | 覆盖语义任务显示状态 |

语义导航组件可以在自身状态变化后写入显示信息。例如，下面只会让 UI 显示“自动售货机”已解析，不会创建 ROS 实体或触发任务：

```bash
curl --fail-with-body \
  --request POST http://127.0.0.1:8092/api/semantic-task \
  --header 'Content-Type: application/json' \
  --data '{"task_id":"demo-1","target_name":"自动售货机","status":"resolved","message":"已匹配保存的地图 Pose","pose":{"frame_id":"map","x":1.2,"y":-0.4,"yaw":0.0}}'
```

允许的显示状态为 `idle`、`received`、`resolving`、`resolved`、`navigating`、`succeeded`、`canceled` 和 `failed`。接口没有“执行”字段，也不会把 Pose 转发到 ROS 或 Robonix navigation service。

## Atlas status 返回

只读 status capability 的 `ok` 仅在 Web 子进程、health endpoint 和 ROS 观察线程都在线时为 `true`。`status_json` 只包含只读标志、子进程状态、最近退出码、health 摘要和已验证的公开 config，不复制父进程环境或凭据。创建子进程时也会移除代理变量及名称包含 password、secret、token、credential、private key 或 API/access key 的环境项；ROS、DDS 和 Python 运行环境保留。查询不会启动、重启或停止任何进程。

## 离线验证

测试只解析内存中的伪造消息字节与源码，不启动 ROS、HTTP 监听或任何硬件连接：

```bash
python3 -m unittest discover -s tests -v
```

覆盖范围包括图像行步长和编码、LaserScan 边界、PointCloud2 字段/大小端、OccupancyGrid 朝向、四元数、数据新鲜度、语义状态校验、manifest/provider 身份、config 到子进程环境映射、只终止自身子进程，以及运行时代码的只读静态约束。
