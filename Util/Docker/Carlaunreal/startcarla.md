# CarlaUnreal Docker 启动指南

## 前置条件

```bash
# 允许容器访问 X11 显示
xhost +local:
```

## 单实例启动（docker run）

### 渲染模式 + ROS2

```bash
sudo docker run \
  --runtime=nvidia \
  --net=host \
  --ipc=host \
  --user=$(id -u):$(id -g) \
  --env=DISPLAY=$DISPLAY \
  --env=QT_X11_NO_MITSHM=1 \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -it --rm \
  carla:0.10.0 bash Linux/CarlaUnreal.sh -nosound --ros2
```

### 无渲染模式 + ROS2

```bash
sudo docker run \
  --runtime=nvidia \
  --net=host \
  --ipc=host \
  --user=$(id -u):$(id -g) \
  --env=DISPLAY=$DISPLAY \
  --env=QT_X11_NO_MITSHM=1 \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -it --rm \
  carla:0.10.0 bash Linux/CarlaUnreal.sh -nosound -RenderOffScreen --ros2
```

默认监听端口：2000（RPC）/ 2001（Streaming）/ 2002（Secondary）

## 多实例并行启动（docker compose）

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/Util/Docker/Carlaunreal
```

### 渲染模式（双实例，端口 2000 / 2010）

```bash
sudo UID=$(id -u) GID=$(id -g) docker compose up -d
```

### 无渲染模式（双实例，端口 2000 / 2010）

```bash
sudo UID=$(id -u) GID=$(id -g) docker compose -f docker-compose-Renderoffscreen.yml up -d
```

### 管理命令

```bash
# 查看状态
sudo docker compose ps

# 查看日志
sudo docker compose logs -f carla1
sudo docker compose logs -f carla2

# 停止全部
sudo docker compose down
```

## 客户端连接

```python
import carla

client1 = carla.Client('localhost', 2000)  # 实例1
client2 = carla.Client('localhost', 2010)  # 实例2

client1.set_timeout(30)
client2.set_timeout(30)
```

## 参数说明

| 参数 | 说明 |
|------|------|
| `-nosound` | 禁用音频 |
| `-RenderOffScreen` | 无头渲染（不弹窗） |
| `--ros2` | 启用 ROS2 通信 |
| `-carla-port=N` | 指定 RPC 端口（Streaming=N+1, Secondary=N+2） |

## 文件说明

| 文件 | 用途 |
|------|------|
| `docker-compose.yml` | 渲染模式，双实例 |
| `docker-compose-Renderoffscreen.yml` | 无渲染模式，双实例 |

## 注意事项

- 使用 `network_mode: host`，容器直接共享宿主机网络，无需端口映射
- 多实例端口间隔建议 ≥10（避免 RPC/Streaming/Secondary 冲突）
- 每个实例约占 2-4 GB 显存，注意 GPU 资源
