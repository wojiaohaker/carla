# CARLA UE5 ROS2 车辆控制 - 快速开始指南

## 概述

本指南介绍如何使用 ROS2 消息控制 CARLA UE5 中的车辆，**无需 carla_bridge**。

## 前提条件

### 1. 安装依赖

```bash
# 安装 colcon（ROS2 构建工具）
sudo apt install python3-colcon-common-extensions
```

### 2. 编译 carla_msgs

```bash
cd /home/qiyuan/Xiangrikui/carla_msgs
colcon build --packages-select carla_msgs
```

### 3. Source 环境

```bash
source /opt/ros/humble/setup.bash
source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash
```

**建议添加到 `~/.bashrc`：**

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash' >> ~/.bashrc
```

### 4. 验证消息类型

```bash
ros2 interface show carla_msgs/msg/CarlaEgoVehicleControl
```

应该看到消息定义（包含 throttle, steer, brake 等字段）。

## 使用方法

### 方式 1：Python 脚本（推荐）

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
python3 ros2_vehicle_control.py --role-name hero
```

**键盘控制：**
- `w` - 油门前进（0.8）
- `s` - 刹车（0.8）
- `a` - 左转（-0.5）
- `d` - 右转（0.5）
- `x` - 停止
- `q` - 退出

### 方式 2：Shell 脚本

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
./test_vehicle_control.sh hero
```

### 方式 3：直接使用 ros2 topic pub

```bash
# 单次发布命令
ros2 topic pub --once /rt/carla/hero/vehicle_control_cmd \
    carla_msgs/msg/CarlaEgoVehicleControl \
    "{throttle: 0.8, steer: 0.0, brake: 0.0, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}"

# 持续发布（10Hz）
ros2 topic pub -r 10 /rt/carla/hero/vehicle_control_cmd \
    carla_msgs/msg/CarlaEgoVehicleControl \
    "{throttle: 0.5, steer: 0.0, brake: 0.0, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}"
```

## 完整测试流程

### 步骤 1：启动 CARLA UE5

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/Build
cmake --build Build --target launch
```

等待编辑器完全加载。

### 步骤 2：生成测试场景（可选）

在 UE5 编辑器中：
1. 放置一辆车辆（如 Lincoln MKZ）
2. 设置其 Role Name 为 "hero"
3. 确保车辆已注册到 ROS2 系统

或者使用 Python API 生成车辆：

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
python3 ros2_native.py -f stack.json --delta 0.02
```

### 步骤 3：运行控制脚本

在另一个终端：

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
python3 ros2_vehicle_control.py --role-name hero
```

### 步骤 4：验证通信

在第三个终端查看 topics：

```bash
# 查看所有 topics
ros2 topic list | grep hero

# 应该看到：
# /rt/carla/hero/vehicle_control_cmd  (Subscriber)
# /rt/carla/hero/INS                  (Publisher - 如果你有 VehicleDataSensor)
# /rt/carla/hero/VEHICLE_STATE        (Publisher)
# /rt/carla/hero/OBSTACLE_LIST        (Publisher)
```

## Topic 命名规则

根据 [ROS2.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/LibCarla/source/carla/ros2/ROS2.cpp#L180) 的实现：

| 方向 | Topic 路径 | 说明 |
|------|-----------|------|
| **下行**（控制） | `/rt/carla/{role_name}/vehicle_control_cmd` | 发送控制命令到 CARLA |
| **上行**（传感器） | `/rt/carla/{role_name}/{sensor_type}` | CARLA 发布传感器数据 |

例如：
- `/rt/carla/hero/vehicle_control_cmd` - 控制英雄车辆
- `/rt/carla/hero/INS` - INS 数据（你的自定义传感器）
- `/rt/carla/hero/VEHICLE_STATE` - 车辆状态
- `/rt/carla/hero/OBSTACLE_LIST` - 障碍物列表

## 消息格式

### CarlaEgoVehicleControl

```yaml
header:
  stamp:
    sec: <timestamp_seconds>
    nanosec: <timestamp_nanoseconds>
  frame_id: "<frame_id>"
throttle: 0.0      # 0.0 - 1.0
steer: 0.0         # -1.0 - 1.0 (左负右正)
brake: 0.0         # 0.0 - 1.0
hand_brake: false  # true/false
reverse: false     # true/false
gear: 1            # 档位（1=前进，-1=倒车）
manual_gear_shift: false  # 是否手动换挡
```

## 常见问题

### Q1: 提示 "carla_msgs.msg import error"

**解决：** 确保已经 source 了 carla_msgs 环境：

```bash
source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash
```

### Q2: 车辆没有响应控制命令

**检查清单：**
1. CARLA UE5 是否正在运行？
2. 车辆是否正确注册（Role Name 匹配）？
3. Topic 名称是否正确（`/rt/carla/{role_name}/vehicle_control_cmd`）？
4. 是否有其他控制器也在发布控制命令（冲突）？

验证 Subscriber 是否存在：

```bash
ros2 topic info /rt/carla/hero/vehicle_control_cmd
# 应该显示 Subscription count: 1 或更多
```

### Q3: 如何同时接收传感器数据和发送控制命令？

打开两个终端：

```bash
# 终端 1: 接收传感器数据
python3 ros2_native.py -f stack.json --delta 0.02

# 终端 2: 发送控制命令
python3 ros2_vehicle_control.py --role-name hero
```

## 技术细节

### C++ 侧实现

- **消息定义**: `LibCarla/source/carla/ros2/types/CarlaEgoVehicleControl.h`
- **Subscriber**: `LibCarla/source/carla/ros2/subscribers/CarlaEgoVehicleControlSubscriber.cpp`
- **注册逻辑**: `LibCarla/source/carla/ros2/ROS2.cpp` (第 189 行)

### Python 侧实现

- **脚本**: `PythonAPI/examples/ros2/ros2_vehicle_control.py`
- **消息包**: `/home/qiyuan/Xiangrikui/carla_msgs`
- **Topic**: `/rt/carla/{role_name}/vehicle_control_cmd`

### 与 UE4 版本的区别

- **UE4**: 需要外部的 `carla_ros_bridge` 包
- **UE5**: 内置 ROS2 Native 支持，只需 `carla_msgs` 提供消息定义

## 相关文档

- [CARLA UE5 ROS2 Native 文档](file:///home/qiyuan/UnrealEngine/CarlaUE5/Docs/ros2_native.md)
- [VehicleDataSensor 实现](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Sensor/VehicleDataSensor.cpp)
- [ROS2 消息类型列表](file:///home/qiyuan/UnrealEngine/CarlaUE5/LibCarla/source/carla/ros2/types/)
