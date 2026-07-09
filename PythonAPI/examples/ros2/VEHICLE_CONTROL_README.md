# CARLA UE5 ROS2 车辆控制测试指南

## 背景

CARLA UE5 版本已经内置了 ROS2 Native 支持，**不需要外部的 carla_bridge**。所有消息类型定义都在项目内部：

- C++ 侧：`LibCarla/source/carla/ros2/types/`
- Subscriber：`LibCarla/source/carla/ros2/subscribers/CarlaEgoVehicleControlSubscriber.cpp`

## 问题

Python 端无法直接访问 C++ 定义的消息类型（没有 Python 绑定），所以之前的 `ros2_vehicle_control.py` 脚本需要外部的 `carla_msgs` 包。

## 解决方案

### 方案 1：使用 ros2 topic pub 命令行工具（最简单）

```bash
# Source ROS2 环境
source /opt/ros/humble/setup.bash
source /home/qiyuan/Xiangrikui/vehicle_msgs/install/setup.bash

# 发布单次控制命令（油门 0.8）
ros2 topic pub --once /rt/carla/hero/vehicle_control_cmd \
    carla_msgs/msg/CarlaEgoVehicleControl \
    "{throttle: 0.8, steer: 0.0, brake: 0.0, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}"

# 发布刹车命令
ros2 topic pub --once /rt/carla/hero/vehicle_control_cmd \
    carla_msgs/msg/CarlaEgoVehicleControl \
    "{throttle: 0.0, steer: 0.0, brake: 0.8, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}"

# 停止
ros2 topic pub --once /rt/carla/hero/vehicle_control_cmd \
    carla_msgs/msg/CarlaEgoVehicleControl \
    "{throttle: 0.0, steer: 0.0, brake: 0.0, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}"
```

### 方案 2：使用交互式 Shell 脚本

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
./test_vehicle_control.sh hero
```

按键盘控制：
- `w` - 油门前进
- `s` - 刹车
- `a` - 左转
- `d` - 右转
- `x` - 停止
- `q` - 退出

### 方案 3：添加 CarlaEgoVehicleControl 到 vehicle_msgs 包

如果你希望继续使用 Python 脚本，需要将消息定义添加到你的 vehicle_msgs 包中：

```bash
# 1. 复制消息定义
cp /home/qiyuan/UnrealEngine/CarlaUE5/LibCarla/source/carla/ros2/types/CarlaEgoVehicleControl.msg \
   /home/qiyuan/Xiangrikui/vehicle_msgs/msg/CarlaEgoVehicleControl.msg

# 注意：C++ 侧的 .h/.cpp 是 FastDDS 生成的，不是标准的 .msg 文件
# 你需要根据 Header.h 中的字段创建标准的 ROS2 .msg 文件
```

但这种方式比较复杂，因为 C++ 侧使用的是 FastDDS 原生类型，不是标准的 ROS2 IDL。

## 推荐测试流程

```bash
# 终端 1: 启动 CARLA UE5
cd /home/qiyuan/UnrealEngine/CarlaUE5/Build
cmake --build Build --target launch

# 终端 2: 运行传感器数据接收
cd /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2
python3 ros2_native.py -f stack.json --delta 0.02

# 终端 3: 发送控制命令（使用方案 2）
./test_vehicle_control.sh hero
```

## Topic 命名规则

根据 [ROS2.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/LibCarla/source/carla/ros2/ROS2.cpp#L180) 的实现：

- **控制命令**: `/rt/carla/{role_name}/vehicle_control_cmd`
- **传感器数据**: `/rt/carla/{role_name}/{sensor_type}`

例如：
- `/rt/carla/hero/vehicle_control_cmd` - 控制英雄车辆
- `/rt/carla/hero/INS` - INS 数据
- `/rt/carla/hero/VEHICLE_STATE` - 车辆状态
- `/rt/carla/hero/OBSTACLE_LIST` - 障碍物列表
