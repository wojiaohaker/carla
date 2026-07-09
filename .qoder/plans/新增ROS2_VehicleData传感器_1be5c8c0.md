# 新增 ROS2 VehicleData 传感器

## 整体架构

```
UE Sensor Actor (VehicleDataSensor)
  |-- PostPhysTick 采集数据
  |-- 调用 ROS2::ProcessDataFromVehicleData(...)
        |-- CarlaInsDataPublisher      -> Topic: INS
        |-- CarlaVehicleStatePublisher -> Topic: VEHICLE_STATE
        |-- CarlaObstacleListPublisher -> Topic: OBSTACLE_LIST
```

涉及 4 个代码层，自底向上：

---

## Task 1: 创建 FastDDS C++ 消息类型

目录: `LibCarla/source/carla/ros2/types/`

参照 `CarlaCollisionEvent.h/.cpp` + `CarlaCollisionEventPubSubTypes.h/.cpp` 的模式，为每个自定义消息创建 3 个文件（消息结构体 + PubSubType）。命名空间使用 `vehicle_msgs::msg`。

### 1.1 InsData 类型
- `InsData.h` / `InsData.cpp` / `InsDataPubSubTypes.h` / `InsDataPubSubTypes.cpp`
- 字段: header, longitude(double), latitude(double), altitude(float), yaw/pitch/roll(float), vx/vy/vt/psd(float), r/p/q(float), gps_fix_state/satellite_num/gps_id/nav_state(uint8), nav_fault_code(uint16), is_valid(bool), timestamp(double)
- 依赖已有的 `Header.h`

### 1.2 VehicleState 类型
- `VehicleState.h` / `VehicleState.cpp` / `VehicleStatePubSubTypes.h` / `VehicleStatePubSubTypes.cpp`
- 字段: header, work_state/work_mode/control_model(uint8), battery_capacity(uint8), voltage/current(uint16), speed/angle/brake(float), fault_code(uint16)

### 1.3 ObstacleItem 类型
- `ObstacleItem.h` / `ObstacleItem.cpp` / `ObstacleItemPubSubTypes.h` / `ObstacleItemPubSubTypes.cpp`
- 字段: id(int32), type(uint8), x/y/length/width/height/course/speed(double)

### 1.4 ObstacleList 类型
- `ObstacleList.h` / `ObstacleList.cpp` / `ObstacleListPubSubTypes.h` / `ObstacleListPubSubTypes.cpp`
- 字段: header, obstacles(vector of ObstacleItem)
- 注意: 包含动态数组，需要处理 CDR 序列化的序列长度字段

每个类型都需要:
- 构造函数/析构函数、拷贝/移动构造、赋值运算符、比较运算符
- getter/setter 方法
- `serialize()` / `deserialize()` / `getCdrSerializedSize()` / `getMaxCdrSerializedSize()`
- `serializeKey()` / `isKeyDefined()` / `getKeyMaxCdrSerializedSize()`
- PubSubType 类继承 `TopicDataType`，实现 serialize/deserialize/getSerializedSizeProvider/getKey/createData/deleteData

---

## Task 2: 创建 ROS2 Publisher

目录: `LibCarla/source/carla/ros2/publishers/`

参照 `CarlaGNSSPublisher` / `CarlaCollisionPublisher` 的模式，创建 3 个 Publisher。

### 2.1 CarlaInsDataPublisher
- `CarlaInsDataPublisher.h` / `CarlaInsDataPublisher.cpp`
- Traits: `msg_type = vehicle_msgs::msg::InsData`, `msg_pubsub_type = vehicle_msgs::msg::InsDataPubSubType`
- `Write()` 参数: seconds, nanoseconds, 以及 InsData 所有字段（或传入一个结构体）

### 2.2 CarlaVehicleStatePublisher
- `CarlaVehicleStatePublisher.h` / `CarlaVehicleStatePublisher.cpp`
- Traits: `msg_type = vehicle_msgs::msg::VehicleState`

### 2.3 CarlaObstacleListPublisher
- `CarlaObstacleListPublisher.h` / `CarlaObstacleListPublisher.cpp`
- Traits: `msg_type = vehicle_msgs::msg::ObstacleList`
- Write 需要接受 obstacles 数组

---

## Task 3: 扩展 ROS2 门面层

### 3.1 修改 `LibCarla/source/carla/ros2/ROS2.h`
- 前向声明: `class CarlaInsDataPublisher; class CarlaVehicleStatePublisher; class CarlaObstacleListPublisher;`
- 新增方法:
```cpp
void ProcessDataFromVehicleData(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    // InsData fields
    double longitude, double latitude, float altitude,
    float yaw, float pitch, float roll,
    float vx, float vy, float vt, float psd,
    float r, float p, float q,
    uint8_t gps_fix_state, uint8_t satellite_num, uint8_t gps_id,
    uint8_t nav_state, uint16_t nav_fault_code,
    bool is_valid, double timestamp,
    // VehicleState fields
    uint8_t work_state, uint8_t work_mode, uint8_t control_model,
    uint8_t battery_capacity, uint16_t voltage, uint16_t current,
    float speed, float angle, float brake, uint16_t fault_code,
    // ObstacleList fields
    const std::vector<ObstacleItemData>& obstacles,
    void *actor = nullptr);
```
- 定义一个简单结构体 `ObstacleItemData` 用于传递障碍物数据（避免 UE 头文件依赖）

### 3.2 修改 `LibCarla/source/carla/ros2/ROS2.cpp`
- 在 `ESensors` 枚举中新增 `VehicleDataSensor`
- 在 `GetOrCreateSensor()` 的 switch 中新增 case，创建 3 个 publisher（注意：一个 sensor 对应 3 个 publisher，需要特殊处理，可以用一个 map 存 3 个 publisher，或者拆成 3 个 GetOrCreateSensor 调用）
- 实现 `ProcessDataFromVehicleData()`，分别调用 3 个 publisher 的 Write + Publish

**设计要点**: 由于一个传感器需要 3 个不同类型的 publisher，建议在 ROS2 类中新增一个 `std::unordered_map<void*, std::array<std::shared_ptr<BasePublisher>, 3>>` 或用 3 个独立的 map 分别存储 INS/VehicleState/ObstacleList publisher。

---

## Task 4: 创建 UE 传感器 Actor

目录: `Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Sensor/`

### 4.1 新建 `VehicleDataSensor.h` / `VehicleDataSensor.cpp`
- 继承 `ASensor`
- 参考 `AGnssSensor` 和 `AInertialMeasurementUnit` 的数据采集模式
- `PostPhysTick()` 中:
  1. **INS 数据**: 从 Owner Vehicle 获取位置（通过 GeoProjection 转经纬度）、姿态（rotation）、速度（velocity 分解到车体系）、角速度（从 IMU 或旋转差分计算）
  2. **VehicleState 数据**: 从 Vehicle 获取速度、转向角、刹车状态；工作模式/电池等用默认值或可配置值
  3. **ObstacleList 数据**: 使用 UE 的 `SphereTraceMulti` 或 `LineTraceMulti` 检测周围障碍物，计算相对位置（车体坐标系），填充 ObstacleItem 列表
- 通过 `#if defined(WITH_ROS2)` 调用 `ROS2::ProcessDataFromVehicleData()`
- 使用 `NoopSerializer`（不需要向 Carla Python 客户端发数据）

### 4.2 关键数据源
- 位置: `GetActorLocation()` + `UCarlaStatics::GetLargeMapManager()` + GeoProjection（同 GnssSensor）
- 姿态: `GetActorRotation()`
- 速度: `GetOwner()->GetVelocity()` + 坐标变换到车体系
- 角速度: 参考 `AInertialMeasurementUnit::ComputeGyroscope()`
- 障碍物: `GetWorld()->SphereTraceMulti()` 检测附近 Actor

---

## Task 5: 注册传感器

### 5.1 修改 `LibCarla/source/carla/sensor/SensorRegistry.h`
- 前向声明: `class AVehicleDataSensor;`
- 注册: `std::pair<AVehicleDataSensor *, s11n::NoopSerializer>`

### 5.2 修改 `ActorBlueprintFunctionLibrary.h/.cpp`
- 新增 `static FActorDefinition MakeVehicleDataDefinition();`
- 定义传感器的 tag（如 `sensor.other.vehicle_data`）和可配置参数（频率、检测半径等）

### 5.3 修改 `LibCarla/source/carla/ros2/ROS2.cpp` 的 `ESensors` 枚举
- 添加 `VehicleDataSensor` 枚举值（注意顺序要与 SensorRegistry 的注册顺序一致）

---

## Task 6: 构建集成

### 6.1 `Ros2Native/LibCarlaRos2Native/CMakeLists.txt`
- 已有的 glob 模式会自动包含 `types/*.cpp` 和 `publishers/*.cpp` 中的新文件，无需修改

### 6.2 `Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Carla.Build.cs`
- 无需修改（已通过 `WITH_ROS2` 宏控制 ROS2 功能）

---

## 文件变更总结

| 层 | 新增文件 | 修改文件 |
|---|---|---|
| FastDDS 类型 | `types/InsData.{h,cpp}`, `types/InsDataPubSubTypes.{h,cpp}` | 无 |
| FastDDS 类型 | `types/VehicleState.{h,cpp}`, `types/VehicleStatePubSubTypes.{h,cpp}` | 无 |
| FastDDS 类型 | `types/ObstacleItem.{h,cpp}`, `types/ObstacleItemPubSubTypes.{h,cpp}` | 无 |
| FastDDS 类型 | `types/ObstacleList.{h,cpp}`, `types/ObstacleListPubSubTypes.{h,cpp}` | 无 |
| Publisher | `publishers/CarlaInsDataPublisher.{h,cpp}` | 无 |
| Publisher | `publishers/CarlaVehicleStatePublisher.{h,cpp}` | 无 |
| Publisher | `publishers/CarlaObstacleListPublisher.{h,cpp}` | 无 |
| ROS2 门面 | 无 | `ROS2.h`, `ROS2.cpp` |
| UE Sensor | `Sensor/VehicleDataSensor.{h,cpp}` | 无 |
| 注册 | 无 | `SensorRegistry.h`, `ActorBlueprintFunctionLibrary.{h,cpp}` |

---

## 注意事项

1. **Topic 命名**: 按消息解析文档，三个 Topic 分别为 `INS`、`VEHICLE_STATE`、`OBSTACLE_LIST`。在 `GetOrCreateSensor` 的 resolve 中对应设置。实际完整 topic 路径为 `rt/carla/<ros_name>/INS` 等。
2. **频率**: 目标 50Hz，通过 UE 的 `PostPhysTick` 驱动（与仿真帧率同步）。
3. **坐标系**: ObstacleList 的 x/y 是车体坐标系（前方为正/左方为正），需要从 UE 世界坐标转换。
4. **VehicleState.speed 约定**: 按消息解析文档中的注释，当前算法适配 `speed` 按 `m/s * 10` 发送。
5. **FastDDS 类型手写工作量大**: 每个类型需要完整的 CDR 序列化/反序列化实现。建议先参照 `CarlaCollisionEvent` 的模板完整复制后修改字段。ObstacleList 含动态数组，序列化稍复杂。
