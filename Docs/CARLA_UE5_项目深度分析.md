# CARLA UE5 项目深度分析

## 目录

- [一、项目规模概览](#一项目规模概览)
- [二、与普通 UE 项目的核心差异（12 个差异点）](#二与普通-ue-项目的核心差异)
- [三、架构总图](#三架构总图)
- [四、可借鉴的设计模式总结（24 种模式）](#四可借鉴的设计模式总结)
- [五、车辆系统 — 多层物理与控制架构](#五车辆系统--多层物理与控制架构)
- [六、行人系统](#六行人系统)
- [七、天气与环境系统](#七天气与环境系统)
- [八、交通管理系统](#八交通管理系统)
- [九、大地图管理系统](#九大地图管理系统)
- [十、OpenDrive 道路数据集成](#十opendrive-道路数据集成)
- [十一、Trigger 系统 — 区域效果](#十一trigger-系统--区域效果)
- [十二、环境对象系统](#十二环境对象系统)
- [十三、LibCarla 跨平台库详解](#十三libcarla-跨平台库详解)
- [十四、PythonAPI 客户端架构](#十四pythonapi-客户端架构)
- [十五、关键源文件索引](#十五关键源文件索引)
- [十六、PythonAPI Examples — 官方示例脚本解析](#十六pythonapi-examples--官方示例脚本解析)
- [十七、Util 工具链 — 构建与运维体系](#十七util-工具链--构建与运维体系)

## 一、项目规模概览

| 层级 | 代码量 | 文件数 | 职责 |
|------|--------|--------|------|
| **Carla 插件** (UE 端) | ~67,500 行 | 392 个 (220 .h + 172 .cpp) | UE 引擎内的仿真逻辑 |
| **LibCarla** (跨平台库) | ~89,200 行 | 635 个 | RPC 协议、ROS2、流传输、道路解析、交通管理 |
| **CarlaUnreal 模块** (项目壳) | ~100 行 | 4 个 | 仅做模块注册，几乎无逻辑 |
| **PythonAPI** | ~15,000 行 | 50+ 个 | Python 客户端接口 + Agent 系统 |

### 目录结构总览

```
CarlaUE5/
├── CMakeLists.txt              ← 顶层 CMake，编排 LibCarla + UE 构建
├── LibCarla/                    ← 跨平台 C++ 库（不依赖 UE 头文件）
│   └── source/carla/
│       ├── rpc/                 ← 序列化协议定义（Actor/Weather/Control 等）
│       ├── ros2/                ← ROS2 原生发布器（非 bridge）
│       ├── streaming/           ← Boost.Asio 数据流服务端
│       ├── road/                ← OpenDRIVE 道路网络解析
│       ├── multigpu/            ← 多 GPU Primary/Secondary 路由
│       ├── sensor/              ← 传感器数据序列化注册表
│       ├── trafficmanager/      ← 交通管理器（AI 车辆/行人）
│       ├── image/               ← 图像后处理（深度解码、语义标签映射）
│       └── geom/                ← 几何工具（GeoProjection、BoundingBox）
│
├── Unreal/CarlaUnreal/          ← UE 项目（薄壳）
│   ├── CarlaUnreal.uproject     ← 项目描述，启用 Carla 插件
│   ├── Source/                  ← 仅 4 个文件（Build.cs + Target.cs）
│   ├── Config/                  ← 引擎/游戏配置
│   ├── Content/Carla/           ← 美术资源（车辆、行人、地图、蓝图）
│   └── Plugins/
│       ├── Carla/               ← 核心插件（67000+ 行）
│       │   └── Source/Carla/
│       │       ├── Game/        ← GameMode/Episode/Engine/Instance
│       │       ├── Server/      ← RPC 服务端
│       │       ├── Actor/       ← ActorRegistry/Dispatcher/Factory
│       │       ├── Sensor/      ← 传感器框架（20+ 传感器类型）
│       │       ├── Vehicle/     ← 车辆物理与控制
│       │       ├── Walker/      ← 行人控制
│       │       ├── Traffic/     ← 交通灯/标志管理
│       │       ├── Weather/     ← 动态天气
│       │       ├── Recorder/    ← 仿真录制/回放
│       │       ├── OpenDrive/   ← 道路数据集成
│       │       ├── MapGen/      ← 大地图管理
│       │       └── Settings/    ← 仿真参数
│       ├── CarlaExporter/       ← 资产导出工具
│       ├── CarlaTools/          ← 编辑器工具
│       └── StreetMap/           ← OpenStreetMap 导入
│
└── PythonAPI/carla/             ← Python 客户端库
```

---

## 二、与普通 UE 项目的核心差异

### 1. "薄壳项目 + 重插件" 架构

**普通 UE 项目**：逻辑写在项目的 `Source/` 目录下，插件是可选辅助。

**CARLA**：`Source/CarlaUnreal/` 只有 4 个文件，仅用于注册模块。所有核心能力都在 `Plugins/Carla/` 中。

```
普通 UE 项目:
  Source/MyGame/
    ├── MyGame.Build.cs       ← 依赖声明
    ├── MyGameMode.cpp         ← 大量游戏逻辑
    ├── MyCharacter.cpp        ← 角色控制
    └── ...

CARLA 项目:
  Source/CarlaUnreal/
    ├── CarlaUnreal.Build.cs   ← 仅依赖声明（51 行）
    ├── CarlaUnreal.cpp         ← 空实现
    ├── CarlaUnreal.Target.cs   ← 构建目标（37 行）
    └── CarlaUnrealEditor.Target.cs ← 编辑器目标（37 行）

  Plugins/Carla/Source/Carla/   ← 67000+ 行核心代码
    ├── Game/ (10+ 文件)
    ├── Actor/ (20+ 文件)
    ├── Sensor/ (40+ 文件)
    └── ...
```

**uproject 配置**：

```json
{
  "Modules": [
    {
      "Name": "CarlaUnreal",
      "Type": "Runtime",
      "AdditionalDependencies": ["Engine", "Carla", "CoreUObject"]
    },
    {
      "Name": "CarlaDeviceProfileSelector",
      "Type": "Runtime",
      "LoadingPhase": "PostConfigInit"
    }
  ],
  "Plugins": [
    { "Name": "Carla", "Enabled": true },
    { "Name": "PythonScriptPlugin", "Enabled": true },
    { "Name": "GeoReferencing", "Enabled": true }
  ]
}
```

**设计优势**：
- 插件可跨项目复用：任何 UE 项目启用 `Carla` 插件即可获得仿真能力
- 关注点分离：UE 项目只负责资源管理和启动配置
- 独立版本控制：插件可以独立于项目升级

**学到什么**：当你的项目需要被多个 UE 项目复用时，把逻辑做成插件而非项目模块。

---

### 2. 自研 Episode 模型取代 UE 默认关卡流

**普通 UE 项目**：`UGameInstance` → `AGameMode` → `ULevel`，用 `OpenLevel()` 切换地图。

**CARLA**：引入了完整的仿真会话抽象：

```
UCarlaGameInstance (跨关卡持久)
  │
  ├─ FCarlaEngine              ← 全局引擎实例
  │    ├─ FCarlaServer         ← RPC 服务端（Boost.Asio）
  │    ├─ FWorldObserver       ← 世界状态观察器
  │    ├─ UCarlaEpisode*       ← 当前仿真 Episode
  │    ├─ FEpisodeSettings     ← 仿真参数
  │    ├─ ACarlaRecorder*      ← 录制器
  │    └─ Secondary/Router     ← 多 GPU 路由
  │
  └─ UCarlaSettings            ← 全局仿真设置
```

#### UCarlaEpisode — 仿真会话核心

```cpp
// CarlaEpisode.h (413 行)
UCLASS(BlueprintType, Blueprintable)
class UCarlaEpisode : public UObject
{
  // --- Actor 管理 ---
  UActorDispatcher *ActorDispatcher;     // 工厂分发 + Registry
  FActorRegistry &GetActorRegistry();    // 获取注册表
  FCarlaActor* FindCarlaActor(IdType);   // 按 ID 查找

  // --- Actor 生命周期 ---
  TPair<EActorSpawnResultStatus, FCarlaActor*> SpawnActorWithInfo(...);
  bool DestroyActor(FCarlaActor::IdType);
  void AttachActors(AActor *Child, AActor *Parent);
  void PutActorToSleep(IdType);          // 休眠（交通管理器用）
  void WakeActorUp(IdType);              // 唤醒

  // --- 仿真状态 ---
  FEpisodeSettings EpisodeSettings;      // 同步模式、帧率等
  double ElapsedGameTime;                // 仿真时间
  double VisualGameTime;                 // 视觉时间（云/特效用）

  // --- 传感器 ---
  FSensorManager SensorManager;          // 传感器生命周期
  FFrameData FrameData;                  // 帧数据缓冲

  // --- ROS2 ---
  #if WITH_ROS2
  carla::ros2::ROS2 ROS2;               // 原生 ROS2 实例
  #endif

  // --- 地图 ---
  std::optional<carla::road::Map> Map;   // OpenDRIVE 道路数据
  carla::geom::GeoProjection MapGeoProjection; // 地理投影
};
```

#### FEpisodeSettings — 仿真参数

```cpp
struct FEpisodeSettings
{
  bool bSynchronousMode = false;       // 同步模式（等待客户端 tick）
  bool bNoRenderingMode = false;       // 无渲染模式（headless）
  bool bSubstepping = true;            // 物理子步进
  TOptional<double> FixedDeltaSeconds; // 固定帧间隔
  double MaxSubstepDeltaTime = 0.01;   // 最大物理子步长
  int MaxSubsteps = 10;                // 最大子步数
  float MaxCullingDistance = 0.0f;     // 最大裁剪距离
  bool bDeterministicRagdolls = true;  // 确定性布娃娃
  float TileStreamingDistance = 300000.f; // 大地图流送距离 (3km)
  float ActorActiveDistance = 200000.f;   // Actor 激活距离 (2km)
  bool SpectatorAsEgo = true;          // 观察者作为自车
};
```

**学到什么**：对于仿真/数字孪生类项目，用 Episode 概念封装一次完整实验比直接操作 Level 更清晰。Episode 包含完整的 Actor 管理、仿真参数、传感器状态、录制器，是一个自包含的仿真上下文。

---

### 3. 内嵌 RPC Server — UE 变成"被控服务端"

**普通 UE 项目**：游戏逻辑在客户端本地运行。

**CARLA**：UE 进程是一个**等待外部指令的 RPC 服务端**。

#### 服务端架构

```cpp
// CarlaServer.h (59 行) — Pimpl 模式隔离 Boost 依赖
class FCarlaServer
{
public:
  FDataMultiStream Start(uint16_t RPCPort, uint16_t StreamingPort, uint16_t SecondaryPort);
  void NotifyBeginEpisode(UCarlaEpisode &Episode);
  void NotifyEndEpisode();
  void AsyncRun(uint32 NumberOfWorkerThreads);  // 后台线程运行 RPC
  void Tick();                                    // 每帧处理命令
  FDataStream OpenStream() const;                 // 为传感器开数据流
  std::shared_ptr<carla::multigpu::Router> GetSecondaryServer();

private:
  class FPimpl;                        // 隐藏 Boost.Asio 细节
  TUniquePtr<FPimpl> Pimpl;
};
```

#### RPC 调用链路

```
Python Client                 UE 进程内
─────────────                 ─────────
world.spawn_actor(bp, transform)
        │
        │  RPC (Boost.Asio TCP)
        ▼
FCarlaServer::Tick()
        │
        │  反序列化 ActorDescription
        ▼
UCarlaEpisode::SpawnActorWithInfo()
        │
        │  查找匹配的 Factory
        ▼
UActorDispatcher::SpawnActor()
        │
        │  调用绑定的 SpawnFunction
        ▼
AVehicleActorFactory::SpawnActor()
        │
        │  UE World->SpawnActor<ACarlaWheeledVehicle>()
        ▼
FActorRegistry::Register()  ← 注册到 Registry
        │
        │  返回 ActorId
        ▼
Python Client 收到 Actor 对象
```

**学到什么**：UE 不仅能做客户端渲染，还能作为 headless 仿真服务端。Pimpl 模式让 Boost 依赖不污染 UE 头文件。

---

### 4. ActorRegistry — 自研 Actor 生命周期管理

**普通 UE 项目**：通过 `UWorld::GetActorList()` 或 `TActorIterator` 遍历场景 Actor。

**CARLA**：自建了 `FActorRegistry`，完整的 Actor 代理体系。

#### 数据结构

```cpp
// ActorRegistry.h (134 行)
class FActorRegistry
{
  // 三张表维护 Actor 关系
  TMap<IdType, AActor *> Actors;                    // Id → UE Actor 指针
  TMap<AActor *, IdType> Ids;                       // UE Actor → Id（反向索引）
  TMap<IdType, TSharedPtr<FCarlaActor>> ActorDatabase; // Id → 代理对象

  static IdType ID_COUNTER;  // 全局递增 ID
};
```

#### FCarlaActor — 轻量代理类

```cpp
// CarlaActor.h (640 行) — 多态代理体系
class FCarlaActor
{
  // --- 身份 ---
  IdType Id;
  ActorType Type;              // Vehicle/Walker/TrafficLight/TrafficSign/Sensor/Other
  AActor *TheActor;            // UE Actor 指针
  TSharedPtr<const FActorInfo> Info;  // 描述 + 包围盒 + 语义标签

  // --- 状态机 ---
  carla::rpc::ActorState State;  // Active/Dormant/PendingKill/Invalid

  // --- 父子关系 ---
  IdType ParentId;
  TArray<IdType> Children;
  carla::rpc::AttachmentType Attachment;

  // --- 通用接口（所有类型共享）---
  FVector GetActorVelocity() const;
  void SetActorLocalTransform(const FTransform&);
  ECarlaServerResponse SetActorSimulatePhysics(bool);

  // --- 类型特定接口（虚函数，默认返回 ActorTypeMismatch）---
  virtual ECarlaServerResponse ApplyControlToVehicle(...) { return ActorTypeMismatch; }
  virtual ECarlaServerResponse SetTrafficLightState(...) { return ActorTypeMismatch; }
  virtual ECarlaServerResponse ApplyControlToWalker(...) { return ActorTypeMismatch; }
};

// 子类只 override 自己类型的方法
class FVehicleActor : public FCarlaActor {
  ECarlaServerResponse ApplyControlToVehicle(...) final;  // ✓ 实现
  ECarlaServerResponse SetTrafficLightState(...) final;   // ✗ 不会调用
};

class FWalkerActor : public FCarlaActor {
  ECarlaServerResponse ApplyControlToWalker(...) final;
  ECarlaServerResponse GetBonesTransform(...) final;      // 骨骼控制
};

class FTrafficLightActor : public FCarlaActor {
  ECarlaServerResponse SetTrafficLightState(...) final;
  ECarlaServerResponse FreezeTrafficLight(bool) final;
};
```

#### FActorInfo — 元数据缓存

```cpp
// ActorInfo.h (33 行)
struct FActorInfo
{
  FActorDescription Description;           // 蓝图 ID + 属性 Variations
  TSet<carla::rpc::CityObjectLabel> SemanticTags;  // 语义标签
  FBoundingBox BoundingBox;                // 包围盒（spawn 时计算一次）
  carla::rpc::Actor SerializedData;        // 序列化缓存
  mutable FVector Velocity;                // 速度缓存
};
```

#### ActorType 自动判定

```cpp
// ActorRegistry.cpp — 通过 Cast<> 链自动分类
static FCarlaActor::ActorType FActorRegistry_GetActorType(const AActor *Actor)
{
  if (Cast<ACarlaWheeledVehicle>(Actor))  return Vehicle;
  if (Cast<ACharacter>(Actor))            return Walker;
  if (Cast<ATrafficLightBase>(Actor))     return TrafficLight;
  if (Cast<ATrafficSignBase>(Actor))      return TrafficSign;
  if (Cast<ASensor>(Actor))              return Sensor;
  else                                    return Other;  // Prop/StaticMesh 等
}
```

**设计优势**：
- **性能**：O(1) 按 Id 查找，避免 O(N) 遍历 World
- **解耦**：RPC 层操作 `FCarlaActor` 代理，不直接依赖 UE Actor
- **类型安全**：虚函数 + `ActorTypeMismatch` 返回值防止类型误操作
- **序列化**：`FActorInfo` 缓存了序列化所需的全部元数据

**学到什么**：当你需要从外部管理 UE 中的 Actor 时，维护一个 Registry + 代理类体系比遍历 World 高效得多。

---

### 5. Actor Factory 体系 — 数据驱动的 Actor 创建

**普通 UE 项目**：直接在代码中 `SpawnActor<MyClass>()`。

**CARLA**：通过 Factory + Description 模式，让外部 RPC 请求决定创建什么 Actor。

#### 工厂继承体系

```
ACarlaActorFactory (抽象基类)
  │  virtual GetDefinitions() → TArray<FActorDefinition>
  │  virtual SpawnActor(Transform, ActorDescription) → FActorSpawnResult
  │
  ├── AVehicleActorFactory    ← 读取 VehicleParameters.json
  ├── AWalkerActorFactory     ← 读取 WalkerParameters.json
  ├── APropActorFactory       ← 读取 PropParameters.json
  └── ABlueprintActorFactory  ← 从蓝图定义创建
```

#### 绑定流程

```cpp
// ActorDispatcher.h — 工厂 → 生成函数绑定
class UActorDispatcher : public UObject
{
  using SpawnFunctionType = TFunction<FActorSpawnResult(const FTransform&, const FActorDescription&)>;

  void Bind(ACarlaActorFactory &Factory);  // 注册工厂的所有 Definition

  TPair<EActorSpawnResultStatus, FCarlaActor*> SpawnActor(
      const FTransform &Transform,
      FActorDescription ActorDescription);  // RPC 调用时匹配 Definition 并 spawn

  FActorRegistry Registry;                  // spawn 后自动注册
};
```

**Prop 工厂示例**：

```cpp
// PropActorFactory.cpp
FActorSpawnResult APropActorFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  // 1. 从 Description 获取 mesh_path 属性
  FString MeshPath = ActorDescription.Variations["mesh_path"].Value;

  // 2. UE 原生 spawn
  AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(
      ActorDescription.Class, SpawnAtTransform, SpawnParameters);

  // 3. 设置网格体
  StaticMeshComponent->SetStaticMesh(LoadMesh(MeshPath));

  // 4. 返回结果 → Dispatcher 自动注册到 Registry
  return { EActorSpawnResultStatus::Success, Actor };
}
```

**学到什么**：Factory + Description 模式让 Actor 创建完全数据驱动，外部客户端只需发送描述符即可创建任意类型的 Actor。

---

### 6. 传感器框架 — 流式数据管线

**普通 UE 项目**：传感器就是 `SceneCaptureComponent` 或自定义 Actor，数据在本地使用。

**CARLA**：自建了完整的传感器继承体系，支持流式输出和 ROS2 桥接。

#### 传感器基类设计

```cpp
// Sensor.h (278 行)
UCLASS(Abstract)
class ASensor : public AActor
{
  // --- Tick 策略 ---
  void Tick(const float DeltaTime) final;              // 锁定，子类不能 override
  virtual void PrePhysTick(float) {}                    // 物理前钩子
  virtual void PostPhysTick(UWorld*, ELevelTick, float) {} // 物理后钩子（子类实现）

  // --- 数据流 ---
  FDataStream Stream;                                   // 同步流
  void SetDataStream(FDataStream InStream);
  auto GetToken() const { return Stream.GetToken(); }   // 客户端订阅 token

  template <typename SensorType>
  FAsyncDataStream GetDataStream(SensorType&& Self) {   // 创建异步流
    return Stream.MakeAsyncDataStream(Self, GetEpisode().GetElapsedGameTime());
  }

  // --- 数据发送（模板，支持任意传感器类型）---
  template <typename SensorType, typename ElementType>
  static void SendDataToClient(
      SensorType&& Sensor,
      TArrayView<ElementType> SensorData,
      uint64_t FrameIndex)
  {
    // 1. 检查是否有客户端监听
    if (!Sensor.AreClientsListening()) return;

    // 2. 从 Buffer Pool 取缓冲区（零拷贝）
    auto Buffer = Stream.PopBufferFromPool();
    Buffer.copy_from(HeaderOffset, ...);

    // 3. 序列化
    auto Serialized = SensorRegistry::Serialize(Sensor, std::move(Buffer));

#if defined(WITH_ROS2)
    // 4. ROS2 发布（异步线程）
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    if (ROS2->IsEnabled())
    {
      std::async(std::launch::async, [&Sensor, ROS2, ...]() {
        ROS2->ProcessDataFromCamera(SensorType, StreamId, Transform, W, H, Fov, BufferView);
      });
    }
#endif

    // 5. CARLA 原生流发送
    if (Sensor.AreClientsListening())
      Stream.Send(Sensor, BufferView);
  }

  // --- 生命周期回调 ---
  virtual void OnFirstClientConnected() {};
  virtual void OnLastClientDisconnected() {};
};
```

#### 完整传感器列表

| 传感器类 | 类型 | 数据输出 |
|---------|------|---------|
| `SceneCaptureCamera` | RGB 相机 | 图像帧 |
| `DepthCamera` | 深度相机 | 深度图 |
| `SemanticSegmentationCamera` | 语义分割 | 标签图 |
| `InstanceSegmentationCamera` | 实例分割 | 实例标签图 |
| `NormalsCamera` | 法线相机 | 法线图 |
| `OpticalFlowCamera` | 光流相机 | 光流图 |
| `DVSCamera` | 事件相机 | 事件流 |
| `RayCastLidar` | 光线投射 LiDAR | 点云 |
| `RayCastSemanticLidar` | 语义 LiDAR | 带标签点云 |
| `HSSLidar` | 混合固态 LiDAR | 点云（Hesai 建模） |
| `Radar` | 毫米波雷达 | 目标列表 |
| `InertialMeasurementUnit` | IMU | 加速度/角速度 |
| `GnssSensor` | GNSS | 经纬度 |
| `CollisionSensor` | 碰撞传感器 | 碰撞事件 |
| `LaneInvasionSensor` | 车道线传感器 | 车道线入侵事件 |
| `ObstacleDetectionSensor` | 障碍物检测 | 障碍物列表 |
| `VehicleDataSensor` | 车辆数据 | INS/VehicleState/ObstacleList |
| `WorldObserver` | 世界观察器 | 全部 Actor 状态 |

#### SensorManager — 统一调度

```cpp
// SensorManager.h (27 行)
class FSensorManager
{
  void RegisterSensor(ASensor* Sensor);     // BeginPlay 时注册
  void DeRegisterSensor(ASensor* Sensor);   // EndPlay 时注销
  void PostPhysTick(UWorld*, ELevelTick, float); // 统一通知所有传感器
  TArray<ASensor*> SensorList;
};
```

**数据流管线**：

```
UE 物理步进完成
  │
  ▼
FCarlaEngine::OnPostTick()
  │
  ▼
FSensorManager::PostPhysTick()
  │
  ├── ASensor::PostPhysTickInternal()  ← 基类分发
  │     │
  │     ▼
  │   子类::PostPhysTick()  ← 采集数据
  │     │
  │     ▼
  │   SendDataToClient()
  │     │
  │     ├─→ Buffer Pool → CARLA Streaming → Python Client
  │     │
  │     └─→ #if WITH_ROS2 → ROS2 Publisher → ROS2 Topic
  │
  ├── ASensor 2::PostPhysTickInternal()
  └── ASensor N::PostPhysTickInternal()
```

**学到什么**：
- `Tick()` final + `PostPhysTick()` virtual：保证数据在物理步进后采集，时序确定
- Buffer Pool：避免每帧分配内存，零拷贝传输
- ROS2 集成在基类模板方法中：新增传感器自动获得 ROS2 发布能力

---

### 7. 混合构建系统 — CMake + UBT 双轨

**普通 UE 项目**：只用 UBT（UnrealBuildTool）。

**CARLA**：CMake 编排全局，UBT 构建 UE 部分。

#### 构建流程

```
cmake --build Build --target carla
  │
  ├── 1. 构建 LibCarla (CMake + Ninja)
  │     ├── carla/rpc/       → 序列化代码生成
  │     ├── carla/ros2/      → ROS2 发布器
  │     ├── carla/streaming/ → 流传输服务端
  │     └── carla/road/      → OpenDRIVE 解析
  │
  └── 2. 构建 CarlaUnreal (UBT)
        └── Plugins/Carla → 链接 LibCarla 产物

cmake --build Build --target launch
  └── 启动 UE5 编辑器
```

#### Carla.Build.cs — 条件编译

```cpp
public class Carla : ModuleRules
{
  // 命令行开关
  [CommandLine("-ros2")]      bool EnableRos2 = false;
  [CommandLine("-carsim")]    bool EnableCarSim = false;
  [CommandLine("-chrono")]    bool EnableChrono = false;
  [CommandLine("-pytorch")]   bool EnablePytorch = false;

  public Carla(ReadOnlyTargetRules Target) : base(Target)
  {
    // 从 .def 文件动态加载路径
    foreach (var Def in File.ReadAllText("Definitions.def").Split(';'))
      PrivateDefinitions.Add(Def);

    foreach (var Path in File.ReadAllText("Includes.def").Split(';'))
      PublicIncludePaths.Add(Path);

    foreach (var Path in File.ReadAllText("Libraries.def").Split(';'))
      PublicAdditionalLibraries.Add(Path);

    // ROS2 动态链接
    if (EnableRos2)
    {
      PrivateDefinitions.Add("WITH_ROS2=1");
      AddDynamicLibrary("libcarla-ros2-native.so");
      // FastRTPS / FastCDR 运行时依赖
      RuntimeDependencies.Add("libfastrtps.so.2.11.2");
      RuntimeDependencies.Add("libfastcdr.so.1.1.0");
    }
  }
}
```

#### Target.cs — 构建模式控制

```cpp
public class CarlaUnrealTarget : TargetRules
{
  [CommandLine("-unity-build")]
  bool EnableUnityBuild = true;

  public CarlaUnrealTarget(TargetInfo Target) : base(Target)
  {
    Type = TargetType.Game;
    ExtraModuleNames.Add("CarlaUnreal");

    if (!EnableUnityBuild)
    {
      bUseUnityBuild = bForceUnityBuild = bUseAdaptiveUnityBuild = false;
    }
  }
}
```

**学到什么**：
- `.def` 文件让构建路径可配置，避免硬编码
- 条件编译开关让可选功能不增加默认构建负担
- CMake + UBT 双轨适合有跨平台库的项目

---

### 8. 同步模式 — 确定性仿真控制

**普通 UE 项目**：帧率不固定，物理步进由引擎控制。

**CARLA**：支持**同步模式**，实现帧级确定性。

#### 同步模式时序

```
非同步模式（默认）:
  UE: Tick → Tick → Tick → Tick → ...  （尽可能快）
  Client: 随时读取数据

同步模式:
  UE: Tick ──暂停──→ Tick ──暂停──→ Tick
          ↑              ↑             ↑
  Client: step() ──→ step() ──→ step()
          │              │             │
          └── 等待 UE 完成帧 ──┘
```

#### 关键配置

```cpp
// FEpisodeSettings
bool bSynchronousMode = false;       // 启用同步
TOptional<double> FixedDeltaSeconds; // 固定帧间隔（如 0.05 = 20Hz）
double MaxSubstepDeltaTime = 0.01;   // 物理子步长
int MaxSubsteps = 10;                // 最大子步数
```

#### 帧同步机制

```cpp
// CarlaEngine.h
class FCarlaEngine
{
  static uint64_t FrameCounter;

  static uint64_t UpdateFrameCounter()
  {
    FCarlaEngine::FrameCounter += 1;
#if defined(WITH_ROS2)
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    if (ROS2->IsEnabled())
      ROS2->SetFrame(FCarlaEngine::FrameCounter);
#endif
    return FCarlaEngine::FrameCounter;
  }

  void OnPreTick(UWorld*, ELevelTick, float);   // 同步模式下阻塞等待
  void OnPostTick(UWorld*, ELevelTick, float);   // 发送帧数据，通知客户端
};
```

**学到什么**：对于训练数据采集，同步模式保证每一帧的传感器数据和 Actor 状态严格对应。

---

### 9. 多 GPU / Primary-Secondary 架构

**普通 UE 项目**：单进程单 GPU。

**CARLA**：支持多 GPU 分布式渲染。

```
┌──────────────────────┐       ┌──────────────────────┐
│   Primary Server     │       │   Secondary Server   │
│   (GPU 0, 主窗口)    │       │   (GPU 1, 传感器)    │
│                      │       │                      │
│  FCarlaEngine        │  TCP  │  carla::multigpu::   │
│  ├─ 渲染主视图       │◄─────►│  Secondary           │
│  ├─ 处理 RPC 命令    │       │  ├─ 渲染传感器视角   │
│  └─ 分发命令给       │       │  └─ 返回像素数据     │
│     Secondary        │       │                      │
└──────────────────────┘       └──────────────────────┘
```

```cpp
// CarlaEngine.h
class FCarlaEngine
{
  std::shared_ptr<carla::multigpu::Router> SecondaryServer;
  std::shared_ptr<carla::multigpu::Secondary> Secondary;

  std::shared_ptr<carla::multigpu::Router> GetSecondaryServer();
};

// CarlaServer.h
class FCarlaServer
{
  FDataMultiStream Start(uint16_t RPCPort, uint16_t StreamingPort,
                          uint16_t SecondaryPort);  // 第三端口用于多 GPU
};
```

**学到什么**：当传感器数量多到单 GPU 无法承受时（如 8+ 相机），可以将传感器渲染分摊到多 GPU。

---

### 10. 录制回放系统

**普通 UE 项目**：通常没有。

**CARLA**：内建完整的仿真录制/回放系统。

#### 录制数据结构

```cpp
// CarlaRecorder.h (250 行)
enum class CarlaRecorderPacketId : uint8_t
{
  FrameStart = 0,
  FrameEnd,
  EventAdd,          // Actor 创建事件
  EventDel,          // Actor 销毁事件
  EventParent,       // 父子关系变更
  Collision,         // 碰撞事件
  Position,          // Actor 位置/旋转
  State,             // Actor 状态（速度等）
  AnimVehicle,       // 车辆动画
  AnimWalker,        // 行人动画
  VehicleLight,      // 车灯状态
  SceneLight,        // 场景灯光
  Kinematics,        // 运动学数据
  BoundingBox,       // 包围盒
  PlatformTime,      // 平台时间
  PhysicsControl,    // 物理控制参数
  TrafficLightTime,  // 交通灯时间
  WalkerBones,       // 行人骨骼
  Weather,           // 天气变化
  VehicleDoor,       // 车门状态
  AnimBiker,         // 骑行者动画
  // ...
};

UCLASS()
class ACarlaRecorder : public AActor
{
  // 录制接口
  std::string StartRecorder(std::string name, bool AdditionalData);
  void AddEvent(CarlaRecorderEventAdd Event);
  void AddEvent(CarlaRecorderEventDel Event);
  void AddPosition(CarlaRecorderPosition Position);
  void AddCollision(CarlaRecorderCollision Collision);
  void AddWeather(CarlaRecorderWeather Weather);
  // ...

  // 回放接口
  CarlaReplayer *GetReplayer();
};
```

**二进制录制格式**：

```
┌──────────────────────────────────────────┐
│ Header (版本、地图名、起始时间)           │
├──────────────────────────────────────────┤
│ Frame 0                                   │
│   ├─ FrameStart                          │
│   ├─ Position[] (所有 Actor 位置)         │
│   ├─ State[] (所有 Actor 状态)            │
│   ├─ Collision[] (碰撞事件)              │
│   ├─ Weather (天气变化)                   │
│   └─ FrameEnd                            │
├──────────────────────────────────────────┤
│ Frame 1                                   │
│   ├─ ...                                 │
├──────────────────────────────────────────┤
│ ...                                       │
└──────────────────────────────────────────┘
```

**学到什么**：仿真录制是调试和数据复现的关键能力。二进制格式比文本高效得多。

---

### 11. 自定义碰撞通道

`DefaultEngine.ini` 中定义了 CARLA 专用的碰撞通道：

```ini
# 自定义通道
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel1, Name="SensorObject")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel2, Name="SensorTrace")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel3, Name="OverlapChannel")

# 传感器专用碰撞 Profile
+Profiles=(Name="CustomSensorCollision",
    CollisionEnabled=QueryOnly,
    ObjectTypeName="SensorObject",
    CustomResponses=(
        (Channel="WorldStatic", Response=ECR_Ignore),
        (Channel="WorldDynamic", Response=ECR_Ignore),
        (Channel="Pawn", Response=ECR_Ignore),
        (Channel="SensorObject"),
        (Channel="SensorTrace")
    ))
```

**设计意图**：复杂网格体（如精细车辆模型）的物理碰撞体开销大。通过 `CustomSensorCollision`，这些网格体只在传感器光线追踪时响应查询，不参与物理模拟，大幅降低物理计算量。

---

### 12. 渲染配置 — 仿真特化

```ini
# DefaultEngine.ini 关键渲染设置

# 语义分割渲染通道
r.CARLA.EnableSegmentationRendering=1

# 速度输出（光流传感器需要）
r.VelocityOutputPass=True

# 关闭不需要的默认特性
r.DefaultFeature.Bloom=False
r.DefaultFeature.AmbientOcclusion=False
r.DefaultFeature.AutoExposure=False

# Lumen 配置
r.ReflectionMethod=1                    # Lumen 反射
r.DynamicGlobalIlluminationMethod=1     # Lumen GI
r.Lumen.TraceMeshSDFs=1

# 硬件光追（Epic 级别）
r.RayTracing=True
r.Lumen.HardwareRayTracing=False        # 默认关闭，Epic 级别开启

# 纹理流送
r.Streaming.PoolSize=4000
r.Streaming.LimitPoolSizeToVRAM=1

# 距离场
r.GenerateMeshDistanceFields=True
r.DistanceFields.AtlasSizeXY=1024
r.DistanceFields.AtlasSizeZ=2048

# 骨骼蒙皮缓存（从 1024MB 降到 256MB）
r.SkinCache.SceneMemoryLimitInMB=256

# GPU 读回超时（传感器数据读取）
g.TimeoutForBlockOnRenderFence=30000
```

---

## 三、架构总图

```
┌─────────────────────────────────────────────────────────────┐
│                     Python/C++ Client                        │
│            (world.spawn_actor, set_weather...)               │
│            agents/ (basic_agent, behavior_agent)             │
└──────────────────────────┬──────────────────────────────────┘
                           │ RPC (Boost.Asio TCP)
┌──────────────────────────▼──────────────────────────────────┐
│                      FCarlaServer                            │
│              (RPC 解析 + 命令分发 + 流管理)                   │
│         ┌─ RPC Port (2000)                                   │
│         ├─ Streaming Port (自动分配)                          │
│         └─ Secondary Port (多 GPU)                           │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│                     FCarlaEngine                              │
│    ┌───────────────────────────────────────────────────┐     │
│    │               UCarlaEpisode                        │     │
│    │                                                    │     │
│    │  ┌────────────────┐  ┌────────────────────────┐   │     │
│    │  │ ActorDispatcher │  │   FActorRegistry       │   │     │
│    │  │ (Factory 绑定)  │→│   (FCarlaActor 代理)   │   │     │
│    │  └────────────────┘  └────────────────────────┘   │     │
│    │                                                    │     │
│    │  ┌────────────────┐  ┌────────────────────────┐   │     │
│    │  │ SensorManager  │  │   FEpisodeSettings     │   │     │
│    │  │(PostPhysTick)  │  │   (同步/帧率/子步进)   │   │     │
│    │  └────────────────┘  └────────────────────────┘   │     │
│    │                                                    │     │
│    │  ┌────────────────┐  ┌────────────────────────┐   │     │
│    │  │ ACarlaRecorder │  │   carla::road::Map     │   │     │
│    │  │ (录制/回放)    │  │   (OpenDRIVE 道路)     │   │     │
│    │  └────────────────┘  └────────────────────────┘   │     │
│    │                                                    │     │
│    │  ┌────────────────────────────────────────────┐   │     │
│    │  │         场景对象层                           │   │     │
│    │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐   │   │     │
│    │  │  │ Vehicles │ │ Walkers  │ │ Traffic  │   │   │     │
│    │  │  │ (Chaos/  │ │ (骨骼   │ │ Lights/  │   │   │     │
│    │  │  │  CarSim/ │ │  控制)  │ │ Signs    │   │   │     │
│    │  │  │  Chrono) │ │          │ │          │   │   │     │
│    │  │  └──────────┘ └──────────┘ └──────────┘   │   │     │
│    │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐   │   │     │
│    │  │  │ Weather  │ │ LargeMap │ │ Triggers │   │   │     │
│    │  │  │ (天气/  │ │ Manager  │ │ (摩擦/  │   │   │     │
│    │  │  │  昼夜)  │ │ (瓦片/  │ │  区域)   │   │   │     │
│    │  │  │          │ │  Rebase) │ │          │   │   │     │
│    │  │  └──────────┘ └──────────┘ └──────────┘   │   │     │
│    │  └────────────────────────────────────────────┘   │     │
│    └───────────────────────────────────────────────────┘     │
│                                                               │
│    ┌───────────────────────────────────────────────────┐     │
│    │           LibCarla (跨平台 C++ 库)                 │     │
│    │  ┌─────────┐ ┌─────────┐ ┌───────────┐           │     │
│    │  │  rpc/   │ │  ros2/  │ │ streaming/│           │     │
│    │  │MsgPack  │ │原生发布 │ │Boost.Asio │           │     │
│    │  │序列化   │ │(非Bridge)│ │数据流    │           │     │
│    │  └─────────┘ └─────────┘ └───────────┘           │     │
│    │  ┌─────────┐ ┌─────────────────┐ ┌─────────┐   │     │
│    │  │  road/  │ │ trafficmanager/ │ │ sensor/ │   │     │
│    │  │OpenDRIVE│ │ Stage管线      │ │序列化   │   │     │
│    │  │道路解析 │ │ 碰撞→规划→灯  │ │注册表  │   │     │
│    │  └─────────┘ └─────────────────┘ └─────────┘   │     │
│    │  ┌─────────┐ ┌─────────────────┐               │     │
│    │  │  geom/  │ │     image/      │               │     │
│    │  │几何工具 │ │ 图像处理       │               │     │
│    │  └─────────┘ └─────────────────┘               │     │
│    └───────────────────────────────────────────────────┘     │
└──────────────────────────┬──────────────────────────────────┘
                           │
              ┌────────────▼────────────┐
              │     UE5 Engine           │
              │  ├─ 渲染 (Lumen/Nanite) │
              │  ├─ 物理 (Chaos)        │
              │  ├─ AI (TrafficManager) │
              │  └─ 动画 (骨骼/面部)    │
              └─────────────────────────┘
```

---

## 四、可借鉴的设计模式总结

| 模式 | CARLA 做法 | 源码位置 | 适用场景 |
|------|-----------|---------|--------|
| **Episode 封装** | 一次仿真 = 一个 Episode 对象 | `CarlaEpisode.h` | 仿真/数字孪生/训练数据采集 |
| **Registry 代理** | FCarlaActor 缓存元数据，避免遍历 World | `ActorRegistry.h` `CarlaActor.h` | 需要频繁查询 Actor 状态 |
| **Factory + Description** | ActorFactory 绑定 Definition → SpawnFunction | `ActorDispatcher.h` `CarlaActorFactory.h` | 数据驱动的 Actor 创建 |
| **多态代理** | FCarlaActor 子类 override 类型特定方法 | `CarlaActor.h` L477-640 | 统一接口 + 类型安全 |
| **策略模式** | 运动组件可替换（Default/CarSim/Chrono） | `BaseCarlaMovementComponent.h` | 可插拔物理引擎 |
| **优先级仲裁** | EVehicleInputPriority 多源输入覆盖 | `VehicleInputPriority.h` | 多控制源冲突解决 |
| **Pimpl 隔离** | FCarlaServer 用 Pimpl 隔离 Boost 依赖 | `CarlaServer.h` | UE 代码引入第三方库 |
| **条件编译开关** | `WITH_ROS2`, `WITH_CARSIM` 等宏 | `Carla.Build.cs` | 可选功能模块 |
| **Tick 锁定 + 钩子** | Tick() final + PostPhysTick() virtual | `Sensor.h` L118-121 | 传感器数据采集时序保证 |
| **数据流零拷贝** | FDataStream + Buffer Pool | `DataStream.h` `AsyncDataStream.h` | 高频传感器数据传输 |
| **编译期注册表** | SensorRegistry 编译期类型映射 | `SensorRegistry.h` | 传感器序列化分发 |
| **单例模式** | ROS2 全局单例 | `ROS2.h` | 全局资源管理 |
| **Stage 管线** | 碰撞→规划→交通灯→车灯，每阶段独立 | `trafficmanager/` | 复杂 AI 决策管线 |
| **瓦片流送** | LargeMapManager 按需加载/卸载瓦片 | `LargeMapManager.h` | 超大世界支持 |
| **Active/Dormant 状态** | Actor 超出范围自动休眠，进入范围自动唤醒 | `LargeMapManager.h` | 大世界中 Actor 性能优化 |
| **世界原点重定位** | 远离原点时自动 Rebase，保持浮点精度 | `LargeMapManager.h` | 大坐标精度问题 |
| **def 文件配置** | `.def` 文件动态注入构建路径 | `Carla.Build.cs` | 多库混合构建 |
| **双轨构建** | CMake (跨平台库) + UBT (UE 插件) | `CMakeLists.txt` + `*.Build.cs` | 有跨平台依赖的 UE 项目 |
| **自定义碰撞通道** | SensorObject/SensorTrace 专用通道 | `DefaultEngine.ini` | 传感器查询与物理模拟分离 |
| **二进制录制** | 按帧录制 Actor 状态到二进制文件 | `CarlaRecorder.h` | 仿真调试和数据复现 |
| **Trigger 区域效果** | FrictionTrigger 进入/离开时修改物理属性 | `FrictionTrigger.h` | 区域性物理效果 |
| **蓝图可绑定事件** | RefreshWeather / DayTimeChanged | `Weather.h` `CarlaLightSubsystem.h` | UE 蓝图扩展 |

---

## 五、车辆系统 — 多层物理与控制架构

### 1. 车辆继承体系

```
AWheeledVehiclePawn (UE 内置)
  └── ACarlaWheeledVehicle (494 行) ← CARLA 车辆基类
        ├── 物理控制: FVehiclePhysicsControl
        ├── 输入控制: FVehicleControl / FVehicleAckermannControl
        ├── 运动组件: UBaseCarlaMovementComponent（可替换）
        ├── 状态机:   ECarlaWheeledVehicleState（AI 调试）
        ├── 灯光:     FVehicleLightState
        ├── 遥测:     FVehicleTelemetryData
        └── 地形物理: UCustomTerrainPhysicsComponent（可选）
```

### 2. 车辆控制输入

```cpp
// VehicleControl.h — 标准控制
struct FVehicleControl
{
  float Throttle = 0.0f;    // 油门 [0, 1]
  float Steer = 0.0f;       // 转向 [-1, 1]
  float Brake = 0.0f;       // 刹车 [0, 1]
  bool bHandBrake = false;   // 手刹
  bool bReverse = false;     // 倒挡
  bool bManualGearShift = false; // 手动换挡
  int32 Gear = 0;            // 挡位
};
```

### 3. 输入优先级系统

```cpp
// VehicleInputPriority.h — 多源输入仲裁
enum class EVehicleInputPriority : uint8
{
  INVALID = 0,
  Lowest,       // 调试用
  Relaxation,   // 控制松弛（非粘性控制）
  Autopilot,    // 内置自动驾驶
  User,         // 用户在模拟器中操作
  Client,       // RPC 客户端
  Highest       // 调试用
};
// 高优先级输入覆盖低优先级输入
```

### 4. 运动组件 — 策略模式

```
UBaseCarlaMovementComponent (抽象基类)
  │  virtual ProcessControl(FVehicleControl&)
  │  virtual GetVehicleCurrentGear()
  │  virtual GetVehicleForwardSpeed()
  │
  ├── UDefaultMovementComponent     ← UE Chaos 物理
  ├── UCarSimManagerComponent       ← CarSim 集成 (#ifdef WITH_CARSIM)
  └── UChronoMovementComponent      ← Chrono 集成 (#ifdef WITH_CHRONO)
```

**设计优势**：通过替换 MovementComponent 即可切换物理引擎，不影响上层控制逻辑。

### 5. 车辆物理参数控制

```cpp
// VehiclePhysicsControl.h — 143 行，完整的车辆物理参数
struct FVehiclePhysicsControl
{
  // === 发动机 ===
  FRichCurve TorqueCurve;       // 扭矩曲线
  float MaxTorque = 300.0f;     // 最大扭矩 (Nm)
  float MaxRPM = 5000.0f;       // 最大转速
  float IdleRPM = 1.0f;         // 怠速转速
  float BrakeEffect = 1.0f;     // 制动效果
  float RevUpMOI = 1.0f;        // 转动惯量
  float RevDownRate = 600.0f;   // 降速速率

  // === 差速器 ===
  uint8 DifferentialType = 0;   // 差速器类型
  float FrontRearSplit = 0.5f;  // 前后分配比

  // === 变速箱 ===
  bool bUseAutomaticGears = true;
  float GearChangeTime = 0.5f;  // 换挡时间
  float FinalRatio = 4.0f;      // 终传比
  TArray<float> ForwardGearRatios;  // 前进挡比
  TArray<float> ReverseGearRatios;  // 倒挡比
  float ChangeUpRPM = 4500.0f;     // 升挡转速
  float ChangeDownRPM = 2000.0f;   // 降挡转速
  float TransmissionEfficiency = 0.9f; // 传动效率

  // === 车身 ===
  float Mass = 1000.0f;          // 质量 (kg)
  float DragCoefficient = 0.3f;  // 风阻系数
  FVector CenterOfMass;          // 质心偏移
  float ChassisWidth = 180.f;
  float ChassisHeight = 140.f;
  float DownforceCoefficient = 0.3f; // 下压力系数
  FVector InertiaTensorScale;    // 惯量张量缩放

  // === 转向 ===
  FRichCurve SteeringCurve;      // 转向曲线

  // === 车轮 ===
  TArray<FWheelPhysicsControl> Wheels; // 每轮独立参数
  bool UseSweepWheelCollision = false;
};
```

### 6. 车辆遥测数据

```cpp
// VehicleTelemetryData.h — 实时遥测
struct FWheelTelemetryData
{
  float LatSlip = 0.0f;   // 横向滑移
  float LongSlip = 0.0f;  // 纵向滑移
  float Omega = 0.0f;     // 角速度
};

struct FVehicleTelemetryData
{
  float Speed = 0.0f;
  float Steer = 0.0f;
  float Throttle = 0.0f;
  float Brake = 0.0f;
  float EngineRPM = 0.0f;
  int32 Gear = 0;
  TArray<FWheelTelemetryData> Wheels; // 每轮数据
};
```

### 7. 自定义地形物理（高级）

```cpp
// CustomTerrainPhysicsComponent.h — 567 行
// 独立的粒子级地形物理模拟系统，支持：

// 核心数据结构
struct FParticle {
  FDVector Position;   // 粒子位置
  FVector Velocity;    // 粒子速度
  float Radius = 0.02f; // 粒子半径
};

struct FDenseTile {
  std::vector<FParticle> Particles;          // 粒子集合
  std::vector<float> ParticlesHeightMap;     // 高度图
  // 支持按半径查询、按 Box 查询
};

class FSparseHighDetailMap {
  std::unordered_map<uint64_t, FDenseTile> Map;      // 活跃瓦片
  std::unordered_map<uint64_t, FDenseTile> CacheMap; // 缓存瓦片
  FHeightMapData Heightmap;                          // 全局高度图
};

// 功能特性：
// - 粒子级地形变形（车辆驶过产生车辙）
// - HeightMap 异步流送（大地图支持）
// - PyTorch 神经网络土壤模型 (#ifdef WITH_PYTORCH)
// - 多线程瓦片加载 (FTilesWorker : FRunnable)
// - 纹理实时更新（GPU 地形变形可视化）
// - 与 ALargeMapManager 集成
```

---

## 六、行人系统

### 1. 行人继承体系

```
ACharacter (UE 内置)
  └── AWalkerBase (35 行) ← CARLA 行人基类
        ├── bAlive = true           // 存活状态
        ├── LifeSpanAfterDeath = 10 // 死后存活时间
        └── StartDeathLifeSpan()    // 触发死亡动画
```

### 2. 行人控制器

```cpp
// WalkerController.h — 69 行
UCLASS()
class AWalkerController : public AController
{
  void OnPossess(APawn *InPawn) override;
  void Tick(float DeltaSeconds) override;

  // 控制接口
  void ApplyWalkerControl(const FWalkerControl &InControl);
  const FWalkerControl GetWalkerControl() const;

  // 骨骼控制（精细动作）
  void GetBonesTransform(FWalkerBoneControlOut &WalkerBones);
  void SetBonesTransform(const FWalkerBoneControlIn &WalkerBones);
  void BlendPose(float Blend);          // 动画混合
  void GetPoseFromAnimation();           // 从动画系统获取姿态

  float GetMaximumWalkSpeed() const { return 4096.0f; } // ~147 km/h
};
```

**设计要点**：骨骼控制接口允许外部客户端精确控制行人每个骨骼的姿态，用于动画研究和特殊场景。

---

## 七、天气与环境系统

### 1. 天气参数

```cpp
// WeatherParameters.h — 14 个可控参数
struct FWeatherParameters
{
  float Cloudiness = 0.0f;           // 云量 [0, 100]
  float Precipitation = 0.0f;        // 降水量 [0, 100]
  float PrecipitationDeposits = 0.0f; // 路面积水 [0, 100]
  float WindIntensity = 0.35f;       // 风力 [0, 100]
  float SunAzimuthAngle = 0.0f;      // 太阳方位角 [0, 360]
  float SunAltitudeAngle = 75.0f;    // 太阳高度角 [-90, 90]
  float FogDensity = 0.0f;           // 雾浓度 [0, 100]
  float FogDistance = 0.0f;          // 雾起始距离 [0, ∞)
  float FogFalloff = 0.2f;           // 雾衰减 [0, 10]
  float Wetness = 0.0f;              // 湿度 [0, 100]
  float ScatteringIntensity = 0.0f;  // 散射强度 [0, 100]
  float MieScatteringScale = 0.0f;   // 米氏散射 [0, 5]
  float RayleighScatteringScale = 0.0331f; // 瑞利散射 [0, 2]
  float DustStorm = 0.0f;            // 沙尘暴 [0, 100]
};
```

### 2. 天气 Actor

```cpp
// Weather.h — 82 行
UCLASS(Abstract)
class AWeather : public AActor
{
  void ApplyWeather(const FWeatherParameters &WeatherParameters); // 应用 + 通知
  void SetWeather(const FWeatherParameters &WeatherParameters);   // 仅设置
  const FWeatherParameters &GetCurrentWeather() const;

  // 昼夜循环
  void SetDayNightCycle(const bool &active);
  const bool &GetDayNightCycle() const;

protected:
  UFUNCTION(BlueprintImplementableEvent)
  void RefreshWeather(const FWeatherParameters &WeatherParameters); // 蓝图实现

private:
  UMaterial* PrecipitationPostProcessMaterial;   // 降水后处理
  UMaterial* DustStormPostProcessMaterial;       // 沙尘暴后处理
  TMap<UMaterial*, float> ActiveBlendables;      // 活跃混合材质
};
```

### 3. 天空系统

```cpp
// Sky.h — 天空组件集合
UCLASS(Abstract)
class ASkyBase : public AActor
{
  UPostProcessComponent* PostProcessComponent;           // 后处理
  UExponentialHeightFogComponent* ExponentialHeightFogComponent; // 指数高度雾
  UDirectionalLightComponent* DirectionalLightComponentSun;      // 太阳光
  UDirectionalLightComponent* DirectionalLightComponentMoon;     // 月光
  USkyLightComponent* SkyLightComponent;                         // 天光
  UVolumetricCloudComponent* VolumetricCloudComponent;           // 体积云
  USkyAtmosphereComponent* SkyAtmosphereComponent;               // 大气散射
};
```

### 4. 灯光子系统

```cpp
// CarlaLightSubsystem.h — UWorldSubsystem
UCLASS()
class UCarlaLightSubsystem : public UWorldSubsystem
{
  void RegisterLight(UCarlaLight* CarlaLight);
  void UnregisterLight(UCarlaLight* CarlaLight);

  // RPC 接口：获取/设置所有灯光状态
  std::vector<carla::rpc::LightState> GetLights(FString Client);
  void SetLights(FString Client, std::vector<carla::rpc::LightState> LightsToSet, bool DiscardClient = false);

  // 昼夜循环事件
  void SetDayNightCycle(const bool active);
  FDayTimeChanged DayTimeChangeEvent; // 蓝图可绑定的委托

private:
  TMap<int, UCarlaLight*> Lights;       // Id → 灯光
  TMap<FString, bool> ClientStates;     // 客户端同步状态
};

// CarlaLight.h — 灯光组件
UCLASS()
class UCarlaLight : public UActorComponent
{
  enum ELightType { Vehicle, Street, Building, Other };
  enum ECarlaLightFlags { Registered = 1, TurnedOn = 2 };

  void SetLightIntensity(float Intensity);
  void SetLightColor(FLinearColor Color);
  void SetLightOn(bool bOn);
  void SetLightType(ELightType Type);
  carla::rpc::LightState GetLightState();
  void SetLightState(carla::rpc::LightState LightState);
};
```

---

## 八、交通管理系统

### 1. 交通灯管理

```cpp
// TrafficLightManager.h — 127 行
UCLASS()
class ATrafficLightManager : public AActor
{
  // 注册接口
  void RegisterLightComponentFromOpenDRIVE(UTrafficLightComponent*); // 从 OpenDRIVE 注册
  void RegisterLightComponentGenerated(UTrafficLightComponent*);     // 从生成器注册

  // 查询接口
  ATrafficLightGroup* GetTrafficGroup(int JunctionId);     // 按路口查组
  UTrafficLightController* GetController(FString Id);      // 按 ID 查控制器
  USignComponent* GetTrafficSign(FString SignId);          // 按 ID 查标志

  // 编辑器工具
  void GenerateSignalsAndTrafficLights();     // 从 OpenDRIVE 生成
  void RemoveGeneratedSignalsAndTrafficLights(); // 移除生成物
  void MatchTrafficLightActorsWithOpenDriveSignals(); // 匹配

  // 全局冻结
  void SetFrozen(bool InFrozen);

private:
  TMap<int, TObjectPtr<ATrafficLightGroup>> TrafficGroups;      // 路口 → 灯组
  TMap<FString, TObjectPtr<UTrafficLightController>> TrafficControllers; // 控制器
  TMap<FString, TObjectPtr<USignComponent>> TrafficSignComponents;       // 标志
  TArray<TObjectPtr<ATrafficSignBase>> TrafficSigns;                     // 所有标志

  // 模型映射
  TSubclassOf<AActor> TrafficLightModel_RHT;   // 右行制式灯模型
  TSubclassOf<AActor> TrafficLightModel_LHT;   // 左行制式灯模型
  TMap<FString, TSubclassOf<AActor>> TrafficSignsModels;    // 标志类型 → 蓝图
  TMap<FString, TSubclassOf<AActor>> SpeedLimitModels;      // 限速牌 → 蓝图
};
```

### 2. LibCarla 交通管理器

```
LibCarla/source/carla/trafficmanager/
  ├── TrafficManager.h          ← 对外门面（423 行），单例
  ├── TrafficManagerBase.h      ← 抽象基类
  ├── TrafficManagerLocal.h     ← 本地模式（同进程）
  ├── TrafficManagerRemote.h    ← 远程模式（独立进程）
  │
  ├── 阶段管线（Pipeline Stages）:
  │   ├── CollisionStage.h      ← 碰撞检测
  │   ├── MotionPlanStage.h     ← 运动规划
  │   ├── TrafficLightStage.h   ← 交通灯决策
  │   └── VehicleLightStage.h   ← 车灯控制
  │
  ├── 数据结构:
  │   ├── SimpleWaypoint.h      ← 路点
  │   ├── InMemoryMap.h         ← 内存道路网络
  │   ├── DataStructures.h      ← Actor 状态、路径等
  │   └── AtomicActorSet.h      ← 线程安全 Actor 集合
  │
  ├── 工具:
  │   ├── LocalizationUtils.h   ← 定位工具
  │   ├── PIDController.h       ← PID 控制器
  │   ├── ALSM.h               ← 自适应纵向速度模型
  │   └── Parameters.h          ← 参数配置
  │
  └── Constants.h               ← 全局常量
```

**架构特点**：
- **Stage 管线**：碰撞检测 → 运动规划 → 交通灯决策 → 车灯控制，每阶段独立
- **Local/Remote 分离**：可同进程运行，也可独立进程运行（避免阻塞主仿真）
- **AtomicActorSet**：多线程安全的 Actor 集合，支持并发注册/注销

---

## 九、大地图管理系统

### 1. LargeMapManager — 超大世界支持

```cpp
// LargeMapManager.h — 354 行
UCLASS()
class ALargeMapManager : public AActor
{
  using TileID = uint64;

  // 地图生成
  void GenerateMap(FString InAssetsPath);  // 从资产路径生成
  void GenerateLargeMap();                  // 生成大地图

  // 坐标转换
  FTransform GlobalToLocalTransform(const FTransform&) const;
  FVector GlobalToLocalLocation(const FVector&) const;
  FTransform LocalToGlobalTransform(const FTransform&) const;
  FVector LocalToGlobalLocation(const FVector&) const;

  // 瓦片管理
  TileID GetTileID(FVector Location) const;
  FCarlaMapTile* GetCarlaMapTile(FIntVector TileVectorID);
  FCarlaMapTile& LoadCarlaMapTile(FString TileMapPath, TileID TileId);
  bool IsTileLoaded(TileID TileId) const;

  // Actor 生命周期
  void OnActorSpawned(const FCarlaActor& CarlaActor);
  void OnActorDestroyed(AActor* DestroyedActor);

  // 每帧更新
  void UpdateTilesState();              // 更新瓦片加载/卸载
  void CheckActiveActors();             // 检查超出范围的活跃 Actor
  void ConvertActiveToDormantActors();  // 活跃 → 休眠
  void CheckDormantActors();            // 检查进入范围的休眠 Actor
  void ConvertDormantToActiveActors();  // 休眠 → 活跃
  void CheckIfRebaseIsNeeded();         // 检查是否需要世界原点重定位

  // 配置参数
  float LayerStreamingDistance = 300000.f;   // 图层流送距离 (3km)
  float ActorStreamingDistance = 200000.f;   // Actor 流送距离 (2km)
  float RebaseOriginDistance = 200000.f;     // 重定位距离 (2km)
  float TileSide = 200000.f;                 // 瓦片边长 (2km)

private:
  TMap<uint64, FCarlaMapTile> MapTiles;      // 所有瓦片
  TArray<FCarlaActor::IdType> ActiveActors;  // 活跃 Actor
  TArray<FCarlaActor::IdType> DormantActors; // 休眠 Actor
  FIntVector CurrentOriginInt;               // 当前世界原点
  FDVector CurrentOriginD;                   // 双精度原点
};
```

**核心机制**：

```
┌─────────────────────────────────────────────────────┐
│                    大地图瓦片系统                      │
│                                                      │
│  ┌──────┬──────┬──────┬──────┐                      │
│  │Tile00│Tile01│Tile02│Tile03│  ← 2km × 2km 瓦片   │
│  ├──────┼──────┼──────┼──────┤                      │
│  │Tile10│Tile11│Tile12│Tile13│  ← 按需流送          │
│  ├──────┼──────┼──────┼──────┤                      │
│  │Tile20│Tile21│Tile22│Tile23│  ← Active/Dormant   │
│  ├──────┼──────┼──────┼──────┤    Actor 状态切换    │
│  │Tile30│Tile31│Tile32│Tile33│                      │
│  └──────┴──────┴──────┴──────┘                      │
│                                                      │
│  距离判断:                                            │
│    < ActorStreamingDistance (2km) → Active           │
│    > ActorStreamingDistance       → Dormant           │
│    > LayerStreamingDistance (3km) → 卸载瓦片          │
│    > RebaseOriginDistance (2km)  → 世界原点重定位     │
└─────────────────────────────────────────────────────┘
```

### 2. 世界原点重定位（Rebase）

UE 使用 `float` 存储位置，大坐标下精度下降。LargeMapManager 通过 `PreWorldOriginOffset` / `PostWorldOriginOffset` 回调，在 Actor 远离原点时自动将世界原点移动到 Actor 附近，保持浮点精度。

---

## 十、OpenDrive 道路数据集成

### 1. UE 端 OpenDrive 接口

```cpp
// OpenDrive.h — 蓝图函数库
UCLASS()
class UOpenDrive : public UBlueprintFunctionLibrary
{
  static FString GetXODR(const UWorld *World);           // 获取当前地图 XODR
  static FString LoadXODR(const FString &MapName);       // 加载 XODR XML
  static UOpenDriveMap *LoadOpenDriveMap(const FString &MapName); // 解析为 Map 对象
};

// OpenDriveMap.h — 道路查询接口
UCLASS()
class UOpenDriveMap : public UObject
{
  bool Load(const FString &XODRContent);                           // 从 XML 加载
  FWaypoint GetClosestWaypointOnRoad(FVector Location, bool &Success); // 最近路点
  TArray<FWaypoint> GenerateWaypoints(float ApproxDistance = 100.0f);  // 全图路点
  TArray<FWaypointConnection> GenerateTopology();                     // 拓扑图
  TArray<FWaypoint> GenerateWaypointsOnRoadEntries();                 // 路入口点
  FTransform ComputeTransform(FWaypoint Waypoint);                    // 路点变换
  TArray<FWaypoint> GetNext(FWaypoint Waypoint, float Distance);      // 前方路点
};
```

### 2. LibCarla 端道路解析

```
LibCarla/source/carla/road/
  ├── Map.h / Map.cpp             ← 完整道路网络（Lane/Road/Junction/Signal）
  └── element/
      └── Waypoint.h              ← 路点（RoadId, LaneId, Distance）

LibCarla/source/carla/opendrive/
  └── OpenDriveParser.h           ← XML 解析器
```

**数据流**：`XODR XML → OpenDriveParser → carla::road::Map → UOpenDriveMap → 蓝图/Python`

---

## 十一、Trigger 系统 — 区域效果

```cpp
// FrictionTrigger.h — 81 行，摩擦力触发器
UCLASS()
class AFrictionTrigger : public AActor
{
  // 当车辆进入区域时，修改轮胎摩擦力
  void OnTriggerBeginOverlap(...);
  void OnTriggerEndOverlap(...);

  void UpdateWheelsFriction(AActor *OtherActor, TArray<float>& NewFriction);

  float Friction = 0.0f;                  // 目标摩擦系数
  TObjectPtr<UBoxComponent> TriggerVolume; // 触发体积
  TArray<float> OldFrictionValues;         // 保存原始摩擦力（离开时恢复）
};

// TriggerFactory.h — 触发器工厂
UCLASS()
class ATriggerFactory : public ACarlaActorFactory
{
  TArray<FActorDefinition> GetDefinitions() final;
  FActorSpawnResult SpawnActor(...) final;
};
```

**设计意图**：通过 Trigger Volume 实现区域性的物理属性修改（如冰雪路面、泥地），用于仿真不同路况对车辆的影响。

---

## 十二、环境对象系统

```cpp
// EnvironmentObject.h — 57 行
enum EnvironmentObjectType { Invalid, Vehicle, Character, TrafficLight, ISMComp, SMComp, SKMComp };

struct FEnvironmentObject
{
  AActor* Actor;
  FString Name;
  FString IdStr;
  FTransform Transform;
  FBoundingBox BoundingBox;
  uint64 Id;
  EnvironmentObjectType Type;
  crp::CityObjectLabel ObjectLabel; // 语义标签
  bool CanTick;
};

// ObjectRegister.h — 75 行
UCLASS()
class UObjectRegister : public UObject
{
  TArray<FEnvironmentObject> GetEnvironmentObjects(uint8 InTagQueried = 0xFF);
  void RegisterObjects(TArray<AActor*> Actors);
  void EnableEnvironmentObjects(const TSet<uint64>& EnvObjectIds, bool Enable);

private:
  void RegisterVehicle(ACarlaWheeledVehicle* Vehicle);
  void RegisterCharacter(ACharacter* Character);
  void RegisterTrafficLight(ATrafficLightBase* TrafficLight);
  void RegisterISMComponents(AActor* Actor);  // 实例化静态网格组件
  void RegisterSMComponents(AActor* Actor);   // 静态网格组件
  void RegisterSKMComponents(AActor* Actor);  // 骨骼网格组件

  TArray<FEnvironmentObject> EnvironmentObjects;
};
```

**设计要点**：将场景中所有有意义的对象（包括 ISM/SM/SKM 组件级别）统一注册，支持按语义标签查询和动态启用/禁用。

---

## 十三、LibCarla 跨平台库详解

### 1. RPC 序列化协议

```
LibCarla/source/carla/rpc/ (30+ 个头文件)
  ├── Actor.h              ← Actor 序列化（Id, Description, BoundingBox, SemanticTags）
  ├── ActorDescription.h   ← Actor 描述符
  ├── ActorState.h         ← Actor 状态（位置/旋转/速度/变换）
  ├── ActorDefinition.h    ← Actor 定义（蓝图 + 属性列表）
  ├── VehicleControl.h     ← 车辆控制指令
  ├── WalkerControl.h      ← 行人控制指令
  ├── EpisodeSettings.h    ← Episode 设置
  ├── TrafficLightState.h  ← 交通灯状态
  ├── Command.h            ← RPC 命令封装
  ├── Client.h             ← RPC 客户端
  ├── Weather.h            ← 天气参数
  ├── LightState.h         ← 灯光状态
  ├── EnvironmentObject.h  ← 环境对象
  ├── VehicleTelemetryData.h ← 遥测数据
  └── ... (更多协议定义)
```

**序列化机制**：使用 MsgPack 进行高效二进制序列化，所有 RPC 类型都实现了 `operator<<` 和 `operator>>`。

### 2. ROS2 原生集成（非 Bridge）

```cpp
// ROS2.h — 266 行，单例模式
class ROS2 {
  static std::shared_ptr<ROS2> GetInstance(); // 单例

  // Actor 注册 API
  void RegisterSensor(void *actor, std::string ros_name, std::string frame_id, bool publish_tf);
  void UnregisterSensor(void *actor);
  void RegisterVehicle(void *actor, std::string ros_name, std::string frame_id,
                       ActorCallback callback, bool enable_ackermann_control = false);
  void UnregisterVehicle(void *actor);

  // 数据处理入口（每种传感器一个）
  void ProcessDataFromCamera(uint64_t sensor_type, stream_id, transform, W, H, Fov, buffer);
  void ProcessDataFromGNSS(uint64_t sensor_type, stream_id, transform, GeoLocation);
  void ProcessDataFromIMU(uint64_t sensor_type, stream_id, transform, accel, gyro, compass);
  void ProcessDataFromLidar(uint64_t sensor_type, stream_id, transform, LidarData);
  void ProcessDataFromSemanticLidar(uint64_t sensor_type, stream_id, transform, SemanticLidarData);
  void ProcessDataFromRadar(uint64_t sensor_type, stream_id, transform, RadarData);
  void ProcessDataFromVehicleData(uint64_t sensor_type, stream_id, transform,
      /* INS: */ lon, lat, alt, yaw, pitch, roll, vx, vy, vt, psd, r, p, q, ...,
      /* VehicleState: */ speed, throttle, steer, brake, gear, ...,
      /* ObstacleList: */ TArray<ObstacleItemData>);

  // Publisher 类型
  CarlaCameraPublisher;        // 图像
  CarlaClockPublisher;         // 时钟
  CarlaTransformPublisher;     // TF 变换
  CarlaInsDataPublisher;       // INS 数据
  CarlaVehicleStatePublisher;  // 车辆状态
  CarlaObstacleListPublisher;  // 障碍物列表
};
```

**关键设计**：
- **非 Bridge 架构**：ROS2 发布器直接嵌入 UE 进程，无需外部 bridge 进程
- **单例模式**：全局唯一 ROS2 实例，所有传感器共享
- **按类型分发**：每种传感器数据类型有独立的 `ProcessDataFrom*` 方法
- **TF 自动发布**：注册时指定 `frame_id` 和 `publish_tf`，自动发布坐标变换
- **Ackermann 控制**：车辆可选择 Ackermann 或直接控制，互斥

### 3. 传感器序列化注册表

```cpp
// SensorRegistry.h — 编译期类型映射
using SensorRegistry = CompositeSerializer<
  std::pair<ACollisionSensor *,           s11n::CollisionEventSerializer>,
  std::pair<ADepthCamera *,               s11n::ImageSerializer>,
  std::pair<ANormalsCamera *,             s11n::NormalsImageSerializer>,
  std::pair<ADVSCamera *,                 s11n::DVSEventArraySerializer>,
  std::pair<AGnssSensor *,                s11n::GnssSerializer>,
  std::pair<AInertialMeasurementUnit *,   s11n::IMUSerializer>,
  std::pair<ALaneInvasionSensor *,        s11n::NoopSerializer>,
  std::pair<AObstacleDetectionSensor *,   s11n::ObstacleDetectionEventSerializer>,
  std::pair<AOpticalFlowCamera *,         s11n::OpticalFlowImageSerializer>,
  std::pair<ARadar *,                     s11n::RadarSerializer>,
  std::pair<ARayCastSemanticLidar *,      s11n::SemanticLidarSerializer>,
  std::pair<ARayCastLidar *,              s11n::LidarSerializer>,
  std::pair<ASceneCaptureCamera *,        s11n::ImageSerializer>,
  std::pair<ASemanticSegmentationCamera *,s11n::ImageSerializer>,
  std::pair<AInstanceSegmentationCamera *,s11n::ImageSerializer>,
  std::pair<FWorldObserver *,             s11n::EpisodeStateSerializer>,
  std::pair<AHSSLidar *,                  s11n::LidarSerializer>,
  std::pair<AVehicleDataSensor *,         s11n::NoopSerializer>
  // ... 共 25 种传感器
>;
```

**设计优势**：编译期注册，零运行时开销。新增传感器只需在注册表中添加一行 `std::pair`。

### 4. 数据流传输

```
LibCarla/source/carla/streaming/
  ├── Server.h       ← Boost.Asio TCP 服务端
  ├── Client.h       ← 数据流客户端
  ├── Stream.h       ← 流句柄
  ├── Token.h        ← 订阅 Token（StreamId + 地址）
  └── EndPoint.h     ← 网络端点
```

### 5. 几何与数学工具

```
LibCarla/source/carla/geom/
  ├── Vector3D.h / Vector2D.h     ← 基础向量
  ├── Transform.h                  ← 变换（位置 + 旋转）
  ├── Rotation.h / Quaternion.h   ← 旋转
  ├── BoundingBox.h               ← 包围盒
  ├── GeoLocation.h               ← 地理坐标（经纬度）
  ├── GeoProjection.h             ← 地理投影转换
  ├── CubicPolynomial.h           ← 三次多项式（道路曲线）
  ├── Math.h                      ← 数学工具
  ├── Mesh.h                      ← 网格
  ├── Rtree.h                     ← R-tree 空间索引
  ├── Velocity.h / Acceleration.h ← 物理量
  └── AngularVelocity.h           ← 角速度
```

### 6. 图像处理

```
LibCarla/source/carla/image/
  ├── ImageView.h          ← 图像视图（零拷贝）
  ├── ImageConverter.h     ← 格式转换（RGB → 灰度 → 深度）
  ├── ColorConverter.h     ← 颜色空间转换
  ├── CityScapesPalette.h  ← CityScapes 语义分割调色板
  ├── ImageIO.h            ← 图像读写
  └── ImageIOConfig.h      ← IO 配置
```

---

## 十四、PythonAPI 客户端架构

```
PythonAPI/carla/
  ├── __init__.py           ← 模块入口
  ├── scene_layout.py       ← 场景布局查询
  │
  ├── agents/               ← 自动驾驶 Agent
  │   ├── navigation/
  │   │   ├── global_route_planner.py  ← 全局路径规划（Dijkstra/A*）
  │   │   ├── local_planner.py         ← 局部路径规划
  │   │   ├── basic_agent.py           ← 基础 Agent
  │   │   ├── behavior_agent.py        ← 行为 Agent（跟车/换道/避障）
  │   │   ├── constant_velocity_agent.py ← 恒速 Agent
  │   │   ├── controller.py            ← PID 控制器
  │   │   └── behavior_types.py        ← 行为参数类型
  │   └── tools/
  │       ├── misc.py                  ← 工具函数
  │       └── hints.py                 ← 类型提示
  │
  ├── libcarla*.so          ← C++ 绑定（Boost.Python 编译产物）
  └── *.py                  ← Python 封装层
```

**架构特点**：
- **C++ 核心 + Python 封装**：`libcarla` 通过 Boost.Python 暴露 C++ API
- **Agent 分层**：全局规划 → 局部规划 → PID 控制，三层管线
- **Behavior Agent**：支持跟车、换道、避障等复杂驾驶行为

---

## 十五、关键源文件索引

### Game 层（仿真控制）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Game/CarlaGameModeBase.h` | 208 | GameMode，Episode 初始化、Actor 工厂管理 |
| `Game/CarlaGameInstance.h` | 137 | GameInstance，持有 FCarlaEngine |
| `Game/CarlaEngine.h` | 146 | 全局引擎，帧循环 + Server + 多 GPU |
| `Game/CarlaEpisode.h` | 413 | 仿真 Episode，Actor/传感器/ROS2 管理 |
| `Settings/EpisodeSettings.h` | 43 | 仿真参数（同步模式、帧率等） |

### Actor 层（Actor 管理）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Actor/ActorRegistry.h` | 134 | Actor 注册表（三张 TMap） |
| `Actor/CarlaActor.h` | 640 | Actor 代理类（多态继承体系） |
| `Actor/ActorDispatcher.h` | 113 | Factory 绑定 + Spawn 分发 |
| `Actor/ActorInfo.h` | 33 | Actor 元数据（描述/包围盒/语义标签） |
| `Actor/CarlaActorFactory.h` | 51 | 工厂基类 |
| `Actor/Factory/VehicleActorFactory.h` | — | 车辆工厂 |
| `Actor/Factory/WalkerActorFactory.h` | — | 行人工厂 |
| `Actor/Factory/PropActorFactory.h` | — | Prop 工厂 |

### Vehicle 层（车辆系统）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Vehicle/CarlaWheeledVehicle.h` | 494 | 车辆基类（控制/物理/灯光/遥测） |
| `Vehicle/VehicleControl.h` | 37 | 车辆控制输入结构 |
| `Vehicle/VehiclePhysicsControl.h` | 143 | 车辆物理参数（发动机/变速箱/车身/车轮） |
| `Vehicle/VehicleTelemetryData.h` | 55 | 实时遥测（速度/RPM/挡位/每轮滑移） |
| `Vehicle/VehicleInputPriority.h` | 32 | 输入优先级枚举（多源输入仲裁） |
| `Vehicle/MovementComponents/BaseCarlaMovementComponent.h` | 48 | 运动组件基类（策略模式） |
| `Vehicle/MovementComponents/DefaultMovementComponent.h` | 35 | 默认运动组件（Chaos 物理） |
| `Vehicle/MovementComponents/CarSimManagerComponent.h` | — | CarSim 集成 |
| `Vehicle/MovementComponents/ChronoMovementComponent.h` | — | Chrono 集成 |
| `Vehicle/CustomTerrainPhysicsComponent.h` | 567 | 自定义地形物理（粒子模拟 + PyTorch） |
| `Vehicle/AckermannController.h` | — | Ackermann 转向控制 |
| `Vehicle/WheeledVehicleAIController.h` | — | AI 车辆控制器 |

### Sensor 层（传感器框架）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Sensor/Sensor.h` | 278 | 传感器基类（Tick 锁定 + 数据流 + ROS2） |
| `Sensor/SensorManager.h` | 27 | 传感器生命周期管理 |
| `Sensor/DataStream.h` | 83 | 数据流（同步/异步） |
| `Sensor/VehicleDataSensor.h` | 93 | 自定义车辆数据传感器 |
| `Sensor/WorldObserver.h` | — | 世界状态观察器 |
| `Sensor/Radar.h` | — | 毫米波雷达 |
| `Sensor/HSSLidar.h` | — | 混合固态 LiDAR |
| `Sensor/CollisionSensor.h` | — | 碰撞传感器 |
| `Sensor/RHIGPUReadbackPool.h` | — | GPU 读回池 |

### Walker 层（行人系统）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Walker/WalkerBase.h` | 35 | 行人基类（存活状态 + 死亡动画） |
| `Walker/WalkerController.h` | 69 | 行人控制器（移动 + 骨骼控制） |
| `Walker/WalkerControl.h` | — | 控制输入结构 |
| `Walker/WalkerBoneControlIn.h` | — | 骨骼输入 |
| `Walker/WalkerBoneControlOut.h` | — | 骨骼输出 |
| `Walker/WalkerAnim.h` | — | 行人动画 |

### Weather & Lights 层（环境系统）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Weather/Weather.h` | 82 | 天气 Actor（参数 + 后处理 + 昼夜循环） |
| `Weather/WeatherParameters.h` | 58 | 天气参数结构（14 个可控参数） |
| `Weather/Sky.h` | 51 | 天空组件集合（太阳/月亮/云/大气） |
| `Lights/CarlaLightSubsystem.h` | 88 | 灯光子系统（WorldSubsystem） |
| `Lights/CarlaLight.h` | 184 | 灯光组件（类型/颜色/强度/状态） |

### Traffic 层（交通管理）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Traffic/TrafficLightManager.h` | 127 | 交通灯/标志管理（OpenDRIVE 集成） |

### MapGen 层（大地图）

| 文件 | 行数 | 职责 |
|------|------|------|
| `MapGen/LargeMapManager.h` | 354 | 大地图管理（瓦片流送/Actor 休眠/Rebase） |
| `MapGen/ProceduralBuilding.h` | — | 程序化建筑 |
| `MapGen/DoublyConnectedEdgeList.h` | — | 双向连接边表（道路拓扑） |
| `MapGen/GraphParser.h` | — | 图解析器 |
| `MapGen/RoadMap.h` | — | 道路地图 |

### OpenDrive 层（道路数据）

| 文件 | 行数 | 职责 |
|------|------|------|
| `OpenDrive/OpenDrive.h` | 53 | OpenDrive 蓝图函数库 |
| `OpenDrive/OpenDriveMap.h` | 99 | 道路查询接口（路点/拓扑/距离） |
| `OpenDrive/OpenDriveGenerator.h` | — | 道路生成器 |
| `OpenDrive/MapLogicParser.h` | — | 地图逻辑解析 |

### Trigger & Util 层

| 文件 | 行数 | 职责 |
|------|------|------|
| `Trigger/FrictionTrigger.h` | 81 | 摩擦力触发器（区域物理属性修改） |
| `Trigger/TriggerFactory.h` | 31 | 触发器工厂 |
| `Util/ObjectRegister.h` | 75 | 环境对象注册（车辆/行人/交通灯/组件） |
| `Util/EnvironmentObject.h` | 57 | 环境对象结构（Actor/包围盒/语义标签） |

### Server 层（RPC 通信）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Server/CarlaServer.h` | 59 | RPC 服务端（Pimpl 模式） |
| `Server/CarlaServerResponse.h` | — | RPC 响应码定义 |

### Recorder 层（录制回放）

| 文件 | 行数 | 职责 |
|------|------|------|
| `Recorder/CarlaRecorder.h` | 250 | 录制器（20+ 事件类型） |

### LibCarla 层（跨平台库）

| 目录 | 文件数 | 职责 |
|------|--------|------|
| `LibCarla/source/carla/rpc/` | 30+ | RPC 序列化协议（MsgPack） |
| `LibCarla/source/carla/ros2/` | 10+ | ROS2 原生发布器（非 Bridge） |
| `LibCarla/source/carla/streaming/` | 5+ | Boost.Asio 数据流服务端 |
| `LibCarla/source/carla/road/` | 10+ | OpenDRIVE 道路网络解析 |
| `LibCarla/source/carla/multigpu/` | 5+ | 多 GPU Primary/Secondary 路由 |
| `LibCarla/source/carla/sensor/` | 10+ | 传感器数据序列化注册表 |
| `LibCarla/source/carla/sensor/s11n/` | 15+ | 各传感器序列化器 |
| `LibCarla/source/carla/trafficmanager/` | 20+ | 交通管理器（Stage 管线） |
| `LibCarla/source/carla/image/` | 6+ | 图像处理（深度解码/语义标签） |
| `LibCarla/source/carla/geom/` | 15+ | 几何工具（向量/变换/投影/空间索引） |

### PythonAPI 层

| 文件 | 职责 |
|------|------|
| `PythonAPI/carla/agents/navigation/global_route_planner.py` | 全局路径规划 |
| `PythonAPI/carla/agents/navigation/local_planner.py` | 局部路径规划 |
| `PythonAPI/carla/agents/navigation/basic_agent.py` | 基础 Agent |
| `PythonAPI/carla/agents/navigation/behavior_agent.py` | 行为 Agent |
| `PythonAPI/carla/agents/navigation/controller.py` | PID 控制器 |

---

## 十六、PythonAPI Examples — 官方示例脚本解析

### 1. 示例总览

```
PythonAPI/examples/
  ├── 核心控制类
  │   ├── manual_control.py          ← 键盘手动控制（1396 行，最完整）
  │   ├── automatic_control.py       ← 自动驾驶控制（849 行，Agent 演示）
  │   └── no_rendering_mode.py       ← 2D 地图可视化（1619 行，无渲染模式）
  │
  ├── 场景生成类
  │   ├── generate_traffic.py        ← 交通流生成（327 行，TM 演示）
  │   ├── vehicle_gallery.py         ← 车辆展示
  │   ├── vehicle_doors_demo.py      ← 车门控制
  │   ├── vehicle_lights_demo.py     ← 车灯控制
  │   └── invertedai_traffic.py      ← InvertedAI 第三方交通
  │
  ├── 传感器可视化类
  │   ├── visualize_multiple_sensors.py ← 多传感器可视化
  │   ├── visualize_radar.py            ← 雷达可视化
  │   ├── open3d_lidar.py               ← Open3D 点云可视化（287 行）
  │   ├── hss_lidar_top_down.py         ← 混合固态 LiDAR 俯视图
  │   └── sensor_synchronization.py     ← 传感器同步采集（124 行）
  │
  ├── 道路/地图类
  │   ├── lane_explorer.py              ← OpenDRIVE 车道可视化（175 行）
  │   └── uegeo.py                      ← UE 地理坐标工具
  │
  ├── 录制回放类
  │   └── recorder_replay.py            ← 录制回放控制（187 行）
  │
  ├── 其他
  │   ├── draw_skeleton.py              ← 行人骨骼绘制
  │   ├── manual_control_fisheye.py     ← 鱼眼相机手动控制
  │   ├── interpolate_camera.py         ← 相机插值动画
  │   └── interpolation.xml             ← 相机插值配置
  │
  └── ros2/                             ← ROS2 原生集成示例
      ├── ros2_native.py                ← ROS2 传感器发布（184 行）
      ├── ros2_vehicle_control.py       ← ROS2 车辆控制（227 行）
      ├── stack.json                    ← 完整传感器配置
      ├── stack_cameras.json            ← 纯相机配置
      ├── stack_vehicle_data.json       ← 纯车辆数据配置
      ├── config/fastrtps-profile.xml   ← FastRTPS QoS 配置
      ├── rviz/ros2_native.rviz         ← RViz2 预设配置
      ├── run_rviz.sh                   ← RViz 启动脚本
      └── run_vehicle_control.sh        ← 控制启动脚本
```

### 2. 核心示例详解

#### manual_control.py — 键盘手动控制（1396 行）

最完整的示例脚本，实现了完整的 HUD、多传感器切换、录制回放、天气切换、车灯控制：

```
功能矩阵:
  ┌───────────────────────┬────────────────────┐
  │ 控制                  │ WASD/箭头/空格/P   │
  │ 传感器切换            │ TAB/数字键/`        │
  │ 天气切换              │ C/Shift+C          │
  │ 灯光控制              │ L/Shift+L/Z/X/I    │
  │ 录制/回放             │ Ctrl+R/Ctrl+P      │
  │ 挡位控制              │ M/,/.              │
  │ HUD                   │ F1                 │
  │ 地图图层              │ V/Shift+V/B        │
  │ 遥测显示              │ T                  │
  │ 车门控制              │ O                  │
  └───────────────────────┴────────────────────┘
```

#### automatic_control.py — 自动驾驶控制（849 行）

演示三种 Agent 的使用：

```python
# 三种 Agent 可选
from agents.navigation.behavior_agent import BehaviorAgent     # 行为 Agent（跟车/换道/避障）
from agents.navigation.basic_agent import BasicAgent           # 基础 Agent（简单路径跟踪）
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # 恒速 Agent

# 使用流程
agent = BehaviorAgent(vehicle, behavior='cautious')  # 创建 Agent
agent.set_destination(location)                        # 设置目的地
while True:
    control = agent.run_step()                         # 每帧获取控制指令
    vehicle.apply_control(control)                     # 应用控制
    world.tick()                                       # 同步模式推进帧
```

#### generate_traffic.py — 交通流生成（327 行）

演示 TrafficManager 的完整使用：

```python
# 核心参数
--number-of-vehicles 30    # 车辆数
--number-of-walkers 10     # 行人数量
--tm-port 8000             # TM 端口
--hybrid                   # 混合模式（远距离休眠）
--asynch                   # 异步模式
--safe                     # 安全生成点
--seed                     # 随机种子

# 工作流程
1. 连接 CARLA 服务器
2. 获取可生成点 (spawn_points)
3. 批量 spawn 车辆 + 设置 autopilot
4. 批量 spawn 行人 + 设置 walker control
5. 设置 TrafficManager 参数（速度偏差/距离/信号灯）
6. 主循环 tick
```

#### sensor_synchronization.py — 传感器同步（124 行）

演示同步模式下多传感器数据对齐：

```python
# 核心机制：Queue + 每帧等待所有传感器
sensor_queue = Queue()

def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data.frame, sensor_name))

# 主循环
while True:
    world.tick()
    # 等待所有传感器数据到达
    for _ in range(num_sensors):
        data = sensor_queue.get(True, 1.0)
    # 此时所有传感器数据已对齐到同一帧
```

### 3. ROS2 示例详解

#### ros2_native.py — ROS2 传感器发布（184 行）

演示 CARLA 原生 ROS2 集成的完整流程：

```python
# 1. 从 JSON 配置文件加载传感器配置
with open(args.stack) as f:
    config = json.load(f)

# 2. Spawn 车辆并设置 ros_name
bp.set_attribute("role_name", config["id"])
bp.set_attribute("ros_name", config["id"])

# 3. Spawn 传感器并启用 ROS2
for sensor in sensors_config:
    bp.set_attribute("ros_name", sensor["id"])
    sensor_actor = world.spawn_actor(bp, transform, attach_to=vehicle)
    sensor_actor.enable_for_ros()  # ← 关键：启用 ROS2 发布

# 4. 启用车辆的 ROS2 控制订阅
vehicle.enable_for_ros()

# 5. 同步模式 tick
while True:
    world.tick()
```

#### ros2_vehicle_control.py — ROS2 车辆控制（227 行）

演示通过 ROS2 Topic 控制 CARLA 车辆：

```python
# ROS2 节点
class CarlaVehicleController(Node):
    def __init__(self, role_name='hero'):
        # 发布控制指令到 /carla/{role_name}/vehicle_control_cmd
        self.publisher_ = self.create_publisher(
            CarlaEgoVehicleControl, f'/carla/{role_name}/vehicle_control_cmd', 10)

# Topic 命名约定:
#   /carla/{role_name}/vehicle_control_cmd     ← 控制指令
#   /carla/{role_name}/vehicle_control_cmd_data ← 控制数据
```

#### ROS2 传感器配置文件

```json
// stack.json — 完整传感器套件
{
  "type": "vehicle.lincoln.mkz",
  "id": "hero",
  "sensors": [
    { "type": "sensor.camera.rgb", "id": "rgb", ... },
    { "type": "sensor.lidar.ray_cast", "id": "lidar", ... },
    { "type": "sensor.other.gnss", "id": "gnss", ... },
    { "type": "sensor.other.imu", "id": "imu", ... },
    { "type": "sensor.other.vehicle_data", "id": "vehicle_data",
      "attributes": { "detection_radius": "5.0", "max_obstacles": "2" } }
  ]
}

// stack_cameras.json — 纯相机套件（6 种相机）
// RGB + Depth + SemanticSegmentation + InstanceSegmentation + Normals + OpticalFlow

// stack_vehicle_data.json — 纯车辆数据（仅 VehicleDataSensor）
```

---

## 十七、Util 工具链 — 构建与运维体系

### 1. 目录总览

```
Util/
  ├── Docker/                  ← Docker 容器化构建
  │   ├── Base.Dockerfile      ← 基础镜像（编译工具 + 依赖）
  │   ├── Development.Dockerfile ← 开发镜像（非 root 用户 + Python 库）
  │   ├── Release.Dockerfile   ← 发布镜像（最小运行时）
  │   ├── build.sh             ← 构建脚本（97 行）
  │   └── run.sh               ← 运行脚本（129 行）
  │
  ├── DockerUtils/             ← Docker 内工具
  │   └── fbx/                 ← FBX → OBJ 模型转换
  │       ├── src/FBX2OBJ.cpp  ← Autodesk FBX SDK 转换
  │       └── CMakeLists.txt
  │
  ├── SetupUtils/              ← 环境配置
  │   ├── InstallPrerequisites.sh  ← Linux 依赖安装（138 行）
  │   └── InstallPrerequisites.bat ← Windows 依赖安装
  │
  ├── Tools/                   ← 构建/部署工具
  │   ├── BuildUtilsDocker.sh  ← Docker 内 FBX SDK 构建
  │   ├── Environment.sh       ← 脚本环境初始化（16 行）
  │   ├── Check.sh             ← 代码检查
  │   ├── Deploy.bat/.sh       ← 部署脚本
  │   ├── Plugins.bat/.sh      ← 插件管理
  │   ├── Prettify.sh          ← 代码美化
  │   ├── Import.py/.sh        ← 资产导入
  │   ├── uncrustify.cfg       ← C++ 格式化配置
  │   └── uncrustify-ue4.cfg   ← UE4 兼容格式化
  │
  ├── Formatting/              ← 代码格式化
  │   ├── clang-format         ← C++ 格式化规则
  │   └── codeformat.py        ← 格式化脚本（430 行）
  │
  ├── ContentVersions.json     ← 资产版本清单
  ├── ContentVersions.txt
  ├── ImportAssets.sh          ← 资产导入脚本
  ├── download_from_gdrive.py  ← Google Drive 下载工具
  └── CARLA.sublime-project    ← Sublime Text 项目配置
```

### 2. Docker 三层镜像体系

```
┌─────────────────────────────────────────────────┐
│               Base.Dockerfile                    │
│  ubuntu:22.04 + 编译工具 + UE5 依赖 + Python    │
│  → carla-base:ue5-22.04                         │
└──────────────────────┬──────────────────────────┘
                       │
         ┌─────────────┼─────────────┐
         │                           │
┌────────▼──────────┐  ┌────────────▼────────────┐
│ Development.Dockerfile │  │ Release.Dockerfile      │
│ + 非 root 用户       │  │ + 最小运行时依赖       │
│ + Docker socket 挂载 │  │ + NVIDIA GPU 配置      │
│ + Python 示例库      │  │ + CarlaUnreal.sh 启动  │
│ + Vulkan 工具       │  │ → 直接运行仿真         │
│ → 开发/调试用       │  │ → 生产部署用           │
└─────────────────────┘  └─────────────────────────┘
```

#### Base.Dockerfile — 基础镜像（103 行）

```dockerfile
FROM ubuntu:22.04
# 核心编译工具
RUN apt-get install -y build-essential make ninja-build
# UE5 运行时依赖
RUN apt-get install -y libvulkan1 libnss3-dev libatk-bridge2.0-dev ...
# Python 环境
RUN apt-get install -y python3 python3-dev python3-pip python-is-python3
# 图像库（PythonAPI 需要）
RUN apt-get install -y libpng-dev libtiff5-dev libjpeg-dev
# 工具链
RUN apt-get install -y wget curl rsync unzip git git-lfs
```

#### Development.Dockerfile — 开发镜像（62 行）

```dockerfile
FROM carla-base:ue5-22.04
# 创建非 root 用户（UID/GID 可配置）
RUN groupadd --gid ${GID} ${USERNAME}
RUN useradd -m --uid ${UID} -g ${USERNAME} ${USERNAME}
# Docker socket 挂载（容器内可运行 Docker）
RUN groupadd -g ${DOCKER_GID} docker
RUN usermod -a -G docker ${USERNAME}
# 开发工具
RUN apt-get install -y vulkan-tools fontconfig xdg-user-dirs
# Python 示例依赖
RUN pip install -r examples_requirements.txt
```

#### Release.Dockerfile — 发布镜像（33 行）

```dockerfile
FROM ubuntu:22.04
# 最小运行时依赖
RUN apt-get install -y libsdl2-2.0 xserver-xorg libvulkan1 libomp5
# NVIDIA GPU 配置
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV SDL_VIDEODRIVER="x11"
# 直接启动 CARLA
CMD ["/bin/bash", "CarlaUnreal.sh"]
```

### 3. Docker 构建/运行脚本

#### build.sh — 构建流程（97 行）

```bash
# 流程
1. 复制 requirements.txt 到 Docker 构建上下文
2. 构建 Base 镜像: carla-base:ue5-22.04
3. 创建 Docker Volume: carla-development-ue5-22.04
4. 构建 Development 镜像: carla-development:ue5-22.04

# 参数
--ubuntu-distro 22.04    # Ubuntu 版本
--user UID:GID           # 容器内用户
--docker-gid GID         # Docker 组
--force-rebuild          # 强制重建
```

#### run.sh — 运行流程（129 行）

```bash
# 开发模式运行
docker run -it --rm \
  --runtime=nvidia \
  --net=host \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \          # X11 显示
  -v /var/run/docker.sock:/var/run/docker.sock \ # Docker socket
  -v ${CARLA_UNREAL_ENGINE_PATH}:/workspaces/unreal-engine \ # UE 引擎
  -v ${CARLA_ROOT}:/workspaces/carla \         # CARLA 项目
  carla-development:ue5-22.04 bash

# 关键挂载:
#   X11 socket → GUI 显示
#   Docker socket → 容器内运行 Docker
#   UE 引擎路径 → 编译 UE 项目
#   CARLA 项目路径 → 源码编辑
```

### 4. 环境配置工具

#### InstallPrerequisites.sh — Linux 依赖安装（138 行）

```bash
# 自动检测 Ubuntu 版本
# 安装 UE5 + CARLA 所需的全部系统依赖
sudo apt-get install -y \
  build-essential make ninja-build libvulkan1 \
  libpng-dev libtiff5-dev libjpeg-dev \
  python3 python3-dev python3-pip \
  autoconf libtool rsync curl ...

# 安装 Python 依赖
python3 -m pip install -r requirements.txt
```

### 5. 代码格式化工具

#### codeformat.py — 统一格式化脚本（430 行）

```python
# 支持多种格式化器
class CodeFormatter:
  clang-format    # C++ 格式化（.h/.cpp）
  autopep8        # Python 格式化（.py）

# 使用方式
python3 codeformat.py --format      # 格式化所有文件
python3 codeformat.py --verify      # 检查格式是否正确
python3 codeformat.py --format-only # 仅格式化不检查
```

### 6. FBX 模型转换工具

```bash
# BuildUtilsDocker.sh — 构建 FBX → OBJ 转换器
# 下载 Autodesk FBX SDK 2020.0.1
# 编译 FBX2OBJ.cpp（使用 Ninja）
# 产物: FBX2OBJ + libfbxsdk.so

# 用途: 将 FBX 格式的 3D 模型转换为 OBJ 格式
# 用于 CARLA 资产导入管线
```

### 7. 工具链总结

| 工具 | 路径 | 功能 | 使用场景 |
|------|------|------|--------|
| `Base.Dockerfile` | `Util/Docker/` | 基础镜像 | 首次构建 |
| `Development.Dockerfile` | `Util/Docker/` | 开发镜像 | 日常开发 |
| `Release.Dockerfile` | `Util/Docker/` | 发布镜像 | 生产部署 |
| `build.sh` | `Util/Docker/` | 构建镜像 | CI/CD |
| `run.sh` | `Util/Docker/` | 运行容器 | 开发/调试 |
| `InstallPrerequisites.sh` | `Util/SetupUtils/` | 安装依赖 | 裸机环境 |
| `codeformat.py` | `Util/Formatting/` | 代码格式化 | 提交前检查 |
| `clang-format` | `Util/Formatting/` | C++ 格式规则 | IDE 集成 |
| `BuildUtilsDocker.sh` | `Util/Tools/` | FBX SDK 构建 | 资产管线 |
| `Environment.sh` | `Util/Tools/` | 脚本环境 | 其他脚本 source |
| `Check.sh` | `Util/Tools/` | 代码检查 | CI/CD |
| `Deploy.sh` | `Util/Tools/` | 部署 | 发布流程 |



十八、命令行启动项目

```
/home/qiyuan/UnrealEngine/UnrealEngine5_carla/Engine/Binaries/Linux/UnrealEditor /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject
```

