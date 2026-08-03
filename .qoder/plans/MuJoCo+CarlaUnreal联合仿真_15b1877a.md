# MuJoCo + CarlaUnreal 联合仿真集成计划

## 目标架构

```
robot_mujoco (standalone)          mc_ctrl (type=5)           CarlaUnreal
  [物理仿真]                     [RL运动控制]                [纯渲染]
      |                              |                          |
      |--- UDP 25001 RobotState ---> | (接收状态,计算RL)        |
      |                              |                          |
      | <--- UDP 25002 RobotCmd ---- | (发送关节PD目标)         |
      |                              |                          |
      |--- UDP 25001 RobotState -----------------------------> | (接收状态,更新mesh)
                                     |                          |
                                     | <--- 43997 HighLevel --- | (虚拟键盘脚本)
```

**关键事实：**
- `robot_mujoco` 和 CarlaUnreal 使用相同的 protobuf 格式 (`robot_sdk::pb::RobotState`, 303 bytes)
- `robot_mujoco` 通过 `sendto()` 广播状态到 port 25001
- mc_ctrl (type=5) 绑定 25001 接收状态，发送 RobotCmd 到 25002
- CarlaUnreal 只需 **接收** 25001 的 RobotState 并更新渲染

---

## Phase 1: 验证 standalone MuJoCo 可运行

**目标：** 确认 `robot_mujoco` 能正常启动并广播 UDP 状态

1. 启动 robot_mujoco:
```bash
cd /home/qiyuan/Softwares/Matrix/src/robot_mujoco/simulate/build
export LD_LIBRARY_PATH="/opt/mujoco/lib:/lib:$(pwd):${LD_LIBRARY_PATH:-}"
./robot_mujoco
```

2. 用 Python 脚本验证 port 25001 有数据广播:
```python
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 25001))
data, addr = s.recvfrom(4096)
print(f"Received {len(data)} bytes from {addr}")
```

3. 确认数据是 303 bytes protobuf RobotState（与 CarlaUnreal 当前发送的格式一致）

**如果 robot_mujoco 不是广播而是发送到特定 IP：** 需要用 `socat` 或修改 CarlaUnreal 监听不同端口，再转发。

---

## Phase 2: CarlaUnreal 添加"外部物理渲染模式"

**目标：** CarlaUnreal 不再跑内部 mj_step，改为从 UDP 接收 RobotState 驱动渲染

### 2.1 添加模式开关

文件: `Plugins/MuJoCoUE/Source/MuJoCoUE/Public/MuJoCoSimulation.h`

```cpp
/** 外部物理模式: 不跑内部 mj_step, 从 UDP 接收 RobotState 驱动渲染 */
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Control")
bool bExternalPhysicsMode = false;

/** 外部物理模式下的 UDP 状态接收器 (监听 port 25001) */
UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UDP Control")
UUdpReceiverComponent* UdpStateReceiver;
```

### 2.2 新增 RobotState 接收与解析

文件: `Plugins/MuJoCoUE/Source/MuJoCoUE/Private/MuJoCoSimulation.cpp`

在 `BeginPlay()` 中:
- 如果 `bExternalPhysicsMode=true`:
  - 仍加载 MuJoCo XML（用于生成 mesh 和 body 层级）
  - 创建 `UdpStateReceiver` 绑定 port 25001
  - 注册回调: 收到 RobotState protobuf → 解析 → 存入缓存
  - 不启动内部物理循环 (`bSimulationRunning = false`)

在 `Tick()` 中:
- 如果 `bExternalPhysicsMode=true`:
  - 从缓存读取最新 RobotState
  - 将 joint_pos[12] 写入 `mData->qpos` (仅用于 FK 计算)
  - 将 base quat/pos 写入 `mData->qpos[0:6]`
  - 调用 `mj_kinematics()` 计算所有 body 的世界坐标
  - 调用 `UpdateSimulationView()` 更新 mesh transform
  - **不调用** `mj_step()` / `SimulateMuJoCo()`

### 2.3 禁用内部物理相关逻辑

在外部模式下跳过:
- `SimulateMuJoCo()` (mj_step)
- `SendStateToMcCtrl()` (不再需要发送，MuJoCo 自己发)
- `ApplyUdpControl()` / `ApplyStandUpControl()` (mc_ctrl 直接控制 MuJoCo)
- `UpdateGaitTargets()` (内部步态生成器)
- 启动延迟逻辑 (`UDP_STARTUP_DELAY`)

### 2.4 复用现有 UdpReceiverComponent 或新建

当前 `UdpReceiverComponent` 解析的是 **RobotCmd** (mc_ctrl → 仿真器)。
外部模式需要解析的是 **RobotState** (MuJoCo → 渲染器)。

两种方案:
- **方案A (推荐):** 在 `UdpReceiverComponent` 中添加 `bParseAsRobotState` 模式，复用线程框架
- **方案B:** 新建 `UdpStateReceiverComponent` 专门解析 RobotState

RobotState 解析逻辑 (参考现有 `SendStateToMcCtrl()` 的逆过程):
```cpp
robot_sdk::pb::RobotState StateMsg;
StateMsg.ParseFromArray(RawData.GetData(), RawData.Num());
// 提取: joint_pos (12), joint_vel (12), quat (4), position (3), gyro (3), acc (3)
```

---

## Phase 3: 坐标映射验证

**目标：** 确认 standalone MuJoCo 发出的 joint_pos 与 CarlaUnreal mesh 的关节映射一致

关键对应关系 (已在内部物理模式验证过):
- RobotState.joint_pos[0:11] → mData->qpos[7:18] (12个关节)
- RobotState.quat[0:3] → mData->qpos[3:6] (base orientation, wxyz)
- RobotState.position[0:2] → mData->qpos[0:2] (base position, xyz)

**注意:** 当前 CarlaUnreal 的 `SendStateToMcCtrl()` 中已经实现了这些字段的序列化，反向解析即可。

---

## Phase 4: 虚拟键盘脚本适配

**目标：** 键盘输入发给 mc_ctrl 的 highlevel port (43997)

当前虚拟键盘脚本如果是发给 CarlaUnreal 的，需要改为发给 mc_ctrl:
- mc_ctrl highlevel 端口: 43997 (参考 `run_mc.sh` 和 `sdk_config.yaml`)
- 协议: 参考 Matrix 的 HighLevel API (standUP/lieDown/move 等)
- 或者使用 Matrix 提供的 `robot_sdk` Python 接口

如果现有脚本已经通过 mc_ctrl SDK 通信，则无需修改。

---

## Phase 5: 启动脚本

**目标：** 一键启动三个进程

创建 `/home/qiyuan/Softwares/Matrix/scripts/run_carlaunreal_sim.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

MATRIX_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 1. 启动 standalone MuJoCo
cd ${MATRIX_ROOT}/src/robot_mujoco/simulate/build
export LD_LIBRARY_PATH="/opt/mujoco/lib:${LD_LIBRARY_PATH:-}"
./robot_mujoco > /tmp/robot_mujoco.log 2>&1 &
MUJOCO_PID=$!
echo "[1/3] robot_mujoco started (PID=$MUJOCO_PID)"

# 2. 等待 MuJoCo 初始化
sleep 2

# 3. 启动 CarlaUnreal (外部物理模式)
# 根据实际启动方式修改 (编辑器/打包二进制)
cd /path/to/CarlaUnreal
./CarlaUnreal.sh -game -ExternalPhysics > /tmp/carlaunreal.log 2>&1 &
UE_PID=$!
echo "[2/3] CarlaUnreal started (PID=$UE_PID)"

# 4. 等待 UE 初始化
sleep 5

# 5. 启动 mc_ctrl (type=5)
cd ${MATRIX_ROOT}/src/robot_mc/build/export/mc/bin
export LD_LIBRARY_PATH="$(pwd):${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=XG
export SDK_CLIENT_IP=127.0.0.1
taskset -c 7 ./mc_ctrl r > /tmp/mc_ctrl.log 2>&1 &
MC_PID=$!
echo "[3/3] mc_ctrl started (PID=$MC_PID)"

echo "All started. Press Ctrl+C to stop."
trap "kill $MUJOCO_PID $UE_PID $MC_PID 2>/dev/null" EXIT
wait
```

---

## Phase 6: 配置检查清单

| 配置项 | 文件 | 值 | 说明 |
|--------|------|-----|------|
| motor_platform_type | xg-user-parameters.yaml | **5** | MujocoCommandInterface |
| mujoco_running | config/config.json | true | MuJoCo 物理启用 |
| state_port | config/config.json | 25001 | 状态广播端口 |
| cmd_port | config/config.json | 25002 | 指令接收端口 |
| robot | simulate/config.yaml | "xgb" | 机器人类型 |
| robot_scene | simulate/config.yaml | 与 CarlaUnreal XML 一致 | 场景文件 |
| bExternalPhysicsMode | CarlaUnreal 代码/蓝图 | true | 外部物理渲染模式 |
| bUdpControlEnabled | CarlaUnreal 代码 | false (外部模式下) | 禁用内部 UDP 控制 |

---

## Phase 7: 集成测试

1. **UDP 连通性:** 启动 robot_mujoco → 用 Python 验证 25001 有数据
2. **渲染同步:** 启动 CarlaUnreal (外部模式) → 确认机器人 mesh 跟随 MuJoCo 状态动
3. **控制回路:** 启动 mc_ctrl → 按 U 键 → 确认 MuJoCo 中机器人站立 → CarlaUnreal 同步显示
4. **行走测试:** WASD → 机器人行走 → CarlaUnreal 渲染同步

---

## 风险与备选

| 风险 | 影响 | 备选方案 |
|------|------|----------|
| robot_mujoco 不是广播，而是发送到特定 IP | CarlaUnreal 收不到数据 | 用 socat 转发，或 iptables DNAT |
| RobotState protobuf 字段顺序/版本不一致 | 解析错误 | 对比 CarlaUnreal 的 SendState 和 MuJoCo 发出的包 |
| 场景 XML 不一致 | mesh 与物理不匹配 | 确保 CarlaUnreal 加载与 robot_mujoco 相同的 XML |
| 帧率不同步 (MuJoCo 1kHz, UE 30fps) | 渲染抖动 | 在 Tick 中插值，或使用最新帧 |

---

## 工作量估计

| Phase | 预计时间 | 难度 |
|-------|----------|------|
| Phase 1: 验证 MuJoCo UDP | 0.5h | 低 |
| Phase 2: CarlaUnreal 外部模式 | 3-4h | 中 |
| Phase 3: 坐标映射验证 | 1h | 低 |
| Phase 4: 键盘脚本适配 | 0.5-1h | 低 |
| Phase 5: 启动脚本 | 0.5h | 低 |
| Phase 6: 配置 | 0.5h | 低 |
| Phase 7: 集成测试 | 1-2h | 中 |
