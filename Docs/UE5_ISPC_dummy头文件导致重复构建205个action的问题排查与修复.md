# UE5 ISPC dummy 头文件缺失导致每次构建重复编译 205 个 action

## 问题现象

在 CARLA UE5 项目中，即使未修改任何代码、`git status` 完全干净，每次执行构建命令都会重新执行 **205 个 action**（约 195 秒），而非预期的 `Target is up to date`。

```
------ Building 205 action(s) started ------
[1/205] Generate Header [x86-64] PBDCollisionSolver.ispc
[2/205] Generate Header [x86-64] BonePose.ispc
...
[204/205] WriteMetadata UnrealEditor.version (CarlaUnrealEditor)
[205/205] WriteMetadata CarlaUnrealEditor.target
Total execution time: 194.87 seconds
```

关闭编辑器后再次构建，问题依旧复现。

---

## 环境信息

| 项目 | 值 |
|------|-----|
| 操作系统 | Ubuntu 22.04 |
| UE5 引擎 | ue5-dev-carla 分支 |
| 构建目标 | CarlaUnrealEditor Linux Development |
| 构建命令 | `Build.sh CarlaUnrealEditor Linux Development -Project="...CarlaUnreal.uproject" -buildscw` |
| ISPC 版本 | Intel ISPC 1.24.0 (LLVM 18.1.6) |
| 物理核心 | 20 cores / 28 logical cores |

---

## 根因分析

### 1. 构建系统的工作机制

UE5 使用 **Adaptive Non-Unity Build** 机制，通过 `git status` 检测工作区变更来决定编译范围。构建产物通过 **Makefile** 进行依赖追踪，Makefile 中每个 ISPC 头文件生成规则定义了**输入**（`.ispc` 源文件）和**输出**（生成的头文件）。

Make 通过比较输出文件与输入文件的**时间戳**来判断是否需要重新执行：
- 如果输出文件不存在 → 需要重新生成
- 如果输出文件比输入文件旧 → 需要重新生成
- 如果输出文件比输入文件新 → 跳过（up to date）

### 2. ISPC 构建管线的输出不匹配

UE5 的 Makefile 为每个 ISPC 文件定义了以下**期望输出**：

```
PBDCollisionSolver.ispc.generated.h          ← 主生成头文件
PBDCollisionSolver.ispc.generated_avx.h      ← AVX 变体
PBDCollisionSolver.ispc.generated_avx2.h     ← AVX2 变体
PBDCollisionSolver.ispc.generated_sse4.h     ← SSE4 变体
PBDCollisionSolver.ispc.generated_avx512skx.h ← AVX512 变体
PBDCollisionSolver.ispc.generated.dummy_avx.h     ← ⚠️ dummy 标记文件
PBDCollisionSolver.ispc.generated.dummy_avx2.h    ← ⚠️ dummy 标记文件
PBDCollisionSolver.ispc.generated.dummy_sse4.h    ← ⚠️ dummy 标记文件
PBDCollisionSolver.ispc.generated.dummy_avx512skx.h ← ⚠️ dummy 标记文件
```

但 ISPC 工具实际只生成了**前 5 个文件**（`generated.h` + 各变体），**从未生成 `dummy_*.h` 文件**。

### 3. 为什么每次都触发重建

```
Makefile 检查:
  dummy_avx.h 存在？ → ❌ 不存在
  → 判定：需要重新生成 ISPC 头文件
  → 触发 "Generate Header" action
  → 但 ISPC 工具仍然不生成 dummy 文件
  → 下次构建重复上述循环
```

**这是 UE5 构建系统的一个 bug**：Makefile 生成逻辑（UBT）期望 ISPC 工具输出 dummy 文件，但 ISPC 工具的命令行参数中并未包含生成 dummy 文件的选项（如 `--dummy-filename`）。

### 4. 影响的模块

共 **152 个 dummy 文件**缺失，涉及以下引擎模块：

| 模块 | dummy 文件数 | 说明 |
|------|-------------|------|
| Chaos | ~80 | PBD 物理求解器 |
| Engine | ~40 | 骨骼蒙皮、GPU 皮肤、动画等 |
| Niagara | ~16 | 粒子系统 |
| ChaosCloth | ~8 | 布料模拟 |
| GeometryCollection | ~8 | 几何体集合 |

这些模块的 ISPC 头文件生成 action 每次都触发，连带后续的 Compile 和 Link 步骤也全部执行，总计 205 个 action。

---

## 排查过程

### 第一步：确认 git 状态干净

```bash
cd /home/qiyuan/UnrealEngine/UnrealEngine5_carla && git status --short  # 0 变更
cd /home/qiyuan/UnrealEngine/CarlaUE5 && git status --short             # 0 变更
```

### 第二步：定位中间产物状态

```bash
# CarlaUnrealEditor 目标的中间目录只有元数据，无编译产物
find .../CarlaUnrealEditor/Development/ -name "*.o"  # 0 个（首次构建前）

# 引擎模块的 ISPC 生成头文件存在
find .../UnrealEditor/Development/Chaos/ -name "*.ispc.generated.h"  # 存在
```

### 第三步：发现时间戳异常

```bash
# 源文件
stat PBDCollisionSolver.ispc              # 时间戳: 1783418768

# Makefile 实际检查的 dummy 文件
stat PBDCollisionSolver.ispc.generated.dummy_avx.h  # ❌ 文件不存在！

# 实际生成的头文件
stat PBDCollisionSolver.ispc.generated.h   # 时间戳: 1784540207（比源文件新）
```

### 第四步：确认 Makefile 引用了不存在的文件

```bash
strings Makefile.bin | grep "dummy_avx"
# 输出: 大量 dummy_avx.h / dummy_avx2.h / dummy_sse4.h / dummy_avx512skx.h 路径
```

### 第五步：确认 ISPC 工具不生成 dummy 文件

```bash
# 查看所有 ISPC 产物
find .../Chaos/ -name "PBDCollisionSolver.ispc*" -type f
# 结果: generated.h, generated_avx.h, generated_sse4.h, .o 文件等
# 缺失: dummy_avx.h, dummy_avx2.h, dummy_sse4.h, dummy_avx512skx.h
```

---

## 修复方案

### 方法：创建缺失的 dummy 文件

从 Makefile 中提取所有引用的 dummy 文件路径，创建为空文件：

```bash
# 1. 从 Makefile 提取缺失的 dummy 文件路径
strings /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Intermediate/Build/Linux/x64/CarlaUnrealEditor/Development/Makefile.bin \
  | grep -oP '[^\s;<>|={}]+\.ispc\.generated\.dummy_[a-z0-9]+\.h' \
  | sort -u > /tmp/missing_dummy_files.txt

# 2. 查看数量
wc -l /tmp/missing_dummy_files.txt  # 152 个

# 3. 创建空文件（自动创建父目录）
while IFS= read -r f; do
  mkdir -p "$(dirname "$f")"
  touch "$f"
done < /tmp/missing_dummy_files.txt

echo "Created $(wc -l < /tmp/missing_dummy_files.txt) dummy files"
```

### 验证修复

```bash
# 运行构建
/home/qiyuan/UnrealEngine/UnrealEngine5_carla/Engine/Build/BatchFiles/Linux/Build.sh \
  CarlaUnrealEditor Linux Development \
  -Project="/home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject" \
  -buildscw

# 预期输出:
# Target is up to date
# Total execution time: 0.xx seconds
```

---

## 注意事项

1. **清理 Intermediate 后需重新修复**：如果执行了清理操作（如删除 `Intermediate/Build/` 目录），dummy 文件会丢失，需要重新执行上述修复脚本。

2. **引擎更新后可能复发**：UE5 引擎代码更新可能改变 ISPC 构建管线行为，如果问题复发，重新运行修复脚本即可。

3. **dummy 文件是空文件不影响编译**：这些文件仅作为 Makefile 的时间戳标记，不参与实际的编译和链接过程。ISPC 工具生成的实际头文件（`generated.h`、`generated_avx.h` 等）才是编译器使用的。

4. **同时适用于引擎目标和项目目标**：引擎的 `UnrealEditor` 和项目的 `CarlaUnrealEditor` 共享同一套引擎模块的 ISPC 中间文件，修复一次即可同时解决两个目标的重复构建问题。

---

## 相关文件路径

| 文件 | 说明 |
|------|------|
| `CarlaUE5/Unreal/CarlaUnreal/Intermediate/Build/Linux/x64/CarlaUnrealEditor/Development/Makefile.bin` | 项目构建目标的 Makefile |
| `UnrealEngine5_carla/Engine/Intermediate/Build/Linux/x64/UnrealEditor/Development/Makefile.bin` | 引擎构建目标的 Makefile |
| `UnrealEngine5_carla/Engine/Intermediate/Build/Linux/x64/UnrealEditor/Development/Chaos/*.ispc.generated.*` | ISPC 生成的头文件和编译产物 |
| `UnrealEngine5_carla/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Collision/*.ispc` | ISPC 源文件 |
