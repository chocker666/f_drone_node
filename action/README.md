# Drone Node Action 接口设计说明

## 1. 文档目的

本目录定义 `drone_node` 对外提供的飞行任务 Action 接口。

Action 是外部客户端操作无人机的主要任务级接口，负责表达任务目标、执行约束、反馈信息以及执行结果。Action 不直接暴露底层飞控指令或控制器内部状态。

本文档用于确定长期接口规划和各 Action 的功能边界。未实现的 Action 可以先保留为设计说明，待对应规划算法和执行能力具备后逐步实现。

## 2. 总体设计原则

### 2.1 任务级抽象

Action 描述“飞机需要完成什么任务”，而非“控制器每一周期应该输出什么”。

例如：

- `GoTo` 表示飞行至指定位置。
- `FollowTrajectory` 表示按照指定轨迹完成运动。
- `Stop` 表示暂停当前运动任务并保留可恢复信息。
- `Continue` 表示重新规划并继续已暂停任务。

具体轨迹生成由 MotionPlanner 负责，轨迹执行及完成判定由 MotionExecutor 负责。

### 2.2 Action 生命周期

每个 Action 均具有明确的接纳、执行、反馈和结束过程。

统一支持的结束结果：

- `SUCCEEDED`：任务完成并满足成功条件。
- `ABORTED`：任务执行失败、超时或发生不可恢复错误。
- `CANCELED`：客户端主动取消当前任务。

Action 的成功必须依据真实状态或明确的执行结果确认，不能仅依据指令发送成功。

### 2.3 任务取消与轨迹停止

**Cancel：** 客户端取消当前 Action。Exec 终止对应执行过程，并丢弃该任务的恢复信息。

**Stop：** 主动中断当前运动轨迹，保留原任务目标和必要进度信息，允许之后通过 Continue 恢复。

**Continue：** 基于保存的任务信息和飞机当前真实状态重新规划轨迹，而非继续发送旧轨迹的剩余部分。

Stop 保留的任务可以被新的运动任务覆盖。新任务正式接纳后，原暂停任务直接丢弃。

### 2.4 坐标系与单位

默认约定：

- 世界坐标系：`map`，ENU。
- 位置：米（m）。
- 线速度：米每秒（m/s）。
- 线加速度：米每二次方秒（m/s²）。
- 加加速度：米每三次方秒（m/s³）。
- Yaw：弧度（rad）。
- 时间：秒（s）。

具体 Action 可以通过 `std_msgs/Header` 指定坐标系，但必须明确支持的坐标系及变换规则。

## 3. 基础飞行操作

### 3.1 Arm

**功能：** 请求无人机解锁。

**Goal：** 无。

**成功条件：** 从真实飞行器状态确认已解锁。

**限制：** 必须满足飞行状态机规定的解锁条件。

**状态：** 第一版实现。

### 3.2 Disarm

**功能：** 请求无人机上锁。

**Goal：** 无。

**成功条件：** 从真实飞行器状态确认已上锁。

**限制：** 正常情况下仅允许在已确认安全着陆后执行。

**状态：** 第一版实现。

### 3.3 Takeoff

**功能：** 从地面起飞至指定高度并建立稳定飞行状态。

**建议 Goal：**

- `height`：目标起飞高度。
- `max_speed`：最大上升速度，可选。
- `max_acceleration`：最大上升加速度，可选。
- `max_jerk`：最大加加速度，可选。
- `target_yaw`：可选目标航向。

**成功条件：** 达到目标高度，并在规定时间内满足位置、速度和姿态稳定条件。

**限制：** 起飞前必须满足已解锁、定位有效和控制可用等条件。

**备注：** 计划采用 S 型速度规划；具体轨迹传输方式取决于现有 Ctrl 的实际支持能力。

**状态：** 第一版重点实现。

### 3.4 Land

**功能：** 执行受控降落。

**建议 Goal：**

- `descent_speed`：目标下降速度。
- `target_position`：可选着陆位置。
- `auto_disarm`：着陆后是否自动上锁。

**成功条件：** 从真实飞行器状态确认已着陆。

**限制：** 必须具有可用的着陆状态或等效可靠判据。

**状态：** 后续实现。

## 4. 位置与悬停操作

### 4.1 GoTo

**功能：** 飞行至指定位置和航向，不承诺自主避障。

**Goal：**

- `header`：目标坐标系。
- `position`：目标位置。
- `yaw`：目标航向。

**后续可选约束：**

- 最大速度。
- 最大加速度。
- 最大加加速度。
- 位置及航向容差。
- 终端速度约束。

**成功条件：** 真实位置及必要的速度、姿态状态满足目标容差。

**完成处理：** MotionExecutor 向 Ctrl 发送 CANCEL，使其由轨迹跟踪转入悬停控制，确认后返回成功。

**状态：** 第一版逐步实现。

### 4.2 Hover

**功能：** 在指定位置或当前位置保持悬停。

**建议 Goal：**

- `use_current_position`：是否以当前实际位置为悬停目标。
- `position`：可选悬停位置。
- `yaw`：可选悬停航向。
- `duration`：持续时间。
- `position_tolerance`：位置容差。

**成功条件：** 在指定时间内满足悬停要求。

**备注：** 持续时间为零是否表示无限期悬停，需要在正式实现前明确。

**状态：** 规划接口。

### 4.3 Wait

**功能：** 在维持安全控制状态的前提下等待时间或条件满足。

**建议 Goal：**

- `duration`：等待时间。
- `condition_type`：等待条件类型。
- `condition_parameters`：条件参数，具体结构待定义。

**成功条件：** 等待时间结束或指定条件满足。

**与 Hover 的区别：** Wait 是任务调度语义，不必生成新的运动轨迹；Hover 是主动的位置保持任务。

**状态：** 规划接口。

## 5. 任务暂停与恢复

### 5.1 Stop

**功能：** 中断当前运动任务并保留可恢复信息。

**Goal：** 第一版可为空，未来可扩展停止策略。

**执行过程：**

1. 请求终止当前轨迹跟踪。
2. 等待 Ctrl 进入悬停控制状态。
3. 通过真实速度、位置等信息确认停止条件。
4. 保存原任务目标及必要的任务进度。

**成功条件：** 当前轨迹已终止且满足规定的稳定条件。

**注意：** Stop 成功不代表原任务完成。

**状态：** 第一版实现。

### 5.2 Continue

**功能：** 恢复 Stop 保留的任务。

**建议 Goal：**

- `task_id`：可选暂停任务标识；第一版仅有一个暂停槽位时可以省略。

**执行过程：**

1. 检查是否存在可恢复任务。
2. 获取飞机当前真实状态。
3. 根据原任务目标和当前状态重新规划。
4. 执行新轨迹。

**成功条件：** 恢复后的原任务最终完成。

**失败条件：** 不存在暂停任务、任务已被覆盖、状态不满足恢复要求或重新规划失败。

**状态：** 第一版实现。

## 6. 复杂路径与轨迹操作

### 6.1 GoToAvoid

**功能：** 在考虑障碍物约束的情况下到达指定位置。

**建议 Goal：**

- 目标位置及航向。
- 安全距离。
- 避障规划策略。
- 速度、加速度等运动约束。
- 是否允许在线重规划。

**成功条件：** 安全到达目标并满足终端约束。

**失败条件：** 无可行路径、规划超时、障碍物环境无法满足安全约束等。

**状态：** 规划接口。

### 6.2 FollowTrajectory

**功能：** 执行指定的时序轨迹，并满足规定的终端状态。

**建议 Goal：**

- 轨迹数据及时间基准。
- 轨迹坐标系。
- 终端位置、速度、航向等约束。
- 跟踪误差容差。
- 是否允许在线重规划。
- 结束后的保持策略。

**成功条件：** 完成轨迹执行，并满足指定终端状态要求。

**备注：** 轨迹终点不一定要求速度为零。应支持以指定速度通过终点。

**状态：** 规划接口。

### 6.3 ExecuteWaypoints

**功能：** 依次执行多个航点任务。

**建议 Goal：**

- 航点序列。
- 各航点的位置、航向及容差。
- 各航点的可选停留时间。
- 路径规划及运动约束。

**成功条件：** 所有要求的航点均完成。

**暂停恢复：** 应保存已完成航点索引及剩余任务目标，并从当前状态重新规划。

**状态：** 规划接口。

### 6.4 ReturnHome

**功能：** 返回预定义返航位置。

**建议 Goal：**

- 返航策略。
- 返航高度。
- 是否自动降落。

**成功条件：** 根据返航策略到达目标位置，或完成返航及降落。

**备注：** 返航位置应由系统配置或可靠的起飞点记录提供。

**状态：** 规划接口。

## 7. 高级运动操作

### 7.1 Orbit

**功能：** 围绕指定空间中心进行绕飞。

**建议 Goal：** 圆心、半径、飞行高度、角速度、圈数或持续时间、航向策略。

**成功条件：** 完成指定圈数或持续时间。

**状态：** 远期规划。

### 7.2 TrackTarget

**功能：** 根据外部目标状态持续跟踪目标。

**建议 Goal：** 目标标识、期望相对位置、速度约束、跟踪持续时间、目标丢失策略。

**成功条件：** 满足指定跟踪时间或终止条件。

**状态：** 远期规划。

### 7.3 VelocityMove

**功能：** 按指定速度目标执行有限时长运动。

**建议 Goal：** 目标线速度、可选偏航角速度、持续时间、加速度限制。

**成功条件：** 完成指定运动时段并满足结束策略。

**状态：** 远期规划。

### 7.4 AttitudeMove

**功能：** 执行具有明确持续时间和终止条件的姿态运动任务。

**建议 Goal：** 目标姿态、角速度约束、推力策略、持续时间。

**成功条件：** 满足姿态运动及终止条件。

**备注：** 需要与连续姿态控制指令区分；底层必须具有对应控制能力。

**状态：** 远期规划。

### 7.5 EmergencyStop

**功能：** 按预定义安全策略执行紧急停止操作。

**建议 Goal：** 应急策略标识。

**成功条件：** 进入预期应急状态并获得可确认结果。

**注意：** 不应将 EmergencyStop 简单等同于普通 Stop 或立即停转电机。实际策略必须根据飞行状态和安全要求确定。

**状态：** 远期规划。

## 8. Action 的公共反馈与结果约定

### 8.1 Result

建议所有 Action 至少包含：

- `success`：任务是否成功。
- `error_code`：错误码。
- `message`：结果描述。

Action 的最终状态仍以 ROS2 Action 的终止状态为准，`success` 作为业务结果字段。

### 8.2 Feedback

按任务类型提供必要信息，包括：

- 当前任务阶段。
- 执行进度。
- 剩余距离或剩余时间。
- 当前高度。
- 位置及航向误差。
- 必要的状态描述。

所有反馈应来自实际执行过程或明确的规划计算，不得使用模拟进度替代真实执行结果。

## 9. 并发与覆盖规则

第一版采用单活动运动任务模型。

- 同一时刻最多执行一个主运动任务。
- Stop 可以中断当前运动任务。
- 客户端 Cancel 放弃对应任务，不保留恢复信息。
- Stop 最多保留一个暂停任务。
- 新运动任务正式接纳后，可以覆盖并清除已有暂停任务。
- Continue 只能恢复当前有效的暂停任务。
- 飞行故障及安全限制优先于普通任务调度。
- 不允许旧轨迹反馈影响新轨迹的执行结果。

## 10. 实现阶段

**第一阶段：** Arm、Disarm、底层通信、状态确认。

**第二阶段：** Takeoff、GoTo、Stop、Continue、Hover。

**第三阶段：** Land、Wait、FollowTrajectory、ExecuteWaypoints、GoToAvoid、ReturnHome。

**第四阶段：** Orbit、TrackTarget、VelocityMove、AttitudeMove、EmergencyStop。

上述阶段是功能规划，不代表当前代码已经支持这些能力。

## 11. 待确定事项

1. Takeoff 的 S 曲线轨迹传输形式。
2. 各动作的默认位置、速度、姿态容差及稳定时间。
3. Hover 无限期执行时的 Action 生命周期。
4. Land 的着陆确认及自动上锁策略。
5. FollowTrajectory 的轨迹消息格式及时间同步方式。
6. Stop 后的暂停任务保留范围与 Continue 的恢复策略。
7. EmergencyStop 的具体安全策略。
8. 新 Action 是否需要扩展公共错误码与任务标识机制。

本说明文档作为 Action 接口设计基线，具体 `.action` 消息定义应在对应功能实现前进一步确认。

## Exec 正式版重构：阶段 5 当前行为

当前已实现：

- `/drone_node/arm`
- `/drone_node/disarm`

以下 Action Server 已注册，但 Goal 明确拒绝：

- `/drone_node/takeoff`
- `/drone_node/goto`
- `/drone_node/land`
- `/drone_node/stop`

拒绝 Goal 不产生已接纳任务的 Result。

### Arm/Disarm 成功条件

1. FSM 允许当前动作。
2. SetArming 服务响应成功。
3. 请求发出后收到新鲜、匹配的实际 armed 观测。
4. FSM 接受 ARM_CONFIRMED 或 DISARM_CONFIRMED。

已处于目标 armed 状态的 Goal 按 FSM 准入规则拒绝。

### Cancel

- SetArming 请求发送前：允许取消，Action 最终为 CANCELED。
- 请求发送后：拒绝取消，原 Action 继续等待真实结果。
- 不自动发送反向 Arm/Disarm 命令。
- 清理本地 pending request 不代表撤回已发送命令。

### Feedback.stage

- 0：Goal 已接纳，尚未发送服务请求。
- 1：等待服务可用。
- 2：请求已发送，等待服务结果。
- 3：服务成功，等待实际 armed 确认。
- 4：服务与实际状态均已确认。

反馈描述真实执行阶段，不提供模拟百分比。

### Result.error_code

- 0：成功。
- 1：发送前取消。
- 2：等待服务可用超时。
- 3：执行或确认超时。
- 4：服务返回失败。
- 5：状态准入或确认被拒绝。
- 6：FSM 处于 FAULT。
- 7：内部异常。

请求发送后的失败或超时采用保守策略，Action 为 ABORTED，
FSM 锁存 FAULT；不会自动重试、反向操作或自动恢复。

正常数据重新到达不会解除 FAULT。

## Exec 正式版重构：阶段 6 Takeoff

当前支持 Arm、Disarm、Takeoff。
GoTo、Land、Stop 的 Goal 仍明确拒绝。

### Takeoff.height

- 大于 0：使用 Goal 给出的相对高度增量。
- 等于 0：使用 action_manager.takeoff.default_height。
- 小于 0、NaN、无穷值：拒绝 Goal。

### 执行

必须已解锁、真实接地、定位和控制反馈有效，
并通过 FlightStateMachine 准入。

开始时固定当前位置和 Yaw：
目标为 (x0, y0, z0 + height)，Yaw 保持 yaw0。
MotionPlanner/FixedPlanner 生成 POINT，
控制器为 position_yaw_pid，配置为 default。

必须确认真实离地，再进行到达稳定判定。
到达后自动 CANCEL，确认 HOVER 和真实稳定状态，
FSM 转为 HOVERING 后，Action 才 SUCCEEDED。

### Feedback

- current_height：相对任务开始位置的真实高度变化，单位 m。
- progress：真实高度变化与请求高度之比，限制在 [0, 1]。
- phase：实际执行或终态确认阶段。

progress=1 不代表 Action 已完成；
仍需稳定、CANCEL/HOVER 和 FSM 确认。

### Cancel

发布轨迹前可直接取消。
发布后请求真实停止，确认接地或稳定悬停后才 CANCELED。
不保留暂停信息，不自动 Disarm。
故障和无法确认安全停止优先报告 ABORTED。

Result.error_code 新增：
- 8：运动执行失败。

本阶段不自动解锁，不自动切换 PX4 Offboard。

## Exec 正式版重构：阶段 7 GoTo

当前支持 Arm、Disarm、Takeoff、GoTo。
Land、Stop 的 Goal 继续明确拒绝。

### GoTo Goal

- header.frame_id 必须严格为 map。
- position 是 map/ENU 绝对位置，单位 m。
- yaw 是 ENU 目标航向，单位 rad。
- 位置和 yaw 必须为有限数值。
- header.stamp 不作为预约执行时间。

### 准入及执行

仅从已确认的 HOVERING 接纳。
同一时间只允许一个活动任务，普通新 Goal 不抢占旧任务。

MotionPlanner/FixedPlanner 生成 POINT，
使用 position_yaw_pid、default 配置和最短 Yaw 旋转方向。

必须经历匹配轨迹 ID 的 TRACKING、真实位置/Yaw/速度稳定、
自动 CANCEL、新鲜 HOVER 和 FSM GOTO_COMPLETED，
Action 才能 SUCCEEDED。

### Feedback

- remaining_distance：当前真实位置到固定目标的三维距离。
- progress：相对于初始三维距离的位置进度，限制在 [0, 1]。
- phase：当前执行或确认阶段。

相同位置、不同 Yaw 的任务仍需姿态和稳定确认。
progress=1 不代表 Action 已完成。

### Cancel 与异常

发布前取消直接 CANCELED。
发布后取消必须等待安全停止及 FSM 确认，再 CANCELED。
不保留恢复上下文。

故障和无法确认安全停止优先报告 ABORTED。
仅处理匹配当前任务轨迹 ID 的执行事件。
暂停、Continue 和 Stop 留待阶段 8。

## Exec 正式版重构：阶段 8

当前支持 Arm、Disarm、Takeoff、GoTo、Stop、Continue、Hover。
Land 仍明确拒绝。

### Stop

Stop 抢占普通运动任务，但不撤销已发送的 Arm/Disarm 服务。
重复的活动 Stop 被拒绝。

必须取得匹配执行事件及请求后的新鲜停止观测，
连续稳定后才能成功。已处于 HOVER 时不重复发送 CANCEL。

Stop 成功后原运动 Goal 为 ABORTED，error_code=9（PREEMPTED），
不会 SUCCEEDED。暂停槽保存目标、约束、剩余运动时间预算及任务进度，
不保存旧 Ctrl 轨迹或旧轨迹 ID。

客户端同时取消原任务时，原任务 CANCELED，不保存暂停信息。
取消 Stop 不恢复原运动：安全停止后 Stop 为 CANCELED，暂停槽清空。
异常优先于普通完成，无法确认安全停止时 ABORTED 并锁存 FAULT。

### Continue

Goal 为空，仅恢复唯一有效暂停槽。
正式接纳即消费暂停槽。

根据当前真实状态调用 MotionPlanner 生成新 POINT，
使用新的轨迹 ID。旧 GoalHandle 不复活。
恢复 Takeoff 使用原绝对目标，不再次增加起飞高度。

新运动 Goal 正式接纳后立即清空旧暂停任务。
观测失效、故障或成功 Disarm 会使暂停任务失效。
剩余运动预算不足时不生成可恢复任务。

### Hover

Goal.duration 必须有限且 >=0，单位秒。
仅保持接纳后开始执行时的当前位置和 Yaw。

先建立真实稳定悬停，再开始保持计时。
duration=0：稳定悬停确认后立即成功，底层继续悬停。
duration>0：持续满足真实目标约束达指定时间后成功。

Hover 可被 Stop；Continue 重新规划到原保持目标，
再完成剩余保持时间。保持中 Cancel 也必须确认安全停止。

### 反馈

Continue 使用 phase、progress、remaining_distance。
Hover 使用 phase、progress、elapsed_time。

进度来自真实距离或实际稳定保持时间；
progress=1 不代替最终 Result。

## Exec 正式版：阶段 9 Land

当前支持 Arm、Disarm、Takeoff、GoTo、Land、Stop、Continue、Hover。

Land 仅从已确认 HOVERING 接纳。
开始时固定 x0、y0、yaw0，POINT 参考目标为 (x0,y0,-1.0)。
该策略要求实际地面高于 map z=-1.0，不是通用地形跟随降落。

descent_speed 仅保留接口兼容，有限非负值均可接受，
但不控制下降速度。实际下降由现有位置控制器配置决定。

新鲜 landed=true 到达后立即 CANCEL，停止追踪地下参考点。
确认新鲜 HOVER、持续接地及位置/Yaw/速度稳定后，
FSM 转为 GROUNDED，Land 才能完成。
不要求位置 z 到达 -1.0。

land.auto_disarm 参数默认 false；当前 config/exec.yaml 显式设为 true。
启用后，Land Result 还需等待异步 Disarm 服务成功和实际 armed=false。
若 PX4 已自行上锁，则依据真实状态确认。
接地前意外失锁仍是故障。

Land Cancel/Stop 沿用安全停止规则。
空中暂停的 Land 可通过 Continue 重新规划恢复；
已接地的暂停 Land 不再有效。

Land Feedback.current_height 表示实际 map z。
progress 是真实下降位置进度；接地确认后为 1。
progress=1 不代替 Result，自动 Disarm 可能仍在进行。

正常启动使用 drone_node.launch.py，同时启动 Ctrl、Inter 和 Exec。
启动本身不自动执行飞行任务。