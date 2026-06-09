# 实验数据记录说明

## 前置条件

1. 每次跑实验前重启 Gazebo: `cd ~/rmf_ws/llm_robot_agent && ./run_gazebo.sh`
2. 确认 `/startup_done` topic 有数据后再输入指令
3. 每个场景跑完后，用 `/status` 查看最终位置并记录

## 场景1：单机单步移动

**指令**: `一号向前移动1米`

**执行**:
1. `./run_gazebo.sh` (确保 robot1 在 (0,0))
2. 另一个终端 `cd ~/rmf_ws/llm_robot_agent && ~/go1.23/bin/go run main.go --simulator=ros2`
3. 输入指令 `一号向前移动1米`
4. 观察执行，记录 `pos_before` (从 `/status` 获取) 和 `pos_after`
5. 重复 10 次，每次重启 `run_gazebo.sh`

## 场景2：单机顺序依赖

**指令**: `先移动到(3,1)，然后再回到原点(0,0)`

## 场景3：双机并行

**指令**: `一号移动到(3,1)，二号移动到(1,3)`

## 场景4：故障恢复

**指令**: `一号先移动到(3,1)，再回到原点(0,0)`

**故障注入方法**: 在 robot1 移动过程中，终端 Ctrl+C 杀掉 `robot_bridge_node.py` 进程

## 演示视频录制

场景3，录制流程：
1. 启动 `run_gazebo.sh`
2. 启动 agent：`~/go1.23/bin/go run main.go --simulator=ros2`
3. 输入指令 `一号移动到(3,1)，二号移动到(1,3)`
4. 等待执行完成
5. 输入 `/status` 展示最终位置
6. 录制工具推荐: `kazam` 或 `obs`
