自己机械臂的控制项目，厂家自带资料：
- 机器人描述URDF: `alicia_d_descriptions` 
    - 查看描述文件: `ros2 launch alicia_d_descriptions display.launch.py`
- 机器人驱动SDK: `alicia_d_driver`
经过可视化发现原厂urdf的x轴朝机械臂正后方，因此需要修改urdf,
- 将原始的`base_link`改名为`base_link_model`，
- 新建空的`base_link`，新建关节，并将`base_link_model`固定到`base_link`上，z轴旋转180度
- 将原来的`joint1`中的`link1`固定到`base_link_model`上

# 制作 MoveIt 配置包

本项目直接使用厂家的 `alicia_d_descriptions` 和 `alicia_d_driver`，自己生成并维护 `moveit_config`。以下步骤用于从厂家的 URDF 重新生成当前配置。

## 1. 启动 Setup Assistant 并加载 URDF

```bash
cd my_arm_project
source /opt/ros/humble/setup.zsh
# 工作空间已经构建过时再执行：source install/setup.zsh
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

- 新建配置时选择 `Create New MoveIt Configuration Package`。
- 加载厂家 URDF：`alicia_d_descriptions/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf`。
- 修改现有配置时选择 `Edit Existing MoveIt Configuration Package`，目录为 `src/moveit_config`。
- 生成配置前先提交或备份已有文件。Setup Assistant 可能覆盖 SRDF、关节限制、控制器配置和手工修改。

## 2. Self-Collisions

- 点击 `Generate Collision Matrix`。
- 使用默认采样参数即可。该步骤生成 SRDF 中允许忽略的自碰撞 link 组合。

## 3. Virtual Joints

添加机器人与世界坐标系之间的固定虚拟关节：

- Name：`virtual_joint`
- Parent Frame：`world`
- Child Link：`base_link`
- Type：`fixed`

`virtual_joint` 只负责连接 `world` 与机器人，不要加入 `arm` 规划组。运行时由 `my_robot_bringup` 发布 `world -> base_link` 静态 TF。

## 4. Planning Groups

机械臂规划组：

- Group Name：`arm`
- Kinematic Solver：`kdl_kinematics_plugin/KDLKinematicsPlugin`
- 使用 `Add Kin. Chain`，不要使用 `Add Joints` 逐个添加 Joint1 到 Joint6。
- Base Link：`base_link`
- Tip Link：`gripper_center`

生成后的 SRDF 必须包含：

```xml
<group name="arm">
    <chain base_link="base_link" tip_link="gripper_center"/>
</group>
```

`gripper_center` 是机械臂位姿规划使用的工具中心点（TCP）。因此 `tf2_echo base_link gripper_center` 获取的姿态，可以作为 `goToPoseTarget` 的目标姿态。

夹爪规划组：

- Group Name：`gripper`
- 使用 `Add Joints` 添加 `Gripper`、`finger` 和 `right_finger`，与当前 SRDF 保持一致。
- 控制器只向主动关节 `Gripper` 发送命令；`right_finger` 是 mimic joint，`finger` 是固定关节。

## 5. Robot Poses 和其他页面

- Robot Poses：按需添加 `home`、`pose_1`、`pose_2`、`gripper_open`、`gripper_half` 和 `gripper_closed`。
- 重新创建 Planning Group 会删除原组关联的命名姿态，生成前需要再次检查。
- End Effectors：当前代码直接以 `gripper_center` 为 chain tip，不依赖 SRDF 的 end-effector 标签，可以跳过。
- Passive Joints：当前没有需要额外声明的被动关节，可以跳过。
- 3D Perception：当前没有接入深度相机，可以跳过。
- ROS 2 Controllers：创建 `arm_controller` 和 `gripper_controller`，类型均为 `FollowJointTrajectory`。

## 6. ros2_control URDF Modifications

厂家的硬件接口为 Joint1 至 Joint6 和 Gripper 导出了 `position`、`velocity` command/state interface，因此在该页面为这些主动关节选择：

- Command Interfaces：`position`、`velocity`
- State Interfaces：`position`、`velocity`

Setup Assistant 生成的 `Alicia_D_v5_6_gripper_50mm.ros2_control.xacro` 使用 mock hardware，主要服务于 `moveit_config` 自带的 demo。项目日常启动使用 `my_robot_bringup/config/alicia_d.urdf.xacro`，由 `use_fake_hardware` 决定使用 mock hardware 还是厂家驱动。

## 7. Configuration Files 和生成包

- 在 Configuration Files 页面填写作者信息。
- 输出目录选择：`src/moveit_config`。
- 如果是在已有包上重新生成，只选择确实需要更新的文件，并在生成后检查 `git diff`。
- 确认 `config/kinematics.yaml` 的顶层键是 `arm`，并使用 KDL：

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

## 8. 生成后恢复本项目参数

Setup Assistant 可能把以下文件恢复成默认值，需要重新检查。

`config/joint_limits.yaml`：

- `default_velocity_scaling_factor: 0.05`
- Joint1 至 Joint5 的 `max_velocity: 5.0 rad/s`
- Joint6 的 `max_velocity: 3.0 rad/s`
- Gripper 的 `max_velocity: 0.05 m/s`（当前 25 mm 单侧行程的保守规划上限）
- 只保留主动关节，不为 `right_finger` 配置独立限制。

`config/ros2_controllers.yaml`：

- `controller_manager.ros__parameters.update_rate: 200`
- 控制器名称必须是 `arm_controller`、`gripper_controller` 和 `joint_state_broadcaster`。
- `arm_controller` 控制 Joint1 至 Joint6；`gripper_controller` 只控制 Gripper。
- 两个轨迹控制器的 command/state interfaces 均为 `position`、`velocity`。
- `arm_controller` 使用 `state_publish_rate: 200.0`、`action_monitor_rate: 20.0`、`open_loop_control: false` 和 `allow_integration_in_goal_trajectories: true`。
- `constraints.stopped_velocity_tolerance: 0.01`，`constraints.goal_time: 0.0`。

`config/moveit_controllers.yaml`：

- MoveIt 控制器名称必须同样是 `arm_controller` 和 `gripper_controller`。
- `action_ns` 必须是 `follow_joint_trajectory`。
- arm 关节为 Joint1 至 Joint6，gripper 关节只包含 Gripper。

## 9. 构建与验证

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-up-to my_robot_bringup
source install/setup.zsh
ros2 launch my_robot_bringup my_robot.launch.py use_fake_hardware:=true
```

启动后检查：

```bash
ros2 control list_controllers
ros2 param get /commander robot_description_kinematics.arm.kinematics_solver
ros2 run tf2_ros tf2_echo base_link gripper_center
```

预期结果：三个 controller 为 `active`，commander 使用 KDL，TF 能持续输出 `base_link -> gripper_center`。日常测试统一使用 `my_robot_bringup`，不需要再单独启动 `moveit_config/demo.launch.py` 或 `commander_node`。

## 新建自己的机器人启动脚本：`my_robot_bringup`
删除src、include,新建config、launch。
对于config，实际上就是在已有的urdf的基础上新增`ros2_control`字段，在该字段内定义hardware部分，厂家原始提供的driver作为一个plugin放入其中。
- config：
    - 将moveit_config中的`alicia_d.ros2_control.xacro`拿过来，并重新整理一下，主要是对于硬件部分添加是否开启真实硬件选项
        - `use_fake_hardware=true`: 开启仿真
        - `use_fake_hardware=false`: 开启真机
    - `alicia_d.urdf.xacro`: 引入机械臂原始urdf,并添加`ros2_control`字段
- launch
    - `my_robot.launch.py`,开启各节点：
        - `robot_state_publisher`: 机器人状态发布节点
        - `world_to_base`: 静态坐标转换节点。主要是在moveit助手中添加了`virtual_joint`
        - `controller_manager`: 启动控制管理器
        - `joint_state_broadcaster`: 开启关节状态广播器
        - `arm_controller`: 开启机械臂控制器
        - `gripper_controller`: 开启夹爪控制器
        - `move_group`: MoveIt move_group 规划节点
        - `rviz`: 开启可视化
- 构建激活并执行启动:
    - 仅仿真: `ros2 launch my_robot_bringup my_robot.launch.py`
    - 真机: `ros2 launch my_robot_bringup my_robot.launch.py use_fake_hardware:=false port:=/dev/ttyACM0`

## 速度参数的分层和统一入口

所有调速入口集中在本项目的 `my_robot.launch.py`，不修改厂家的描述包和驱动包。

- 控制速度参数：
    - `velocity_scaling_factor`：速度缩放比例，范围 `(0.0, 1.0]`。项目默认 `0.05`，表示使用各关节速度上限的 5%。
        - `velocity_scaling_factor` 由 `my_robot.launch.py` 传给 `commander_node`，Commander 同时应用到 `arm` 和 `gripper` 规划组。
    - `speed_deg_s` 只用于真机，单位为 `deg/s`。本项目的 bringup 将它作为 `default_speed_deg_s` 硬件参数传给未修改的厂家插件；厂家插件使用同一个速度控制六个机械臂关节。项目默认 `5.0`，略高于厂家协议约 `4.39 deg/s` 的下限。
    - 两者关系: 
        - `velocity_scaling_factor`是moveit规划的速度缩放比例，各关节规划速度上限为其 max_velocity 乘以该缩放比例。
        - `speed_deg_s`是真机最大速度。
        - 要求：**真机硬件速度上限应覆盖 MoveIt 规划轨迹中的最大关节速度，避免硬件速度限制低于规划速度**。
- 其他说明：
    - `moveit_config/config/joint_limits.yaml` 中的 `max_velocity` 是各关节绝对规划上限，不作为日常调速旋钮。旋转关节单位是 `rad/s`，直线移动的 Gripper 单位是 `m/s`。
    - 当前所有关节都设置为 `has_acceleration_limits: false`，没有可信的绝对加速度上限。因此本项目暂不暴露 `acceleration_scaling_factor`，以后取得厂家加速度数据并配置 `max_acceleration` 后再恢复。
    - 夹爪的 MoveIt 规划上限为 `0.05 m/s`，默认 5% 缩放后是 `0.0025 m/s`，25 mm 行程的规划时间约为 10 秒。
    - 真机夹爪协议仍由未修改的厂家驱动使用固定原始速度 `5500`。厂家插件没有暴露夹爪速度参数，仓库也没有提供该值到 `m/s` 的换算关系。因此本项目目前只能控制夹爪的 MoveIt 规划时间，不能承诺真机夹爪协议层速度可调；在厂家提供接口或协议说明前应单独谨慎测试夹爪。

### 启动方式
仿真： 只需设置 MoveIt 规划速度。
```bash
ros2 launch my_robot_bringup my_robot.launch.py \
  use_fake_hardware:=true \
  velocity_scaling_factor:=0.05
```
真机： 同时设置 MoveIt 规划速度和厂家插件的机械臂速度。
```bash
ros2 launch my_robot_bringup my_robot.launch.py \
  use_fake_hardware:=false \
  port:=/dev/ttyACM0 \
  velocity_scaling_factor:=0.05 \
  speed_deg_s:=5.0
```
总结： velocity_scaling_factor 控制 MoveIt 轨迹的规划速度；speed_deg_s 控制真机机械臂的硬件速度。仿真只需设置前者，真机需要同时设置两者。

`velocity_scaling_factor` 决定 MoveIt 轨迹的时间和速度；`speed_deg_s` 是真机厂家协议的速度。仿真只使用前者，真机两者都要保守设置。

## 自定义控制后端：commander
基于**多线程执行器、并行规划组、Action、自定义消息接口**实现控制后端
- arm规划：
    - action name: `goToPoseTarget`：
        - 消息接口：
            - 请求
            - 结果
            - 反馈
    - joint_target控制，逆运动学达到目标角度
- gripper规划：
    - gripper_open: 夹爪打开
    - gripper_close: 夹爪关闭
    - grriper_move: 夹爪移动

## 前端测试:test_action
如何获取目标位姿？
- 启动机器人描述脚本：`ros2 launch alicia_d_descriptions display.launch.py`
- 手动调整joint,然后在另一个终端：`ros2 run tf2_ros tf2_echo base_link gripper_center`


## 最终启动方式
- 启动机器人、MoveIt 和 commander：
    - `source install/setup.zsh` && `ros2 launch my_robot_bringup my_robot.launch.py use_fake_hardware:=true`
- `commander_node` 已由上述 launch 启动，不要再单独运行，否则会出现重名节点和重复的 Action Server。
# TODO:List
- 如何引入深度相机
- 如何进行相机标定
- 最后才考虑真机
- 当前commander实现的是任务级控制，给定末端位姿，机械臂规划执行。之后考虑加入基于MoveIt Servo的连续伺服控制。
