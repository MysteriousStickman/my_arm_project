自己机械臂的控制项目，厂家自带资料：
- 机器人描述URDF: `alicia_d_descriptions` 
    - 查看描述文件: `ros2 launch alicia_d_descriptions display.launch.py`
- 机器人驱动SDK: `alicia_d_driver`

# 制作moveit驱动包
运行命令: `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
各部分如下：
- Self-Collisions: 直接点Generate Collision Matrix
- Vitrual Joints: 虚拟关节
    - name: virtual_joint
    - Child Link: base_link
    - Parnet Frame: world
    - Type: fixed
- Palnning Groups:
    - arm
        - name: arm
        - Kinematic Solver(逆运动学求解器): kdl
        - 剩下全部默认，Add joint 把基座、所有的arm link都加进去
    - gripper
        - name: gripper
        - 直接Add joint,把所有关于gripper的关节加进去
- Passive Joints(被动关节): 跳过
- ros2_control URDF Modifications: 这一项根据厂家提供的驱动接口：`export_state_interfaces`、`export_command_interfaces`
    - 在`alicia_d_hardware_interface.cpp`的两个方法，command和state的interface都看到了如下内容：
        - `state_interfaces.emplace_back（..., HW_IF_POSITION)`
        - `state_interfaces.emplace_back（..., HW_IF_VELOCITY)`
## 修改moveit包
- `config/joint_limits.yaml`，修改最大速度:
    - 将12改成5.0
    - Joint6改成3.0
    - Gripper:1.0
    - 删除`right_finger`
- `config/ros2_controllers.yaml`
    - `update_rate`(控制频率): 200
    - `arm_controller`和`gripper_controller`下的
        - `command_interfaces`（命令接口）和`state_interfaces`（状态接口），空列表改为如下内容：
            - `position`
            - `velocity`
    - arm_controller下新增字段
        - `state_publish_rate(状态发布频率): 200.0`
        - `action_monitor_rate(动作监听频率): 20.0`   默认就是20.0   只不过官方显示写出来了
        - `open_loop_control: false`  false 接收到一条新轨迹时，JointTrajectoryController 使用 State Interface 中的实际关节状态，作为轨迹插值的起始
        状态   true表示接收到新轨迹时，控制器更倾向于使用上一条命令状态作为新轨迹起点
        - `allow_integration_in_goal_trajectories(Controller 在收到的轨迹缺少某类数据时，通过积分补全): true`
        - `constraints`中再添加如下内容:
            - `stopped_velocity_tolerance: 0.01`  轨迹到达终点时，如果关节实际速度小于0.01rad/s,就可以认为关节已经停止。 这个值本身就是默认值，所以不写也一样。
            - `goal_time: 0.0` 轨迹计划结束时间之后，还允许机械臂额外花多长时间达到最终目标。
            

- 运行`ros2 launch moveit_config demo.launch.py`

## 新建自己的机器人启动脚本：`my_robot_bringup`
删除src、include,新建config、launch。
对于config，实际上就是在已有的urdf的基础上新增`ros2_control`字段，在该字段内定义hardware部分，厂家原始提供的driver作为一个plugin放入其中。
- config：
    - 将moveit_config中的`alicia_d.ros2_control.xacro`拿过来，并重新整理一下，主要是对于硬件部分添加是否开启真实硬件选项
        - `use_fake_hardware=true`: 开启仿真
        - `use_fake_hardware=true`: 开启真机
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
    - 真机: `ros2 launch my_robot_bringup my_robot.launch.py use_fake_hardware:=false port:=/dev/ttyACM0 speed_deg_s:=0.1`

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


## 最终启动方式
- 启动机器人、MoveIt 和 commander：
    - `source install/setup.zsh` && `ros2 launch my_robot_bringup my_robot.launch.py use_fake_hardware:=true`
- `commander_node` 已由上述 launch 启动，不要再单独运行，否则会出现重名节点和重复的 Action Server。
# TODO:List
- 如何添加速度控制参数
- 如何定义环境碰撞
- 如何引入深度相机
- 如何进行相机标定
- 最后才考虑真机


- 新建功能包C++，  但是使用python编写前端调用，并执行pose   joint   gripper 动作调用
