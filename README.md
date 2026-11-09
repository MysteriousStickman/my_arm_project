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

# TODO:List
- 将当前项目的ros2_controllers.yaml   moveit_controllers.yaml  joint_limits.yaml 备份起来，重新处理预定义姿态pose_1  pose_2
- 学习ros2 action 编写基于C++ API的action  commander
- 考虑如何构建commander 哪些是订阅  哪些是服务  哪些是动作
- 如何定义环境碰撞
- 如何引入深度相机
- 如何进行相机标定
- 最后才考虑真机