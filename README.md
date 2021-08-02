# UR手臂 package來源參考https://github.com/fmauch/universal_robot.git ＆ https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
# 安裝教學參考https://blog.csdn.net/zxxxiazai/article/details/103568577

## 1.install moveIt ＆ trac_ik
  ```bash
  $ sudo apt-get install ros-melodic-moveit
  $ sudo apt-get install ros-melodic-trac-ik-kinematics-plugin
  ```
  
## 2. gazebo test
  ### 請參考補充1
  ```bash
  $ roslaunch ur_gazebo ur5.launch
  $ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
    rviz控制（二則一）
  $ roslaunch ur5_moveit_config moveit_rviz.launch config:=true
    python控制（二則一）
  $ rosrun ur_move_test ur_move_test_node.py
  ```
## 3. real robot control
    ######注意執行順序不能改變######
  ### 請參考補充1
  ```bash
  1.設定電腦ip 為192.168.0.100 （可以更改,如果更改電腦ip,手臂external_control.urp程序的ip也要做調整）
  2.開起手臂電源
  4.執行 roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:= 192.168.0.12(robot_ip)
  5.開起手臂程序
    示教器，運行程序 —> 文件 —> 加载程序 —> external_control.urp程序，打開—>運行
    可以看到终端显示：
	[ INFO]: Robot mode is now POWER_ON
	[ INFO]: Robot mode is now IDLE
	[ INFO]: Robot mode is now RUNNING
	[ INFO]: Robot requested program
	[ INFO]: Sent program to robot
	[ INFO]: Robot ready to receive control commands.
  6.roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
  7.roslaunch ur5_moveit_config moveit_rviz.launch config:=true 
  如果想用python控制手臂控制步驟7可以更換成python檔
    rosrun ur_move_test ur_move_test_node.py
  ```  


# 補充：
   ````bash
   1.真實手臂與gazebo間的切換
     控制真實手臂與gazebo必須要更改 /universal_robot/ur5_moveit_config/config/controllers.yaml 文件
     真實手臂必須在檔案內的  name: 後添加 scaled_pos_joint_traj_controller
     gazebo必須在檔案內的  name: 後添加 ""
     
     否則會報錯 [ERROR] : Action client not connected: /follow_joint_trajectory

   2.如何修改運動學演算法：
     原本官方是KDL演算法,此package為trak_ik演算法
     想換演算法則必須更改 /universal_robot/ur5_moveit_config/config/kinematics.yaml 文件
     kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin 取代成
     kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin

   3.限制關節運動角度：
     修改 /universal_robot/ur_description/urdf/ur5_joint_limited_robot.urdf.xacro文件
     shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
     shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
     elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
     wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
     wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
     wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
     預設是-pi to pi 可以去設定想要的角度
     如果想要讓gazebo初始狀態顯示為有限制過的角度 更改 ur5.launch文件 將limited的default設為true
   ```




