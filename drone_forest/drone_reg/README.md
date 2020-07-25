# drone_reg

## drone_reg.py
Позиционный DP регулятор для квадрокоптера PX4. <br>
На выходе публикует уставку по скорости.

#### Subscribed Topics:

**if** ***NOT use_planner_flag:***<br>
goal_pose ([drone_msgs:Goal](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/Goal.msg)): Целевая позиция дрона<br> 
mavros/local_position/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html)) Текущая позиция дрона <br>
**if** ***use_planner_flag:***<br>
&emsp;goal_pose_to_reg ([drone_msgs:Goal](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/Goal.msg)): Целевая позиция дрона при уставке от планировщика<br> 
**if** ***use_geo_mode:***<br>
&emsp;geo/goal_pose ([drone_msgs:Goal](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/Goal.msg)):  Целевая позиция дрона от GPS<br/> 
&emsp;geo/local_pose ([geometry_msgs/PoseStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html)): Позиция дрона от GPS<br>

local_position/velocity ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html)): Скорость дрона м/с^2 в NED<br>
state ([mavros/State](http://docs.ros.org/hydro/api/mavros/html/msg/State.html)): Состояние FCU PX4<br>
extended_state ([mavros/ExtendedState](http://docs.ros.org/api/mavros_msgs/html/msg/ExtendedState.html)): Расширенное состояник FCU PX4 <br>

#### Publisher Topics:
mavros_root/setpoint_velocity/cmd_vel ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html)):  Уставка по скорости <br>
marker_reg_point ([visualization_msgs/Marker](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html)): Маркер целевой точки

#### Parameters:

~use_planner_flag (bool, default: false)<br/>
&emsp;&emsp;*Флаг для переназначения топика целевой точки, при использовании позиционного планировщика.<br/>*
~use_geo_mode (bool, default: false)<br/>
&emsp;&emsp;*Флаг для переназначения топика целевой точки, при использовании GPS навигации..<br/>*

**Параметры dynamic reconfigure :**
   
angular_p: (float, default: 2.5) P коэффициент регулятора по курсу
angular_d: (float, default: 0.1) D коэффициент регулятора по курсу
max_hor_vel: (float, default: 0.5) Максимальная горизонтальная скорость 
hor_kp: (float, default: 1.5) P коэффициент регулятора горизонтальной скороти
hor_kd: (float, default: 0.3) D коэффициент регулятора горизонтальной скороти
max_ver_vel: (float, default: 1.0) Максимальная вертикальная скорость
ver_kp: (float, default: 1.5) P коэффициент регулятора вертикальной скороти
ver_kd: (float, default: 1.0) D коэффициент регулятора вертикальной скороти
