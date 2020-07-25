# потенциальные поля


### Subscribed Topics:

**if use_point2 == True**<br>
point2/near (PointCloud2)  облако точек относительно Map<br>
**if use_point2 == False**<br>
scan (LaserScan)  Данные с лидара<br>


goal_pose ([drone_msgs:Goal](/drone_msgs/msg/Goal.msg)): Целевая позиция дрона<br> 
mavros/local_position/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html)) Текущая позиция дрона <br>
local_position/velocity ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html)): Скорость дрона м/с^2 в NED<br>

#### Publisher Topics:
field_vel ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html)):  Вектор отталкивания <br>
drone/local_planner/potential/active (bool) Состояния, включен / выключен планировщик <br>
drone/local_planner/potential/status (bool) Состояния, рабает ли отталкивание <br>


#### Service Topics:
drone/local_planner/potential/set_active (bool) вкл/выкл планировщика <br>



#### Parameters:
~yaml_path (string, default: "$(find potencial_planner)/cfg/params.yaml")<br/>
&emsp;&emsp;*Путь до yaml файла конфигурации полей.<br/>*
~use_point2 (bool, default: false)<br/>
&emsp;&emsp;*Если true подписываемся на облако точек, иначе на лидар (laserscan).<br>*



**Параметры dynamic reconfigure :**
   
r_field (float, default: 1.5)       радиус действия полей
c_rep (float, default: 0.4)         коэф ф-ии отталкивания
k_vel (float, default: 0.5)         коэф по скорости
angle_vec (float, default: 0.0)     угол поворота вектора отталкивания (не исп)