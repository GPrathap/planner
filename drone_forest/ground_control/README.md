# ground_control

Настроенный нпу под RVIZ для управления дроном.

Запуск для SLAM режима:

    roslaunch ground_control slam_gc.launch
    

## swarm_visualization.py
Сервер для отображения дронов в RVIZ в групповом режиме полёта.

**Subsctibe:**<br>
drone/list - список дронов

**Publisher:**<br>
drone/list/markers - Marker Array в RVIZ

