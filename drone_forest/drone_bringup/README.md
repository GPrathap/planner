# drone_bringup

Пакет содерижит основные файлы запуска для полёта в автономной режиме.<br>

Доступно **3** режима полёта:<br>
  1. полёт по GPS(RTK)
  2. полёт по RTLS
  3. полёт по Slam (по лидару) 

<details><summary>подробнее..</summary><p>

### 1. Полёт по GPS(RTK)

Запуск:

    roslaunch drone_bringup bringup_gps.launch

Включает:
  * необходимые настройки TF
  * запуск mavros и соединение Pixhawk / GCS
  * получение диагностической информации
  * систему безопасности
  * конвертацию GPS в локальные координаты  
  * децентрализованный сервер обмена навигацией
  * клиент для получения поправки с РТК
  * регулятор с интерактичным управлением в rviz
  
  
### 2. Полёт по RTLS

Команда:

    roslaunch drone_bringup bringup_rtls.launch

**Регулятор запускается отдельно**
    
Включает:
  * необходимые настройки TF
  * запуск mavros и соединение Pixhawk / GCS
  * получение диагностической информации
  * систему безопасности
  * навигацию от РТЛС
  * запуск RPLIDAR A2
  * удалённые управление с телефона через rosbridge

  
### 3. Полёт по SLAM

Команда:

    roslaunch drone_bringup bringup_slam.launch

**SLAM и регулятор запускаются отдельно**

Включает:
  * необходимые настройки TF
  * запуск mavros и соединение Pixhawk / GCS
  * получение диагностической информации
  * систему безопасности
  * запуск RPLIDAR A2
  * удалённые управление с телефона через rosbridge


### Запуск регулятора

Команда:

      roslaunch drone_bringup  drone_reg.launch

Включает:
    * интерактичное управление через RVIZ
    * позиционный регулятор
    * локальный планировщик (опционально, см. файл перед запуском)
  
### Запуск SLAM (Google Cartographer)

Команда:

    roslaunch drone_bringup cartograpther_slam.launch

Запускать после **bringup_slam.launch**
    
Включает:
  * необходимые настройки для навигации по RPLIDAR
  

</p></details>


### Содержание **Launch** файлов:
 * **bringup_gps.launch**: файл запуска дрона с навигацией по GPS
 * **bringup_rtls.launch**: файл запуска дрона с навигацией по РТЛС
 * **bringup_slam.launch**: файл запуска дрона с навигацией по SLAM
 * **cartograpther_slam.launch**: запуск SLAM google cartograpther
 * **drone_reg.launch**: запуск регулятора с интерактивным плагином RVIZ 
 * **fixed_joint.launch**: настройка дерева FT
 * **octomap_mapping.launch**: octomap сервер
 * **rplidar.launch**: запуск rplidar A2
 
 
### Содержание **Nodes**:
 
* **ground_lidar_seek_ros.py**: считывает высоту с лидара, фильтрует и публикует в топик
* **drone_geo_reference.py**: преобразует GPS в локальные координаты относительно origin точки
* **footprint_tf_node.py**: формирует проекцию base_link на плоскость (base_link --> base_stabilized --> base_footprint)
* **drone_diagnostics.py**: формирует диагностическую информацию о состоянии дрона
* **drone_safery_node.py**: система безопасности
* **init_state.py**: инициализация pixhawk, необходимо для получения IMU при навигации от SLAM
* **msg_to_tf2.py**: публикуем ft (map --> map_slam)
* **rtls_averaging.py**: получение навигации от РТЛС

### Файлы /configuration_files:
 * **fixed_joint.urdf**: конфигурация дерева TF дрона
 * **octomap.yaml**: параметры для octomap сервера
 * **rplidar_2d.lua**: параметры для SLAM google cartograpther (rplidar)

