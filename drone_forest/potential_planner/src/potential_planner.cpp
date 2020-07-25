#include <iostream>
#include <fstream>

#include <ros/package.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


#include <drone_msgs/DronePose.h>
#include <drone_msgs/Goal.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolResponse.h>

#include <dynamic_reconfigure/server.h>
#include "potential_planner/PotPlannerConfig.h"

#include <yaml-cpp/yaml.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

/*
 * Трехмерный вектор
 */
struct Vector3
{
  double x, y, z;

  Vector3() : x(0.0), y(0.0), z(0.0) {}
  Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

  const Vector3 operator*(const double& rv) const { return Vector3(x*rv, y*rv, z*rv); }
  const Vector3 operator/(const double& rv) const { return Vector3(x/rv, y/rv, z/rv); }
};

/*
 * Функция перевоа градусов в радианы
 */
static double deg2rad(double deg)
{
  return deg * M_PI/180.0;
}

/*
 * Функция перевоа радиан в градусы
 */
static double rad2deg(double rad)
{
  return rad * 180.0/M_PI;
}

/*
 * Функция вычисления длины вектора
 */
static double vecLen(Vector3 vec)
{
  double len = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
  return len;
}

/*
 * Функция нормализации вектора
 */
static Vector3 vecNorm(Vector3 vec)
{
  return vec / vecLen(vec);
}

/*
 * Поворот вектора на угол
 */
static Vector3 vecRot(Vector3 vec, double ang_deg)
{
  double ang_rad = deg2rad(ang_deg);
  Vector3 res;
  res.x = vec.x*cos(-ang_rad) - vec.y*sin(-ang_rad);
  res.y = vec.x*sin(-ang_rad) + vec.y*cos(-ang_rad);
  res.z = vec.z;
  return res;
}

/*
 * Функция проверки существования файла
 */
bool fileExists(const std::string& filename)
{
  std::ifstream ifile(filename);
  return static_cast<bool>(ifile);
}

// --- Параметры планировщика

double r_field = 1.5;   // радиус действия полей
double c_rep = 0.4;     // коэф ф-ии отталкивания
double k_vel = 0.5;     // коэф по скорости
double angle_vec = 0.0; // угол поворота вектора отталкивания

float max_dist_lidar = 8.0;
bool input_point2 = false;
bool init_server = false;
bool active_flag = true;

//sensor_msgs::PointCloud cloud;
pcl::PointCloud<pcl::PointXYZ> cloud;
drone_msgs::Goal target_pose;
Vector3 current_pose;
double current_course = 0.0;

double mod_vel;
double dist_to_goal;

//tf::TransformListener listener;




//yaml
YAML::Node pot_planner_yaml;
std::string yaml_path = "/home/op/Desktop/pot_planner_params.yaml";


/*
 *  Функция считываения параметров регулятора
 */
void configCb(potential_planner::PotPlannerConfig &config, uint32_t level)
{
  //считываем параметры
  r_field = config.r_field;
  c_rep = config.c_rep;
  k_vel = config.k_vel;
  angle_vec = config.angle_vec;

  //сохраняем в yaml
  if (init_server)
  {
    pot_planner_yaml["r_field"] = r_field;
    pot_planner_yaml["c_rep"] = c_rep;
    pot_planner_yaml["k_vel"] = k_vel;
    pot_planner_yaml["angle_vec"] = angle_vec;

    std::ofstream fout(yaml_path);
    fout << pot_planner_yaml;

  }
  init_server = true;
}

/*
 * Получение параметров из ямла
 */
potential_planner::PotPlannerConfig getYamlCfg()
{
  //загружаем файл
  YAML::Node y_cfg = YAML::LoadFile(yaml_path);
  //объявляем конфиг
  potential_planner::PotPlannerConfig cfg;
  //заносим значения из ЯМЛА в переменные
  r_field = y_cfg["r_field"].as<double>();
  c_rep = y_cfg["c_rep"].as<double>();
  k_vel = y_cfg["k_vel"].as<double>();
  angle_vec = y_cfg["angle_vec"].as<double>();
  //заносим значения в конфиг
  cfg.r_field = r_field;
  cfg.c_rep = c_rep;
  cfg.k_vel = k_vel;
  cfg.angle_vec = angle_vec;
  //уазуращуаем уасся!
  return cfg;
}


/*
 * Функция получения облака точек из лазерскана
 */
void lidarCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Convert laserscan to pointcloud2
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 sensor_tmp_cloud;
  projector.projectLaser(*msg, sensor_tmp_cloud);


  // Convert pointcloud2 to pcl pointcloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(sensor_tmp_cloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_tmp_cloud);

  // Next code from here: http://pointclouds.org/documentation/tutorials/matrix_transform.php
  // Using METHOD #1 with transform matrix
  //            |-------> This column is the translation
  //     | 1 0 0 x |  \
  //     | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
  //     | 0 0 1 z |  /
  //     | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  float theta = -current_course; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);

  // Executing the transformation
  pcl::transformPointCloud (*pcl_tmp_cloud, cloud, transform_1);
}

/*
 * Функция получения облака точек
 */
void octomapCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //конвертим PC2 to PC
  // sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

  // from here: https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);
}

/*
 * Функция получения текущей позиции
 */
void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose.x = msg->pose.position.x;
  current_pose.y = msg->pose.position.y;
  current_pose.z = msg->pose.position.z;

  if (!input_point2)
  {
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_course = yaw;
  }
}

/*
 * Функция получения текущей траекторной скорости
 */
void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  mod_vel = sqrt(msg->twist.linear.x * msg->twist.linear.x +
                 msg->twist.linear.y * msg->twist.linear.y +
                 msg->twist.linear.z * msg->twist.linear.z);
}

/*
 * Функция получения координат ЦТ
 */
void goalCb(const drone_msgs::Goal::ConstPtr& msg)
{
  target_pose.pose = msg->pose;
  double dx = target_pose.pose.point.x - current_pose.x;
  double dy = target_pose.pose.point.y - current_pose.y;
  double dz = target_pose.pose.point.z - current_pose.z;
  dist_to_goal = sqrt(dx*dx + dy*dy + dz*dz);

  std::cout << "dtg = " << dist_to_goal << std::endl;
}

/*
 * Функция получения вектора
 */
Vector3 get_vector(Vector3 drone, geometry_msgs::Point32 obstacle)
{
  Vector3 vec;
  vec.x = drone.x - obstacle.x;
  vec.y = drone.y - obstacle.y;
  vec.z = drone.z - obstacle.z;
  return vec;
}

Vector3 get_vector(Vector3 drone, pcl::PointXYZ obstacle)
{
  Vector3 vec;
  vec.x = drone.x - obstacle.x;
  vec.y = drone.y - obstacle.y;
  vec.z = drone.z - obstacle.z;
  return vec;
}

/*
 * Функция вычисления отталкивающей силы
 */
double rep_force(double dist_to_obs)
{
  return (c_rep / dist_to_obs - c_rep / r_field);
}

/*
 * Сервис отключения планировщика
 */
bool set_active(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& response)
{
  active_flag = req.data;
  response.success = req.data;
  return true;
}

/*
 * Функция публикации булимовского сообщения
 */
void publish_bool_msg(ros::Publisher publisher, bool val)
{
  std_msgs::Bool msg;
  msg.data = val;

  publisher.publish(msg);
}

/*
 * Функция поиска ближайшей точки из облака точек
 */
Vector3 findNearestPoint(const pcl::PointCloud<pcl::PointXYZ>& cloud, double &cur_min)
{
  Vector3 cur_vec;
  for (auto point_obs : cloud.points)
  {
    // находим вектор между дроном и препятствием
    Vector3 vec_i = get_vector(current_pose, point_obs);

    // находим длину этого вектора
    double len_i = vecLen(vec_i);

    // находим минимальный
    if (len_i < cur_min)
    {
      cur_min = len_i;
      cur_vec = vec_i;
      std::cout << "[" << ros::Time::now() << "]: cur min = " << cur_min << std::endl;
    }
  }
  return cur_vec;
}

/*
 * ГЛАВНАЯ ФУНКЦИЯ
 */
int main(int argc, char** argv)
{
  std::cout << "Start pot_planner_node" << std::endl;

  // инициализация узла
  ros::init(argc, argv, "pot_planner_node");
  ros::NodeHandle n("~");

  // инициализация сервера динамической реконцигурации
  n.getParam("yaml_path", yaml_path);
  n.getParam("use_point2", input_point2);
  std::cout << "yaml_path: " << yaml_path << std::endl;
  dynamic_reconfigure::Server<potential_planner::PotPlannerConfig> cfg_server;
  dynamic_reconfigure::Server<potential_planner::PotPlannerConfig>::CallbackType f;
  f = boost::bind(&configCb, _1, _2);

  // проверяем существование файла настроек
  if (fileExists(yaml_path))
  {
    // апдейтим значения на сервере из ямла
    potential_planner::PotPlannerConfig yaml_cfg = getYamlCfg();
    cfg_server.updateConfig(yaml_cfg);
  }

  // получаем данные с сервера
  cfg_server.setCallback(f);

  //считываем параметры
  std::cout << "r_field: " << r_field << std::endl;
  std::cout << "c_rep: " << c_rep << std::endl;
  std::cout << "k_vel: " << k_vel << std::endl;
  std::cout << "angle_vec: " << angle_vec << std::endl;


  // подписываемся на топики
  ros::Subscriber sub_lidar;
  ros::Subscriber sub_octomap;
  if (input_point2)
  {
    sub_octomap = n.subscribe("/point2/near", 10, octomapCb);
  }
  else
  {
    sub_lidar = n.subscribe("/scan", 10, lidarCb);
  }
  ros::Subscriber sub_pose = n.subscribe("/mavros/local_position/pose", 10, poseCb);
  ros::Subscriber sub_goal = n.subscribe("/goal_pose", 10, goalCb);
  ros::Subscriber sub_vel = n.subscribe("/mavros/local_position/velocity", 10, velCb);

  // объявляем паблишеры
  ros::Publisher pub_vel = n.advertise<geometry_msgs::TwistStamped>("/field_vel", 10);
  ros::Publisher pub_status = n.advertise<std_msgs::Bool>("/drone/local_planner/status", 1);
  ros::Publisher pub_active = n.advertise<std_msgs::Bool>("/drone/local_planner/active", 1);

  // сервис для отключения планировщика
  ros::ServiceServer service = n.advertiseService("/drone/local_planner/set_active", set_active);

  // главный цикл
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    // публикуем активность
    publish_bool_msg(pub_active, active_flag);

    // Проверяем активность планировщика
    if (!active_flag)
    {
      publish_bool_msg(pub_status, false);
      std::cout << "[" << ros::Time::now() << "]: Potencial planner is not active" << std::endl;
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

    // находим ближайшую точку в pointcloud и расстояние до нее
    double cur_min = max_dist_lidar + 1.0;
    Vector3 cur_vec = findNearestPoint(cloud, cur_min);

    // если расстояние до препятствия меньше порогового
    Vector3 rep_vec;
    bool planning_status = false; // флаг срабатывания планировщика
    if (cur_min < r_field)
    {
      // нормализуем вектор
      Vector3 norm_vec = vecNorm(cur_vec);
      // умножаем на отталкивающую силу
      double force = (rep_force(cur_min) + k_vel*mod_vel);
      rep_vec = norm_vec * force;

      // планировщик стработал
      planning_status = true;
    }

    // поворачиваем вектор отталкивания на угол
    rep_vec = vecRot(rep_vec, angle_vec);

    // суём его в сообщение
    geometry_msgs::TwistStamped field_vel_msg;
    field_vel_msg.twist.linear.x = rep_vec.x;
    field_vel_msg.twist.linear.y = rep_vec.y;
    field_vel_msg.twist.linear.z = 0;

    // публикуем сообщение
    pub_vel.publish(field_vel_msg);

    // публикуем статус планировщика
    publish_bool_msg(pub_status, planning_status);

    //крутим цикл
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


