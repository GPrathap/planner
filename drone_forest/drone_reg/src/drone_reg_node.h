#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <drone_msgs/DronePose.h>
#include <drone_msgs/Goal.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <drone_reg/DroneRegConfig.h>
#include <std_msgs/Float32.h>

/*----Global params----*/

bool use_planner_flag = false;
bool use_geo_mode = false;
bool use_alt_sonar = false;
bool calc_vel_from_pos = false;

/*----Глобальные переменные----*/

// Коэффициенты регулятора
double_t hor_kp = 1.5;
double_t hor_kd = 0.3;
double_t max_hor_vel = 0.5;
double_t ver_kp = 1.5;
double_t ver_kd = 1.0;
double_t max_ver_vel = 1.0;
double_t angular_p = 2.5;
double_t angular_d = 0.1;
double_t prev_error = 0.0;

int ctr_type;                                       // по умолчанию используем управление по координатам
drone_msgs::DronePose drone_pose;                   // координаты дрона
drone_msgs::DronePose goal;                         // целевая позиция дрона
drone_msgs::DronePose prev_goal;
drone_msgs::DronePose prev_vec;
geometry_msgs::TwistStamped current_vel;            // Текущая скорость дрона
geometry_msgs::TwistStamped vel_field;
mavros_msgs::State drone_status;

std::string mavros_root = "/mavros";
std::string OFFBOARD = "OFFBOARD";

// topics
std::string goal_local_topic = "/goal_pose";             // напрямую
std::string goal_posestamped_local_topic = "/goal";             // напрямую

std::string goal_planner_topic = "/goal_pose_to_reg";    // для планировщика
std::string goal_global_topic = "/geo/goal_pose";         // через geolib

std::string local_pose_topic = "/mavros/local_position/pose";
std::string geo_pose_topic = "/geo/local_pose";
std::string alt_sonar_topic = "/drone/alt";

// yaml
YAML::Node yaml_node;
std::string yaml_path = "~/drone_reg_params.yaml";

// таймеры
double goal_timer = 0.0;
double pose_timer = 0.0;
double goal_lost_time = 1.0;
double pose_lost_time = 0.5;
double print_delay = 3.0;
double print_timer = 0.;
const int queue_size = 10;

bool init_server = false;

/*----Функции подписки на топики----*/

void cfg_callback(drone_reg::DroneRegConfig &config, uint32_t level);

void nav_pos_cb(const geometry_msgs::PoseStampedConstPtr &data);

void goal_cb(const drone_msgs::GoalConstPtr &data);

void goal_posestamped_cb(const geometry_msgs::PoseStamped &data);


void state_cb(mavros_msgs::StateConstPtr &data);

void extended_state_cb(mavros_msgs::ExtendedStateConstPtr &data);

void imu_cb(sensor_msgs::ImuConstPtr &data);

void on_shutdown_cb();

void vel_field_cb(const geometry_msgs::TwistStampedConstPtr &data);

void nav_vel_cb(const geometry_msgs::TwistStampedConstPtr &data);

void alt_sonar_cb(const std_msgs::Float32ConstPtr &data);

/*----Функции управления дроном----*/

std::vector<double_t> get_control(drone_msgs::DronePose data);

void arm();

void disarm();

void set_mode(std::string new_mode);

/*----Вспомогательные функции----*/

double_t angle_between(std::vector<double_t> vec1, std::vector<double_t> vec2);

std::vector<double_t> limit_vector(std::vector<double_t> r, double_t max_val);

std::vector<double_t> get_linear_vel_vec(std::vector<double_t> r, std::vector<double_t> vel);

double_t get_angular_vel(double_t ang, double_t vel, double_t k, double_t d);

visualization_msgs::Marker setup_marker(drone_msgs::DronePose point);

void set_server_value();

drone_msgs::DronePose lerp_point(std::vector<double_t> current_pos, drone_msgs::DronePose newGoal, double_t step);

/*----Python function replacements----*/

double_t norm_d(std::vector<double_t> r);

double_t degToRad(double_t rad);

/*-------------------------------------*/
