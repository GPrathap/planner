#include "drone_reg_node.h"

using namespace std;

/*----Функции подписки на топики----*/

void cfg_callback(drone_reg::DroneRegConfig &config, uint32_t level) {
    // dynamic_reconfigurate callback

    hor_kp = config.hor_kp;                         //берем данные из .cfg файла
    hor_kd = config.hor_kd;
    max_hor_vel = config.max_hor_vel;

    ver_kp = config.ver_kp;
    ver_kd = config.ver_kd;
    max_ver_vel = config.max_ver_vel;

    angular_p = config.angular_p;
    angular_d = config.angular_d;

    if (init_server == true) {                      //сохраняем в .yaml файл
        yaml_node["angular_d"] = angular_d;
        yaml_node["angular_p"] = angular_p;
        yaml_node["hor_kd"] = hor_kd;
        yaml_node["hor_kp"] = hor_kp;
        yaml_node["max_hor_vel"] = max_hor_vel;
        yaml_node["max_ver_vel"] = max_ver_vel;
        yaml_node["ver_kd"] = ver_kd;
        yaml_node["ver_kp"] = ver_kp;

        std::ofstream fout(yaml_path);
        fout << yaml_node;
    }

    init_server = true;
}

////получение параметров из ямла
drone_reg::DroneRegConfig getYamlCfg()
{
    //загружаем файл
    YAML::Node y_cfg = YAML::LoadFile(yaml_path);
    //объявляем конфиг
    drone_reg::DroneRegConfig cfg;
    //заносим значения из ЯМЛА в переменные
    hor_kp = y_cfg["hor_kp"].as<double>();
    hor_kd = y_cfg["hor_kd"].as<double>();
    max_hor_vel = y_cfg["max_hor_vel"].as<double>();

    ver_kp = y_cfg["ver_kp"].as<double>();
    ver_kd = y_cfg["ver_kd"].as<double>();
    max_ver_vel = y_cfg["max_ver_vel"].as<double>();

    angular_p = y_cfg["angular_p"].as<double>();
    angular_d = y_cfg["angular_d"].as<double>();

    //заносим значения в конфиг
    cfg.hor_kp = hor_kp;
    cfg.hor_kd = hor_kd;
    cfg.max_hor_vel = max_hor_vel;

    cfg.ver_kp = ver_kp;
    cfg.ver_kd = ver_kd;
    cfg.max_ver_vel = max_ver_vel;

    cfg.angular_p = angular_p;
    cfg.angular_d = angular_d;
    //уазуращуаем уасся!
    return cfg;
}

//функция проверки существования файла
bool fileExists(const std::string& filename)
{
    std::ifstream ifile(filename);
    return (bool) ifile;
}

void calculate_velocity_from_pose()
{
  static double last_time;
  static drone_msgs::DronePose last_pose;

  double dt = ros::Time::now().toSec() - last_time;

  //current_vel.twist.linear.x = (drone_pose.point.x-last_pose.point.x) / dt;
  //current_vel.twist.linear.y = (drone_pose.point.y-last_pose.point.y) / dt;
  current_vel.twist.linear.z = (drone_pose.point.z-last_pose.point.z) / dt;

  last_time = ros::Time::now().toSec();
  last_pose = drone_pose;
}

void nav_pos_cb(const geometry_msgs::PoseStampedConstPtr &data) {
    // navigation-position callback
    tf::Quaternion quat(data->pose.orientation.x,
                        data->pose.orientation.y,
                        data->pose.orientation.z,
                        data->pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    drone_pose.course = yaw;                        //getYaw
    drone_pose.point.x = data->pose.position.x;
    drone_pose.point.y = data->pose.position.y;

    if(!use_alt_sonar)
      drone_pose.point.z = data->pose.position.z;

    pose_timer = 0.0;

    if(calc_vel_from_pos)
      calculate_velocity_from_pose();
}

void goal_cb(const drone_msgs::GoalConstPtr &data) {
    // goal_pose callback
    ctr_type = data->ctr_type;
    goal = data->pose;
    goal_timer = 0.0;
    goal_timer = 0.0;
}


void goal_posestamped_cb(const geometry_msgs::PoseStamped &data) {
// goal_pose callback
    goal.point = data.pose.position;

    // navigation-position callback
    tf::Quaternion quat(data.pose.orientation.x,
                        data.pose.orientation.y,
                        data.pose.orientation.z,
                        data.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goal.course = yaw;

    goal_timer = 0.0;
    goal_timer = 0.0;
}



void state_cb(mavros_msgs::StateConstPtr &data) {
    // state callback
    drone_status.armed = data->armed;
    drone_status.mode = data->mode;
}

void extended_state_cb(mavros_msgs::ExtendedStateConstPtr &data) {
    /***/
}

void imu_cb(sensor_msgs::ImuConstPtr &data) {
    /***/
}

void on_shutdown_cb() {

}

void vel_field_cb(const geometry_msgs::TwistStampedConstPtr &data) {
    // velocity field callback
    vel_field.twist = data->twist;
}

void nav_vel_cb(const geometry_msgs::TwistStampedConstPtr &data) {
    // navigation velocity callback
  current_vel.twist.linear.x = data->twist.linear.x;
  current_vel.twist.linear.y = data->twist.linear.y;
  if(!calc_vel_from_pos)
	  current_vel.twist.linear.z = data->twist.linear.z;
}

void alt_sonar_cb(const std_msgs::Float32ConstPtr &data)
{
  if(use_alt_sonar)
    drone_pose.point.z = data->data;
}

/*----Функции управления дроном----*/

std::vector<double_t> get_control(drone_msgs::DronePose data) {
    //
    std::vector<double_t> coords_vec = {data.point.x - drone_pose.point.x,
                                        data.point.y - drone_pose.point.y,
                                        data.point.z - drone_pose.point.z};

    double_t diff_ang = data.course - drone_pose.course;

    std::vector<double_t> current_acc_vel = {current_vel.twist.linear.x,
                                             current_vel.twist.linear.y,
                                             current_vel.twist.linear.z};

    std::vector<double_t> vel_ctr_vec = get_linear_vel_vec(coords_vec, current_acc_vel);

    double_t ang = get_angular_vel(diff_ang, current_vel.twist.angular.z, angular_p, angular_d);
    // std::vector<double_t> res_ctr_vec = limit_vector(vel_ctr_vec, max_hor_vel);
    vel_ctr_vec.push_back(ang);
    return vel_ctr_vec;
}

void arm() {
    if (drone_status.armed != true) {
        ROS_INFO("arming");
        /***/
    }
}

void disarm() {
    if (drone_status.armed = true) {
        ROS_INFO("disarming");
        /***/
    }
}

void set_mode(string new_mode) {
    if (drone_status.mode != new_mode) {
        /***/
    }
}

/*----Вспомогательные функции----*/

double_t angle_between(std::vector<double_t> vec1, std::vector<double_t> vec2) {
    // Возвращает угол между двумя двухмерными векторами

    double_t x = (vec2[1] - vec1[1]);
    double_t y = -(vec2[0] - vec1[0]);
    double_t res = atan2(x, y) + M_PI;

    if (res > M_PI)
        res -= 2 * M_PI;
    if (res < -M_PI)
        res += 2 * M_PI;

    return res;
}

std::vector<double_t> limit_vector(std::vector<double_t> r, double_t max_val) {
    // ограничивает управление (НЕ ИСПОЛЬЗУЕТСЯ)

    double_t l = norm_d(r);
    if (l > max_val) {
        r[0] = r[0] * max_val / l;
        r[1] = r[1] * max_val / l;
        r[2] = r[2] * max_val / l;
    }
    return r;
}

std::vector<double_t> get_linear_vel_vec(std::vector<double_t> r, std::vector<double_t> vel) {
    // возвращает вектор линейных скоростей

    double_t error = norm_d(r);
    std::vector<double_t> v = {0.0, 0.0, 0.0};
    v[0] = r[0] * hor_kp - vel[0] * hor_kd;
    v[1] = r[1] * hor_kp - vel[1] * hor_kd;
    v[2] = r[2] * ver_kp - vel[2] * ver_kd;
    for (int i = 0; i < (v.size() - 1); i++) {
        if (v[i] < -max_hor_vel)
            v[i] = -max_hor_vel;
        else if (v[i] > max_hor_vel)
            v[i] = max_hor_vel;
    }
    if (v[2] < -max_ver_vel)
        v[2] = -max_ver_vel;
    else if (v[2] > max_ver_vel)
        v[2] = max_ver_vel;

    prev_error = error;
    return v;
}

double_t get_angular_vel(double_t ang, double_t vel, double_t k, double_t d) {
    // возвращает угловую скорость

    if (ang == 0) {
        return 0;
    }

    if (ang > M_PI)
        ang -= 2 * M_PI;
    if (ang < -M_PI)
        ang += 2 * M_PI;
    return k * ang - vel * d;
}

visualization_msgs::Marker setup_marker(drone_msgs::DronePose point) {
    // возвращает маркер в rviz

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_test_reg";
    marker.id = 0;
    marker.action = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.x = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.type = visualization_msgs::Marker::Type::SPHERE;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;

    return marker;
}

void set_server_value() {
    /***/
}

drone_msgs::DronePose lerp_point(std::vector<double_t> current_pos, drone_msgs::DronePose newGoal, double_t step) {
    // интерполяция точки (НЕ ИСПОЛЬЗУЕТСЯ)

    std::vector<double_t> current_vec (3);
    current_vec[0] = newGoal.point.x - current_pos[0];
    current_vec[1] = newGoal.point.y - current_pos[1];
    current_vec[2] = newGoal.point.z - current_pos[2];

    prev_vec.point.x = prev_goal.point.x - current_pos[0];
    prev_vec.point.y = prev_goal.point.y - current_pos[1];
    prev_vec.point.z = prev_goal.point.z - current_pos[2];

    double_t dist = norm_d(current_vec);
    std::vector<double_t> prev_vec_temp = {prev_vec.point.x, prev_vec.point.y, prev_vec.point.z};
    double_t angle_between_goal = abs(angle_between(current_vec, prev_vec_temp));
    double_t dist_to_prev_point = norm_d(prev_vec_temp);

    if (dist < step) {
        prev_goal = newGoal;
        return newGoal;
    }
    if (((angle_between_goal > degToRad(30)) && (angle_between_goal < degToRad(180))) &&
            (dist_to_prev_point > (step / 2.0)) &&
            (dist > dist_to_prev_point))
        return prev_goal;
    std::vector<double_t> val(3);
    for (int i = 0; i < 3; i++)
        val[i] = current_vec[i] / dist * step;

    newGoal.point.x = val[0] + current_pos[0];
    newGoal.point.y = val[1] + current_pos[1];
    newGoal.point.z = val[2] + current_pos[2];

    prev_goal = newGoal;
    return newGoal;
}

/*----Python function replacements----*/

double_t norm_d(std::vector<double_t> r) {
    // норма вектора
    return sqrt((r[0] * r[0]) + (r[1] * r[1]) + (r[2] * r[2]));
}

double_t degToRad(double_t deg) {
    // перевод из градусов в радианы
    return deg * M_PI / 180.0;
}

/*----Главная функция----*/

int main(int argc, char** argv) {
    goal.point.z = 1.0;

    ros::init(argc, argv, "drone_reg_vel_node");
    ros::NodeHandle n("~");

    ros::Rate loop_rate(50);

    ros::Publisher vel_pub, pub_marker;
    ros::Subscriber goalSub, goalPsSub, navPosSub, navVelSub, stateSub, exStateSub, velFieldSub, altSonarSub;

    //инициализация сервера динамической реконцигурации
    n.getParam("yaml_path", yaml_path);
    n.getParam("use_planner", use_planner_flag);
    n.getParam("use_geo_mode", use_geo_mode);
    n.getParam("use_alt_sonar", use_alt_sonar);
    n.getParam("calc_vel_from_pos", calc_vel_from_pos);

    dynamic_reconfigure::Server<drone_reg::DroneRegConfig> server;
    dynamic_reconfigure::Server<drone_reg::DroneRegConfig>::CallbackType f;
    f = boost::bind(&cfg_callback, _1, _2);
    //проверяем существование файла настроек
    if (fileExists(yaml_path)) {
        //апдейтим значения на сервере из ямла
        drone_reg::DroneRegConfig yaml_cfg = getYamlCfg();
        server.updateConfig(yaml_cfg);
    }
    //получаем данные с сервера
    server.setCallback(f);


    //stateSub = n.subscribe(mavros_root + "/state", queue_size, state_cb);
    //exStateSub = n.subscribe(mavros_root + "/extended_state", queue_size, extended_state_cb);
    velFieldSub = n.subscribe("/field_vel", queue_size, vel_field_cb);
    navVelSub = n.subscribe(mavros_root + "/local_position/velocity_local", queue_size, nav_vel_cb);

    if (use_geo_mode == true)
        goalSub = n.subscribe(goal_global_topic, queue_size, goal_cb);
    else {
        if (use_planner_flag == true)
            goalSub = n.subscribe(goal_planner_topic, queue_size, goal_cb);
        else {
            goalSub = n.subscribe(goal_local_topic, queue_size, goal_cb);
            goalPsSub = n.subscribe(goal_posestamped_local_topic, queue_size, goal_posestamped_cb);
        }
    }

    if (use_geo_mode == true)
        navPosSub = n.subscribe(geo_pose_topic, queue_size, nav_pos_cb);
    else
        navPosSub = n.subscribe(local_pose_topic, queue_size, nav_pos_cb);

    altSonarSub = n.subscribe(alt_sonar_topic, queue_size, alt_sonar_cb);

    vel_pub = n.advertise<geometry_msgs::TwistStamped> (mavros_root + "/setpoint_velocity/cmd_vel", queue_size);
    pub_marker = n.advertise<visualization_msgs::Marker> ("/marker_reg_point", queue_size);



    geometry_msgs::TwistStamped ctr_msg;

    double old_time = ros::Time::now().toSec();
    double dt = 0.0;

    std::vector<double_t> control;

    while (ros::ok()) {
        dt = ros::Time::now().toSec() - old_time;
        goal_timer += dt;
        pose_timer += dt;
        print_timer += dt;
        old_time = ros::Time::now().toSec();

        pub_marker.publish(setup_marker(goal));

        control = get_control(goal);

        ctr_msg.twist.linear.x = control[0] + vel_field.twist.linear.x;
        ctr_msg.twist.linear.y = control[1] + vel_field.twist.linear.y;
        ctr_msg.twist.linear.z = control[2] + vel_field.twist.linear.z;
        ctr_msg.twist.angular.z = control[3];

        if (pose_timer < pose_lost_time)
            vel_pub.publish(ctr_msg);
        else {
            if (print_timer > print_delay) {
                //ROS_INFO("lost goal callback: %f %f %f %f", goal_timer, goal_timer, pose_timer, pose_timer);
                print_timer = 0;
            }
            }

        ros::spinOnce();
        loop_rate.sleep();
    }
    std_msgs::Float32MultiArray ctr_msg_sd;
    ctr_msg_sd.data = {0, 0, 0, 0};
    vel_pub.publish(ctr_msg_sd);
    ROS_INFO("shutdown");

    return 0;
}
