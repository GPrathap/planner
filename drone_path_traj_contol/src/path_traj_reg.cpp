#include "path_traj_reg.h"

using namespace std;

/*----Функции подписки на топики----*/

void cfg_callback(drone_path_traj_contol::DroneRegConfig &config, uint32_t level) {
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
drone_path_traj_contol::DroneRegConfig getYamlCfg()
{
    //загружаем файл
    YAML::Node y_cfg = YAML::LoadFile(yaml_path);
    //объявляем конфиг
    drone_path_traj_contol::DroneRegConfig cfg;
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
  static Pose last_pose;

  double dt = ros::Time::now().toSec() - last_time;

  //current_vel.twist.linear.x = (drone_pose.point.x-last_pose.point.x) / dt;
  //current_vel.twist.linear.y = (drone_pose.point.y-last_pose.point.y) / dt;
  current_vel.twist.linear.z = (drone_pose.position.z-last_pose.position.z) / dt;

  last_time = ros::Time::now().toSec();
  last_pose = drone_pose;
}


double getYawFromQuat(const Quaternion &data)
{
    tf::Quaternion quat(data.x,
                        data.y,
                        data.z,
                        data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void nav_pos_cb(const geometry_msgs::PoseStampedConstPtr &data) {
    // navigation-position callback

    drone_pose = data->pose;
    pose_timer = 0.0;
}

void goal_cb(const quadrotor_msgs::PositionCommand &data) {
    // goal_pose callback
    goal = data;
    goal_timer = 0.0;
    is_goal_is_set = true;
    // goal_yaw = data.yaw;
}

void stopping_callback(std_msgs::Empty msg) {
  /* stop moving */
//   std::cout<< "===================================stopping...." << std::endl;
  const double time_out = 0.25;
  is_regulating = false;
}

void starting_callback(std_msgs::Empty msg) {
  /* stop moving */
//   std::cout<< "===================================wait for goal...." << std::endl;
  const double time_out = 0.25;
  is_regulating = true;
}

void goal_ps_cb(const geometry_msgs::PoseStamped &data) {
    // goal_pose callback
    tf::Quaternion quat(data.pose.orientation.x,
                        data.pose.orientation.y,
                        data.pose.orientation.z,
                        data.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goal_yaw = yaw;
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
	  current_vel.twist.linear.z = data->twist.linear.z;
}


/*----Функции управления дроном----*/
std::vector<double_t> get_control(quadrotor_msgs::PositionCommand data) {
    std::vector<double_t> coords_vec = {data.position.x - drone_pose.position.x,
                                        data.position.y - drone_pose.position.y,
                                        data.position.z - drone_pose.position.z};
    double current_yaw = getYawFromQuat(drone_pose.orientation);
    double_t diff_ang = data.yaw - current_yaw;
    std::vector<double_t> current_acc_vel = {current_vel.twist.linear.x,
                                             current_vel.twist.linear.y,
                                             current_vel.twist.linear.z};
    std::vector<double_t> vel_ctr_vec = get_linear_vel_vec(coords_vec, current_acc_vel);
    // std::cout<< "angular_p: " << angular_p << " angular_d:" << angular_d << std::endl;
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

visualization_msgs::Marker setup_marker(geometry_msgs::Point point) {
    // возвращает маркер в rvizset_server_value

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
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    return marker;
}

void set_server_value() {
    /***/
}

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
    ros::init(argc, argv, "drone_reg_vel_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(30);
    ros::Publisher vel_pub, pub_marker;
    ros::Subscriber goalSub, navPosSub, navVelSub, stateSub, exStateSub, velFieldSub, altSonarSub, goal_ps_Sub
        , stopping_sub, starting_sub;
    //инициализация сервера динамической реконцигурации
    if (n.getParam("yaml_path", yaml_path))
    {
        n.param<std::string>("yaml_path", yaml_path, "");
    }
    if (n.getParam("use_rotate", use_rotate))
    {
        n.param<bool>("use_rotate", use_rotate, use_rotate);
    }
    if (n.getParam("speed_rotate", speed_rotate))
    {
        n.param<double>("speed_rotate", speed_rotate, speed_rotate);
    }
    dynamic_reconfigure::Server<drone_path_traj_contol::DroneRegConfig> server;
    dynamic_reconfigure::Server<drone_path_traj_contol::DroneRegConfig>::CallbackType f;
    f = boost::bind(&cfg_callback, _1, _2);
    //проверяем существование файла настроек
    if (fileExists(yaml_path)) {
        //апдейтим значения на сервере из ямла
        drone_path_traj_contol::DroneRegConfig yaml_cfg = getYamlCfg();
        server.updateConfig(yaml_cfg);
    }
    //получаем данные с сервера
    server.setCallback(f);
    velFieldSub = n.subscribe("/field_vel", queue_size, vel_field_cb);
    navVelSub = n.subscribe(mavros_root + "/local_position/velocity_local", queue_size, nav_vel_cb);
    goalSub = n.subscribe("/planning/pos_cmd", queue_size, goal_cb);
    goal_ps_Sub = n.subscribe("/goal", queue_size, goal_ps_cb);
    navPosSub = n.subscribe(local_pose_topic, queue_size, nav_pos_cb);
    vel_pub = n.advertise<geometry_msgs::TwistStamped> (mavros_root + "/setpoint_velocity/cmd_vel", queue_size);
    pub_marker = n.advertise<visualization_msgs::Marker> ("/marker_reg_point", queue_size);
    stopping_sub = n.subscribe("/planning/wait_for_goal", 10, stopping_callback);
    starting_sub = n.subscribe("/planning/start_moving", 10, starting_callback);
    geometry_msgs::TwistStamped ctr_msg;
    double old_time = ros::Time::now().toSec();
    double dt = 0.0;
    std::vector<double_t> control;
    double angle = 0.;
    while (ros::ok()) {
        dt = ros::Time::now().toSec() - old_time;
        goal_timer += dt;
        pose_timer += dt;
        print_timer += dt;
        old_time = ros::Time::now().toSec();
        pub_marker.publish(setup_marker(goal.position));
        control = get_control(goal);
        ctr_msg.twist.linear.x = control[0] + vel_field.twist.linear.x + goal.velocity.x;
        ctr_msg.twist.linear.y = control[1] + vel_field.twist.linear.y + goal.velocity.y;
        ctr_msg.twist.linear.z = control[2] + vel_field.twist.linear.z + goal.velocity.z;
        ctr_msg.twist.angular.z = control[3];
        if (use_rotate)
        {
            angle +=  speed_rotate*dt*0.0174533;
            ctr_msg.twist.angular.z = angle;
        }
        // if(!is_regulating){
        //     ctr_msg.twist.linear.x = 0;
        //     ctr_msg.twist.linear.y = 0;
        //     ctr_msg.twist.linear.z = 0;
        //     ctr_msg.twist.angular.z = 0;
        // }
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
    geometry_msgs::TwistStamped ctr_msg_sd;
    vel_pub.publish(ctr_msg_sd);
    ROS_INFO("shutdown");
    return 0;
}
