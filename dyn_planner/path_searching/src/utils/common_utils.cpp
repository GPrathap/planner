#include "common_utils.h"

namespace kamaz {
namespace hagen{


  //https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
  void CommonUtils::get_roration_matrix(Eigen::Vector3d a
      , Eigen::Vector3d b, Eigen::Matrix3d& r){
        a = a/a.norm();
        double b_norm = b.norm();
        b = b/b_norm;
        Eigen::Vector3d v = a.cross(b);
        double s = v.norm();
        double c = a.dot(b);
        Eigen::Matrix3d vx;
        vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        r = Eigen::Matrix3d::Identity(3,3);
        if(s != 0 ){
            r = r + vx + vx*vx*((1-c)/std::pow(s, 2));
        }
  }

  void CommonUtils::get_point_on_the_trajectory(Eigen::Vector3d way_point, Eigen::Vector3d start_point,  Eigen::Vector3d& path_position){
        path_position << way_point[0]*voxel_side_length + init_min_point[0]
        , way_point[1]*voxel_side_length + init_min_point[1]
        , way_point[2]*voxel_side_length + init_min_point[2], 1.0;
        path_position = path_position + start_point;
  }

  double CommonUtils::get_cost_of_path(std::vector<Eigen::Vector3d> path1){
    int size_of_path = path1.size();
    Eigen::Vector3d path1_dis(size_of_path);
    for(int i=0; i< path1.size(); i++){
        path1_dis[i] = path1[i].head(3).norm();
    }
    Eigen::MatrixXd smoothed_map  = Eigen::MatrixXd::Zero(size_of_path, size_of_path);
    for(int i=0; i<size_of_path-1; i++){
      smoothed_map(i,i) = 2;
      smoothed_map(i,i+1) = smoothed_map(i+1,i) = -1;
    }
    smoothed_map(size_of_path-1, size_of_path-1) = 2;
    return path1_dis.transpose()*smoothed_map*path1_dis;
  }

  // https://geus.wordpress.com/2011/09/15/how-to-represent-a-3d-normal-function-with-ros-rviz/
  // https://ma.ttpitk.in/blog/?p=368&cpage=1
  void CommonUtils::generate_samples_from_ellipsoid(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat, 
            Eigen::Vector3d cent, Eigen::MatrixXd& container){

        int ndims = container.cols();
        int npts = container.rows();
        std::cout<< "====11" << std::endl;
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
        std::cout<< "====11" << std::endl;
        eigensolver.compute(covmat);
        std::cout<< "====11" << std::endl;
        Eigen::Vector3d eigen_values = eigensolver.eigenvalues().real();
        std::cout<< "====11" << std::endl;
        Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
        std::cout<< "====11" << std::endl;
        std::vector<std::tuple<double, Eigen::Vector3d>> eigen_vectors_and_values; 
        std::cout<< "====11" << std::endl;
        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<double, Eigen::Vector3d> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::cout<< "====22" << std::endl;
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(), 
            [&](const std::tuple<double, Eigen::Vector3d>& a, const std::tuple<double, Eigen::Vector3d>& b) -> bool{ 
                return std::get<0>(a) <= std::get<0>(b); 
        });
        std::cout<< "====22" << std::endl;
        int index = 0;
        for(auto const vect : eigen_vectors_and_values){
            eigen_values(index) = std::get<0>(vect);
            eigen_vectors.row(index) = std::get<1>(vect);
            index++;
        }
        std::cout<< "====33" << std::endl;

        Eigen::MatrixXd eigen_values_as_matrix = eigen_values.asDiagonal();

        std::random_device rd{};
        std::mt19937 gen{rd()};  
        std::uniform_real_distribution<double> dis(0, 1);
        std::normal_distribution<double> normal_dis{0.0f, 1.0f};
        std::cout<< "====44" << std::endl;
        Eigen::MatrixXd pt = Eigen::MatrixXd::Zero(npts, ndims).unaryExpr([&](double dummy){return (double)normal_dis(gen);});
        std::cout<< "====22" << std::endl;
        Eigen::VectorXd rs = Eigen::VectorXf::Zero(npts).unaryExpr([&](double dummy){return dis(gen);});
        std::cout<< "====22" << std::endl;
        Eigen::VectorXd fac = pt.array().pow(2).rowwise().sum();
        std::cout<< "====22" << std::endl;
        Eigen::VectorXd fac_sqrt = fac.array().sqrt();
        std::cout<< "====22" << std::endl;
        Eigen::VectorXd rs_pow = rs.array().pow(1.0/ndims);
        std::cout<< "====22" << std::endl;
        fac = rs_pow.array()/fac_sqrt.array();
        std::cout<< "====22" << std::endl;
        Eigen::VectorXd d = eigen_values_as_matrix.diagonal().array().sqrt();
        // std::cout << "============================================>>>>>>" << npts << std::endl;
        for(auto i(0); i<npts; i++){
            container.row(i) = fac(i)*pt.row(i).array();
            Eigen::MatrixXd  fff = (container.row(i).array()*d.transpose().array());
            Eigen::VectorXd bn = rotation_mat*fff.transpose();
            container.row(i) = bn.array() + cent.head(3).array();
        }
        // std::cout << "points: " << container << std::endl;
    }



    // dji_sdk::Gimbal CommonUtils::get_gimbal_msg(int mode, double roll, double pitch
    //         , double yaw){
    //     dji_sdk::Gimbal gimbal_angle;
    //     gimbal_angle.header.stamp = ros::Time::now();
    //     gimbal_angle.header.frame_id = world_frame_id;
    //     gimbal_angle.mode     = mode;
    //     gimbal_angle.roll     = roll;
    //     gimbal_angle.pitch    = pitch;
    //     gimbal_angle.yaw      = yaw;
    //     return gimbal_angle;
       
    // // }
    // // visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::Vector3d _point_on_path,
    //     Eigen::MatrixXd covmat, int id_, std::string name_space){ 
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = world_frame_id;
    //     marker.header.stamp = ros::Time();
    //     marker.ns = name_space;
    //     marker.id = id_;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = _point_on_path[0];
    //     marker.pose.position.y = _point_on_path[1];
    //     marker.pose.position.z = _point_on_path[2];
    //     marker.pose.orientation.x = 0.1;
    //     marker.pose.orientation.y = 0.1;
    //     marker.pose.orientation.z = 0.1;
    //     marker.pose.orientation.w = 0.2;
    //     marker.scale.x = covmat(0,0)*voxel_side_length;
    //     marker.scale.y = covmat(1,1)*voxel_side_length;
    //     marker.scale.z = covmat(2,2)*voxel_side_length;
    //     marker.color.a = 0.2;
    //     marker.color.r = 0.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.8;
    //     marker.lifetime = ros::Duration(); 
    //     return marker;
    // }

    // visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::Vector3d _point_on_path, ColorRGBA color_of_qupter, int id_, std::string name_space){ 
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = world_frame_id;
    //     marker.type = visualization_msgs::Marker::CUBE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.scale.x = 0.5;
    //     marker.scale.y = 0.5;
    //     marker.scale.z = 0.5;
    //     marker.pose.position.x = _point_on_path[0];
    //     marker.pose.position.y = _point_on_path[1];
    //     marker.pose.position.z = _point_on_path[2];
    //     marker.color.r = color_of_qupter.r;
    //     marker.color.g = color_of_qupter.g;
    //     marker.color.b = color_of_qupter.b;
    //     marker.color.a = 1.0;
    //     marker.ns = name_space;
    //     marker.id = id_;
    //     marker.lifetime = ros::Duration();
    //     return marker;
    // }

    // visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::Vector3d _point_on_path,
    //     Eigen::MatrixXd covmat, Eigen::Quaternion<double> q, int id_, std::string name_space){ 
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = world_frame_id;
    //     marker.header.stamp = ros::Time();
    //     marker.ns = name_space;
    //     marker.id = id_;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = _point_on_path[0];
    //     marker.pose.position.y = _point_on_path[1];
    //     marker.pose.position.z = _point_on_path[2];
    //     marker.pose.orientation.x = q.x();
    //     marker.pose.orientation.y = q.y();
    //     marker.pose.orientation.z = q.z();
    //     marker.pose.orientation.w = q.w();
    //     marker.scale.x = covmat(0,0)*voxel_side_length;
    //     marker.scale.y = covmat(1,1)*voxel_side_length;
    //     marker.scale.z = covmat(2,2)*voxel_side_length;
    //     marker.color.a = 0.2;
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;
    //     marker.lifetime = ros::Duration(); 
    //     return marker;
    // }

    // geometry_msgs::PoseStamped CommonUtils::constructPoseStamped(Eigen::Vector3d path_position){
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.stamp = ros::Time::now();
    //     pose.header.frame_id = world_frame_id;
    //     pose.pose.position.x = path_position[0];
    //     pose.pose.position.y = path_position[1];
    //     pose.pose.position.z = path_position[2];
    //     return pose;
    // }

    // void CommonUtils::printStampedTf(tf::StampedTransform sTf){
    //     tf::Transform tf;
    //     BOOST_LOG_TRIVIAL(info) << "frame_id: "<<sTf.frame_id_;
    //     BOOST_LOG_TRIVIAL(info) << "child_frame_id: "<<sTf.child_frame_id_; 
    //     tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf  
    //     printTf(tf);       
    // }

    // void CommonUtils::printTf(tf::Transform tf) {
    //     tf::Vector3 tfVec;
    //     tf::Matrix3x3 tfR;
    //     Eigen::Matrix3d e;
        
    //     tf::Quaternion quat;
    //     tfVec = tf.getOrigin();
    //     BOOST_LOG_TRIVIAL(info) << "Vector from reference frame to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();
    //     tfR = tf.getBasis();
    //     BOOST_LOG_TRIVIAL(info) << "Orientation of child frame w/rt reference frame: ";
    //     tfVec = tfR.getRow(0);
    //     BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();
    //     tfVec = tfR.getRow(1);
    //     BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();    
    //     tfVec = tfR.getRow(2);
    //     BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ(); 
    //     quat = tf.getRotation();
    //     BOOST_LOG_TRIVIAL(info) << "quaternion: " <<quat.x()<<", "<<quat.y()<<", "
    //     <<quat.z()<<", "<<quat.w();
    //     tf::matrixTFToEigen(tfR, e);
    //     std::cout << e << std::endl;   
    // }

    // tf::Transform CommonUtils::get_tf_from_stamped_tf(tf::StampedTransform sTf) {
    //     tf::Transform tf(sTf.getBasis(), sTf.getOrigin());
    //     return tf;
    // }

    // void CommonUtils::PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //     fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
    //     fprintf(stderr, "received msg   %d\n", msg->header.seq);
    //     fprintf(stderr, "height:        %d\n", msg->height);
    //     fprintf(stderr, "width:         %d\n", msg->width);
    //     fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
    //     fprintf(stderr, "fields of each point:\n");
    //     for (auto const& pointField : msg->fields) {
    //         fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
    //         fprintf(stderr, "\toffset:   %d\n", pointField.offset);
    //         fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
    //         fprintf(stderr, "\tcount:    %d\n", pointField.count);
    //         fprintf(stderr, "\n");
    //     }
    //     fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
    //     fprintf(stderr, "point step:    %d\n", msg->point_step);
    //     fprintf(stderr, "row step:      %d\n", msg->row_step);
    //     fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
    //     fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
    //     fprintf(stderr, "=========================================\n");
    // }

}
}