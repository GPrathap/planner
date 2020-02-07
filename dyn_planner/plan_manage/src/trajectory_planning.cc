#include <plan_manage/trajectory_planning.h>

namespace kamaz {
namespace hagen {

    TrajectoryPlanning::TrajectoryPlanning(double speed){
        max_speed = speed;
        is_set = false;
    }

    bool TrajectoryPlanning::generate_ts(Eigen::MatrixXd path){
        int size_of_the_path = path.cols()-3;

        if(size_of_the_path < 1){
            return false;
        } 
        int vector_dim = path.rows();
        // way_points = Eigen::MatrixXd::Zero(size_of_the_path, vector_dim); 
        // way_points = path.transpose().block(1, 0, size_of_the_path-1, vector_dim)
        // std::cout<< "-======2 " << std::endl;
        way_points = path.block(0, 0, vector_dim, size_of_the_path).transpose();
        // std::cout<< "-======3 " << std::endl;
        // int row_index = 0;
        // for(auto way_point : path){
        //     way_points.row(row_index) = way_point;
        //     row_index++;
        // }
        Eigen::MatrixXd dis = (way_points.block(1, 0, size_of_the_path-1, vector_dim).array() 
                - way_points.block(0, 0, size_of_the_path-1, vector_dim).array()).pow(2).rowwise().sum().sqrt();
        double path_len = dis.sum();
        total_time = path_len/max_speed;
        Eigen::MatrixXd path_seg_length = dis.array().sqrt();
        for(int i=1; i<size_of_the_path-1; i++){
          path_seg_length(i, 0)= path_seg_length(i-1, 0) + path_seg_length(i, 0);
        }
        path_seg_length = path_seg_length.array()/path_seg_length(size_of_the_path-2, 0);
        
        time_segs.push_back(0.0);
        for(int i=1; i<size_of_the_path; i++){
          time_segs.push_back(path_seg_length(i-1, 0)*total_time);
        }
        is_set = true;
        return true;
    }

    void TrajectoryPlanning::save_status(std::vector<std::vector<Eigen::VectorXd>> status
    ,  std::string file_name){
       std::vector<double> quad_status; 
       for(auto sector: status){
            // std::cout<< status.size() << std::endl;
            for(auto val : sector){
                quad_status.push_back(val[0]);
                quad_status.push_back(val[1]);
                quad_status.push_back(val[2]);
            }
       }
    //    cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }

    void TrajectoryPlanning::save_trajectory(std::vector<Eigen::VectorXd> trajectory_of_drone
    ,  std::string file_name){
       std::vector<double> quad_status; 
       for(auto sector: trajectory_of_drone){
            // std::cout<< status.size() << std::endl;
            quad_status.push_back(sector[0]);
            quad_status.push_back(sector[1]);
            quad_status.push_back(sector[2]);
       }
    //    cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }


    void TrajectoryPlanning::get_desired_state(double time, std::vector<Eigen::VectorXd>& states){
        
        if(time >= total_time){
            std::cout<< "Out of total count..." << total_time << std::endl;
            Eigen::MatrixXd point  = way_points.block(way_points.rows()-1, 0, 1, 3);
            Eigen::VectorXd pos = Eigen::Map<Eigen::RowVectorXd>(point.data(), 3);
            states.push_back(pos);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(3);
            Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);
            states.push_back(vec);
            states.push_back(acc);
            return;
        }

        int k = closest(time).second;

        // std::cout<< "======||" << k << std::endl;

        Eigen::MatrixXd pose_coeff(1, 8);
        pose_coeff << std::pow(time, 7.0)
            ,std::pow(time, 6.0)
            ,std::pow(time, 5.0)
            ,std::pow(time, 4.0)
            ,std::pow(time, 3.0)
            ,std::pow(time, 2.0)
            ,time
            ,1.0;
        
        Eigen::MatrixXd velocity_coeff(1, 8);
        velocity_coeff << 7.0*std::pow(time, 6.0)
                , 6.0*std::pow(time, 5.0)
                , 5.0* std::pow(time, 4.0)
                , 4.0* std::pow(time, 3.0)
                , 3.0* std::pow(time, 2.0)
                , 2.0* std::pow(time, 1.0)
                ,1.0
                ,0.0;
        
        Eigen::MatrixXd acceleration_coeff(1, 8);
        acceleration_coeff << 42.0*std::pow(time, 5.0)
                    , 30.0*std::pow(time, 4.0)
                    , 20.0* std::pow(time, 3.0)
                    , 12.0* std::pow(time, 2.0)
                    , 6.0* std::pow(time, 1.0)
                    , 2.0
                    , 0.0
                    , 0.0;

        Eigen::MatrixXd position = pose_coeff*X.block(8*k, 0, 8, 3);
        Eigen::MatrixXd velocity = velocity_coeff*X.block(8*k, 0, 8, 3);
        Eigen::MatrixXd acceleration = acceleration_coeff*X.block(8*k, 0, 8, 3);

        Eigen::VectorXd pos = Eigen::Map<Eigen::RowVectorXd>(position.data(), 3);
        Eigen::VectorXd vel = Eigen::Map<Eigen::RowVectorXd>(velocity.data(), 3);
        Eigen::VectorXd acc = Eigen::Map<Eigen::RowVectorXd>(acceleration.data(), 3);

        states.push_back(pos);
        states.push_back(vel);
        states.push_back(acc);
        // std::cout << " states  " << states.size() << std::endl;

        // std::cout << "pose_coeff: "<< pose_coeff << std::endl;
        // std::cout << "velocity_coeff: "<< velocity_coeff << std::endl;
        // std::cout << "acceleration_coeff: "<< acceleration_coeff << std::endl;

        // std::cout << "position: "<< pos.transpose() << std::endl;
        // std::cout << "velocity: "<< vel.transpose() << std::endl;
        // std::cout << "acceleration: "<< acc.transpose() << std::endl;
    }

    std::pair<double, int > TrajectoryPlanning::closest(double value) {
        std::pair<double, int > result;
        int index = 0;
        for(auto point: time_segs){
            // std::cout << "point: "<< point  << " value: "<< value << std::endl;
            if(point > value){
                break;
            }
            index++;
        }
        if (index == (int)time_segs.size()) { 
            result.first = -1;
		    result.second = -1;
            return result; 
        }
        result.first = time_segs[index];
		result.second = index > 1 ? index -1 : 0;
        return result;
    }

    void TrajectoryPlanning::generate_target_trajectory(std::vector<Eigen::VectorXd>&  target_trajectory
  , std::string trajectory_to_be_flown_file_name){
        Eigen::VectorXd path_position(4);
        std::fstream infile;
        infile.open(trajectory_to_be_flown_file_name, std::ios::in);
        std::string line, word, temp;
        std::string delimiter = ",";
        while (std::getline(infile, line)) {
            size_t pos = 0;
            std::string token;
            int index = 0;
            while ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                path_position(index) = std::atof(token.c_str());
                index++;
                line.erase(0, pos + delimiter.length());
            }
            path_position(index) = std::atof(line.c_str());
            path_position(3) = 1.0;
            target_trajectory.push_back(path_position);
        }
        return;
    }

    void TrajectoryPlanning::traj_opt7(){
        int m = way_points.rows();
        int n = way_points.cols();
        m = m - 1;
        int x_max = 8*m;
        X = Eigen::MatrixXd::Zero(x_max, n);
        A = Eigen::MatrixXd::Zero(n, x_max*x_max);
        Y = Eigen::MatrixXd::Zero(x_max, n);
        for(int i=0; i<n; i++){
            for(int b=0; b<x_max; b++){
                A(i, b*x_max+b) = 1*2.2204e-16;
            }
            int idx = 0;
            Eigen::MatrixXd coeff(1, 8);
            int colum_count = 0;
            int row_index = -1;
            for(int k = 0; k < m-1; k++){
                coeff << std::pow(time_segs[k+1], 7.0f)
                    ,std::pow(time_segs[k+1], 6.0f)
                    ,std::pow(time_segs[k+1], 5.0f)
                    ,std::pow(time_segs[k+1], 4.0f)
                    ,std::pow(time_segs[k+1], 3.0f)
                    ,std::pow(time_segs[k+1], 2.0f)
                    ,time_segs[k+1]
                    ,1.0;

                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                Y(idx, i) = way_points(k+1, i);
                idx = idx + 1;
                row_index++;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                Y(idx, i) = way_points(k+1, i);
                idx = idx + 1;

                
                
            }

            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 7.0*std::pow(time_segs[k+1], 6.0f)
                    , 6.0*std::pow(time_segs[k+1], 5.0f)
                    , 5.0* std::pow(time_segs[k+1], 4.0f)
                    , 4.0* std::pow(time_segs[k+1], 3.0f)
                    , 3.0* std::pow(time_segs[k+1], 2.0f)
                    , 2.0* std::pow(time_segs[k+1], 1.0f)
                    ,1.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;

                
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 42.0*std::pow(time_segs[k+1], 5.0f)
                    , 30.0*std::pow(time_segs[k+1], 4.0f)
                    , 20.0* std::pow(time_segs[k+1], 3.0f)
                    , 12.0* std::pow(time_segs[k+1], 2.0f)
                    , 6.0* std::pow(time_segs[k+1], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
               
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 210.0*std::pow(time_segs[k+1], 4.0f)
                    , 120.0*std::pow(time_segs[k+1], 3.0f)
                    , 60.0* std::pow(time_segs[k+1], 2.0f)
                    , 24.0* std::pow(time_segs[k+1], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
                 
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 840.0*std::pow(time_segs[k+1], 3.0f)
                    , 360.0*std::pow(time_segs[k+1], 2.0f)
                    , 120.0* std::pow(time_segs[k+1], 1.0f)
                    , 24.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
                
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 2520.0*std::pow(time_segs[k+1], 2.0f)
                    , 720.0*std::pow(time_segs[k+1], 1.0f)
                    , 120.0
                    , 0.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
               
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 5040.0*time_segs[k+1]
                    , 720.0
                    , 0.0
                    , 0.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
                 
            }

            
            int k = 0;
            colum_count = 0;
            coeff << std::pow(time_segs[k], 7.0f)
                    ,std::pow(time_segs[k], 6.0f)
                    ,std::pow(time_segs[k], 5.0f)
                    ,std::pow(time_segs[k], 4.0f)
                    ,std::pow(time_segs[k], 3.0f)
                    ,std::pow(time_segs[k], 2.0f)
                    ,time_segs[k]
                    ,1.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = way_points(k, i);
            idx = idx + 1;
            coeff << 7.0*std::pow(time_segs[k], 6.0f)
                    , 6.0*std::pow(time_segs[k], 5.0f)
                    , 5.0* std::pow(time_segs[k], 4.0f)
                    , 4.0* std::pow(time_segs[k], 3.0f)
                    , 3.0* std::pow(time_segs[k], 2.0f)
                    , 2.0* std::pow(time_segs[k], 1.0f)
                    ,1.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            coeff << 42.0*std::pow(time_segs[k], 5.0f)
                    , 30.0*std::pow(time_segs[k], 4.0f)
                    , 20.0* std::pow(time_segs[k], 3.0f)
                    , 12.0* std::pow(time_segs[k], 2.0f)
                    , 6.0* std::pow(time_segs[k], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            coeff << 210.0*std::pow(time_segs[k], 4.0f)
                    , 120.0*std::pow(time_segs[k], 3.0f)
                    , 60.0* std::pow(time_segs[k], 2.0f)
                    , 24.0* std::pow(time_segs[k], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            k = m-1;
            colum_count = 8*k;
            coeff << std::pow(time_segs[k+1], 7.0f)
                    ,std::pow(time_segs[k+1], 6.0f)
                    ,std::pow(time_segs[k+1], 5.0f)
                    ,std::pow(time_segs[k+1], 4.0f)
                    ,std::pow(time_segs[k+1], 3.0f)
                    ,std::pow(time_segs[k+1], 2.0f)
                    ,time_segs[k+1]
                    ,1.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = way_points(k+1, i);
            idx = idx + 1;

            coeff << 7.0*std::pow(time_segs[k+1], 6.0f)
                    , 6.0*std::pow(time_segs[k+1], 5.0f)
                    , 5.0* std::pow(time_segs[k+1], 4.0f)
                    , 4.0* std::pow(time_segs[k+1], 3.0f)
                    , 3.0* std::pow(time_segs[k+1], 2.0f)
                    , 2.0* std::pow(time_segs[k+1], 1.0f)
                    ,1.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            coeff << 42.0*std::pow(time_segs[k+1], 5.0f)
                    , 30.0*std::pow(time_segs[k+1], 4.0f)
                    , 20.0* std::pow(time_segs[k+1], 3.0f)
                    , 12.0* std::pow(time_segs[k+1], 2.0f)
                    , 6.0* std::pow(time_segs[k+1], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            coeff << 210.0*std::pow(time_segs[k+1], 4.0f)
                    , 120.0*std::pow(time_segs[k+1], 3.0f)
                    , 60.0* std::pow(time_segs[k+1], 2.0f)
                    , 24.0* std::pow(time_segs[k+1], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            
            Eigen::MatrixXd x_flat_mat = A.block(i, 0, 1, x_max*x_max);
            Eigen::MatrixXd y_flat_mat = Y.block(0, i, x_max, 1);
            Eigen::Map<Eigen::MatrixXd> x_flat_mat_map(x_flat_mat.data(), x_max, x_max);
            Eigen::MatrixXd x_flat_mat_map_new = x_flat_mat_map.transpose();

            Eigen::MatrixXd x_flat_mat_map_inv = x_flat_mat_map_new.cast <double>();
            Eigen::MatrixXd x_flat_mat_map_invi = x_flat_mat_map_inv.inverse();
            
            Eigen::MatrixXd y_flat_matd = y_flat_mat.cast<double>();
            Eigen::MatrixXd vvv = x_flat_mat_map_invi*y_flat_matd;
            X.block(0, i, x_max, 1) = vvv.cast<double>();
            
            // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
            
            // std::cout << "===============================================================================================================================================" << std::endl;

            // std::cout << "x_flat_mat" << x_flat_mat_map_invi.format(CleanFmt)  << std::endl;
            // std::cout << "coeff==>" << y_flat_mat << std::endl;

        } 
    }

    
//   TrajectoryPlanning trajectory_planning;

// //   trajectory_planning.generate_target_trajectory(path, "/dataset/result/9/path1.csv");
//   int size_of_the_path = path.size();
//   trajectory_planning.generate_ts(path);

//   for (auto time_stamp : trajectory_planning.time_segs){
//     std::cout<< "---" << time_stamp << std::endl;
//   }

//   std::cout<< "Total time: " << trajectory_planning.total_time << std::endl;
//   trajectory_planning.traj_opt7();

//   double cstep = 0.05;
//   double time = 0.0;
//   double tstep = 0.01;

//   int max_iter = (int) (trajectory_planning.total_time / cstep); 
//   std::cout<< "===============max_iter============"<< max_iter << std::endl;
  
//   std::vector<std::vector<Eigen::VectorXd>> paths_points;
//   for (int iter =1; iter < max_iter; iter++){
//     std::vector<Eigen::VectorXd> desired_state;
//     trajectory_planning.get_desired_state(time+cstep, desired_state);
//     paths_points.push_back(desired_state);
//     std::cout<< "==>: "<<  desired_state[0] << std::endl;
//     time = time + cstep;
//   }


}
}