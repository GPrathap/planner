#include "mpc_opt/trajectory_tracker_validate.h"
#include <ros/ros.h>

namespace hagen_planner
{
  
  TrajectoryTrackerValidate::TrajectoryTrackerValidate()
  {
    
  }


  void TrajectoryTrackerValidate::setTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_utils_ = manager;
  }

  void TrajectoryTrackerValidate::solver_init(){

      SX x = SX::sym("x");
      SX y = SX::sym("y");
      SX z = SX::sym("z");
      SX theta = SX::sym("theta");
      states = vertcat(x, y, z, theta);
      n_states = states.size1();

      SX v_x = SX::sym("v_x");
      SX v_y = SX::sym("v_y");
      SX v_z = SX::sym("v_z");
      SX omega = SX::sym("omega");
      controls = vertcat(v_x, v_y, v_z, omega);
      n_controls = controls.size1();

      Eigen::MatrixXd k_A(n_states, n_states); // System dynamics matrix
      Eigen::MatrixXd k_C(n_controls, n_states); // Output matrix
      Eigen::MatrixXd k_Q(n_states, n_states); // Process noise covariance
      Eigen::MatrixXd k_R(n_controls, n_controls); // Measurement noise covariance
      Eigen::MatrixXd k_P(n_states, n_states); // Estimate error covariance
      k_A << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      k_C << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      
      bool passed = passer->passing_matrix("covariance_matrix_for_control_input_q", k_Q);
      if(!passed){
        double q_cov = 1;
        k_Q << q_cov, 0, .032, 0, 0, q_cov, .0, .032, .08,0.0, q_cov, 0, 0,0,0,1;
      }
      passed = passer->passing_matrix("covariance_matrix_for_control_input_r", k_R);
      if(!passed){
        double r_cov = 5000;
        k_R << r_cov, 0, .0, 0, 0, r_cov, .0, .0, .0, 0, r_cov, 0, 0,0,0, r_cov;
      }
      k_P << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      std::cout << "A: \n" << k_A << std::endl;
      std::cout << "C: \n" << k_C << std::endl;
      std::cout << "Q: \n" << k_Q << std::endl;
      std::cout << "R: \n" << k_R << std::endl;
      std::cout << "P: \n" << k_P << std::endl;
      // Construct the filter
      kf = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

      Eigen::MatrixXd nmpc_A(n_states, n_states); // System dynamics matrix
      Eigen::MatrixXd nmpc_C(n_controls, n_states); // Output matrix
      Eigen::MatrixXd nmpc_Q(n_states, n_states); // Process noise covariance
      Eigen::MatrixXd nmpc_R(n_controls, n_controls); // Measurement noise covariance
      Eigen::MatrixXd nmpc_P(n_states, n_states); // Estimate error covariance
      nmpc_A << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      nmpc_C << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_q", nmpc_Q);
      if(!passed){
        double q_cov = 1;
        nmpc_Q << q_cov, 0, .032, 0, 0, q_cov, .0, .032, .08,0.0, q_cov, 0, 0,0,0,1;
      }
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_r", nmpc_R);
      if(!passed){
        double r_cov = 1;
        nmpc_R << r_cov, 0, .0, 0, 0, r_cov, .0, .0, .0, 0, r_cov, 0, 0,0,0, r_cov;
      }
      nmpc_P << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
      std::cout << "A: \n" << nmpc_A << std::endl;
      std::cout << "C: \n" << nmpc_C << std::endl;
      std::cout << "Q: \n" << nmpc_Q << std::endl;
      std::cout << "R: \n" << nmpc_R << std::endl;
      std::cout << "P: \n" << nmpc_P << std::endl;
      // Construct the filter
      kf_nmpc = new KalmanFilter(delta_t, nmpc_A, nmpc_C, nmpc_Q, nmpc_R, nmpc_P);

      rhs = vertcat(v_x*cos(theta)-v_y*sin(theta), v_y*cos(theta) + v_x*sin(theta), v_z, omega);
      f = Function("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});

      SX U = SX::sym("U", n_controls, prediction_horizon);
      SX P = SX::sym("P", n_states + prediction_horizon*(n_states+n_controls));
     

      SX Q = DM::zeros(n_states, n_states);
      SX R = DM::zeros(n_controls, n_controls);
      Eigen::MatrixXd Q_tmp(n_states, n_states); 
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_q", Q_tmp);
      if(!passed){
        Q(0,0) = 1;
        Q(1,1) = 1;
        Q(2,2) = 1;
        Q(3,3) = 3;
      }else{
        Q(0,0) = Q_tmp(0,0);
        Q(1,1) = Q_tmp(1,1);
        Q(2,2) = Q_tmp(2,2);
        Q(3,3) = Q_tmp(3,3);
      }
      
      Eigen::MatrixXd R_tmp(n_states, n_states); 
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_r", R_tmp);
      if(!passed){
        R(0,0) = 1;
        R(1,1) = 1;
        R(2,2) = 1;
        R(3,3) = 0.05;
      }else{
        R(0,0) = R_tmp(0,0);
        R(1,1) = R_tmp(1,1);
        R(2,2) = R_tmp(2,2);
        R(3,3) = R_tmp(3,3);
      }

      if(!use_collocation){
        obj = 0;
        g = SX::sym("g", prediction_horizon+1, n_states);
        X = SX::sym("X", n_states, prediction_horizon+1);
        SX st = X(Slice(0, X.size1()), Slice(0));

        g(Slice(0), Slice(0, g.size2())) = st - P(Slice(0, n_states));
        int ibj = 1;
        SX con = 0;
        int k_index = n_controls + n_states;
        for(int k=0; k<prediction_horizon; k++){
          st = X(Slice(0, X.size1()), Slice(k));
          con = U(Slice(0, U.size1()), Slice(k));
          
          // obj = obj + mtimes((st-P(Slice(n_states,n_states*2))).T(), mtimes(Q,(st-P(Slice(n_states,n_states*2))))) + mtimes(con.T(), mtimes(R, con));
          obj = obj + mtimes((st-P(Slice(k_index*(k+1)-n_controls, k_index*(k+1)))).T(), mtimes(Q,(st-P(Slice(k_index*(k+1)-n_controls, k_index*(k+1)))))) 
                + mtimes((con-P(Slice(k_index*(k+1), k_index*(k+1)+n_states))).T(), mtimes(R, con-P(Slice(k_index*(k+1), k_index*(k+1)+ n_states))));
          
          SX st_next = X(Slice(0, X.size1()), Slice(k+1));
          SXDict f_in = {{"x", st}, {"u", con}};
          SXDict f_value = f(f_in);
          SX st_next_euler = st + delta_t*f_value["rhs"];
          g(Slice(ibj), Slice(0, g.size2())) = st_next - st_next_euler;
          ibj += 1;
        }

        g = reshape(g, n_states*(prediction_horizon+1), 1);
        SX OPT_variables = vertcat(reshape(X, n_states*(prediction_horizon+1), 1), reshape(U, n_controls*prediction_horizon, 1));
        
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 1000;
        opts["expand"] = true;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-4;

        nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g",g}};

        DM lbx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);
        DM ubx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);

        lbx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[0];
        ubx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[0];
        lbx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[1];
        ubx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[1];
        lbx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[2];
        ubx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[2];
        lbx(Slice(3, n_states*(prediction_horizon+1), n_states), Slice(0)) = -inf;
        ubx(Slice(3, n_states*(prediction_horizon+1), n_states), Slice(0)) = inf;
        
        lbx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(prediction_horizon+1)+3, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_min;
        ubx(Slice(n_states*(prediction_horizon+1)+3, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_max;

        DM p(n_states*2);
        args["lbx"] = lbx;
        args["ubx"] = ubx;
        args["p"] = p;

      }else{
        
        cout<< "Number of points: "<<  collocation_degree << endl;
        C = vector<vector<double>>(collocation_degree+1,vector<double>(collocation_degree+1,0));
        D = vector<double>(collocation_degree+1, 0);
        B = vector<double>(collocation_degree+1, 0);
        int num_states = prediction_horizon + 1 + collocation_degree*prediction_horizon;
        getCollocationPoints(B, C, D);

        obj = 0;
        X = SX::sym("X", n_states, num_states);
        g = SX::sym("g", num_states, n_states);

        SX Xk = X(Slice(0, X.size1()), Slice(0));
        
        g(Slice(0), Slice(0, g.size2())) = Xk - P(Slice(0, n_states));
        // SX con = 0;
        int pos_index = 0;
        int constraints = 1;
        for(int k=0; k<prediction_horizon; k++){
          SX Uk = U(Slice(0, U.size1()), Slice(k));
          vector<SX> Xc;
          for(int j=0; j<collocation_degree; j++){
            SX Xkj = X(Slice(0, X.size1()), Slice(pos_index+1+j));
            Xc.push_back(Xkj);
          }
          pos_index += collocation_degree+1;
          SX Xk_end = D[0]*Xk;
          for (int j=1; j<collocation_degree+1; j++){
            SX xp = C[0][j]*Xk;
            for (int r=0; r<collocation_degree; r++){
              xp = xp + C[r+1][j]*Xc[r];
            }
            SXDict f_in = {{"x", Xc[j-1]}, {"u", Uk}};
            SXDict f_value = f(f_in);
            auto qj = mtimes((Xc[j-1]-P(Slice(n_states,n_states*2))).T(), mtimes(Q,(Xc[j-1]-P(Slice(n_states,n_states*2))))) + mtimes(Uk.T(), mtimes(R, Uk));
            g(Slice(constraints), Slice(0, g.size2())) = delta_t*f_value["rhs"] - xp;
            constraints +=1;
            Xk_end = Xk_end + D[j]*Xc[j-1];
            obj += B[j]*qj*delta_t;
          }
          Xk =  X(Slice(0, X.size1()), Slice(pos_index));
          g(Slice(constraints), Slice(0, g.size2())) = Xk_end-Xk;
          constraints +=1;
        }

        g = reshape(g, n_states*(num_states), 1);
        SX OPT_variables = vertcat(reshape(X, n_states*(num_states), 1), reshape(U, n_controls*prediction_horizon, 1));
        
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 1000;
        opts["expand"] = true;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-4;

        nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g", g}};

        DM lbx = DM(n_states*(num_states)+n_controls*prediction_horizon, 1);
        DM ubx = DM(n_states*(num_states)+n_controls*prediction_horizon, 1);

        lbx(Slice(0, n_states*(num_states), n_states), Slice(0)) = min_range_[0];
        ubx(Slice(0, n_states*(num_states), n_states), Slice(0)) = max_range_[0];
        lbx(Slice(1, n_states*(num_states), n_states), Slice(0)) = min_range_[1];
        ubx(Slice(1, n_states*(num_states), n_states), Slice(0)) = max_range_[1];
        lbx(Slice(2, n_states*(num_states), n_states), Slice(0)) = min_range_[2];
        ubx(Slice(2, n_states*(num_states), n_states), Slice(0)) = max_range_[2];
        lbx(Slice(3, n_states*(num_states), n_states), Slice(0)) = -inf;
        ubx(Slice(3, n_states*(num_states), n_states), Slice(0)) = inf;
        
        lbx(Slice(n_states*(num_states), n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states), n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(num_states)+1, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states)+1, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(num_states)+2, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states)+2, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(num_states)+3, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_min;
        ubx(Slice(n_states*(num_states)+3, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_max;

        DM p(n_states*2);
        args["lbx"] = lbx;
        args["ubx"] = ubx;
        args["p"] = p;
      }
      vehicle_current_state = std::make_shared<std::vector<double>>();  
  }


  std::vector<Eigen::Vector3d> TrajectoryTrackerValidate::mpc_solver(){
    if(use_collocation){
      return mpc_solver_with_collocation();
    }else{
      return mpc_solver_with_multiple_shooting();
    }
  }


  std::vector<Eigen::Vector3d> TrajectoryTrackerValidate::mpc_solver_with_multiple_shooting(){

    double t0 = 0;
    vector<double> t;
    t.push_back(t0);
    // DM u0 = DM::zeros(prediction_horizon, n_controls);
    DM U0 = repmat(u0, 1, prediction_horizon).T();
    DM X0 = repmat(x0, 1, prediction_horizon+1).T();
    int sim_time = simulated_duration;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;
    args["x0"] = x0;
    // double error(norm_2(x0-xs));

    still_running = true;
    // bool init_kalman_filter = false;
    Eigen::VectorXd k_x0(n_states);
    k_x0 << 0, 0, 0, 0;
    kf->init(0, k_x0);

    Eigen::VectorXd k_p1(4);
    k_p1 <<  (double)x0(0,0), (double)x0(1,0), (double)x0(2,0), (double)x0(3,0);
    kf_nmpc->init(0, k_p1);

    clock_t t2, t1 = clock();
    previos_pose<< (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);

    double t_cmd_start, t_cmd_end;
    bspline_utils_->traj_pos_.getTimeSpan(t_cmd_start, t_cmd_end);
    bspline_utils_->retrieveTrajectory();
    ros::Time time_traj_start = ros::Time::now();
    double error(norm_2(x0-xs));
    traj_cmd = new boost::circular_buffer<Eigen::Vector3d>(10000);
    std::vector<Eigen::Vector3d> final_path;
    while(error > maximum_acceptable_error && mpciter < sim_time/delta_t){
        error = (double)(norm_2(x0-xs));
        // std::cout<<  "============14===============" << maximum_acceptable_error << "  " << error << std::endl;

        t2 = clock();
        int k_index = n_controls + n_states;
        if((t2-t1) > delta_t){
          
          args["p"] = DM::zeros(n_states + prediction_horizon*(n_states+n_controls));
          args["p"](Slice(0, n_states)) = x0;
          // args["p"] = vertcat(x0, xs);
          ros::Time time_now = ros::Time::now();
    //       // std::cout<< "========1" << std::endl;
          double current_time = (time_now - time_traj_start).toSec();
          std::vector<Eigen::Vector3d> vis_horizon_desired;
          for(int k=0; k<prediction_horizon; k++){
    //         // std::cout<< "========2" << std::endl;
            double t_predict = current_time + k*(delta_t_desired);
            Eigen::Vector3d current_pose = bspline_utils_->traj_pos_.evaluateDeBoor(t_predict);
            vis_horizon_desired.push_back(current_pose);
            // Eigen::Vector3d current_velocity = bspline_utils_->traj_vel_.evaluateDeBoor(t_predict);
            double x_ref = current_pose[0];
            double y_ref = current_pose[1];
            double z_ref = current_pose[2];
            double theta_ref = 0;
            double v_x_ref = 0;
            double v_y_ref = 0;
            double v_z_ref = 0;
            double omega_ref = 0;
            // std::cout<< "========3"<< current_pose.transpose() << std::endl;
            DM ref_state_vector = DM::zeros(1, n_states);
            DM ref_control_inputs = DM::zeros(1, n_states);
            
            ref_state_vector(0) = x_ref;
            ref_state_vector(1) = y_ref;
            ref_state_vector(2) = z_ref;
            ref_state_vector(3) = theta_ref;

            ref_control_inputs(0) = v_x_ref;
            ref_control_inputs(1) = v_y_ref;
            ref_control_inputs(2) = v_z_ref;
            ref_control_inputs(3) = omega_ref;

            args["p"](Slice(k_index*(k+1)-n_controls, k_index*(k+1))) = ref_state_vector;
            args["p"](Slice(k_index*(k+1), k_index*(k+1)+n_states)) = ref_control_inputs;
          }
          
          visualization_->drawPath(vis_horizon_desired, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 190);
          args["x0"] = vertcat(reshape(X0.T(), n_states*(prediction_horizon+1), 1), reshape(U0.T(), n_controls*prediction_horizon, 1));
          
          // obs_length = (stuck_point <= 0)? 1: stuck_point*2;
         
          vector<Eigen::VectorXd> obs_map_;
          double previous_dis = 20;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            // std::cout<< "=============4===============" << std::endl;
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            Eigen::Vector3d pos((double)X0(trj_i,0), (double)X0(trj_i,1), (double)X0(trj_i,2));
            Eigen::Vector3d obs;
            double obs_dis;
            Eigen::VectorXd obs_pose(4);
            edt_env_->get_close_obstacle(pos, obs, obs_dis);
            // cout<< "Obs pose: " << obs.transpose() << "Obs distance: "<< obs_dis << endl;
            if(std::abs(obs_dis-previous_dis)< obs_min_allowed_length){
              // cout<< "Obs at the same pose" << obs.transpose() << endl;
            }else{
              obs_pose.head(3) = obs;
              obs_pose[3] = obs_dis;
              obs_map_.push_back(obs_pose);
              previous_dis = obs_dis;
            }
          }
          std::sort(std::begin(obs_map_), std::end(obs_map_), [](Eigen::VectorXd a
              , Eigen::VectorXd b) {return a[3] > b[3]; });
          
          if(obs_map_.size() <= obs_max_allowed_length){
            obs_length = obs_map_.size();
          }else{
            obs_length = obs_max_allowed_length;
          }
          // obs_length = obs_map_.size();
          DM lbg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          DM ubg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          lbg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          ubg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          lbg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = -inf;
          ubg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = 0;
          args["lbg"] = lbg; 
          args["ubg"] = ubg;
          SX obs_g = SX::sym("obs_g",(prediction_horizon+1)*obs_length);
          // std::cout<< "=============3===============" << std::endl;
          int obs_cout=0;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            // std::cout<< "=============4===============" << std::endl;
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            // for(auto obs : obs_map_){
            for(int obs_index = 0; obs_index<obs_length; obs_index++){
              Eigen::VectorXd obs = obs_map_[obs_index];
              obs_g(obs_cout) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + avoidance_distance;
              obs_cout++;
            }
          }

          std::vector<Eigen::Vector3d> vis_obs;
          for(auto obs : obs_map_){
            vis_obs.push_back(obs.head(3));
          }
          visualization_->drawPath(vis_obs, 0.2, Eigen::Vector4d(0.0, 0.2 ,0.9, 1), 345);

          // std::cout<< "=============5===============" << std::endl;
          SX final_g = vertcat(g, obs_g);
          // std::cout<< "=============6===============" << std::endl;
          nlp_prob["g"] = final_g;
          // nlp_prob["f"] = obj + obj_cucost;
          Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);
          // std::cout<< "=============7===============" << std::endl;
          res = solver(args);
          // std::cout<< "=================="<< solver.stats().at("return_status") << std::endl;
          DM u = reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T();
          vector<double> u11(reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T());
          // vector<double> xx11(reshape(res["x"](Slice(0, (n_states*(N+1)))).T(), n_states, N+1).T());
          // xx1.push_back(xx11);
          vector<double> xx11(res["x"](Slice(0, (n_states*(prediction_horizon+1)))).T());
          xx1.push_back(xx11);
          std::vector<Eigen::Vector3d> vis_horizon;

          for(int kl=0; kl< int(xx11.size()/n_states); kl+=4){
            Eigen::Vector3d poss(xx11[kl], xx11[kl+1], xx11[kl+2]);
            vis_horizon.push_back(poss);
          }

          visualization_->drawPath(vis_horizon, 0.2, Eigen::Vector4d(0.2, 1 ,0.5, 1), 0);
          // std::cout<< res["x"](Slice(0, (n_states*(N+1)))).T() << std::endl;
          u_cl.push_back(u11);
          t.push_back(t0);

          DM current_control = u(Slice(0), Slice(0, u.size2())).T();
          DM current_pose = x0;
          // std::cout<< "============12==============="<< (double)current_pose(0) << "   "<< (double)current_control(1) <<  std::endl;
        
          if(force_terminate){
            still_running = false;
            std::cout<< "Solver has been interrupted..." <<  std::endl;
            return final_path;
          }

          Eigen::VectorXd k_y(n_controls);
          std::vector<double> k_estimated_values;
          k_y << (double)current_control(0), (double)current_control(1), (double)current_control(2), (double)current_control(3);
          kf->update(k_y);

          Eigen::MatrixXd B_(n_controls, n_controls);
          B_ << (double)cos(x0(3,0)), (double)-sin(x0(3,0)), 0, 0, (double)sin(x0(3,0)), (double)cos(x0(3,0)), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
          B_ = delta_t*B_;
          Eigen::VectorXd k_p_(n_states);
          k_p_ << (double)x0(0), (double)x0(1), (double)x0(2), (double)x0(3);
          kf_nmpc->update(k_p_, B_, k_y);

          x0(0,0) = current_projected_pose.pose.pose.position.x =  kf_nmpc->state()[0];
          x0(1,0) = current_projected_pose.pose.pose.position.y = kf_nmpc->state()[1];
          x0(2,0) = current_projected_pose.pose.pose.position.z = kf_nmpc->state()[2];
          x0(3,0) = current_projected_pose.pose.pose.orientation.x = kf_nmpc->state()[3];
          
          current_projected_pose.twist.twist.linear.x = kf->state()[0];
          current_projected_pose.twist.twist.linear.y = kf->state()[1];
          current_projected_pose.twist.twist.linear.z = kf->state()[2];
          current_projected_pose.twist.twist.angular.z = kf->state()[3];

          Eigen::Vector3d pos;
          pos(0) = current_projected_pose.pose.pose.position.x;
          pos(1) = current_projected_pose.pose.pose.position.y;
          pos(2) = current_projected_pose.pose.pose.position.z;
          traj_cmd->push_back(pos);
          visualization_->displayTrajWithColor(*traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 21);
          final_path.push_back(pos);
          pos_current_pos_pub.publish(current_projected_pose);

          tuple<double, DM, DM> shiftted_val = shift(delta_t, t0, x0, u, f); 
          x0 = get<1>(shiftted_val);
          u = get<2>(shiftted_val);
          t0 = get<0>(shiftted_val);

          X0 = reshape(res["x"](Slice(0, n_states*(prediction_horizon+1))).T(), n_states, prediction_horizon+1).T();
          // xx(Slice(0, xx.size1()),Slice(mpciter+2)) = x0;
          vector<double> xxxo(x0);
          // std::cout<<  "============14===============" << x0 << std::endl;
          xx0.push_back(xxxo);

          SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
          SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
          X0 = vertcat(x0_rest, x0_last);
          mpciter = mpciter + 1;
          t1 = clock();
        }else{
          usleep(5000);
        }
    }

    still_running = false;
    need_intermediate_goal = false;
    cout<< "Finish mpc solver...."<< endl;
    // std::cout<< "Start saving files" << std::endl;
    // string  stotage_location = "/dataset/data/";

    // string file_name = stotage_location + "xx0.npy";
    // save_double_vector(xx0, file_name, 1);

    // file_name = stotage_location + "xx1.npy";
    // std::cout<< xx1.size() << "   " << xx1[0].size() << std::endl;
    // save_double_vector(xx1, file_name, 1);

    // // file_name = stotage_location + "xx.npy";
    // // save_dm(xx, file_name, 1);

    // file_name = stotage_location + "u_cl.npy";
    // save_double_vector(u_cl, file_name, 4);

    // file_name = stotage_location + "t.npy";
    // save_vector(t, file_name, 1);

    // file_name = stotage_location + "xs.npy";
    // save_dm(xs, file_name, 1);
    return final_path;
  
  }

  std::vector<Eigen::Vector3d> TrajectoryTrackerValidate::mpc_solver_with_collocation(){

    double t0 = 0;
    vector<double> t;
    t.push_back(t0);
    int num_of_states = (prediction_horizon*collocation_degree) + (prediction_horizon + 1);
    DM U0 = repmat(u0, 1, prediction_horizon).T();
    DM X0 = repmat(x0, 1, num_of_states).T();
    int sim_time = simulated_duration;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;
    args["x0"] = x0;

    still_running = true;
    Eigen::VectorXd k_x0(n_states);
    k_x0 << 0, 0, 0, 0;
    kf->init(0, k_x0);

    // Eigen::VectorXd k_p0(3);
    // k_p0 <<  (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);
    // kf_position->init(0, k_p0);
    Eigen::VectorXd k_p1(4);
    k_p1 <<  (double)x0(0,0), (double)x0(1,0), (double)x0(2,0), (double)x0(3,0);
    kf_nmpc->init(0, k_p1);

    clock_t t2, t1 = clock();
    previos_pose<< (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);
   
    double t_cmd_start, t_cmd_end;
    bspline_utils_->traj_pos_.getTimeSpan(t_cmd_start, t_cmd_end);
    bspline_utils_->retrieveTrajectory();

    ros::Time time_traj_start = ros::Time::now();
    std::vector<Eigen::Vector3d> final_path;
    double error(norm_2(x0-xs));
     while(error > maximum_acceptable_error && mpciter < sim_time/delta_t){
        error = (double)(norm_2(x0-xs));
        t2 = clock();
        int k_index = n_controls + n_states;
        if((t2-t1) > delta_t){
          
          args["p"] = DM::zeros(n_states + prediction_horizon*(n_states+n_controls));
          args["p"](Slice(0, n_states)) = x0;
          ros::Time time_now = ros::Time::now();
          
          double current_time = (time_now - time_traj_start).toSec();
          std::vector<Eigen::Vector3d> vis_horizon_desired;
          for(int k=0; k<prediction_horizon; k++){
            double t_predict = current_time + k*(delta_t_desired);
            Eigen::Vector3d current_pose = bspline_utils_->traj_pos_.evaluateDeBoor(t_predict);
            vis_horizon_desired.push_back(current_pose);
            double x_ref = current_pose[0];
            double y_ref = current_pose[1];
            double z_ref = current_pose[2];
            double theta_ref = 0;
            double v_x_ref = 0;
            double v_y_ref = 0;
            double v_z_ref = 0;
            double omega_ref = 0;
           
            DM ref_state_vector = DM::zeros(1, n_states);
            DM ref_control_inputs = DM::zeros(1, n_states);
            
            ref_state_vector(0) = x_ref;
            ref_state_vector(1) = y_ref;
            ref_state_vector(2) = z_ref;
            ref_state_vector(3) = theta_ref;

            ref_control_inputs(0) = v_x_ref;
            ref_control_inputs(1) = v_y_ref;
            ref_control_inputs(2) = v_z_ref;
            ref_control_inputs(3) = omega_ref;

            args["p"](Slice(k_index*(k+1)-n_controls, k_index*(k+1))) = ref_state_vector;
            args["p"](Slice(k_index*(k+1), k_index*(k+1)+n_states)) = ref_control_inputs;
          }

          visualization_->drawPath(vis_horizon_desired, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 190);
          args["x0"] = vertcat(reshape(X0.T(), n_states*(num_of_states), 1), reshape(U0.T(), n_controls*prediction_horizon, 1));
          
          vector<Eigen::VectorXd> obs_map_;
          double previous_dis = 20;
          int pose_select_index = 0;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            // std::cout<< "=============4===============" << std::endl;
            SX st = X(Slice(0, X.size1()), Slice(pose_select_index));
            Eigen::Vector3d pos((double)X0(pose_select_index,0), (double)X0(pose_select_index,1), (double)X0(pose_select_index,2));
            Eigen::Vector3d obs;
            double obs_dis;
            Eigen::VectorXd obs_pose(4);
            edt_env_->get_close_obstacle(pos, obs, obs_dis);
            // cout<< "Obs pose: " << obs.transpose() << "Obs distance: "<< obs_dis << endl;
            if(std::abs(obs_dis-previous_dis)< obs_min_allowed_length){
              // cout<< "Obs at the same pose" << obs.transpose() << endl;
            }else{
              obs_pose.head(3) = obs;
              obs_pose[3] = obs_dis;
              obs_map_.push_back(obs_pose);
              previous_dis = obs_dis;
            }
            pose_select_index += (collocation_degree +1);
          }
          std::sort(std::begin(obs_map_), std::end(obs_map_), [](Eigen::VectorXd a
              , Eigen::VectorXd b) {return a[3] > b[3]; });
          
          // std::cout<< "===========obs_map_.size()============="<< obs_map_.size() << std::endl;
          if(obs_map_.size() <= obs_max_allowed_length){
            obs_length = obs_map_.size();
          }else{
            obs_length = obs_max_allowed_length;
          }

          obs_length = obs_map_.size();
          DM lbg = DM(1, n_states*(num_of_states) + (prediction_horizon+1)*obs_length);
          DM ubg = DM(1, n_states*(num_of_states) + (prediction_horizon+1)*obs_length);
          lbg(Slice(0), Slice(0, n_states*(num_of_states))) = 0;
          ubg(Slice(0), Slice(0, n_states*(num_of_states))) = 0;
          lbg(Slice(0), Slice(n_states*(num_of_states), n_states*(num_of_states) + (prediction_horizon+1)*obs_length)) = -inf;
          ubg(Slice(0), Slice(n_states*(num_of_states), n_states*(num_of_states) + (prediction_horizon+1)*obs_length)) = 0;
          args["lbg"] = lbg; 
          args["ubg"] = ubg;
          SX obs_g = SX::sym("obs_g",(prediction_horizon+1)*obs_length);
          // std::cout<< "=============3===============" << std::endl;
          int obs_cout=0;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            // std::cout<< "=============4===============" << std::endl;
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            for(auto obs : obs_map_){
              obs_g(obs_cout) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + (avoidance_distance);
              obs_cout++;
            }
            
          }
          // std::cout<< "=============5===============" << std::endl;
          SX final_g = vertcat(g, obs_g);
          // std::cout<< "=============6===============" << std::endl;
          nlp_prob["g"] = final_g;
          // nlp_prob["f"] = obj + obj_cucost;
          Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);
          // std::cout<< "=============7===============" << std::endl;
          res = solver(args);
          // std::cout<< "=================="<< solver.stats().at("return_status") << std::endl;
          DM u = reshape(res["x"](Slice(n_states*(num_of_states), res.at("x").size1())).T(), n_controls, prediction_horizon).T();
          vector<double> u11(reshape(res["x"](Slice(n_states*(num_of_states), res.at("x").size1())).T(), n_controls, prediction_horizon).T());
          vector<double> xx11(res["x"](Slice(0, (n_states*(num_of_states)))).T());
          xx1.push_back(xx11);
          std::vector<Eigen::Vector3d> vis_horizon;

          for(int kl=0; kl< int(xx11.size()/n_states); kl+=4){
            Eigen::Vector3d poss(xx11[kl], xx11[kl+1], xx11[kl+2]);
            vis_horizon.push_back(poss);
          }

          visualization_->drawPath(vis_horizon, 0.2, Eigen::Vector4d(0.2, 1 ,0.5, 1), 0);
          // std::cout<< res["x"](Slice(0, (n_states*(N+1)))).T() << std::endl;
          u_cl.push_back(u11);
          t.push_back(t0);

          DM current_control = u(Slice(0), Slice(0, u.size2())).T();
          DM current_pose = x0;
          // std::cout<< "============12==============="<< (double)current_pose(0) << "   "<< (double)current_control(1) <<  std::endl;
        
          if(force_terminate){
            still_running = false;
            std::cout<< "Solver has been interrupted..." <<  std::endl;
            return final_path;
          }

          // Eigen::VectorXd k_p(3);
          // k_p << (double)current_pose(0), (double)current_pose(1), (double)current_pose(2);
          // kf_position->update(k_p);

          // current_projected_pose.pose.pose.position.x =  kf_position->state()[0];
          // current_projected_pose.pose.pose.position.y = kf_position->state()[1];
          // current_projected_pose.pose.pose.position.z = kf_position->state()[2];
          // current_projected_pose.pose.pose.orientation.x = (double)current_pose(3);
          // current_projected_pose.twist.twist.linear.x = (double)(current_control(0)*cos(current_pose(3)) - current_control(1)*sin(current_pose(3)));
          // current_projected_pose.twist.twist.linear.y = (double)(current_control(0)*sin(current_pose(3)) + current_control(1)*cos(current_pose(3)));
          
          Eigen::VectorXd k_y(n_controls);
          std::vector<double> k_estimated_values;
          k_y << (double)current_control(0), (double)current_control(1), (double)current_control(2), (double)current_control(3);
          kf->update(k_y);

          Eigen::MatrixXd B_(n_controls, n_controls);
          B_ << (double)cos(x0(3,0)), (double)-sin(x0(3,0)), 0, 0, (double)sin(x0(3,0)), (double)cos(x0(3,0)), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
          B_ = delta_t*B_;
          Eigen::VectorXd k_p_(n_states);
          k_p_ << (double)x0(0), (double)x0(1), (double)x0(2), (double)x0(3);
          kf_nmpc->update(k_p_, B_, k_y);

          current_projected_pose.twist.twist.linear.x = kf->state()[0];
          current_projected_pose.twist.twist.linear.y = kf->state()[1];
          current_projected_pose.twist.twist.linear.z = kf->state()[2];
          current_projected_pose.twist.twist.angular.z = kf->state()[3];

          // Eigen::VectorXd k_p(3);
          // k_p << (double)current_pose(0), (double)current_pose(1), (double)current_pose(2);
          // kf_position->update(k_p);
            
          // x0(0,0) =  current_projected_pose.pose.pose.position.x =  kf_position->state()[0];
          // x0(1,0) =  current_projected_pose.pose.pose.position.y = kf_position->state()[1];
          // x0(2,0) =  current_projected_pose.pose.pose.position.z = kf_position->state()[2];
          x0(0,0) = current_projected_pose.pose.pose.position.x =  kf_nmpc->state()[0];
          x0(1,0) = current_projected_pose.pose.pose.position.y = kf_nmpc->state()[1];
          x0(2,0) = current_projected_pose.pose.pose.position.z = kf_nmpc->state()[2];
          x0(3,0) = current_projected_pose.pose.pose.orientation.x = kf_nmpc->state()[3];

          Eigen::Vector3d pos;
          pos(0) = current_projected_pose.pose.pose.position.x;
          pos(1) = current_projected_pose.pose.pose.position.y;
          pos(2) = current_projected_pose.pose.pose.position.z;
          final_path.push_back(pos);
          pos_current_pos_pub.publish(current_projected_pose);

          // std::cout<< "============13==============="<< std::endl;
          tuple<double, DM, DM> shiftted_val = shift(delta_t, t0, x0, u, f); 
          x0 = get<1>(shiftted_val);
          u = get<2>(shiftted_val);
          t0 = get<0>(shiftted_val);

          X0 = reshape(res["x"](Slice(0, n_states*(num_of_states))).T(), n_states, num_of_states).T();
          // xx(Slice(0, xx.size1()),Slice(mpciter+2)) = x0;
          vector<double> xxxo(x0);
          // std::cout<<  "============14===============" << x0 << std::endl;
          xx0.push_back(xxxo);

          SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
          SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
          X0 = vertcat(x0_rest, x0_last);
          mpciter = mpciter + 1;
          t1 = clock();
        }else{
          usleep(5000);
        }
    }

    still_running = false;
    need_intermediate_goal = false;
    cout<< "Finish mpc solver...."<< endl;
    // std::cout<< "Start saving files" << std::endl;
    // string  stotage_location = "/dataset/data/";

    // string file_name = stotage_location + "xx0.npy";
    // save_double_vector(xx0, file_name, 1);

    // file_name = stotage_location + "xx1.npy";
    // std::cout<< xx1.size() << "   " << xx1[0].size() << std::endl;
    // save_double_vector(xx1, file_name, 1);

    // // file_name = stotage_location + "xx.npy";
    // // save_dm(xx, file_name, 1);

    // file_name = stotage_location + "u_cl.npy";
    // save_double_vector(u_cl, file_name, 4);

    // file_name = stotage_location + "t.npy";
    // save_vector(t, file_name, 1);

    // file_name = stotage_location + "xs.npy";
    // save_dm(xs, file_name, 1);
    return final_path;
  
  }

  void TrajectoryTrackerValidate::init(ros::NodeHandle& nh){
      node_ = nh;
      NonLinearMPCOpt::init(nh);
      node_.param("mpc_opt/delta_t_desired", delta_t_desired, 0.02);
      solver_init();
  }

}  // namespace hagen_planner
