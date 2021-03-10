#include "mpc_opt/nonlinear_mpc_opt.h"
#include <ros/ros.h>

namespace hagen_planner
{
  NonLinearMPCOpt::NonLinearMPCOpt()
  {
    
  }

  void NonLinearMPCOpt::solver_init(){
      // SX x = SX::sym("x");
      // SX y = SX::sym("y");
      // SX z = SX::sym("z");
      // SX theta = SX::sym("theta");
      // states = vertcat(x, y, z, theta);
      // n_states = states.size1();

      // SX v_x = SX::sym("v_x");
      // SX v_y = SX::sym("v_y");
      // SX v_z = SX::sym("v_z");
      // SX omega = SX::sym("omega");
      // controls = vertcat(v_x, v_y, v_z, omega);
      // n_controls = controls.size1();

      // obs_map = {{-2.5, 1.5, 2, 2},{3.5, 5, 3, 2}};
      // obs_length = 1;

      // rhs = vertcat(v_x*cos(theta)-v_y*sin(theta), v_y*cos(theta) + v_x*sin(theta), v_z, omega);
      // f = Function("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});

      // SX U = SX::sym("U", n_controls, prediction_horizon);
      // SX P = SX::sym("P", n_states + n_states);
      // X = SX::sym("X", n_states, prediction_horizon+1);

      // SX Q = DM::zeros(n_states, n_states);
      // Q(0,0) = 1;
      // Q(1,1) = 1;
      // Q(2,2) = 1;
      // Q(3,3) = 3;
      // SX R = DM::zeros(n_controls, n_controls);
      // R(0,0) = 1;
      // R(1,1) = 1;
      // R(2,2) = 1;
      // R(3,3) = 0.05;

      // obj = 0;
      // g = SX::sym("g", prediction_horizon+1, n_states);
      // SX st = X(Slice(0, X.size1()), Slice(0));

      // g(Slice(0), Slice(0, g.size2())) = st - P(Slice(0, n_states));
      // int ibj = 1;
      // SX con = 0;
      // for(int k=0; k<prediction_horizon; k++){
      //   st = X(Slice(0, X.size1()), Slice(k));
      //   con = U(Slice(0, U.size1()), Slice(k));
      //   obj = obj + mtimes((st-P(Slice(n_states,n_states*2))).T(), mtimes(Q,(st-P(Slice(n_states,n_states*2))))) + mtimes(con.T(), mtimes(R, con));
      //   SX st_next = X(Slice(0, X.size1()), Slice(k+1));
      //   SXDict f_in = {{"x", st}, {"u", con}};
      //   SXDict f_value = f(f_in);
      //   SX st_next_euler = st + delta_t*f_value["rhs"];
      //   g(Slice(ibj), Slice(0, g.size2())) = st_next - st_next_euler;
      //   ibj += 1;
      // }

      // g = reshape(g, n_states*(prediction_horizon+1), 1);
      // SX OPT_variables = vertcat(reshape(X, n_states*(prediction_horizon+1), 1), reshape(U, n_controls*prediction_horizon, 1));
      
      // opts["ipopt.tol"] = 1e-4;
      // opts["ipopt.max_iter"] = 1000;
      // opts["expand"] = true;
      // opts["ipopt.print_level"] = 0;
      // opts["print_time"] = 0;
      // opts["ipopt.acceptable_tol"] = 1e-8;

      // nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g",g}};
      
      // // DM lbg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
      // // DM ubg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);

      // // lbg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
      // // ubg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;

      // // lbg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = -inf;
      // // ubg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = 0;

      // DM lbx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);
      // DM ubx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);

      // lbx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[0];
      // ubx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[0];
      // lbx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[1];
      // ubx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[1];
      // lbx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[2];
      // ubx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[2];
      // lbx(Slice(3, n_states*(prediction_horizon+1), n_states), Slice(0)) = -inf;
      // ubx(Slice(3, n_states*(prediction_horizon+1), n_states), Slice(0)) = inf;
      
      // lbx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
      // ubx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
      // lbx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
      // ubx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
      // lbx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
      // ubx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
      // lbx(Slice(n_states*(prediction_horizon+1)+3, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_min;
      // ubx(Slice(n_states*(prediction_horizon+1)+3, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = omega_max;

      // DM p(n_states*2);
      // args["lbx"] = lbx;
      // args["ubx"] = ubx;
      // args["p"] = p;

      // vehicle_current_state = std::make_shared<std::vector<double>>();  
  }

  void NonLinearMPCOpt::getCollocationPoints(std::vector<double>& B, std::vector<std::vector<double>>& C, std::vector<double>& D){
    
    int d = collocation_degree;
    // Choose collocation points
    vector<double> tau_root = collocation_points(d, "legendre");
    tau_root.insert(tau_root.begin(), 0);

    // Coefficients of the collocation equation
    // vector<vector<double> > C(d+1,vector<double>(d+1,0));

    // Coefficients of the continuity equation
    // vector<double> D(d+1,0);

    // For all collocation points
    for(int j=0; j<d+1; ++j){
      // Construct Lagrange polynomials to get the polynomial basis at the collocation point
      Polynomial p = 1;
      for(int r=0; r<d+1; ++r){
        if(r!=j){
          p *= Polynomial(-tau_root[r],1)/(tau_root[j]-tau_root[r]);
        }
      }

      // Evaluate the polynomial at the final time to get the coefficients of the continuity equation
      D[j] = p(1.0);

      // Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
      Polynomial dp = p.derivative();
      for(int r=0; r<d+1; ++r){
        C[j][r] = dp(tau_root[r]);
      }

      Polynomial ip = p.anti_derivative();
      B[j] = ip(1.0);

    }
    return;
  }


  double NonLinearMPCOpt::getYawFromQuat(const geometry_msgs::Quaternion &data)
  {
      tf::Quaternion quat(data.x, data.y, data.z, data.w);
      tf::Matrix3x3 m(quat);
      double_t roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
  }

  // void NonLinearMPCOpt::mpc_solver(){

  //   cout<< "Calling parent method" << endl;
  //   // double dt = 1.0/30; // Time step
  //   // Eigen::MatrixXd k_A(n_states, n_states); // System dynamics matrix
  //   // Eigen::MatrixXd k_C(n_controls, n_states); // Output matrix
  //   // Eigen::MatrixXd k_Q(n_states, n_states); // Process noise covariance
  //   // Eigen::MatrixXd k_R(n_controls, n_controls); // Measurement noise covariance
  //   // Eigen::MatrixXd k_P(n_states, n_states); // Estimate error covariance
  //   // k_A << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  //   // k_C << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  //   // // Reasonable covariance matrices
  //   // double q_cov = 1;
  //   // double r_cov = 5000;
  //   // k_Q << q_cov, 0, .032, 0, 0, q_cov, .0, .032, .08,0.0, q_cov, 0, 0,0,0,1;
  //   // k_R << r_cov, 0, .0, 0, 0, r_cov, .0, .0, .0, 0, r_cov, 0, 0,0,0, r_cov;
  //   // k_P << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  //   // std::cout << "A: \n" << k_A << std::endl;
  //   // std::cout << "C: \n" << k_C << std::endl;
  //   // std::cout << "Q: \n" << k_Q << std::endl;
  //   // std::cout << "R: \n" << k_R << std::endl;
  //   // std::cout << "P: \n" << k_P << std::endl;
  //   // //  // Construct the filter
  //   // KalmanFilter kf(0, k_A, k_C, k_Q, k_R, k_P);

  //   // double t0 = 0;
  //   // vector<double> t;
  //   // t.push_back(t0);
  //   // // DM u0 = DM::zeros(prediction_horizon, n_controls);
  //   // DM U0 = repmat(u0, 1, prediction_horizon).T();
  //   // DM X0 = repmat(x0, 1, prediction_horizon+1).T();
  //   // int sim_time = simulated_duration;
  //   // int mpciter = 0;
  //   // vector<vector<double>> xx1, u_cl, xx0, xx;
  //   // args["x0"] = x0;
  //   // double error(norm_2(x0-xs));


  //   // Cumulative_Sum<double, double, 5> cumulative_sum_checker;
  //   // // SX obs_g = SX::sym("obs_g",(prediction_horizon+1), 1);
  //   //  // TODO where to set obs ...?
  //   // // SX obs = SX::sym("obs", 3, (prediction_horizon+1)); 
  //   // // for(int k=0; k< prediction_horizon+1; k++){
  //   // //     SX st = X(Slice(0, X.size1()), Slice(k));
  //   // //     obs_g(k) = -sqrt(pow((st(0)-obs(0,k)),2) + pow((st(1)-obs(1,k)),2) + pow((st(2)-obs(2,k)),2)) + (avoidance_distance + robot_diam); 
  //   // // }
  //   // // std::cout<< "=============1===============" << std::endl;
  //   // still_running = true;
  //   // bool init_kalman_filter = false;
  //   // Eigen::VectorXd k_x0(n_states);
  //   // k_x0 << 0, 0, 0, 0;
  //   // kf.init(0, k_x0);

  //   // clock_t t2, t1 = clock();
  //   // previos_pose<< (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);
  //   // int stuck_point = 0;
  //   // double previous_angle = 0;
  //   // double previous_error =0;
  //   // while(error > maximum_acceptable_error && mpciter < sim_time/delta_t){
  //   //     // std::cout<< "current error" << error << std::endl;
  //   //     if(error>previous_error){
  //   //       stuck_point++;
  //   //     }else{
  //   //       stuck_point--;
  //   //     }
  //   //     if(stuck_point<0){
  //   //       stuck_point = 0;
  //   //     }
  //   //     previous_error = error;
  //   //     if(stuck_point> fluctuation_length && error > 1.0){
  //   //       need_intermediate_goal = true;
  //   //       still_running = false;
  //   //       std::cout<< "Solver need intermediate goal..." <<  std::endl;
  //   //       return;
  //   //     }
  //   //     cout<< stuck_point << "  "  << error << endl;
  //   //       // if(stuck_point>fluctuation_length && error > 1.0){
  //   //       //   need_intermediate_goal = true;
  //   //       //   still_running = false;
  //   //       //   std::cout<< "Solver need intermediate goal..." <<  std::endl;
  //   //       //   return;
  //   //       // }

  //   //     t2 = clock();
  //   //     if((t2-t1) > delta_t){
  //   //       args["p"] = vertcat(x0, xs);
  //   //       args["x0"] = vertcat(reshape(X0.T(), n_states*(prediction_horizon+1), 1), reshape(U0.T(), n_controls*prediction_horizon, 1));
          
  //   //       // obs_length = (stuck_point <= 0)? 1: stuck_point*2;
  //   //       obs_length = 1;
  //   //       DM lbg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
  //   //       DM ubg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
  //   //       lbg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
  //   //       ubg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
  //   //       lbg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = -inf;
  //   //       ubg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = 0;
  //   //       args["lbg"] = lbg; 
  //   //       args["ubg"] = ubg;
  //   //       SX obs_g = SX::sym("obs_g",(prediction_horizon+1)*obs_length);
  //   //       // std::cout<< "=============3===============" << std::endl;
  //   //       int obs_cout=0;
  //   //       SX obj_cucost = 0;
  //   //       for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
  //   //         // std::cout<< "=============4===============" << std::endl;
  //   //         SX st = X(Slice(0, X.size1()), Slice(trj_i));
  //   //         Eigen::Vector3d pos((double)X0(trj_i,0), (double)X0(trj_i,1), (double)X0(trj_i,2));
  //   //         double obs_dis;
  //   //         // vector<Eigen::Vector3d> obs_vec = edt_env_->nearest_obstacles_to_current_pose(pos, obs_length);
  //   //         // for(auto obs : obs_vec){
  //   //         //   // std::cout<< obs.transpose() << " pose: "<< pos.transpose() << std::endl;
  //   //         //   obs_g(obs_cout) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + (avoidance_distance);
  //   //         //   obs_cout++;
  //   //         // }
  //   //         Eigen::Vector3d obs;
  //   //         edt_env_->get_close_obstacle(pos, obs, obs_dis);
            
  //   //         obj_cucost += 1.0/((pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + 0.0005);
  //   //         obs_g(trj_i) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + (avoidance_distance);
  //   //       }
  //   //       // std::cout<< "=============5===============" << std::endl;
  //   //       SX final_g = vertcat(g, obs_g);
  //   //       // std::cout<< "=============6===============" << std::endl;
  //   //       nlp_prob["g"] = final_g;
  //   //       nlp_prob["f"] = obj + obj_cucost;
  //   //       Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);
  //   //       // std::cout<< "=============7===============" << std::endl;
  //   //       res = solver(args);
  //   //       // std::cout<< "============9==============="<< res["x"] << std::endl;
  //   //       DM u = reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T();
  //   //       vector<double> u11(reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T());
  //   //       // vector<double> xx11(reshape(res["x"](Slice(0, (n_states*(N+1)))).T(), n_states, N+1).T());
  //   //       // xx1.push_back(xx11);
  //   //       vector<double> xx11(res["x"](Slice(0, (n_states*(prediction_horizon+1)))).T());
  //   //       xx1.push_back(xx11);
  //   //       std::vector<Eigen::Vector3d> vis_horizon;

  //   //       for(int kl=0; kl< int(xx11.size()/n_states); kl+=4){
  //   //         Eigen::Vector3d poss(xx11[kl], xx11[kl+1], xx11[kl+2]);
  //   //         vis_horizon.push_back(poss);
  //   //       }

          
  //   //       visualization_->drawPath(vis_horizon, 0.1, Eigen::Vector4d(0.2, 1 ,0.5, 1), 0);
  //   //       // std::cout<< res["x"](Slice(0, (n_states*(N+1)))).T() << std::endl;
  //   //       u_cl.push_back(u11);
  //   //       t.push_back(t0);

  //   //       DM current_control = u(Slice(0), Slice(0, u.size2())).T();
  //   //       // std::cout<< "======u: "<< u << std::endl;
  //   //       // std::cout<< "======current_control: "<< current_control << std::endl;
  //   //       DM current_pose = x0;
  //   //       // std::cout<< "============12==============="<< (double)current_pose(0) << "   "<< (double)current_control(1) <<  std::endl;
        
  //   //       if(force_terminate){
  //   //         still_running = false;
  //   //         std::cout<< "Solver has been interrupted..." <<  std::endl;
  //   //         return;
  //   //       }
  //   //       current_projected_pose.pose.pose.position.x = (double)current_pose(0);
  //   //       current_projected_pose.pose.pose.position.y = (double)current_pose(1);
  //   //       current_projected_pose.pose.pose.position.z = (double)current_pose(2);
  //   //       current_projected_pose.pose.pose.orientation.x = (double)current_pose(3);
  //   //       // current_projected_pose.twist.twist.linear.x = (double)(current_control(0)*cos(current_pose(3)) - current_control(1)*sin(current_pose(3)));
  //   //       // current_projected_pose.twist.twist.linear.y = (double)(current_control(0)*sin(current_pose(3)) + current_control(1)*cos(current_pose(3)));
          
          
  //   //       Eigen::VectorXd k_y(n_controls);
  //   //       std::vector<double> k_estimated_values;
  //   //       k_y << (double)current_control(0), (double)current_control(1), (double)current_control(2), (double)current_control(3);
  //   //       kf.update(k_y);
          
         
  //   //       // Eigen::VectorXd v2(3);
  //   //       // v2<< (double)current_control(0), (double)current_control(1), (double)current_control(2);
  //   //       // v2 = v1 + v2;
         
         
  //   //       // current_projected_pose.twist.twist.linear.x = (double)current_control(0);
  //   //       // current_projected_pose.twist.twist.linear.y = (double)current_control(1);
  //   //       // current_projected_pose.twist.twist.linear.z = (double)current_control(2);
  //   //       // current_projected_pose.twist.twist.angular.z = (double)current_control(3);
  //   //       current_projected_pose.twist.twist.linear.x = kf.state()[0];
  //   //       current_projected_pose.twist.twist.linear.y = kf.state()[1];
  //   //       current_projected_pose.twist.twist.linear.z = kf.state()[2];
  //   //       current_projected_pose.twist.twist.angular.z = kf.state()[3];
  //   //       pos_current_pos_pub.publish(current_projected_pose);

  //   //       // int index_end = xx11.size()-4;
  //   //       // Eigen::Vector3d v1((double)current_pose(0), (double)current_pose(1), (double)current_pose(2));
  //   //       // Eigen::Vector3d v2(xx11[index_end], xx11[index_end+1], xx11[index_end+2]);
  //   //       // cout << "================3" << endl;
  //   //       // double angle = acos(v1.dot(v2)/(v1.norm()*v2.norm()));
  //   //       // double diff_angle = std::abs(previous_angle - angle);
  //   //       // cout << "================4" << endl;
  //   //       // double dis = (v1-previos_pose).norm();
  //   //       // cumulative_sum_checker(dis);
  //   //       // previos_pose = v1;
  //   //       // cout << "================:"<< dis << endl;
  //   //       // cout<< cumulative_sum_checker << endl;

  //   //       // if(cumulative_sum_checker> max_fluctuatio_allowed && error > 1.0){
  //   //       // //   stuck_point++;
  //   //       // // }else{
  //   //       // //   stuck_point = 0;
  //   //       //   need_intermediate_goal = true;
  //   //       //   still_running = false;
  //   //       //   std::cout<< "Solver need intermediate goal..." <<  std::endl;
  //   //       //   return;
  //   //       // }
  //   //       // // if(stuck_point>fluctuation_length && error > 1.0){
  //   //       // //   need_intermediate_goal = true;
  //   //       // //   still_running = false;
  //   //       // //   std::cout<< "Solver need intermediate goal..." <<  std::endl;
  //   //       // //   return;
  //   //       // // }

         

  //   //       // std::cout<< "============13==============="<< std::endl;
  //   //       tuple<double, DM, DM> shiftted_val = shift(delta_t, t0, x0, u, f); 
  //   //       x0 = get<1>(shiftted_val);
  //   //       u = get<2>(shiftted_val);
  //   //       t0 = get<0>(shiftted_val);

  //   //       error = (double)(norm_2(x0-xs));
        
  //   //       X0 = reshape(res["x"](Slice(0, n_states*(prediction_horizon+1))).T(), n_states, prediction_horizon+1).T();
  //   //       // xx(Slice(0, xx.size1()),Slice(mpciter+2)) = x0;
  //   //       vector<double> xxxo(x0);
  //   //       // std::cout<<  "============14===============" << x0 << std::endl;
  //   //       xx0.push_back(xxxo);

  //   //       SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
  //   //       SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
  //   //       X0 = vertcat(x0_rest, x0_last);
  //   //       mpciter = mpciter + 1;
  //   //       t1 = clock();
  //   //     }else{
  //   //       usleep(5000);
  //   //     }
  //   // }

  //   // still_running = false;
  //   // need_intermediate_goal = false;
  //   // cout<< "Finish mpc solver...."<< endl;
  //   // // std::cout<< "Start saving files" << std::endl;
  //   // // string  stotage_location = "/dataset/data/";

  //   // // string file_name = stotage_location + "xx0.npy";
  //   // // save_double_vector(xx0, file_name, 1);

  //   // // file_name = stotage_location + "xx1.npy";
  //   // // std::cout<< xx1.size() << "   " << xx1[0].size() << std::endl;
  //   // // save_double_vector(xx1, file_name, 1);

  //   // // // file_name = stotage_location + "xx.npy";
  //   // // // save_dm(xx, file_name, 1);

  //   // // file_name = stotage_location + "u_cl.npy";
  //   // // save_double_vector(u_cl, file_name, 4);

  //   // // file_name = stotage_location + "t.npy";
  //   // // save_vector(t, file_name, 1);

  //   // // file_name = stotage_location + "xs.npy";
  //   // // save_dm(xs, file_name, 1);
  
  // }

  void NonLinearMPCOpt::setEnvironment(const EDTEnvironment::Ptr& env){
    edt_env_ = env;
  }


  void NonLinearMPCOpt::init(ros::NodeHandle& nh){
      node_ = nh;
      /* ---------- param ---------- */
      node_.param("sdf_map/origin_x", origin_(0), -20.0);
      node_.param("sdf_map/origin_y", origin_(1), -20.0);
      node_.param("sdf_map/origin_z", origin_(2), 0.0);

      node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
      node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
      node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

      node_.param("mpc_opt/delta_t", delta_t, 0.2);
      node_.param("mpc_opt/prediction_horizon", prediction_horizon, 5);
      node_.param("mpc_opt/estimation_horizon", estimation_horizon, 5);
      node_.param("mpc_opt/v_max", v_max, 0.4);
      node_.param("mpc_opt/v_min", v_min, -0.4);
      node_.param("mpc_opt/omega_max", omega_max, pi);
      node_.param("mpc_opt/omega_min", omega_min, -pi);
      node_.param("mpc_opt/robot_diam", robot_diam, 0.3);
      node_.param("mpc_opt/avoidance_distance", avoidance_distance, 0.3);
      node_.param("mpc_opt/maximum_acceptable_error", maximum_acceptable_error, 0.3);
      node_.param("mpc_opt/simulated_duration", simulated_duration, 40);
      node_.param("mpc_opt/fluctuation_length", fluctuation_length, 5);
      node_.param("mpc_opt/max_fluctuatio_allowed", max_fluctuatio_allowed, 1.4);
      node_.param("mpc_opt/obs_min_allowed_length", obs_min_allowed_length, 0.2);
      node_.param("mpc_opt/obs_max_allowed_length", obs_max_allowed_length, 10.0);
      node_.param("mpc_opt/collocation_degree", collocation_degree, 3);
      node_.param("mpc_opt/use_collocation", use_collocation, false);


      pos_current_pos_pub = node_.advertise<nav_msgs::Odometry>("/planning/current_state", 5);

      min_range_ = origin_;
      max_range_ = origin_ + map_size_;
  }

  tuple<double, SX, SX> NonLinearMPCOpt::shift(double T, double t0, SX x0, SX u, Function f){
      SX st = x0;
      SX con = u(Slice(0), Slice(0, u.size2())).T();
      SXDict f_in = {{"x", st}, {"u", con}};
      SXDict f_value = f(f_in);
      st = st + T*f_value["rhs"];
      x0 = st;
      t0 = t0 + T;
      SX u_rest = u(Slice(1, u.size1()), Slice(0, u.size2()));
      SX u_last = u(Slice(u.size1()-1, u.size1()), Slice(0, u.size2()));
      SX ui = vertcat(u_rest, u_last);
      tuple<double, SX, SX> next_state(t0, x0, ui);
      return next_state;
  }

}  // namespace hagen_planner
