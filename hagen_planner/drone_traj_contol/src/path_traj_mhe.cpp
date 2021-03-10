#include "path_traj_mhe.h"

namespace hagen_planner
{
  

  NonLinearMHEOpt::NonLinearMHEOpt()
  {
    
  }

  void NonLinearMHEOpt::solver_init(){
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

      rhs = vertcat(v_x*cos(theta)-v_y*sin(theta), v_y*cos(theta) + v_x*sin(theta), v_z, omega);
      f = Function("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});

      con_cov = SX::sym("con_cov", n_controls, 1);
      con_cov(0) = 234;
      con_cov(1) = 453;
      con_cov(2) = 345;
      con_cov(3) = (pi/180)*2.0;
      con_cov = SX::diag(con_cov);

      meas_cov = SX::sym("meas_cov", n_controls,1);
      meas_cov(0) = 200;
      meas_cov(1) = 100;
      meas_cov(2) = 200;
      meas_cov(3) = (pi/180)*2.0;
      meas_cov = SX::diag(meas_cov);

      V = SX::sym("con_cov1", n_controls, 1);
      V(0) = 1;
      V(1) = 1;
      V(2) = 1;
      V(3) = (pi/180)*2.0;
      V = SX::diag(V);

      W = SX::sym("meas_cov1", n_controls,1);
      W(0) = 1;
      W(1) = 1;
      W(2) = 1;
      W(3) = (pi/180)*2.0;
      W = SX::diag(W);

      mhe_estimator_init();
  }

  void NonLinearMHEOpt::mhe_estimator_init(){
    
    // std::cout<< "==1" << std::endl;
    SX X = SX::sym("X", n_states, estimation_horizon+1);
    SX U = SX::sym("U", n_controls, estimation_horizon);
    SX P = SX::sym("P", n_controls, (estimation_horizon+1) + estimation_horizon);
    // std::cout<< "==2" << std::endl;
    SX obj = 0;
    mhe_g = SX::sym("mhe_g", estimation_horizon, n_states);
    // std::cout<< "==3" << std::endl;
    for(int k=0; k< estimation_horizon+1; k++){
      SX st = X(Slice(0, X.size1()), Slice(k));
      // SX h_x = X_sensory(Slice(0, X_sensory.size1()), Slice(k));
      SX y_tilda = P(Slice(0, P.size1()), Slice(k));
      obj += mtimes((y_tilda-st).T(), mtimes(V, (y_tilda-st)));
    }
    // std::cout<< "==4" << std::endl;
    for(int k=0; k< estimation_horizon; k++){
      SX con = U(Slice(0, U.size1()), Slice(k));
      SX u_tilda = P(Slice(0, P.size1()), Slice(estimation_horizon+k));
      obj += mtimes((u_tilda-con).T(), mtimes(W, (u_tilda-con)));
    }
    // std::cout<< "==5" << std::endl;
    for(int k=0; k<estimation_horizon; k++){
      SX st = X(Slice(0, X.size1()), Slice(k));
      // std::cout<< "==5" << st.size() << std::endl;
      SX con = U(Slice(0, U.size1()), Slice(k));
      //  std::cout<< "==5" << con.size() << std::endl;
      SX st_next = X(Slice(0, X.size1()), Slice(k+1));
      //  std::cout<< "==5" << st_next.size() << std::endl;
      SXDict f_in = {{"x", st}, {"u", con}};
      // std::cout<< "==5" << st_next.size() << std::endl;
      SXDict f_value = f(f_in);
      //  std::cout<< "==5" << st_next.size() << std::endl;
      SX st_next_euler = st + delta_t*f_value["rhs"];
      //  std::cout<< "==5" << st_next.size() << std::endl;
      mhe_g(Slice(k), Slice(0, mhe_g.size2())) = (st_next - st_next_euler);
    }
    // std::cout<< "==6" << std::endl;
    mhe_g = reshape(mhe_g, n_states*(estimation_horizon), 1);
    // std::cout<< "==6" << std::endl;
    SX OPT_variables = vertcat(reshape(X, n_states*(estimation_horizon+1), 1), reshape(U, n_controls*estimation_horizon, 1));
    // std::cout<< "==6" << std::endl;  
    opts_mhe["ipopt.tol"] = 1e-4;
    opts_mhe["ipopt.max_iter"] = 100;
    opts_mhe["expand"] = true;
    opts_mhe["ipopt.print_level"] = 0;
    opts_mhe["print_time"] = 0;
    opts_mhe["ipopt.acceptable_tol"] = 1e-8;
    //  std::cout<< "==6" << std::endl;
    nlp_prob_mhe = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g", mhe_g}};
    // std::cout<< "==6" << std::endl;
    solver_mhe = nlpsol("nlpsol", "ipopt", nlp_prob_mhe, opts_mhe);
    // std::cout<< "==6" << std::endl;
    DM lbg = DM(1, n_states*(estimation_horizon));
    DM ubg = DM(1, n_states*(estimation_horizon));

    lbg(Slice(0), Slice(0, n_states*(estimation_horizon))) = 0;
    ubg(Slice(0), Slice(0, n_states*(estimation_horizon))) = 0;

    DM lbx = DM(n_states*(estimation_horizon+1)+n_controls*estimation_horizon, 1);
    DM ubx = DM(n_states*(estimation_horizon+1)+n_controls*estimation_horizon, 1);
// std::cout<< "==6" << std::endl;
    lbx(Slice(0, n_states*(estimation_horizon+1), n_states), Slice(0)) = min_range_[0];
    ubx(Slice(0, n_states*(estimation_horizon+1), n_states), Slice(0)) = max_range_[0];
    lbx(Slice(1, n_states*(estimation_horizon+1), n_states), Slice(0)) = min_range_[1];
    ubx(Slice(1, n_states*(estimation_horizon+1), n_states), Slice(0)) = max_range_[1];
    lbx(Slice(2, n_states*(estimation_horizon+1), n_states), Slice(0)) = min_range_[2];
    ubx(Slice(2, n_states*(estimation_horizon+1), n_states), Slice(0)) = max_range_[2];
    lbx(Slice(3, n_states*(estimation_horizon+1), n_states), Slice(0)) = -pi/2;
    ubx(Slice(3, n_states*(estimation_horizon+1), n_states), Slice(0)) = pi/2;
      // std::cout<< "==6" << std::endl;
    lbx(Slice(n_states*(estimation_horizon+1), n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_states*(estimation_horizon+1), n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_states*(estimation_horizon+1)+1, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_states*(estimation_horizon+1)+1, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_states*(estimation_horizon+1)+2, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_states*(estimation_horizon+1)+2, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_states*(estimation_horizon+1)+3, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = omega_min;
    ubx(Slice(n_states*(estimation_horizon+1)+3, n_states*(estimation_horizon+1) + n_controls*estimation_horizon, n_controls), Slice(0)) = omega_max;
// std::cout<< "==6" << std::endl;
    args_mhe["lbx"] = lbx;
    args_mhe["ubx"] = ubx;
    args_mhe["lbg"] = lbg; 
    args_mhe["ubg"] = ubg;

    x0_mhe = std::make_shared<boost::circular_buffer<double>>((estimation_horizon+1)*n_states);
    u0_mhe = std::make_shared<boost::circular_buffer<double>>(estimation_horizon*n_controls);
    p_x_mhe = std::make_shared<boost::circular_buffer<double>>((estimation_horizon+1)*n_states);
    p_u_mhe = std::make_shared<boost::circular_buffer<double>>(estimation_horizon*n_controls);
    current_estimated_state =  std::make_shared<vector<double>>();
    current_estimated_state_previous = std::make_shared<vector<double>>();
// std::cout<< "==6" << std::endl;
    x0_mhe_init = DM::zeros(n_states, estimation_horizon+1);
    u0_mhe_init = DM::zeros(n_controls, estimation_horizon);
    p_mhe_init = DM::zeros(n_controls, (estimation_horizon+1) + estimation_horizon);

    x0_mhe_init = DM::zeros(n_states, estimation_horizon+1);
    u0_mhe_init = DM::zeros(n_controls, estimation_horizon);
    p_mhe_init = DM::zeros(n_controls, (estimation_horizon+1) + estimation_horizon);

    current_estimated_state->push_back(0);
    current_estimated_state->push_back(0);
    current_estimated_state->push_back(0);
    current_estimated_state->push_back(0);
    current_estimated_state_previous = current_estimated_state;
    // std::cout<< "==6" << std::endl;

  }

  double NonLinearMHEOpt::getYawFromQuat(const geometry_msgs::Quaternion &data)
  {
      tf::Quaternion quat(data.x, data.y, data.z, data.w);
      tf::Matrix3x3 m(quat);
      double_t roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
  }

  void NonLinearMHEOpt::get_current_state(const hagen_msgs::PoseCommand& odom_){
        // cout<< "message --->>" << endl;
        current_projected_pose = odom_;
  }


  void NonLinearMHEOpt::actual_vehilce_odometry(const nav_msgs::Odometry& msg) {
      odom = msg;
      if(stop_estimating){
        init_buffer = false;
        return;
      }

      x0_mhe->push_back(odom.pose.pose.position.x);
      x0_mhe->push_back(odom.pose.pose.position.y);
      x0_mhe->push_back(odom.pose.pose.position.z);     
      x0_mhe->push_back(getYawFromQuat(odom.pose.pose.orientation));
      u0_mhe->push_back(odom.twist.twist.linear.x);
      u0_mhe->push_back(odom.twist.twist.linear.y);
      u0_mhe->push_back(odom.twist.twist.linear.z);     
      u0_mhe->push_back(odom.twist.twist.angular.z); 

      auto current_desired = current_projected_pose;
      p_x_mhe->push_back(current_desired.position.x);
      p_x_mhe->push_back(current_desired.position.y);
      p_x_mhe->push_back(current_desired.position.z);     
      p_x_mhe->push_back(current_desired.yaw);
      p_u_mhe->push_back(current_desired.state_vector.x);
      p_u_mhe->push_back(current_desired.state_vector.y);
      p_u_mhe->push_back(current_desired.state_vector.z);     
      p_u_mhe->push_back(current_desired.yaw_dot); 

      if(x0_mhe->size() != x0_mhe->capacity()){
        // cout<< "Still not yet fillted x0_mhe" << endl;
        init_buffer = false;
        return;
      }

      if(u0_mhe->size() != u0_mhe->capacity()){
        // cout<< "Still not yet fillted u0_mhe" << endl;
        init_buffer = false;
        return;
      }

      if(p_x_mhe->size() != p_x_mhe->capacity()){
        // cout<< "Still not yet fillted p_x_mhe" << endl;
        init_buffer = false;
        return;
      }

      if(p_u_mhe->size() != p_u_mhe->capacity()){
        init_buffer = false;
        return;
      }
     
      // if(!init_buffer){
        int k = 0;
        int state_size =  (estimation_horizon+1)*n_states;
        int control_size =  (estimation_horizon)*n_states;
        cout<< "start: " << (*x0_mhe)[0]  << "," << (*x0_mhe)[1] << "," << (*x0_mhe)[2] << endl;
        cout<< "end: " << (*x0_mhe)[control_size]  << "," << (*x0_mhe)[control_size+1] << "," << (*x0_mhe)[control_size+2] << endl;
       
        for(int i=0; i< control_size; i+=4){
          x0_mhe_init(0,k) = (*x0_mhe)[i];
          x0_mhe_init(1,k) = (*x0_mhe)[i+1];
          x0_mhe_init(2,k) = (*x0_mhe)[i+2];
          x0_mhe_init(3,k) = (*x0_mhe)[i+3];
          u0_mhe_init(0,k) = (*u0_mhe)[i];
          u0_mhe_init(1,k) = (*u0_mhe)[i+1];
          u0_mhe_init(2,k) = (*u0_mhe)[i+2];
          u0_mhe_init(3,k) = (*u0_mhe)[i+3];
          k++;
        }
        int last_index = control_size;
        x0_mhe_init(0,k) = (*x0_mhe)[last_index];
        x0_mhe_init(1,k) = (*x0_mhe)[last_index+1];
        x0_mhe_init(2,k) = (*x0_mhe)[last_index+2];
        x0_mhe_init(3,k) = (*x0_mhe)[last_index+3];
        k = 0;
        for(int i=0; i< state_size; i+=4){
            p_mhe_init(0,k) = (*p_x_mhe)[i];
            p_mhe_init(1,k) = (*p_x_mhe)[i+1];
            p_mhe_init(2,k) = (*p_x_mhe)[i+2];
            p_mhe_init(3,k) = (*p_x_mhe)[i+3];
            k++;
        }
        int fg = state_size-4;
        cout<< "end real: " << (*p_x_mhe)[fg]  << "," << (*p_x_mhe)[fg+1] << "," << (*p_x_mhe)[fg+2] << endl;
        for(int i=0; i< control_size; i+=4){
            p_mhe_init(0,k) = (*p_u_mhe)[i];
            p_mhe_init(1,k) = (*p_u_mhe)[i+1];
            p_mhe_init(2,k) = (*p_u_mhe)[i+2];
            p_mhe_init(3,k) = (*p_u_mhe)[i+3];
            k++;
        }
        init_buffer = true;
        
      // }else{
      //   //   cout<< "||====5" << p_mhe_init.size()  << "<----->" << estimation_horizon << endl;
      //   DM p_rest_x = p_mhe_init(Slice(0, n_controls), Slice(0, estimation_horizon));
      //   //   cout<< "||====5" << p_rest_x.size() << endl;
      //   DM p_rest_u = p_mhe_init(Slice(0, n_controls), Slice(estimation_horizon+1, estimation_horizon+1+estimation_horizon-1));
      //   //   cout<< "||====5" << p_rest_u.size() << endl;
      //   DM px0_new = DM(n_states, 1);
      //   DM pu0_new = DM(n_controls, 1);
      //   //   cout<< "====4" << endl;
      //   DM x0_rest = x0_mhe_init(Slice(0, n_states), Slice(0, estimation_horizon+1-1));
      //   //   cout<< "====5" << endl;
      //   DM u0_rest = u0_mhe_init(Slice(0, n_controls), Slice(0, estimation_horizon-1));
      //   //   cout<< "====6" << endl;
      //   // DM p_rest_x = p_mhe_init(Slice(0, n_controls), Slice(0, estimation_horizon));
      //   // DM p_rest_u = p_mhe_init(Slice(0, n_controls), Slice(estimation_horizon+1, estimation_horizon+1+estimation_horizon-1));
      //   DM x0_new = DM(n_states, 1);
      //   DM u0_new = DM(n_controls, 1);
      //   // DM px0_new = DM(n_states, 1);
      //   // DM pu0_new = DM(n_controls, 1);
      //   //   cout<< "====7" << endl;
      //   for (int i=0; i< n_states; i++){
      //     if(x0_mhe->empty()){
      //           cout<< "x0_mhe is empty" << endl;
      //     }
      //     if(u0_mhe->empty()){
      //       cout<< "u0_mhe is empty" << endl;
      //     }
      //     x0_new(i) = x0_mhe->front();
      //     x0_mhe->pop_front();
      //     u0_new(i) = u0_mhe->front();
      //     u0_mhe->pop_front();

      //     // px0_new(i) = p_x_mhe->front();
      //     // p_x_mhe->pop_front();
      //     // pu0_new(i) = p_u_mhe->front();
      //     // p_u_mhe->pop_front();
      //   }
        //   cout<< "====8" << endl;
        // cout<< "====111" << endl;
      //   x0_mhe_init = horzcat(x0_new, x0_rest);
      //   // cout<< "====122" << endl;
      //   u0_mhe_init = horzcat(u0_new, u0_rest);
      //   // cout<< "====133" << endl;
      //   // cout<< horzcat(px0_new, p_rest_x, pu0_new, p_rest_u).size() << endl;
      //   // p_mhe_init = horzcat(px0_new, p_rest_x, pu0_new, p_rest_u);
      //   // cout<< horzcat(px0_new, p_rest_x, pu0_new, p_rest_u).size() << endl;

      //   //  cout<< "||====6 rest " << p_rest_u.size() << endl;
      //   for (int i=0; i< n_states; i++){
      //       if(p_x_mhe->empty()){
      //         cout<< "p_x is empty" << endl;
      //       }
      //       if(p_u_mhe->empty()){
      //         cout<< "p_u_mhe is empty" << endl;
      //       }
      //       px0_new(i) = p_x_mhe->front();
      //       p_x_mhe->pop_front();
      //       pu0_new(i) = p_u_mhe->front();
      //       p_u_mhe->pop_front();
      //   }
      //   // cout<< "||====7" << endl;
      //   //   cout<< horzcat(px0_new, p_rest_x, pu0_new, p_rest_u).size() << endl;
      //   p_mhe_init = horzcat(px0_new, p_rest_x, pu0_new, p_rest_u);
      //   //   cout<< horzcat(px0_new, p_rest_x, pu0_new, p_rest_u).size() << endl;
      //   //   cout<< "||====8" << endl;
      // }
      x0_mhe_init_previous = x0_mhe_init;
      u0_mhe_init_previous = u0_mhe_init;
      p_mhe_init_previous = p_mhe_init;
      mhe_estimator();
      // cout<< "||====9" << endl;
      // if(init_buffer_p){
      // }
  }

  void NonLinearMHEOpt::mhe_estimator(){
    // static int counter = 0;

    // std::cout<< "Start saving files" << std::endl;
    // string  stotage_location = "/dataset/data/";
    // string file_name = stotage_location + std::to_string(counter) + "p_mhe_init_previous.npy";
    // cout<< "before ---->1" << endl;
    cout<< "before ---->: " << p_mhe_init_previous.size() << endl;
    args_mhe["p"] = p_mhe_init_previous;
    // save_dm(p_mhe_init_previous, file_name, 1);


    std::vector<Eigen::Vector3d> vis_horizon_p;
    // for(int kl=0; kl< (estimation_horizon+1); kl++){
    //       Eigen::Vector3d poss((double)p_mhe_init_previous(0,kl), (double)p_mhe_init_previous(1,kl), (double)p_mhe_init_previous(2,kl));
    //       vis_horizon_p.push_back(poss);
    // }
    Eigen::Vector3d poss((double)p_mhe_init_previous(0, estimation_horizon), (double)p_mhe_init_previous(1, estimation_horizon), (double)p_mhe_init_previous(2, estimation_horizon));
    vis_horizon_p.push_back(poss);
    visualization_->drawPath(vis_horizon_p, 0.9, Eigen::Vector4d(1, 0.0 ,0.5, 1), 679);

    // cout<< "before ---->2" << endl;
    args_mhe["x0"] = vertcat(reshape(x0_mhe_init_previous.T(), n_states*(estimation_horizon+1), 1), reshape(u0_mhe_init_previous.T(), n_controls*estimation_horizon, 1));
    // cout<< "before ---->" << endl;
    // file_name = stotage_location + std::to_string(counter) + "x0_u0_mhe_init_previous.npy";
    // save_dm(args_mhe["x0"], file_name, 1);
    
    res_mhe = solver_mhe(args_mhe);
    // cout<< "after ---->" << endl;
    vector<double> estimated_controls(reshape(res_mhe["x"](Slice(n_states*(estimation_horizon+1), res_mhe.at("x").size1())).T(), n_controls, estimation_horizon).T());
    // cout<< "after ---->1" << endl;
    vector<double> estimated_pose(res_mhe["x"](Slice(0, (n_states*(estimation_horizon+1)))).T());
    // cout<< "after ---->2" << estimated_pose<< endl;
    std::vector<Eigen::Vector3d> vis_horizon;
    // for(int kl=0; kl< int(estimated_pose.size()/n_states); kl+=4){
    //       Eigen::Vector3d poss(estimated_pose[kl], estimated_pose[kl+1], estimated_pose[kl+2]);
    //       vis_horizon.push_back(poss);
    // }
    int kl = estimated_pose.size()-4;
    Eigen::Vector3d poss1(estimated_pose[kl], estimated_pose[kl+1], estimated_pose[kl+2]);
    vis_horizon.push_back(poss1);
    // file_name = stotage_location + std::to_string(counter) +  "estimated_pose.npy";
    // save_vector(estimated_pose, file_name, 1);
    // int control_index = estimated_controls.size()-4;
    int control_index = estimated_controls.size()-4;
    Eigen::VectorXd state_vec(4);
  
    state_vec << estimated_controls[control_index], estimated_controls[control_index+1]
              , estimated_controls[control_index+2], estimated_controls[control_index+3];
    double current_yaw = estimated_controls[control_index+3];

    // std::ofstream ofs;
    // ofs.open("estimated_controls.txt", std::ofstream::out | std::ofstream::app);
    // ofs << estimated_controls[control_index] << ","<< estimated_controls[control_index+1] << ","<< estimated_controls[control_index+2] << "," << estimated_controls[control_index+3] << "\n";
    // ofs.close();
    
    // std::ofstream ofs1;
    // ofs1.open("actual_controls.txt", std::ofstream::out | std::ofstream::app);
    // ofs1 << odom.twist.twist.linear.x << ","<< odom.twist.twist.linear.y << ","<< odom.twist.twist.linear.z << "," << odom.twist.twist.angular.z << "\n";
    // ofs1.close();

    // std::ofstream ofs2;
    // ofs2.open("estimated_pose.txt", std::ofstream::out | std::ofstream::app);
    // ofs2 << odom.twist.twist.linear.x << ","<< odom.twist.twist.linear.y << ","<< odom.twist.twist.linear.z << "," << odom.twist.twist.angular.z << "\n";
    // ofs2.close();

    // std::ofstream ofs3;
    // ofs3.open("actual_pose.txt", std::ofstream::out | std::ofstream::app);
    // ofs3 << odom.pose.pose.position.x << ","<< odom.pose.pose.position.y << ","<< odom.pose.pose.position.z  << "\n";
    // ofs3.close();
 
    current_estimated_state->clear();
    current_estimated_state->push_back((double)(state_vec(0)*cos(current_yaw) - state_vec(1)*sin(current_yaw)));
    current_estimated_state->push_back((double)(state_vec(0)*sin(current_yaw) + state_vec(1)*cos(current_yaw)));
    current_estimated_state->push_back(estimated_controls[control_index+2]);
    current_estimated_state->push_back(estimated_controls[control_index+3]);
    current_estimated_state_previous = current_estimated_state;
    visualization_->drawPath(vis_horizon, 0.9, Eigen::Vector4d(1, 1 ,0.5, 1), 678);
    // counter += 1;
  }

  // void NonLinearMHEOpt::save_dm(DM trees, std::string file_name, int index){
  //       std::vector<double> edges; 
  //       int count = 0;
  //       for(int k=0; k< trees.size1(); k++){
  //         for(int j=0; j< trees.size2(); j++){
  //             edges.push_back((double)trees(k,j));
  //         }
  //       }
  //       cnpy::npy_save(file_name, &edges[0],{(unsigned int)trees.size1(), (unsigned int)trees.size2()/index, (unsigned int)index},"w");
  // }
  
  void NonLinearMHEOpt::start_moving(std_msgs::Empty mes){
    // cout<< "start moving" << endl;
    stop_estimating = false;
  }
  
  void NonLinearMHEOpt::init(ros::NodeHandle& nh){
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
      node_.param("mpc_opt/omega_max", omega_max, pi/4);
      node_.param("mpc_opt/omega_min", omega_min, -pi/4);
      node_.param("mpc_opt/robot_diam", robot_diam, 0.3);
      node_.param("mpc_opt/avoidance_distance", avoidance_distance, 0.3);
      node_.param("mpc_opt/maximum_acceptable_error", maximum_acceptable_error, 0.3);
      node_.param("mpc_opt/simulated_duration", simulated_duration, 40);

      visualization_.reset(new PlanningVisualization(nh));

      // pos_current_pos_pub = node_.advertise<nav_msgs::Odometry>("/planning/current_state", 5);
      pos_current_sub = node_.subscribe("/planning/pos_cmd", 1, &NonLinearMHEOpt::get_current_state, this);
      // odometry_pose_sub = node_.subscribe("/odom_world", 10, &NonLinearMHEOpt::actual_vehilce_odometry, this);
      odometry_pose_sub = node_.subscribe("/odom_world", 10, &NonLinearMHEOpt::actual_vehilce_odometry, this);
      restart_estimator_sub = node_.subscribe("/planning/stop_moving", 10, &NonLinearMHEOpt::restart_estimator, this);
      wait_for_goal_sub = node_.subscribe("/planning/start_moving", 10, &NonLinearMHEOpt::start_moving, this);

      min_range_ = origin_;
      max_range_ = origin_ + map_size_;
      solver_init();
      cout<< "---------------------pos_current_pos_pub----------------------->." << endl;
      // estimator_thread = startEstimatorThread();
  }

  // std::thread NonLinearMHEOpt::startEstimatorThread() {
  //   return std::thread([&] { estimaorThread(); });
  // }

  // void NonLinearMHEOpt::estimaorThread() {
  //   std::cout<< "Starting the estimator thread..." << std::endl;
  //   while(is_allowed_for_execution_estimator){
  //     // std::unique_lock<std::mutex> lk(lock_on_estimator);
  //     // condition_on_solver.wait(lk, [&]{return granted_execution_estimator;});
  //     if(init_buffer){
  //       mhe_estimator();
  //     }else{
  //       usleep(4000);
  //     }
  //     // lk.unlock();
  //     // condition_on_solver.notify_all();

  //     // std::cout<< "Start saving files" << std::endl;
  //     // string  stotage_location = "/dataset/data/";
  //     // string file_name = stotage_location + "regulated.npy";
  //     // save_double_vector(trees, file_name, 4);
  //     // file_name = stotage_location + "desired.npy";
  //     // save_double_vector(trees_real, file_name, 4);
  //   }
  //   return;
  // }

  
  void NonLinearMHEOpt::restart_estimator(std_msgs::Empty msg) {
            // cout<< "clearing data" << endl;
            x0_mhe->clear();
            u0_mhe->clear();
            p_u_mhe->clear();
            p_x_mhe->clear();
            stop_estimating = true;      
            double current_yaw = (*current_estimated_state)[3];
            current_estimated_state->clear();
            current_estimated_state->push_back(0);
            current_estimated_state->push_back(0);
            current_estimated_state->push_back(0);
            current_estimated_state->push_back(current_yaw);
            current_estimated_state_previous = current_estimated_state;


  }

  tuple<double, SX, SX> NonLinearMHEOpt::shift(double T, double t0, SX x0, SX u, Function f){
      SX st = x0;
      SX con = u(Slice(0), Slice(0, u.size2())).T();
      SXDict f_in = {{"x", st}, {"u", con}};
      SXDict f_value = f(f_in);
      st = st + T*f_value["rhs"];
      x0 = st;
      t0 = t0 + T;
      SX u_rest = u(Slice(1, u.size1()), Slice(0, u.size2()));
      SX u_last = u(Slice(u.size1()-1, u.size1()), Slice(0, u.size2()));
      SX u0 = vertcat(u_rest, u_last);
      tuple<double, SX, SX> next_state(t0, x0, u0);
      return next_state;
  }


  // void NonLinearMHEOpt::save_vector(vector<double> trees, std::string file_name, int index){
  //       std::vector<double> edges; 
  //       int count = 0;
  //       for(int k=0; k< trees.size(); k++){
  //             edges.push_back(trees[k]);
  //       }
  //       cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)trees.size(), (unsigned int)index},"w");
  // }

}  // namespace hagen_planner


using namespace hagen_planner;

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "hagen_planner_node");
//   ros::NodeHandle node;
//   ros::NodeHandle nh("~");
//   NonLinearMHEOpt fsm;
//   fsm.init(nh);
//   ros::spin();
//   return 0;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_mhe_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(30);
    NonLinearMHEOpt fsm;
    fsm.init(n);

    double goal_timer = 0.0;
    double pose_timer = 0.0;
    double goal_lost_time = 1.0;
    double pose_lost_time = 0.5;
    double print_delay = 3.0;
    double print_timer = 0.;
    const int queue_size = 10;
    double goal_yaw = 0.0;

    ros::Publisher vel_pub, pub_marker;
    ros::Subscriber goalSub, navPosSub, navVelSub, stateSub, exStateSub, velFieldSub, altSonarSub, goal_ps_Sub
        , stopping_sub, starting_sub;
   
    // vel_pub = n.advertise<geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 10);
    
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
        
        auto current_control = *(fsm.current_estimated_state_previous);
        ctr_msg.twist.linear.x = current_control[0];
        ctr_msg.twist.linear.y = current_control[1];
        ctr_msg.twist.linear.z = current_control[2];
        ctr_msg.twist.angular.z = current_control[3];
        
        // if (pose_timer < pose_lost_time){
          //  cout<< current_control << endl;
        // vel_pub.publish(ctr_msg);
        // }else {
        //     if (print_timer > print_delay) {
        //         //ROS_INFO("lost goal callback: %f %f %f %f", goal_timer, goal_timer, pose_timer, pose_timer);
        //         print_timer = 0;
        //     }
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }
    geometry_msgs::TwistStamped ctr_msg_sd;
    vel_pub.publish(ctr_msg_sd);
    ROS_INFO("shutdown");
    return 0;
}
