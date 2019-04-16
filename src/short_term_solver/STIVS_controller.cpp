#include "STIVS_controller.h"

using namespace std;

//namespace shortTerm{

  stivsController::stivsController(const std::string& yaml_file) 
                                : BaseibvsController(yaml_file){

    W_.setZero();
    WN_.setZero();
    input_.setZero();
    state_.setZero();
    reference_.setZero();
    referenceN_.setZero();
  }

  stivsController::~stivsController(){}

  bool stivsController::setCommandPose(const nav_msgs::Odometry odom_msg){

    if( ( mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position).head(2) - pObst_vert1 ).norm() < .5 ||
        ( mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position).head(2) - pObst_vert2 ).norm() < .5 ||
        ( Eigen::Vector2d(mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position)(0),mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position)(1)) - pObst_horiz ).norm() < .5 ){
            std::cout << FRED("Trajectory Point not allowed! Too much close to the obstacle!\n\n");
      return false;
    }

    mav_msgs::eigenOdometryFromMsg(odom_msg, &trajectory_point);  
    computeDesiredState(); 

    std::cerr << FBLU("Short Term Final State Set to: ") << trajectory_point.position_W.transpose() << 
                 " " << trajectory_point.orientation_W_B.w() << " " << trajectory_point.orientation_W_B.vec().transpose() << "\n";  
          
    return true;
  }

  void stivsController::computeDesiredState(){

    Eigen::Vector3d Xr = pT_W_ - ( pCam_B_ + trajectory_point.position_W );
    trajectory_point.position_W(2) = Xr(0) * ( 1 / ( cos(0.21) ) ) * sin(0.21) + pT_W_(2);

    float yaw = 0.f;
    if( fabs(Xr(1)) > 1e-3 )
      yaw = atan2( Xr(1), Xr(0) );  

    Eigen::Quaterniond q_des( Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ) );
    trajectory_point.orientation_W_B = Eigen::Quaterniond ( Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ) );;
  }


  void stivsController::calculateRollPitchYawRateThrustCommands(Eigen::Vector4d& command_roll_pitch_yawrate_thrust_)
  {
      
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);
    
    for (size_t i = 0; i < ACADO_N; i++) {
      // Possibility to add here the FeedForward Term in "reference_" velocities 
      reference_.block(i, 0, 1, ACADO_NY) << trajectory_point.position_W.transpose(), 
                                             0, 0, 0, 
                                             0, 0, trajectory_point.getYaw(), 
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0, 0;

    }    
    
    referenceN_ << trajectory_point.position_W.transpose(), 0, 0, 0, 0, 0, trajectory_point.getYaw(), 0, 0;

    Eigen::Matrix<double, ACADO_NX, 1> x_0;
    x_0 << odometry.position_W, odometry.getVelocityWorld(), euler_angles, 0;

    Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(shortTermAcadoVariables.x0)) = x_0;
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(shortTermAcadoVariables.y)) = reference_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(shortTermAcadoVariables.yN)) = referenceN_.transpose();
    
    //acado_timer t;
    acado_preparationStep();
    //acado_tic( &t );
    auto start = chrono::steady_clock::now();

    for(iter = 0; iter < NUM_STEPS; ++iter)
    {
      acado_feedbackStep( );

      if(acado_getKKT() < KKT_THRESHOLD)
        break;

      acado_preparationStep();
    }
    //real_t te = acado_toc( &t );
    auto end = chrono::steady_clock::now();
    solve_time = chrono::duration_cast<chrono::microseconds>(end - start).count();

    if(verbosity_ == Verbosity_Level::VERBOSE){   
      std::cout << FBLU("Short Term time elapsed: ") << solve_time << "microsecs\n";
      std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
    } else if (verbosity_ == Verbosity_Level::FEMMINA_CAGACAZZI ) {
      std::cout << FBLU("Short Term time of one real-time iteration: ") <<  solve_time << " microsecs\n";
      std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
      std::cout << FBLU("Short Term Differential Variables: ") << "\n";
      printDifferentialVariables();
      std::cout << FBLU("Short Term Control Variables: ") << "\n";
      printControlVariables();
    }

    //real_t time_elapsed = iter * te;
    //sendToLog( time_elapsed );

    ComputeIntegralAction();  

    command_roll_pitch_yawrate_thrust_(0) = shortTermAcadoVariables.u[0]; // + integral_action_(0) * integral_action_weights_(0);
    command_roll_pitch_yawrate_thrust_(1) = shortTermAcadoVariables.u[1]; // + integral_action_(1) * integral_action_weights_(1);
    command_roll_pitch_yawrate_thrust_(2) = 2*shortTermAcadoVariables.u[3] - yaw_rate_damping*odometry.getYawRate() + integral_action_(3) * integral_action_weights_(3);
    command_roll_pitch_yawrate_thrust_(3) = shortTermAcadoVariables.u[2] + integral_action_(2) * integral_action_weights_(2);
                                          
  }

  void stivsController::printDifferentialVariables(){
    int i, j;
    for (i = 0; i < ACADO_N + 1; ++i)
    {
      for (j = 0; j < ACADO_NX; ++j)
        std::cout << shortTermAcadoVariables.x[i * ACADO_NX + j] << " ";
      std::cout << "\n";
    }
    std::cout << "\n\n";
  }

  void stivsController::printControlVariables(){
    int i, j;
    for (i = 0; i < ACADO_N; ++i)
    {
      for (j = 0; j < ACADO_NU; ++j)
        std::cout << shortTermAcadoVariables.u[i * ACADO_NU + j] << " ";
      std::cout << "\n";
    }
    std::cout << "\n\n";
  }

  bool stivsController::InitializeController()
  {
    
    acado_initializeSolver();

    W_.block(0, 0, 3, 3) = q_position_.asDiagonal();
    W_.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
    W_.block(6, 6, 3, 3) = q_attitude_.asDiagonal();
    W_.block(9, 9, 3, 3) = q_command_.head(3).asDiagonal();
    W_.block(12, 12, 2, 2) = q_target_.asDiagonal();
    W_(14,14) = 5;
    W_(15,15) = 5;
    W_(16,16) = 5;
    W_(17,17) = 50;
    W_(18,18) = 5;

    WN_.block(0, 0, 3, 3) = qf_position_.asDiagonal();
    WN_.block(3, 3, 3, 3) = qf_velocity_.asDiagonal();
    WN_.block(6, 6, 3, 3) = qf_attitude_.asDiagonal();
    WN_.block(9, 9, 2, 2) = qf_target_.asDiagonal();
    
    if(true){
      std::cout << FBLU("Short Term controller W matrix: ") << "\n";    
      std::cout << W_.diagonal().transpose() << "\n" << "\n";
      std::cout << FBLU("Short Term controller WN matrix: ") << "\n";   
      std::cout << WN_.diagonal().transpose() << "\n" << "\n";
    }

    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(shortTermAcadoVariables.W)) = W_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(shortTermAcadoVariables.WN)) = WN_.transpose();
      
    for (size_t i = 0; i < ACADO_N; ++i) {
      
      shortTermAcadoVariables.lbValues[ACADO_NU * i] = roll_ref_bnds_(0);        // min roll
      shortTermAcadoVariables.lbValues[ACADO_NU * i + 1] = pitch_ref_bnds_(0);   // min pitch
      shortTermAcadoVariables.lbValues[ACADO_NU * i + 2] = thrust_bnds_(0);      // min thrust
      shortTermAcadoVariables.lbValues[ACADO_NU * i + 3] = yaw_rate_bnds_(0);    // min yaw rate
      shortTermAcadoVariables.ubValues[ACADO_NU * i] = roll_ref_bnds_(1);        // max roll
      shortTermAcadoVariables.ubValues[ACADO_NU * i + 1] = pitch_ref_bnds_(1);   // max pitch
      shortTermAcadoVariables.ubValues[ACADO_NU * i + 2] = thrust_bnds_(1);      // max thrust
      shortTermAcadoVariables.ubValues[ACADO_NU * i + 3] = yaw_rate_bnds_(1);     // max yaw rate

      shortTermAcadoVariables.lbAValues[ACADO_NPAC * i] = .3;                   // min obst1 dist
      shortTermAcadoVariables.lbAValues[ACADO_NPAC * i + 1] = .3;               // min obst2 dist
      shortTermAcadoVariables.lbAValues[ACADO_NPAC * i + 2] = .3;               // min obst3 dist
      shortTermAcadoVariables.lbAValues[ACADO_NPAC * i + 3] = .3;               // min obst3 dist
      shortTermAcadoVariables.lbAValues[ACADO_NPAC * i + 4] = .3;
      shortTermAcadoVariables.ubAValues[ACADO_NPAC * i] = 1000;                 // max obst1 dist
      shortTermAcadoVariables.ubAValues[ACADO_NPAC * i + 1] = 1000;             // max obst2 dist
      shortTermAcadoVariables.ubAValues[ACADO_NPAC * i + 2] = 1000;             // max obst3 dist
      shortTermAcadoVariables.ubAValues[ACADO_NPAC * i + 3] = 1000;             // max obst3 dist
      shortTermAcadoVariables.ubAValues[ACADO_NPAC * i + 4] = 1000;             // max obst3 dist
    }

    std::cout << FBLU("Short Term controller Upper Bound Limits: ") << "\n"; 
    std::cout << shortTermAcadoVariables.lbValues[0] << " " << shortTermAcadoVariables.lbValues[1] << " " << shortTermAcadoVariables.lbValues[2] << " " << shortTermAcadoVariables.lbValues[3] << " " << "\n" << "\n";

    std::cout << FBLU("Short Term controller Lower Bound Limits: ") << "\n"; 
    std::cout << shortTermAcadoVariables.ubValues[0] << " " << shortTermAcadoVariables.ubValues[1] << " " << shortTermAcadoVariables.ubValues[2] << " " << shortTermAcadoVariables.ubValues[3] << " " << "\n" << "\n";

    for (int i = 0; i < ACADO_N + 1; i++) {
      acado_online_data_.block(i, 0, 1, ACADO_NOD) << roll_time_constant_, roll_gain_, pitch_time_constant_, pitch_gain_,  // Horizontal motion model
                                                      qCam_B_Cam_(0), qCam_B_Cam_(1), qCam_B_Cam_(2),                      // Relative orientation of Cam frame wrt B frame
                                                      pCam_B_(0), pCam_B_(1), pCam_B_(2),                                  // Relative translation between Cam frame wrt B frame in B frame
                                                      pT_W_(0), pT_W_(1), pT_W_(2),                                        // Target position in W frame
                                                      camera_instrinsics_(0), camera_instrinsics_(1),                      // Camera Intrinsic Params
                                                      pObst_vert1(0), pObst_vert1(1),
                                                      pObst_vert1_WMat(0), pObst_vert1_WMat(1),
                                                      pObst_vert2(0), pObst_vert2(1),
                                                      pObst_vert2_WMat(0), pObst_vert2_WMat(1),
                                                      pObst_horiz(0), pObst_horiz(1),
                                                      pObst_horiz_WMat(0), pObst_horiz_WMat(1),
                                                      pDyn_Obst(0), pDyn_Obst(1), pDyn_Obst(2),
                                                      pDyn_Obst_WMat(0), pDyn_Obst_WMat(1), pDyn_Obst_WMat(2),
                                                      pObst_horiz2(0), pObst_horiz2(1),
                                                      pObst_horiz2_WMat(0), pObst_horiz2_WMat(1);
    }

    std::cout << FBLU("Short Term controller Online Data matrix: ") << "\n"; 
    std::cout << acado_online_data_.row(0) << "\n" << "\n";

    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(shortTermAcadoVariables.od)) = acado_online_data_.transpose(); 
    restartSolver();   
  }

  void stivsController::restartSolver(){
    
    // Initialize the states and controls. 
    for (unsigned int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)  shortTermAcadoVariables.x[ i ] = 0.0;
    for (unsigned int i = 0; i < ACADO_NU * ACADO_N; ++i)  shortTermAcadoVariables.u[ i ] = 0.0;

    // Initialize the measurements/reference. 
    for (unsigned int i = 0; i < ACADO_NY * ACADO_N; ++i)  shortTermAcadoVariables.y[ i ] = 0.0;
    for (unsigned int i = 0; i < ACADO_NYN; ++i)  shortTermAcadoVariables.yN[ i ] = 0.0;

    #if ACADO_INITIAL_STATE_FIXED
      for (unsigned int i = 0; i < ACADO_NX; ++i) shortTermAcadoVariables.x0[ i ] = 0.0;
    #endif    

    std::cout << FBLU("NY: ") << ACADO_NY << 
                 FBLU(" NYN: ") << ACADO_NYN << 
                 FBLU(" NX: ") << ACADO_NX << 
                 FBLU(" NU: ") << ACADO_NU << 
                 FBLU(" N: ") << ACADO_N << 
                 FBLU(" NOD: ") << ACADO_NOD << "\n" << "\n";

  }















  Eigen::Vector4d stivsController::computeControls(){
    
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);

    Eigen::Vector3d g(0.f, 0.f, -9.8066);
    Eigen::Vector3d Dpos_(2,2,2);
    Eigen::Vector3d Dvel_(4,4,4);

    // Feed Forward term
    // ...

    Eigen::Vector3d des_pos(shortTermAcadoVariables.x[0 + ACADO_NX], shortTermAcadoVariables.x[1 + ACADO_NX], shortTermAcadoVariables.x[2 + ACADO_NX]);
    Eigen::Vector3d des_vel(shortTermAcadoVariables.x[3 + ACADO_NX], shortTermAcadoVariables.x[4 + ACADO_NX], shortTermAcadoVariables.x[5 + ACADO_NX]);


    Eigen::Vector3d acc_des_w_(Dpos_.cwiseProduct(des_pos-odometry.position_W) 
            + Dvel_.cwiseProduct(des_vel - odometry.getVelocityWorld()) - g);

    Eigen::Vector3d acc_des_b_ = odometry.orientation_W_B.toRotationMatrix().inverse()*(acc_des_w_/acc_des_w_.norm());
    double alpha = acos( odometry.orientation_W_B.toRotationMatrix().inverse().col(2).dot(acc_des_b_) );
    Eigen::Vector3d n = odometry.orientation_W_B.toRotationMatrix().inverse().col(2).cross(acc_des_b_);
    n = n/n.norm();
    Eigen::Quaterniond q_e_rp_(Eigen::AngleAxisd(alpha, n));
    q_e_rp_.normalize();

    Eigen::Vector3d euler_angles_des;
    if( q_e_rp_.w() < 0 )
    {
      mav_msgs::getEulerAnglesFromQuaternion(q_e_rp_, &euler_angles_des);
      euler_angles *= -1;
    }
    else if( q_e_rp_.w() >= 0 )
    {
      mav_msgs::getEulerAnglesFromQuaternion(q_e_rp_, &euler_angles_des);
    }

    float des_yaw = shortTermAcadoVariables.x[8 + ACADO_NX];
    Eigen::Vector3d e_x_C_( cos(des_yaw), sin(des_yaw), 0 );
    Eigen::Vector3d e_y_C_( -sin(des_yaw), cos(des_yaw), 0 );
    Eigen::Vector3d e_x_des_B_ = e_y_C_.cross(acc_des_b_);

    Eigen::Matrix3d R_des_;
    Eigen::Vector3d yaw_angle_des;
        
    if(e_x_des_B_.norm() > 0.01) {
      
      e_x_des_B_ = e_x_des_B_/e_x_des_B_.norm();
      Eigen::Vector3d e_y_des_B_ = acc_des_b_.cross(e_x_des_B_);
      e_y_des_B_ = e_y_des_B_/e_y_des_B_.norm();
      
      Eigen::Matrix3d R_des_;
      R_des_.col(0) = e_x_des_B_;
      R_des_.col(1) = e_y_des_B_;
      R_des_.col(2) = acc_des_b_;
      
      Eigen::Quaterniond q_e_yaw_( (odometry.orientation_W_B.toRotationMatrix()*q_e_rp_.toRotationMatrix()).inverse()*R_des_ );
      
      if( q_e_yaw_.w() < 0 )
      {
        mav_msgs::getEulerAnglesFromQuaternion(q_e_yaw_, &yaw_angle_des);
        euler_angles *= -1;
      }
      else if( q_e_yaw_.w() >= 0 )
      {
        mav_msgs::getEulerAnglesFromQuaternion(q_e_yaw_, &yaw_angle_des);
      }
          
    }

    float thrust = acc_des_w_.dot(odometry.orientation_W_B.toRotationMatrix().col(2)); 
    return Eigen::Vector4d( 2*euler_angles_des(0), 2*euler_angles_des(1),3*yaw_angle_des(2) - 1.5*odometry.angular_velocity_B(2),thrust );
  }

//}; // end shortTerm workspace
