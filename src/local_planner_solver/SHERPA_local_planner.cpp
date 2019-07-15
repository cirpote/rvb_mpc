#include "SHERPA_local_planner.h"

using namespace std;

SherpaAckermannPlanner::SherpaAckermannPlanner(const std::string& yaml_file) 
                                              : BaseibvsController(yaml_file) {

  W_.setZero();
  WN_.setZero();
  input_.setZero();
  state_.setZero();
  reference_.setZero();
  referenceN_.setZero();
}

SherpaAckermannPlanner::~SherpaAckermannPlanner(){}

bool SherpaAckermannPlanner::setCommandPose(const nav_msgs::Odometry odom_msg){

  // if( ( mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position).head(2) - pObst_vert1 ).norm() < .5 ||
  //     ( mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position).head(2) - pObst_vert2 ).norm() < .5 ||
  //     ( Eigen::Vector2d(mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position)(0),mav_msgs::vector3FromPointMsg(odom_msg.pose.pose.position)(1)) - pObst_horiz ).norm() < .5 ){
  //         std::cout << FRED("Trajectory Point not allowed! Too much close to the obstacle!\n\n");
  //   return false;
  // }

  utils::eigenOdometryFromMsg(odom_msg, &trajectory_point);  

  std::cerr << FBLU("Short Term Final State Set to: ") << trajectory_point.position_W.transpose() << 
               " " << trajectory_point.orientation_W_B.w() << " " << trajectory_point.orientation_W_B.vec().transpose() << "\n";  
        
  return true;
}

void SherpaAckermannPlanner::calculateRollPitchYawRateThrustCommands(Eigen::Vector4d& command_roll_pitch_yawrate_thrust_)
{
    
  Eigen::Vector3d euler_angles;
  odometry.getEulerAngles(&euler_angles);
  
  tf::StampedTransform leftWheelTf, rightWheelTf;
  tfListner_.lookupTransform("/sherpa/base_link", "/sherpa/right_front_wheel", ros::Time(0), rightWheelTf);
  tfListner_.lookupTransform("/sherpa/base_link", "/sherpa/left_front_wheel", ros::Time(0), leftWheelTf);

  steer_angle_ = ( utils::yawFromTfQuaternion( leftWheelTf.getRotation() ) + utils::yawFromTfQuaternion( rightWheelTf.getRotation() ) ) / 2;

  for (size_t i = 0; i < ACADO_N; i++) {
    // Possibility to add here the FeedForward Term in "reference_" velocities 
    reference_.block(i, 0, 1, ACADO_NY) << trajectory_point.position_W.block<2,1>(0,0).transpose(), 
                                           trajectory_point.getYaw(), 0, 
                                           0, 0;
  }    
  
  referenceN_ << trajectory_point.position_W.block<2,1>(0,0).transpose(), trajectory_point.getYaw(), 0;

  Eigen::Matrix<double, ACADO_NX, 1> x_0;
  x_0 << odometry.position_W.block<2,1>(0,0), trajectory_point.getYaw(), steer_angle_, 0;

  // Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  // Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = reference_.transpose();
  // Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = referenceN_.transpose();
  
  // //acado_timer t;
  // acado_preparationStep();
  // //acado_tic( &t );
  // auto start = chrono::steady_clock::now();

  // for(iter = 0; iter < NUM_STEPS; ++iter)
  // {
  //   acado_feedbackStep( );

  //   if(acado_getKKT() < KKT_THRESHOLD)
  //     break;

  //   acado_preparationStep();
  // }
  // //real_t te = acado_toc( &t );
  // auto end = chrono::steady_clock::now();
  // solve_time = chrono::duration_cast<chrono::microseconds>(end - start).count();

  // if(verbosity_ == Verbosity_Level::VERBOSE){   
  //   std::cout << FBLU("Short Term time elapsed: ") << solve_time << "microsecs\n";
  //   std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
  // } else if (verbosity_ == Verbosity_Level::FEMMINA_CAGACAZZI ) {
  //   std::cout << FBLU("Short Term time of one real-time iteration: ") <<  solve_time << " microsecs\n";
  //   std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
  //   std::cout << FBLU("Short Term Differential Variables: ") << "\n";
  //   printDifferentialVariables();
  //   std::cout << FBLU("Short Term Control Variables: ") << "\n";
  //   printControlVariables();
  // }

  //real_t time_elapsed = iter * te;
  //sendToLog( time_elapsed );

  // ComputeIntegralAction();  

  // command_roll_pitch_yawrate_thrust_(0) = acadoVariables.u[0]; // + integral_action_(0) * integral_action_weights_(0);
  // command_roll_pitch_yawrate_thrust_(1) = acadoVariables.u[1]; // + integral_action_(1) * integral_action_weights_(1);
  // command_roll_pitch_yawrate_thrust_(2) = 2*acadoVariables.u[3] - yaw_rate_damping*odometry.getYawRate() + integral_action_(3) * integral_action_weights_(3);
  // command_roll_pitch_yawrate_thrust_(3) = acadoVariables.u[2] + integral_action_(2) * integral_action_weights_(2);
                                        
}

void SherpaAckermannPlanner::printDifferentialVariables(){
  int i, j;
  for (i = 0; i < ACADO_N + 1; ++i)
  {
    for (j = 0; j < ACADO_NX; ++j)
      std::cout << acadoVariables.x[i * ACADO_NX + j] << " ";
    std::cout << "\n";
  }
  std::cout << "\n\n";
}

void SherpaAckermannPlanner::printControlVariables(){
  int i, j;
  for (i = 0; i < ACADO_N; ++i)
  {
    for (j = 0; j < ACADO_NU; ++j)
      std::cout << acadoVariables.u[i * ACADO_NU + j] << " ";
    std::cout << "\n";
  }
  std::cout << "\n\n";
}

bool SherpaAckermannPlanner::InitializeController()
{
  
  acado_initializeSolver();

  W_.block(0, 0, 2, 2) = q_position_.asDiagonal();
  W_(2, 2) = q_orientation_;
  W_(3, 3) = q_steering_angle_;
  W_.block(4, 4, 2, 2) = q_command_.asDiagonal();

  WN_.block(0, 0, 2, 2) = qf_position_.asDiagonal();
  WN_(2, 2) = qf_orientation_;
  WN_(3, 3) = qf_steering_angle_;
  
  std::cout << FBLU("Short Term controller W matrix: ") << "\n";    
  std::cout << W_.diagonal().transpose() << "\n" << "\n";
  std::cout << FBLU("Short Term controller WN matrix: ") << "\n";   
  std::cout << WN_.diagonal().transpose() << "\n" << "\n";
  
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) = WN_.transpose();
    
  for (size_t i = 0; i < ACADO_N; ++i) {
    
    acadoVariables.lbValues[ACADO_NU * i] = vel_bnds_(0);           // min vel
    acadoVariables.lbValues[ACADO_NU * i + 1] = phi_cmd_bnds_(0);   // min phi_cmd
    acadoVariables.ubValues[ACADO_NU * i] = vel_bnds_(1);           // max vel
    acadoVariables.ubValues[ACADO_NU * i + 1] = phi_cmd_bnds_(1);   // max phi_cmd

    acadoVariables.lbAValues[ACADO_NPAC * i] = 1;                    // min obst1 dist
    acadoVariables.ubAValues[ACADO_NPAC * i] = 1000;                 // max obst1 dist

  }

  std::cout << FBLU("Short Term controller Lower Bound Limits: ") << "\n"; 
  std::cout << acadoVariables.lbValues[0] << " " << acadoVariables.lbValues[1] << " " << "\n" << "\n";

  std::cout << FBLU("Short Term controller Upper Bound Limits: ") << "\n"; 
  std::cout << acadoVariables.ubValues[0] << " " << acadoVariables.ubValues[1] << " " << "\n" << "\n";

  for (int i = 0; i < ACADO_N + 1; i++) {
    acado_online_data_.block(i, 0, 1, ACADO_NOD) << l_,                               // vehicle lenghts
                                                    pObst_vert1(0), pObst_vert1(1);   // Obstacle Position
  }

  std::cout << FBLU("Short Term controller Online Data matrix: ") << "\n"; 
  std::cout << acado_online_data_.row(0) << "\n" << "\n";

  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) = acado_online_data_.transpose(); 
  restartSolver();   
}

void SherpaAckermannPlanner::restartSolver(){
  
  // Initialize the states and controls. 
  for (unsigned int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  for (unsigned int i = 0; i < ACADO_NU * ACADO_N; ++i)  acadoVariables.u[ i ] = 0.0;

  // Initialize the measurements/reference. 
  for (unsigned int i = 0; i < ACADO_NY * ACADO_N; ++i)  acadoVariables.y[ i ] = 0.0;
  for (unsigned int i = 0; i < ACADO_NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

  #if ACADO_INITIAL_STATE_FIXED
    for (unsigned int i = 0; i < ACADO_NX; ++i) acadoVariables.x0[ i ] = 0.0;
  #endif    

  std::cout << FBLU("NY: ") << ACADO_NY << 
                FBLU(" NYN: ") << ACADO_NYN << 
                FBLU(" NX: ") << ACADO_NX << 
                FBLU(" NU: ") << ACADO_NU << 
                FBLU(" N: ") << ACADO_N << 
                FBLU(" NOD: ") << ACADO_NOD << "\n" << "\n";

}