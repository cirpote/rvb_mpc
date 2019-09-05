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

  eigenOdometryFromMsg(odom_msg, &trajectory_point);  

  std::cerr << FBLU("Short Term Final State Set to: ") << trajectory_point.position_W.transpose().head(2) << 
               " " << utils::yawFromQuaternion(trajectory_point.orientation_W_B) << "\n";  
        
  return true;
}

void SherpaAckermannPlanner::calculateRollPitchYawRateThrustCommands(trajectory_msgs::JointTrajectory& trajectory_pts)
{
    
  /*  NON LINEAR CONTROLLER INITIAL STATE AND CONSTRAINTS  */
  for (size_t i = 0; i < ACADO_N; i++) {
    reference_.block(i, 0, 1, ACADO_NY) << trajectory_point.position_W(0), 
                                           trajectory_point.position_W(1), 
                                           utils::yawFromQuaternion(trajectory_point.orientation_W_B),
                                           0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                                           0.f, 0.f;
  }    

  referenceN_ << trajectory_point.position_W(0), trajectory_point.position_W(1), utils::yawFromQuaternion(trajectory_point.orientation_W_B);

  Eigen::Matrix<double, ACADO_NX, 1> x_0;
  x_0 << odometry.position_W(0), odometry.position_W(1), utils::yawFromQuaternion(odometry.orientation_W_B), 0.f;

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = referenceN_.transpose();
  
  acado_preparationStep();
  auto start = chrono::steady_clock::now();

  for(iter = 0; iter < NUM_STEPS; ++iter)
  {
    acado_feedbackStep( );

    if(acado_getKKT() < KKT_THRESHOLD)
      break;

    acado_preparationStep();
  }
  auto end = chrono::steady_clock::now();
  solve_time = chrono::duration_cast<chrono::microseconds>(end - start).count();

  if(verbosity_ == Verbosity_Level::VERBOSE){   
    std::cout << FBLU("Short Term time elapsed: ") << solve_time*1e-6 << " secs\n";
    std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
  } else if (verbosity_ == Verbosity_Level::FEMMINA_CAGACAZZI ) {
    std::cout << FBLU("Short Term time of one real-time iteration: ") <<  solve_time*1e-6 << " secs\n";
    std::cout << FBLU("Short Term N° Iteration Performed: ") << iter << "\n\n"; 
    std::cout << FBLU("Short Term Differential Variables: ") << "\n";
    printDifferentialVariables();
    std::cout << FBLU("Short Term Control Variables: ") << "\n";
    printControlVariables();
    std::cout << FBLU("Short Term Desired State: ") << "\n";
    std::cout << referenceN_.transpose() << "\n";
  }
                                        
  getTrajectoryVector(trajectory_pts);
  return;

}

void SherpaAckermannPlanner::getTrajectoryVector(trajectory_msgs::JointTrajectory& trajectory_pts){
  for (unsigned int i = 1; i < ACADO_N + 1; ++i) {
    trajectory_pts.points[0].positions[(ACADO_NX-1) * (i-1) ] = acadoVariables.x[i*ACADO_NX];
    trajectory_pts.points[0].positions[(ACADO_NX-1) * (i-1) + 1 ] = acadoVariables.x[i*ACADO_NX + 1];
    trajectory_pts.points[0].positions[(ACADO_NX-1) * (i-1) + 2 ] = acadoVariables.x[i*ACADO_NX + 2];
  }
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

  W_(0,0) = q_p_(0);
  W_(1,1) = q_p_(1);
  W_(2,2) = q_theta_;
  W_(3,3) = q_obst_;
  W_(4,4) = q_obst_;
  W_(5,5) = q_obst_;
  W_(6,6) = q_obst_;
  W_(7,7) = q_obst_;
  W_(8,8) = q_obst_;
  W_(9,9) = q_obst_;
  W_(10,10) = 200;
  W_(11,11) = 200;

  WN_(0,0) = qf_p_(0);
  WN_(1,1) = qf_p_(1);
  WN_(2,2) = qf_theta_;
  
  std::cout << FBLU("Short Term controller W matrix: ") << "\n";    
  std::cout << W_.diagonal().transpose() << "\n" << "\n";
  std::cout << FBLU("Short Term controller WN matrix: ") << "\n";   
  std::cout << WN_.diagonal().transpose() << "\n" << "\n";
  
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) = WN_.transpose();
    
  for (size_t i = 0; i < ACADO_N; ++i) {

    acadoVariables.lbValues[ACADO_NU * i] = vel_bnds_(0);        // min vel_x
    acadoVariables.lbValues[ACADO_NU * i + 1] = vel_bnds_(0);    // min vel_y
    acadoVariables.ubValues[ACADO_NU * i] = phi_bnds_(1);        // max vel_x
    acadoVariables.ubValues[ACADO_NU * i + 1] = phi_bnds_(1);    // max vel_y              

  }

  std::cout << FBLU("Short Term controller Lower Bound Limits: ") << "\n"; 
  std::cout << acadoVariables.lbValues[0] << " " << acadoVariables.lbValues[1] << " " << "\n" << "\n";

  std::cout << FBLU("Short Term controller Upper Bound Limits: ") << "\n"; 
  std::cout << acadoVariables.ubValues[0] << " " << acadoVariables.ubValues[1] << " " << "\n" << "\n";

  for (int i = 0; i < ACADO_N + 1; i++) {
    acado_online_data_.block(i, 0, 1, ACADO_NOD) << obst1_(0), obst1_(1),     // 1st Obstacle x-y position
                                                    obst2_(0), obst2_(1),     // 2st Obstacle x-y position
                                                    obst3_(0), obst3_(1),     // 3st Obstacle x-y position
                                                    obst4_(0), obst4_(1),     // 4st Obstacle x-y position
                                                    obst5_(0), obst5_(1),     // 5st Obstacle x-y position
                                                    obst6_(0), obst6_(1),     // 6st Obstacle x-y position
                                                    obst7_(0), obst7_(1),     // tst Obstacle x-y position
                                                    l_;                       // vehicle lenght
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