#include "base_IBVS_controller.h"

BaseibvsController::BaseibvsController(const std::string& yaml_file) {

    initializeControllerfromYAML(yaml_file);
}
  
void BaseibvsController::initializeControllerfromYAML(const std::string& yaml_file){

  std::cout << FBLU("Reading Input params from: ") << yaml_file << "\n";

  YAML::Node configuration = YAML::LoadFile(yaml_file);
  std::vector<double> q_position, q_command, qf_position,  
                      integral_action_weights, 
                      vel_bnds, phi_cmd_bnds,
                      p_vert1, p_vert1_weights;

  q_position = configuration["path_constraints"]["q_p"].as<std::vector<double>>();
  q_orientation_ = configuration["path_constraints"]["q_orientation"].as<double>();
  q_steering_angle_ = configuration["path_constraints"]["q_steering_angle"].as<double>();
  q_command = configuration["path_constraints"]["q_control"].as<std::vector<double>>();

  qf_position = configuration["terminal_constraints"]["qf_p"].as<std::vector<double>>();
  qf_orientation_ = configuration["terminal_constraints"]["qf_orientation"].as<double>();
  qf_steering_angle_ = configuration["terminal_constraints"]["qf_steering_angle"].as<double>();

  // Path constraints factors assignment
  q_command_ << q_command.at(0), q_command.at(1);
  q_position_ << q_position.at(0), q_position.at(1);
  
  // Terminal constraints factors assignment
  qf_position_ << qf_position.at(0), qf_position.at(1);

  l_ = configuration["vehicle_parameters"]["l"].as<double>();
  anti_windup_ball_ = configuration["integral_action"]["anti_windup_ball"].as<double>();
  attraction_ball_ = configuration["integral_action"]["attraction_ball"].as<double>();
  integral_action_weights = configuration["integral_action"]["integral_weights"].as<std::vector<double>>();
  yaw_rate_damping = configuration["integral_action"]["yaw_rate_damping_factor"].as<double>();


  vel_bnds = configuration["boundary_constraints"]["vel_bounds"].as<std::vector<double>>();
  phi_cmd_bnds = configuration["boundary_constraints"]["phi_cmd_bounds"].as<std::vector<double>>();


  vel_bnds_ << vel_bnds.at(0), vel_bnds.at(1);
  phi_cmd_bnds_ << phi_cmd_bnds.at(0), phi_cmd_bnds.at(1);


  verbosity_ = configuration["general_params"]["verbosity"].as<int>();

  p_vert1 = configuration["obstacles"]["p_vert1_W"].as<std::vector<double>>();

  p_vert1_weights = configuration["obstacles"]["p_vert1_WMat"].as<std::vector<double>>();
  
  pObst_vert1 << p_vert1.at(0), p_vert1.at(1);

  pObst_vert1_WMat << p_vert1_weights.at(0), p_vert1_weights.at(1);

  // Obstacle World Position Check
  if( fabs(pObst_vert1(0)) < 1e-2 )
    pObst_vert1(0) = 1e-2;

  // Init additional variables
  integral_action_.setZero();
  integral_action_weights_ << integral_action_weights.at(0), integral_action_weights.at(1), integral_action_weights.at(2), integral_action_weights.at(3);
}

void BaseibvsController::ComputeIntegralAction(){

  Eigen::Vector3d delta_position( trajectory_point.position_W - odometry.position_W );
  Eigen::Quaterniond q_err( Eigen::AngleAxisd( odometry.getYaw(), Eigen::Vector3d::UnitZ() ).inverse() * 
                         Eigen::AngleAxisd( trajectory_point.getYaw(), Eigen::Vector3d::UnitZ()) );

  if( delta_position.norm() < attraction_ball_ ){

    if( integral_action_.norm() < anti_windup_ball_ ){
      integral_action_.head(3) += delta_time_ * delta_position;
      integral_action_(3) += delta_time_ * utils::yawFromQuaternion(q_err);
    }

  } else if( delta_position.norm() > attraction_ball_ ) {
    integral_action_.setZero();
  }

}

BaseibvsController::~BaseibvsController(){}

void BaseibvsController::setOdometry(const nav_msgs::OdometryConstPtr& odom_msg) {

  eigenOdometryFromMsg(*odom_msg, &odometry);
  if(is_first_odometry_set_){
    prev_time_ = odom_msg->header.stamp.toSec();
    is_first_odometry_set_ = false;
  } else {
    float curr_time = odom_msg->header.stamp.toSec();
    delta_time_ = curr_time - prev_time_;
    prev_time_= curr_time;
  }

}
