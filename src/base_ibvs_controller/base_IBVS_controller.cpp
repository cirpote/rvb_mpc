#include "base_IBVS_controller.h"

BaseibvsController::BaseibvsController(const std::string& yaml_file) {

    initializeControllerfromYAML(yaml_file);
}
  
void BaseibvsController::initializeControllerfromYAML(const std::string& yaml_file){

  YAML::Node configuration = YAML::LoadFile(yaml_file);
  std::vector<double> q_position, q_velocity, q_attitude, q_command, q_target,
                      qf_position, qf_velocity, qf_attitude, qf_target, 
                      integral_action_weights, 
                      pT_W, pCam_B, qCam_B_Cam, camera_instrinsics,
                      roll_ref_bnds, pitch_ref_bnds, yaw_rate_bnds, thrust_bnds, uv_bnds,
                      p_vert1, p_vert2, p_horiz, p_horiz2, p_dyn, p_vert1_weights, p_vert2_weights,
                      p_horiz_weights, p_dyn_obst_weights, p_horiz2_weights;

  q_position = configuration["path_constraints"]["q_p"].as<std::vector<double>>();
  q_velocity = configuration["path_constraints"]["q_v"].as<std::vector<double>>();
  q_attitude = configuration["path_constraints"]["q_attitude"].as<std::vector<double>>();
  q_command = configuration["path_constraints"]["q_control"].as<std::vector<double>>();
  q_target = configuration["path_constraints"]["q_target"].as<std::vector<double>>();

  qf_position = configuration["terminal_constraints"]["qf_p"].as<std::vector<double>>();
  qf_velocity = configuration["terminal_constraints"]["qf_v"].as<std::vector<double>>();
  qf_attitude = configuration["terminal_constraints"]["qf_attitude"].as<std::vector<double>>();
  qf_target = configuration["terminal_constraints"]["qf_target"].as<std::vector<double>>();

  // Path constraints factors assignment
  q_target_ << q_target.at(0), q_target.at(1);
  q_command_ << q_command.at(0), q_command.at(1), q_command.at(2), q_command.at(3);
  q_velocity_ << q_velocity.at(0), q_velocity.at(1) , q_velocity.at(2);
  q_attitude_ << q_attitude.at(0), q_attitude.at(1), q_attitude.at(2);
  q_position_ << q_position.at(0), q_position.at(1) , q_position.at(2);
  
  // Terminal constraints factors assignment
  qf_position_ << qf_position.at(0), qf_position.at(1),qf_position.at(2);
  qf_velocity_ << qf_velocity.at(0), qf_velocity.at(1),qf_velocity.at(2);
  qf_attitude_ << qf_attitude.at(0), qf_attitude.at(1),qf_attitude.at(2);
  qf_target_ << qf_target.at(0), qf_target.at(1);

  mass_ = configuration["vehicle_parameters"]["mass"].as<double>();
  roll_gain_ = configuration["vehicle_parameters"]["roll_gain"].as<double>();
  roll_time_constant_ = configuration["vehicle_parameters"]["roll_tau"].as<double>();
  pitch_gain_ = configuration["vehicle_parameters"]["pitch_gain"].as<double>();
  pitch_time_constant_ = configuration["vehicle_parameters"]["pitch_tau"].as<double>();

  anti_windup_ball_ = configuration["integral_action"]["anti_windup_ball"].as<double>();
  attraction_ball_ = configuration["integral_action"]["attraction_ball"].as<double>();
  integral_action_weights = configuration["integral_action"]["integral_weights"].as<std::vector<double>>();
  yaw_rate_damping = configuration["integral_action"]["yaw_rate_damping_factor"].as<double>();

  pT_W = configuration["target"]["pT_W"].as<std::vector<double>>();
  pCam_B = configuration["target"]["pCam_B"].as<std::vector<double>>();
  qCam_B_Cam = configuration["target"]["qCam_B__Cam"].as<std::vector<double>>();
  camera_instrinsics = configuration["target"]["camera_instrinsics"].as<std::vector<double>>();

  pT_W_ << pT_W.at(0), pT_W.at(1), pT_W.at(2);
  pCam_B_ << pCam_B.at(0), pCam_B.at(1), pCam_B.at(2);
  qCam_B_Cam_ << qCam_B_Cam.at(0), qCam_B_Cam.at(1), qCam_B_Cam.at(2);
  camera_instrinsics_ << camera_instrinsics.at(0), camera_instrinsics.at(1);

  roll_ref_bnds = configuration["boundary_constraints"]["roll_bounds"].as<std::vector<double>>();
  pitch_ref_bnds = configuration["boundary_constraints"]["pitch_bounds"].as<std::vector<double>>();
  yaw_rate_bnds = configuration["boundary_constraints"]["yaw_rate_bounds"].as<std::vector<double>>();
  thrust_bnds = configuration["boundary_constraints"]["thrust_bounds"].as<std::vector<double>>();
  uv_bnds = configuration["boundary_constraints"]["uv_bounds"].as<std::vector<double>>();

  roll_ref_bnds_ << roll_ref_bnds.at(0), roll_ref_bnds.at(1);
  pitch_ref_bnds_ << pitch_ref_bnds.at(0), pitch_ref_bnds.at(1);
  yaw_rate_bnds_ << yaw_rate_bnds.at(0), yaw_rate_bnds.at(1);
  thrust_bnds_ << thrust_bnds.at(0), thrust_bnds.at(1);
  uv_bnds_ << uv_bnds.at(0), uv_bnds.at(1), uv_bnds.at(2), uv_bnds.at(3);

  verbosity_ = configuration["general_params"]["verbosity"].as<int>();

  p_vert1 = configuration["obstacle"]["p_vert1_W"].as<std::vector<double>>();
  p_vert2 = configuration["obstacle"]["p_vert2_W"].as<std::vector<double>>();
  p_horiz = configuration["obstacle"]["p_horiz_W"].as<std::vector<double>>();
  p_horiz2 = configuration["obstacle"]["p_horiz2_W"].as<std::vector<double>>();
  p_dyn = configuration["obstacle"]["p_dyn_W"].as<std::vector<double>>();

  p_vert1_weights = configuration["obstacle"]["p_vert1_WMat"].as<std::vector<double>>();
  p_vert2_weights = configuration["obstacle"]["p_vert2_WMat"].as<std::vector<double>>();
  p_horiz_weights =  configuration["obstacle"]["p_horiz_WMat"].as<std::vector<double>>();
  p_horiz2_weights =  configuration["obstacle"]["p_horiz2_WMat"].as<std::vector<double>>();
  p_dyn_obst_weights = configuration["obstacle"]["p_dyn_WMat"].as<std::vector<double>>();

  output_log_file_ = configuration["log_params"]["output_log_file"].as<std::string>();
  log_file_.open (ros::package::getPath("ovs_controller") + "/log_output_folder/" + output_log_file_);

  pObst_vert1 << p_vert1.at(0), p_vert1.at(1);
  pObst_vert2 << p_vert2.at(0), p_vert2.at(1);
  pObst_horiz << p_horiz.at(0), p_horiz.at(1);
  pObst_horiz2 << p_horiz.at(0), p_horiz.at(1);
  pDyn_Obst << p_dyn.at(0), p_dyn.at(1), p_dyn.at(2);

  pObst_vert1_WMat << p_vert1_weights.at(0), p_vert1_weights.at(1);
  pObst_vert2_WMat << p_vert2_weights.at(0), p_vert2_weights.at(1);
  pObst_horiz_WMat << p_horiz_weights.at(0), p_horiz_weights.at(1);
  pObst_horiz2_WMat << p_horiz2_weights.at(0), p_horiz2_weights.at(1);
  pDyn_Obst_WMat   << p_dyn_obst_weights.at(0), p_dyn_obst_weights.at(1), p_dyn_obst_weights.at(2);



  // Obstacle World Position Check
  if( fabs(pObst_vert1(0)) < 1e-2 )
    pObst_vert1(0) = 1e-2;

  if( fabs(pObst_vert2(1)) < 1e-2 )
    pObst_vert2(1) = 1e-2;

  if( fabs(pObst_vert1(0)) < 1e-2 )
    pObst_vert2(0) = 1e-2;

  if( fabs(pObst_vert2(1)) < 1e-2 )
    pObst_vert2(1) = 1e-2;

  if( fabs(pObst_horiz(0)) < 1e-2 )
    pObst_horiz(0) = 1e-2;

  if( fabs(pObst_horiz(1)) < 1e-2 )
    pObst_horiz(1) = 1e-2;

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
      integral_action_(3) += delta_time_ * mav_utils::yawFromQuaternion(q_err);
    }

  } else if( delta_position.norm() > attraction_ball_ ) {
    integral_action_.setZero();
  }

}

BaseibvsController::~BaseibvsController(){}

void BaseibvsController::setOdometry(const nav_msgs::OdometryConstPtr& odom_msg) {

  mav_msgs::eigenOdometryFromMsg(*odom_msg, &odometry);
  if(is_first_odometry_set_){
    prev_time_ = odom_msg->header.stamp.toSec();
    is_first_odometry_set_ = false;
  } else {
    float curr_time = odom_msg->header.stamp.toSec();
    delta_time_ = curr_time - prev_time_;
    prev_time_= curr_time;
  }

}
