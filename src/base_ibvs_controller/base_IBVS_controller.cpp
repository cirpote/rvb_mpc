#include "base_IBVS_controller.h"

BaseibvsController::BaseibvsController(const std::string& yaml_file) {

    initializeControllerfromYAML(yaml_file);
}
  
void BaseibvsController::initializeControllerfromYAML(const std::string& yaml_file){

  std::cout << FBLU("Reading Input params from: ") << yaml_file << "\n";

  YAML::Node configuration = YAML::LoadFile(yaml_file);
  std::vector<double> q_p, qf_p, vel_bnds, phi_bnds, obst1, obst2, obst3, obst4, obst5, obst6;

  
  // Path constraints factors assignment
  q_p = configuration["path_constraints"]["q_p"].as<std::vector<double>>();
  q_theta_ = configuration["path_constraints"]["q_theta"].as<double>();
  q_obst_ = configuration["path_constraints"]["q_obst"].as<double>();

  q_p_ << q_p.at(0), q_p.at(1);

  // Terminal constraints factors assignment
  qf_p = configuration["terminal_constraints"]["qf_p"].as<std::vector<double>>();
  qf_theta_ = configuration["terminal_constraints"]["qf_theta"].as<double>();
  qf_p_ << qf_p.at(0), qf_p.at(1);

  l_ = configuration["vehicle_parameters"]["l"].as<double>();
  verbosity_ = configuration["general_params"]["verbosity"].as<int>();

  // Inputs Contraints
  vel_bnds = configuration["boundary_constraints"]["vel_bounds"].as<std::vector<double>>();
  phi_bnds = configuration["boundary_constraints"]["phi_bounds"].as<std::vector<double>>();
  vel_bnds_ << vel_bnds.at(0), vel_bnds.at(1);
  phi_bnds_ << phi_bnds.at(0), phi_bnds.at(1);


  // Obstacles
  obst1 = configuration["obstacles"]["obst1"].as<std::vector<double>>();
  obst2 = configuration["obstacles"]["obst2"].as<std::vector<double>>();
  obst3 = configuration["obstacles"]["obst3"].as<std::vector<double>>();
  obst4 = configuration["obstacles"]["obst4"].as<std::vector<double>>();
  obst5 = configuration["obstacles"]["obst5"].as<std::vector<double>>();
  obst6 = configuration["obstacles"]["obst6"].as<std::vector<double>>();
  safety_distance_ = configuration["obstacles"]["safety_dist"].as<double>();
  obst1_ << obst1.at(0), obst1.at(1);
  obst2_ << obst2.at(0), obst2.at(1);
  obst3_ << obst3.at(0), obst3.at(1);
  obst4_ << obst4.at(0), obst4.at(1);
  obst5_ << obst5.at(0), obst5.at(1);
  obst6_ << obst6.at(0), obst6.at(1);

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
