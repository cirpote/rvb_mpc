#include <src/nodes/IBVS_random_controller_node.h>

using namespace std;

IBVSRandomNode::IBVSRandomNode(ros::NodeHandle& nh, const std::string& yaml_short_file, const std::string& gui_file)
  : MavGUI(nh, gui_file), nh_(nh), first_trajectory_cmd_(false), commands_(0,0), SHERPA_planner_(yaml_short_file), 
    ang_vel_ref(SHERPA_planner_.odometry.angular_velocity_B)
{

  odom_sub_ = nh_.subscribe( "/odom", 1, &IBVSRandomNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay() );
  cmd_pose_sub_ = nh_.subscribe("/command/pose", 1, &IBVSRandomNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay() );
  //command_vel_steering_angle_pub_ = nh_.advertise<geometry_msgs::Twist>("/sherpa/akrm_cmd", 1);

  std::cerr << "\n" << FBLU("Initializing short term Controller from:") << " " << yaml_short_file << "\n";
  SHERPA_planner_.InitializeController();
  this->init3DObjRendering( ros::package::getPath("rvb_mpc") );

  // int iter = 0;
  // while(true){
  //   if (utils::exists( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" ) ){
  //       iter++;
  //   } else {
  //     logFileStream.open( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" );
  //     break;
  //   }
  // }
}

IBVSRandomNode::~IBVSRandomNode(){
  logFileStream.close();
}

void IBVSRandomNode::resetSolver(){
  SHERPA_planner_.InitializeController();
}


void IBVSRandomNode::CommandPoseCallback(const nav_msgs::OdometryConstPtr& cmd_pose_msg)
{
  ROS_INFO_ONCE("Optimal IBVS controller got first command message.");
  SHERPA_planner_.setCommandPose(*cmd_pose_msg);
  
  if(first_trajectory_cmd_){
    new_comand = true;
    return;
  }

  startTime = ros::Time::now().toSec();
  first_trajectory_cmd_ = true;  

  return;
}

void IBVSRandomNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  
  ROS_INFO_ONCE("Optimal IBVS controller got first odometry message.");

  SHERPA_planner_.setOdometry(odom_msg);
  _current_orientation = utils::quaternionFromMsg(odom_msg->pose.pose.orientation); 
  _current_yaw_orientation = utils::yawFromQuaternion(_current_orientation);
  _current_odom_position = utils::vector3FromPointMsg(odom_msg->pose.pose.position);  

  // Check Is the first trajectory msg has been received
  if(!first_trajectory_cmd_)
    return;

  SHERPA_planner_.calculateRollPitchYawRateThrustCommands(commands_);
  
  //std::cout << FRED("current command: ") << commands_.transpose() << "\n"; 
  //command.linear.x = fmax( -.5, fmin(.5, commands_(0)) );
  //command.angular.z = fmax( -.5, fmin(.5, commands_(1)) );
  //command_vel_steering_angle_pub_.publish(command);
  //std::cout << FRED("current command: ") << command << "\n";

  return;
}

static void error_callback(int error, const char* description)
{
  fprintf(stderr, "Error %d: %s\n", error, description);
}

int main(int argc, char** argv)
{

  if(argc < 5) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_short_term_file> <gui_params> <waypoint_generation> <dyn_obst_spawn>" << "\n";
        std::exit(1);
  }

  std::string yaml_short_filename = argv[1];
  std::string gui_filename = argv[2];
  std::string waypoint_generation = argv[3];
  std::string dynamic_obstacle_spawning = argv[4];

  if(!strcmp(waypoint_generation.c_str(), "true") ){
    randomWaypointGeneration = true;
    std::cout << FGRN(" Random Waypoint Generation ON ") << "\n";
  } else {
    randomWaypointGeneration = false;
    std::cout << FGRN(" Random Waypoint Generation OFF ") << "\n";
  }

    if(!strcmp(dynamic_obstacle_spawning.c_str(), "true") ){
    randomSpawnDynObj = true;
    std::cout << FGRN(" Random Dynamic Object Spawning ON ") << "\n";
  } else {
    randomSpawnDynObj = false;
    std::cout << FGRN(" Random Dynamic Object Spawning OFF ") << "\n";
  }

  // Setup window
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
    return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  #if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  #endif
  GLFWwindow* window = glfwCreateWindow(1000, 1000, "mav_gui_app", NULL, NULL);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  gl3wInit();

  // Setup ImGui binding
  ImGui_ImplGlfwGL3_Init(window, true);

  bool show_test_window = false;
  ImVec4 clear_color = ImColor(114, 144, 154);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if(argc < 2) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_file.txt>" << "\n";
        std::exit(1);
  }

  ros::init(argc, argv, "mav_gnomic_gui_node");
  ros::NodeHandle nh("~");

  IBVSRandomNode IBVS_node(nh, yaml_short_filename, gui_filename);
  bool show_gnomic_GUI = true;

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    ImGui_ImplGlfwGL3_NewFrame();

    if (show_test_window) {
      ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
      ImGui::ShowTestWindow(&show_test_window);
    }

    if(show_gnomic_GUI) {
      ImGui::SetNextWindowPos(ImVec2(650,650), ImGuiCond_FirstUseEver);
      IBVS_node.showGUI(&show_gnomic_GUI);
    }

    // Rendering
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui::Render();
    glfwSwapBuffers(window);

    ros::spinOnce();
  }

  // Cleanup
  ImGui_ImplGlfwGL3_Shutdown();
  glfwTerminate();

  ros::spin();

  return 0;
}
