#include <demo_grasping/demo_grasping.h>

#include <gazebo_msgs/SetPhysicsProperties.h>
#include <math.h>
#include <time.h>
#include <boost/assign/std/vector.hpp>
#include <limits>

#include <yumi_hw/YumiGrasp.h>

namespace demo_grasping {
using namespace boost::assign;
//-----------------------------------------------------------------
DemoGrasping::DemoGrasping()
    : task_error_tol_(0.0),
      task_diff_tol_(1e-5),
      task_timeout_tol_(1.5),
      n_jnts(14) {
  // handle to home
  nh_ = ros::NodeHandle("~");
  // global handle
  n_ = ros::NodeHandle();

  // get params
  nh_.param<bool>("with_gazebo", with_gazebo_, false);
  if (with_gazebo_) ROS_INFO("Grasping experiments running in Gazebo.");

  if (!nh_.getParam("bag_path", bag_path_))
    ROS_WARN("Could not find bag path on the parameter server!");

  write_jnts_ = false;
  write_img_ = false;
  write_tf_ = false;
  write_cluster_ = false;

  // initialize variables
  task_status_changed_ = false;
  task_success_ = false;

  // register general callbacks
  start_demo_srv_ =
      nh_.advertiseService("start_demo", &DemoGrasping::startDemo, this);
  look_what_i_found_srv_ = nh_.advertiseService(
      "dual_grasp_script", &DemoGrasping::lookWhatIFound, this);

  task_status_sub_ = n_.subscribe("task_status_array", 1,
                                  &DemoGrasping::taskStatusCallback, this);
  joint_state_sub_ =
      n_.subscribe("joint_states", 1, &DemoGrasping::jointStateCallback, this);
  img_sub_ = n_.subscribe("/camera/rgb/image_raw", 1,
                          &DemoGrasping::imgCallback, this);
  cluster_sub_ = n_.subscribe("/gplanner/fused_pc", 1,
                              &DemoGrasping::clusterCallback, this);
  tf_sub_ = n_.subscribe("tf", 1, &DemoGrasping::tfCallback, this);
  set_tasks_clt_ =
      n_.serviceClient<hqp_controllers_msgs::SetTasks>("set_tasks");
  remove_tasks_clt_ =
      n_.serviceClient<hqp_controllers_msgs::RemoveTasks>("remove_tasks");
  activate_hqp_control_clt_ =
      n_.serviceClient<hqp_controllers_msgs::ActivateHQPControl>(
          "activate_hqp_control");
  visualize_task_geometries_clt_ =
      n_.serviceClient<hqp_controllers_msgs::VisualizeTaskGeometries>(
          "visualize_task_geometries");
  set_gazebo_physics_clt_ = n_.serviceClient<gazebo_msgs::SetPhysicsProperties>(
      "set_physics_properties");
  load_tasks_clt_ =
      n_.serviceClient<hqp_controllers_msgs::LoadTasks>("load_tasks");
  reset_hqp_control_clt_ =
      n_.serviceClient<std_srvs::Empty>("reset_hqp_control");

  task_feedback_pub_ =
      n_.advertise<hqp_controllers_msgs::Task>("task_feedback", 10);

  // hardcode graciously the frame, pose and bbox of object
  nh_.param<std::string>("grasp_req_frame",
                         grasp_plan_request.request.header.frame_id, "world");
  double px, py, pz, ox, oy, oz, ow, object_radius, object_height;
  nh_.param<double>("grasp_req_radius", object_radius, 0);
  nh_.param<double>("grasp_req_height", object_height, 0);
  nh_.param<double>("grasp_req_px", px, 0);
  nh_.param<double>("grasp_req_py", py, 0);
  nh_.param<double>("grasp_req_pz", pz, 0);
  nh_.param<double>("grasp_req_ox", ox, 0);
  nh_.param<double>("grasp_req_oy", oy, 0);
  nh_.param<double>("grasp_req_oz", oz, 0);
  nh_.param<double>("grasp_req_ow", ow, 0);

  grasp_plan_request.request.object_radius = object_radius;
  grasp_plan_request.request.object_height = object_height;
  grasp_plan_request.request.objectPose.position.x = px;
  grasp_plan_request.request.objectPose.position.y = py;
  grasp_plan_request.request.objectPose.position.z = pz;
  grasp_plan_request.request.objectPose.orientation.x = ox;
  grasp_plan_request.request.objectPose.orientation.y = oy;
  grasp_plan_request.request.objectPose.orientation.z = oz;
  grasp_plan_request.request.objectPose.orientation.w = ow;
  grasp_plan_request.request.approach_frame = "world";

  switch_controller_clt_ =
      n_.serviceClient<controller_manager_msgs::SwitchController>(
          "switch_controller");

  if (!with_gazebo_) {
    close_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("close_gripper");
    open_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("open_gripper");
    get_grasp_interval_clt_ =
        n_.serviceClient<grasp_planner::PlanGrasp>("get_grasp_interval");

    reset_map_clt_ = n_.serviceClient<std_srvs::Empty>("reset_map");
    close_gripper_clt_.waitForExistence();
    open_gripper_clt_.waitForExistence();
    reset_map_clt_.waitForExistence();
    get_grasp_interval_clt_.waitForExistence();
  } else {
    // if gazebo is used, set the simulated gravity to zero in order to prevent
    // gazebo's joint drifting glitch
    set_gazebo_physics_clt_.waitForExistence();
    gazebo_msgs::SetPhysicsProperties properties;
    properties.request.time_step = 0.001;
    properties.request.max_update_rate = 1000;
    properties.request.gravity.x = 0.0;
    properties.request.gravity.y = 0.0;
    properties.request.gravity.z = 0.0;
    properties.request.ode_config.auto_disable_bodies = false;
    properties.request.ode_config.sor_pgs_precon_iters = 0;
    properties.request.ode_config.sor_pgs_iters = 50;
    properties.request.ode_config.sor_pgs_w = 1.3;
    properties.request.ode_config.sor_pgs_rms_error_tol = 0.0;
    properties.request.ode_config.contact_surface_layer = 0.001;
    properties.request.ode_config.contact_max_correcting_vel = 100.0;
    properties.request.ode_config.cfm = 0.0;
    properties.request.ode_config.erp = 0.2;
    properties.request.ode_config.max_contacts = 20.0;

    set_gazebo_physics_clt_.call(properties);
    if (!properties.response.success) {
      ROS_ERROR("Couldn't set Gazebo physics properties, status message: %s!",
                properties.response.status_message.c_str());
      ros::shutdown();
    } else
      ROS_INFO("Disabled gravity in Gazebo.");
  }

  set_tasks_clt_.waitForExistence();
  remove_tasks_clt_.waitForExistence();
  activate_hqp_control_clt_.waitForExistence();
  visualize_task_geometries_clt_.waitForExistence();
  load_tasks_clt_.waitForExistence();
  reset_hqp_control_clt_.waitForExistence();

  // PRE-DEFINED JOINT CONFIGURATIONS
  // configs have to be within the safety margins of the joint limits

  // fill in link names
  link_frame_names.push_back("yumi_link_1_r");
  link_frame_names.push_back("yumi_link_2_r");
  link_frame_names.push_back("yumi_link_3_r");
  link_frame_names.push_back("yumi_link_4_r");
  link_frame_names.push_back("yumi_link_5_r");
  link_frame_names.push_back("yumi_link_6_r");
  link_frame_names.push_back("yumi_link_7_r");
  link_frame_names.push_back("yumi_link_1_l");
  link_frame_names.push_back("yumi_link_2_l");
  link_frame_names.push_back("yumi_link_3_l");
  link_frame_names.push_back("yumi_link_4_l");
  link_frame_names.push_back("yumi_link_5_l");
  link_frame_names.push_back("yumi_link_6_l");
  link_frame_names.push_back("yumi_link_7_l");

  sensing_config_ = std::vector<double>(n_jnts);
  sensing_config_[0] = 0.42;
  sensing_config_[1] = -1.48;
  sensing_config_[2] = -1.21;
  sensing_config_[3] = 0.75;  // 0.37;
  sensing_config_[4] = 0.8;   //-3.41;
  sensing_config_[5] = 0.45;  //-0.23;
  sensing_config_[6] = 1.21;  // 0.13;

  sensing_config_[7] = -0.42;
  sensing_config_[8] = -1.48;
  sensing_config_[9] = 1.21;
  sensing_config_[10] = 0.75;  // 0.37;
  sensing_config_[11] = -0.8;  //-3.41;
  sensing_config_[12] = 0.45;  //-0.23;
  sensing_config_[13] = 1.21;  // 0.13;

  // DEFAULT GRASP
  grasp_.obj_frame_ = "world";         // object frame
  grasp_.e_frame_ = "gripper_r_base";  // endeffector frame
  grasp_.e_.setZero();  // endeffector point expressed in the endeffector frame
  grasp_.isSphereGrasp = false;
  grasp_.isDefaultGrasp = true;

  grasp_.v_(0) = 0.0;
  grasp_.v_(1) = 0.0;
  grasp_.v_(2) = 1.0;  // cylinder normal
  grasp_.p_(0) = 0.45;
  grasp_.p_(1) = 0;
  grasp_.p_(2) = 0.1;  // reference point on the cylinder axis
  grasp_.r1_ = 0.115;
  grasp_.r2_ = 0.125;  // cylinder radii
  grasp_.n1_ = grasp_.v_;
  grasp_.n2_ = -grasp_.v_;  // plane normals
  grasp_.d1_ = 0.12;
  grasp_.d2_ = -0.2;  // plane offsets

  /*
  //PLACEMENT ZONES
  PlaceInterval place;
  place.place_frame_ = "world";
  place.e_frame_ = "gripper_r_base";
  place.e_(0) = 0.16; place.e_(1) = 0.0; place.e_(2) = 0.0;
  place.v_(0) = 0.0; place.v_(1) = 0.0; place.v_(2) = 1.0;
  place.p_(0) = 0.75; place.p_(1) = 0.2; place.p_(2) = 0.16;
  place.r_ = 0.02;
  place.n_(0) = 0.0; place.n_(1) = 0.0; place.n_(2) = 1.0;
  place.d_ = 0.26;
  place.joints_ += 1.81, 1.01, -0.75, -1.28, 0.79, 0.85, -2.26;
  //place_zones_.push_back(place);

  place.joints_.clear();
  place.p_(1) = 0.0;
  place.joints_ += 2.11, 0.58, -1.00, -1.71, 0.58, 0.82, -2.20;
  place_zones_.push_back(place);

  place.joints_.clear();
  place.p_(1) = -0.2;
  place.joints_ += 0.038, -0.26, 0.94, -1.88, 0.51, 1.01, -2.29;
  //place_zones_.push_back(place);
  */
}
//-----------------------------------------------------------------
// bool DemoGrasping::setCartesianStiffness(double sx, double sy, double sz,
// double sa, double sb, double sc)
// {
//     if(!with_gazebo_)
//     {
//         lbr_fri::SetStiffness cart_stiffness;

//         cart_stiffness.request.sx = sx;
//         cart_stiffness.request.sy = sy;
//         cart_stiffness.request.sz = sz;
//         cart_stiffness.request.sa = sa;
//         cart_stiffness.request.sb = sb;
//         cart_stiffness.request.sc = sc;
//         if(!set_stiffness_clt_.call(cart_stiffness))
//         {
//             ROS_ERROR("Could not set the cartesian stiffness!");
//             return false;
//         }
//     }

//     return true;
// }
//-----------------------------------------------------------------
void DemoGrasping::activateHQPControl() {
  hqp_controllers_msgs::ActivateHQPControl controller_status;
  controller_status.request.active = true;
  activate_hqp_control_clt_.call(controller_status);
}
//-----------------------------------------------------------------
void DemoGrasping::deactivateHQPControl() {
  hqp_controllers_msgs::ActivateHQPControl controller_status;
  controller_status.request.active = false;
  activate_hqp_control_clt_.call(controller_status);
}
//-----------------------------------------------------------------
void DemoGrasping::safeShutdown() {
  deactivateHQPControl();
  resetState();
  std_srvs::Empty srv;
  reset_hqp_control_clt_.call(srv);
  pers_task_vis_ids_.clear();
  ROS_BREAK();  // I must break you ... ros::shutdown() doesn't seem to do the
                // job
}
//-----------------------------------------------------------------
void DemoGrasping::safeReset() {
  deactivateHQPControl();
  resetState();
  std_srvs::Empty srv;
  reset_hqp_control_clt_.call(srv);
  pers_task_vis_ids_.clear();
}
//-----------------------------------------------------------------
bool DemoGrasping::getGraspInterval() {
  // get the grasp intervall
  get_grasp_interval_clt_.waitForExistence();

  grasp_planner::PlanGrasp grasp_plan_request_right = grasp_plan_request;
  grasp_planner::PlanGrasp grasp_plan_request_left = grasp_plan_request;
  // RIGHT ARM
  grasp_plan_request_right.request.approach_vector[0] = 0;
  grasp_plan_request_right.request.approach_vector[1] = 1;
  grasp_plan_request_right.request.approach_vector[2] = 0;
  grasp_plan_request_right.request.approach_angle = 1.57;
  // LEFT ARM
  grasp_plan_request_left.request.approach_vector[0] = 0;
  grasp_plan_request_left.request.approach_vector[1] = -1;
  grasp_plan_request_left.request.approach_vector[2] = 0;
  grasp_plan_request_left.request.approach_angle = 1.57;

#if 0 
    ROS_INFO("Request is:");
    std::cout<<grasp_plan_request_right.request.object_radius <<" "<<
               grasp_plan_request_right.request.object_height <<" "<<
               grasp_plan_request_right.request.objectPose.position.x <<" "<<
               grasp_plan_request_right.request.objectPose.position.y <<" "<<
               grasp_plan_request_right.request.objectPose.position.z <<" "<<
               grasp_plan_request_right.request.objectPose.orientation.x <<" "<<
               grasp_plan_request_right.request.objectPose.orientation.y <<" "<<
               grasp_plan_request_right.request.objectPose.orientation.z <<" "<<
               grasp_plan_request_right.request.objectPose.orientation.w <<" "<<
               grasp_plan_request_right.request.approach_frame <<" "<< 
	       grasp_plan_request_right.request.approach_vector[0] <<" "<< 
	       grasp_plan_request_right.request.approach_vector[1] <<" "<< 
	       grasp_plan_request_right.request.approach_vector[2] <<" "<< 
	       grasp_plan_request_right.request.approach_angle <<"\n";
#endif

  if (!get_grasp_interval_clt_.call(grasp_plan_request_right)) {
    ROS_ERROR("Could not call grasp client");
    return false;
  }

  if (!grasp_plan_request_right.response.success) {
    ROS_WARN("Planning was NOT successful! (RIGHT)");
  }

  if (!get_grasp_interval_clt_.call(grasp_plan_request_left)) {
    ROS_ERROR("Could not call grasp client");
    return false;
  }

  if (!grasp_plan_request_left.response.success) {
    ROS_WARN("Planning was NOT successful! (LEFT)");
  }

  if (!grasp_plan_request_left.response.success &&
      !grasp_plan_request_right.response.success) {
    ROS_ERROR("Planning was NOT successful with either arm, ABORT!");
    return false;
  }

  if (grasp_plan_request_right.response.volume >
      grasp_plan_request_left.response.volume) {
    // grasping with right
    ROS_INFO("Grasping with right");
    grasp_plan_request.response = grasp_plan_request_right.response;
    grasp_.e_frame_ = "gripper_r_base";  // endeffector frame
  } else {
    // grasping with left
    ROS_INFO("Grasping with left");
    grasp_plan_request.response = grasp_plan_request_left.response;
    grasp_.e_frame_ = "gripper_l_base";  // endeffector frame
  }

  ROS_INFO("Got grasp plan, processing");
  grasp_.isDefaultGrasp = false;

  ROS_ASSERT(grasp_plan_request.response.constraints.size() == 6);
  std::vector<double> data;
  grasp_.obj_frame_ = grasp_plan_request.response.frame_id;

  // BOTTOM PLANE
  ROS_ASSERT(grasp_plan_request.response.constraints[0].g_type ==
             hqp_controllers_msgs::TaskGeometry::PLANE);
  data = grasp_plan_request.response.constraints[0].g_data;
  ROS_ASSERT(data.size() == 4);
  grasp_.n1_(0) = data[0];
  grasp_.n1_(1) = data[1];
  grasp_.n1_(2) = data[2];
  grasp_.d1_ = data[3];

  // TOP PLANE
  ROS_ASSERT(grasp_plan_request.response.constraints[1].g_type ==
             hqp_controllers_msgs::TaskGeometry::PLANE);
  data = grasp_plan_request.response.constraints[1].g_data;
  ROS_ASSERT(data.size() == 4);
  grasp_.n2_(0) = data[0];
  grasp_.n2_(1) = data[1];
  grasp_.n2_(2) = data[2];
  grasp_.d2_ = data[3];

  // LEFT PLANE
  ROS_ASSERT(grasp_plan_request.response.constraints[2].g_type ==
             hqp_controllers_msgs::TaskGeometry::PLANE);
  data = grasp_plan_request.response.constraints[2].g_data;
  ROS_ASSERT(data.size() == 4);
  grasp_.n3_(0) = data[0];
  grasp_.n3_(1) = data[1];
  grasp_.n3_(2) = data[2];
  grasp_.d3_ = data[3];

  // RIGHT PLANE
  ROS_ASSERT(grasp_plan_request.response.constraints[3].g_type ==
             hqp_controllers_msgs::TaskGeometry::PLANE);
  data = grasp_plan_request.response.constraints[3].g_data;
  ROS_ASSERT(data.size() == 4);
  grasp_.n4_(0) = data[0];
  grasp_.n4_(1) = data[1];
  grasp_.n4_(2) = data[2];
  grasp_.d4_ = data[3];

  // INNER GRASP CYLINDER
  ROS_ASSERT(grasp_plan_request.response.constraints[4].g_type ==
                 hqp_controllers_msgs::TaskGeometry::CYLINDER ||
             grasp_plan_request.response.constraints[4].g_type ==
                 hqp_controllers_msgs::TaskGeometry::SPHERE);

  grasp_.isSphereGrasp = grasp_plan_request.response.constraints[4].g_type ==
                         hqp_controllers_msgs::TaskGeometry::SPHERE;

  data = grasp_plan_request.response.constraints[4].g_data;
  if (!grasp_.isSphereGrasp) {
    ROS_ASSERT(data.size() == 7);
    grasp_.p_(0) = data[0];
    grasp_.p_(1) = data[1];
    grasp_.p_(2) = data[2];
    grasp_.v_(0) = data[3];
    grasp_.v_(1) = data[4];
    grasp_.v_(2) = data[5];
    grasp_.r1_ = data[6];

    // OUTER GRASP CYLINDER
    ROS_ASSERT(grasp_plan_request.response.constraints[5].g_type ==
               hqp_controllers_msgs::TaskGeometry::CYLINDER);
    data = grasp_plan_request.response.constraints[5].g_data;
    ROS_ASSERT(data.size() == 7);
    grasp_.r2_ = data[6];
  } else {
    ROS_ASSERT(data.size() == 4);
    grasp_.p_(0) = data[0];
    grasp_.p_(1) = data[1];
    grasp_.p_(2) = data[2];
    grasp_.r1_ = data[3];

    // OUTER GRASP CYLINDER
    ROS_ASSERT(grasp_plan_request.response.constraints[5].g_type ==
               hqp_controllers_msgs::TaskGeometry::SPHERE);
    data = grasp_plan_request.response.constraints[5].g_data;
    ROS_ASSERT(data.size() == 4);
    grasp_.r2_ = data[3];
  }
  /*
      //set the angle to the max opening allowed
      grasp_.angle = grasp_plan_request.response.max_oa;
      //if we are too open, add a little safety margin
      if(grasp_.angle < MIN_OPENING) grasp_.angle = MIN_OPENING;
      //make sure we are never closed more than the allowed angle
      if(grasp_.angle >
     grasp_plan_request.response.min_oa-OPENING_SAFETY_MARGIN) grasp_.angle =
     grasp_plan_request.response.min_oa-OPENING_SAFETY_MARGIN;
      //if we are too open, add a little safety margin
      if(grasp_.angle < MIN_OPENING) grasp_.angle = MIN_OPENING;
  */
  ROS_INFO("GRIPPER WILL GO TO %f", grasp_.angle);

  // Plane normals need to point in opposit directions to give a closed interval
  ROS_ASSERT((grasp_.n1_.transpose() * grasp_.n2_) <= 0.0);

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::resetState() {
  hqp_controllers_msgs::RemoveTasks rem_t_srv;
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    rem_t_srv.request.ids.push_back(tasks_.response.ids[i]);

  remove_tasks_clt_.call(rem_t_srv);
  if (!rem_t_srv.response.success) {
    ROS_ERROR("DemoGrasping::resetStateTasks(): could not remove tasks!");
    return false;
  }
  // clean up the task message which is used as a container
  tasks_.response.ids.clear();
  tasks_.response.success = false;
  tasks_.request.tasks.clear();

  // clean up the monitored tasks
  monitored_tasks_.clear();

  // clean up the previous task progress vector;
  t_prog_prev_.resize(0);

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::visualizeStateTasks(std::vector<unsigned int> const& ids) {
  // send a visualization message to show the tasks in Rviz
  hqp_controllers_msgs::VisualizeTaskGeometries vis_srv;
  for (unsigned int i = 0; i < ids.size(); i++)
    vis_srv.request.ids.push_back(ids[i]);

  visualize_task_geometries_clt_.call(vis_srv);
  if (!vis_srv.response.success) {
    ROS_ERROR("DemoGrasping::setStateTasks(): could not start visualization!");
    return false;
  }
  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::sendStateTasks() {
  // sends the tasks to the controller
  set_tasks_clt_.call(tasks_);
  if (!tasks_.response.success) {
    ROS_ERROR("DemoGrasping::setStateTasks(): could not set tasks!");
    return false;
  }
  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::setJointConfiguration(std::vector<double> const& joints) {
  ROS_ASSERT(joints.size() == 14);

  hqp_controllers_msgs::Task task;
  hqp_controllers_msgs::TaskLink t_link;
  hqp_controllers_msgs::TaskGeometry t_geom;

  // SET JOINT VALUES
  t_link.geometries.resize(1);
  t_geom.g_data.resize(1);
  task.t_links.clear();
  task.dynamics.d_data.clear();
  task.t_type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
  task.priority = 3;
  task.name = "joint_setpoints";
  task.is_equality_task = true;
  task.task_frame = "world";
  task.ds = 0.0;
  task.di = 1;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN);
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;

  for (int i = 0; i < n_jnts; i++) {
    t_geom.g_data[0] = joints[i];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = link_frame_names[i];
    task.t_links.push_back(t_link);
  }

  tasks_.request.tasks.push_back(task);

  // send the filled task message to the controller
  if (!sendStateTasks()) return false;

  // monitor only the last task
  monitored_tasks_.push_back(tasks_.response.ids.back());

  // visualize all tasks except of the last one
  std::vector<unsigned int> ids = pers_task_vis_ids_;
  for (unsigned int i = 0; i < tasks_.response.ids.size() - 1; i++)
    ids.push_back(tasks_.response.ids[i]);

  if (!visualizeStateTasks(ids)) return false;

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::setObjectPlace(PlaceInterval const& place) {
  hqp_controllers_msgs::Task task;
  hqp_controllers_msgs::TaskLink t_link;
  hqp_controllers_msgs::TaskGeometry t_geom;

  // EE ON HORIZONTAL PLANE
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.name = "ee_on_horizontal_plane (place)";
  task.is_equality_task = true;
  task.task_frame = place.place_frame_;
  task.ds = 0.0;
  task.di = 1;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
  t_geom.g_data.push_back(place.n_(0));
  t_geom.g_data.push_back(place.n_(1));
  t_geom.g_data.push_back(place.n_(2));
  t_geom.g_data.push_back(place.d_);
  t_link.link_frame = place.place_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(place.e_(0));
  t_geom.g_data.push_back(place.e_(1));
  t_geom.g_data.push_back(place.e_(2));
  t_link.link_frame = place.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // PLACEMENT_CYLINDER
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.name = "ee_in_placement_cylinder (place)";
  task.is_equality_task = false;
  task.task_frame = place.place_frame_;
  task.ds = 0.0;
  task.di = 0.05;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN / 5);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(place.e_(0));
  t_geom.g_data.push_back(place.e_(1));
  t_geom.g_data.push_back(place.e_(2));
  t_link.link_frame = place.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
  t_geom.g_data.push_back(place.p_(0));
  t_geom.g_data.push_back(place.p_(1));
  t_geom.g_data.push_back(place.p_(2));
  t_geom.g_data.push_back(place.v_(0));
  t_geom.g_data.push_back(place.v_(1));
  t_geom.g_data.push_back(place.v_(2));
  t_geom.g_data.push_back(place.r_);
  t_link.link_frame = "world";
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // GRIPPER APPROACH AXIS ALIGNMENT
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PARALLEL;
  task.priority = 2;
  task.name = "gripper_approach_axis_alignment";
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 0.05;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN / 6);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(-0.707);
  t_geom.g_data.push_back(0.707);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(ALIGNMENT_ANGLE * 10);
  t_link.link_frame = place.place_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(1);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // GRIPPER VERTICAL AXIS ALIGNMENT
  task.t_links.clear();
  task.dynamics.d_data.clear();
  task.name = "gripper_vertical_axis_alignment";
  task.t_type = hqp_controllers_msgs::Task::PARALLEL;
  task.priority = 2;
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 1;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN * 2);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(1);
  t_geom.g_data.push_back(0.0);
  t_link.link_frame = place.place_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(1);
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // send the filled task message to the controller
  if (!sendStateTasks()) return false;

  // monitor all tasks
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    monitored_tasks_.push_back(tasks_.response.ids[i]);

  // visualize all tasks
  std::vector<unsigned int> ids = pers_task_vis_ids_;
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    ids.push_back(tasks_.response.ids[i]);

  if (!visualizeStateTasks(ids)) return false;

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::setObjectExtract() {
  hqp_controllers_msgs::Task task;
  hqp_controllers_msgs::TaskLink t_link;
  hqp_controllers_msgs::TaskGeometry t_geom;

  ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);
  ROS_ASSERT(grasp_.n1_.transpose() * grasp_.n2_ < 0.0);

  // LOWER GRASP INTERVAL PLANE
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 0.02;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
  t_geom.g_data.push_back(grasp_.n1_(0));
  t_geom.g_data.push_back(grasp_.n1_(1));
  t_geom.g_data.push_back(grasp_.n1_(2));
  t_geom.g_data.push_back(grasp_.d1_ + EXTRACT_OFFSET * grasp_.n1_(2));
  ROS_INFO("Stay above task %lf %lf %lf %lf", t_geom.g_data[0],
           t_geom.g_data[1], t_geom.g_data[2], t_geom.g_data[3]);

  t_link.link_frame = grasp_.obj_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(grasp_.e_(0));
  t_geom.g_data.push_back(grasp_.e_(1));
  t_geom.g_data.push_back(grasp_.e_(2));
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // UPPER GRASP INTERVAL PLANE
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 0.05;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(2 * DYNAMICS_GAIN);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
  t_geom.g_data.push_back(grasp_.n2_(0));
  t_geom.g_data.push_back(grasp_.n2_(1));
  t_geom.g_data.push_back(grasp_.n2_(2));
  t_geom.g_data.push_back(grasp_.d2_ + EXTRACT_OFFSET * grasp_.n2_(2));
  ROS_INFO("Stay below task %lf %lf %lf %lf", t_geom.g_data[0],
           t_geom.g_data[1], t_geom.g_data[2], t_geom.g_data[3]);
  t_link.link_frame = grasp_.obj_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(grasp_.e_(0));
  t_geom.g_data.push_back(grasp_.e_(1));
  t_geom.g_data.push_back(grasp_.e_(2));
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // Left and Right limits only for non-hardcoded grasps
  if (!grasp_.isDefaultGrasp) {
    // LEFT GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n3_(0));
    t_geom.g_data.push_back(grasp_.n3_(1));
    t_geom.g_data.push_back(grasp_.n3_(2));
    t_geom.g_data.push_back(grasp_.d3_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // RIGHT GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n4_(0));
    t_geom.g_data.push_back(grasp_.n4_(1));
    t_geom.g_data.push_back(grasp_.n4_(2));
    t_geom.g_data.push_back(grasp_.d4_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
  }

  if (!grasp_.isSphereGrasp) {
    // INNER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // OUTER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // COPLANAR LINES CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::COPLANAR;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(4 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(-1);
    t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
  } else {
    // INNER SPHERE CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.r1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // OUTER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.r2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // TODO: add gripper alignment constraints
  }

#if 0
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;


    //UPPER GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n2_(0)); t_geom.g_data.push_back(grasp_.n2_(1)); t_geom.g_data.push_back(grasp_.n2_(2));
    t_geom.g_data.push_back(grasp_.d2_+EXTRACT_OFFSET);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
    
    //COPLANAR LINES CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::COPLANAR;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0)); t_geom.g_data.push_back(grasp_.v_(1)); t_geom.g_data.push_back(grasp_.v_(2));
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(4 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(-1); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

#endif

#if 0
    //EE ON ATTACK POINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_attack_point";
    task.is_equality_task = true;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 0.5);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.p_(0) - grasp_.a_(0)*0.25); t_geom.g_data.push_back(grasp_.p_(1) - grasp_.a_(1)*0.2); t_geom.g_data.push_back(grasp_.p_(2) + 0.2);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER APPROACH AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.name = "gripper_approach_axis_alignment";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 4);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    //project the approach vector on the x/y plane
    Eigen::Vector3d a;
    a.setZero();
    a(0) = grasp_.a_(0); a(1) = grasp_.a_(1);
    a.normalize();
    t_geom.g_data.push_back(a(0)); t_geom.g_data.push_back(a(1)); t_geom.g_data.push_back(a(2));
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE / 4);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
#endif

  // send the filled task message to the controller
  if (!sendStateTasks()) return false;

  // monitor all tasks
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    monitored_tasks_.push_back(tasks_.response.ids[i]);

  // visualize all tasks
  std::vector<unsigned int> ids = pers_task_vis_ids_;
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    ids.push_back(tasks_.response.ids[i]);

  if (!visualizeStateTasks(ids)) return false;
  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::setGripperExtract(PlaceInterval const& place) {
  hqp_controllers_msgs::Task task;
  hqp_controllers_msgs::TaskLink t_link;
  hqp_controllers_msgs::TaskGeometry t_geom;

  // EE ON ATTACK POINT
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.name = "ee_on_attack_point";
  task.is_equality_task = true;
  task.task_frame = place.place_frame_;
  task.ds = 0.0;
  task.di = 1;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN * 1.5);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(place.p_(0));
  t_geom.g_data.push_back(place.p_(1) - 0.15);
  t_geom.g_data.push_back(0.6);
  t_link.link_frame = place.place_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(place.e_(0));
  t_geom.g_data.push_back(place.e_(1));
  t_geom.g_data.push_back(place.e_(2));
  t_link.link_frame = place.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // GRIPPER APPROACH AXIS ALIGNMENT
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PARALLEL;
  task.priority = 2;
  task.name = "gripper_approach_axis_alignment";
  task.is_equality_task = false;
  task.task_frame = place.place_frame_;
  task.ds = 0.0;
  task.di = 0.05;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(DYNAMICS_GAIN / 6);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
  t_geom.g_data.push_back(place.p_(0));
  t_geom.g_data.push_back(place.p_(1));
  t_geom.g_data.push_back(place.p_(2));
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(1);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(ALIGNMENT_ANGLE * 10);
  t_link.link_frame = place.place_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(1);
  t_geom.g_data.push_back(0);
  t_geom.g_data.push_back(0);
  t_link.link_frame = place.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);
#if 0
    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = place.place_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE / 4);
    t_link.link_frame = place.place_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = place.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
#endif

  // send the filled task message to the controller
  if (!sendStateTasks()) return false;

  // monitor all tasks
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    monitored_tasks_.push_back(tasks_.response.ids[i]);

  // visualize all tasks
  std::vector<unsigned int> ids = pers_task_vis_ids_;
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    ids.push_back(tasks_.response.ids[i]);

  if (!visualizeStateTasks(ids)) return false;

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::setGraspApproach() {
  if (!with_gazebo_) {
    if (grasp_.isDefaultGrasp) {
      ROS_WARN("Grasp is default grasp!");
      return false;
    }
  }

  hqp_controllers_msgs::Task task;
  hqp_controllers_msgs::TaskLink t_link;
  hqp_controllers_msgs::TaskGeometry t_geom;

  ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);
  ROS_ASSERT(grasp_.n1_.transpose() * grasp_.n2_ < 0.0);

  // LOWER GRASP INTERVAL PLANE
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 0.02;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(1.5 * DYNAMICS_GAIN);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
  t_geom.g_data.push_back(grasp_.n1_(0));
  t_geom.g_data.push_back(grasp_.n1_(1));
  t_geom.g_data.push_back(grasp_.n1_(2));
  t_geom.g_data.push_back(grasp_.d1_);
  t_link.link_frame = grasp_.obj_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(grasp_.e_(0));
  t_geom.g_data.push_back(grasp_.e_(1));
  t_geom.g_data.push_back(grasp_.e_(2));
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // UPPER GRASP INTERVAL PLANE
  task.t_links.clear();
  task.dynamics.d_data.clear();

  task.t_type = hqp_controllers_msgs::Task::PROJECTION;
  task.priority = 2;
  task.is_equality_task = false;
  task.task_frame = grasp_.obj_frame_;
  task.ds = 0.0;
  task.di = 0.05;
  task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
  task.dynamics.d_data.push_back(3 * DYNAMICS_GAIN);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
  t_geom.g_data.push_back(grasp_.n2_(0));
  t_geom.g_data.push_back(grasp_.n2_(1));
  t_geom.g_data.push_back(grasp_.n2_(2));
  t_geom.g_data.push_back(grasp_.d2_);
  t_link.link_frame = grasp_.obj_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  t_link.geometries.clear();
  t_geom.g_data.clear();
  t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
  t_geom.g_data.push_back(grasp_.e_(0));
  t_geom.g_data.push_back(grasp_.e_(1));
  t_geom.g_data.push_back(grasp_.e_(2));
  t_link.link_frame = grasp_.e_frame_;
  t_link.geometries.push_back(t_geom);
  task.t_links.push_back(t_link);

  tasks_.request.tasks.push_back(task);

  // Left and Right limits only for non-hardcoded grasps
  if (!grasp_.isDefaultGrasp) {
    // LEFT GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n3_(0));
    t_geom.g_data.push_back(grasp_.n3_(1));
    t_geom.g_data.push_back(grasp_.n3_(2));
    t_geom.g_data.push_back(grasp_.d3_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // RIGHT GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n4_(0));
    t_geom.g_data.push_back(grasp_.n4_(1));
    t_geom.g_data.push_back(grasp_.n4_(2));
    t_geom.g_data.push_back(grasp_.d4_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
  }

  if (!grasp_.isSphereGrasp) {
    // INNER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // OUTER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // COPLANAR LINES CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::COPLANAR;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0));
    t_geom.g_data.push_back(grasp_.v_(1));
    t_geom.g_data.push_back(grasp_.v_(2));
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(4 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(-1);
    t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);
  } else {
    // INNER SPHERE CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.r1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // OUTER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0));
    t_geom.g_data.push_back(grasp_.e_(1));
    t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(grasp_.p_(0));
    t_geom.g_data.push_back(grasp_.p_(1));
    t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.r2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // TODO: add gripper alignment constraints
  }

  for (int xx = 0; xx < tasks_.request.tasks.size(); xx++) {
    task_feedback_pub_.publish(tasks_.request.tasks[xx]);
  }

  // send the filled task message to the controller
  if (!sendStateTasks()) return false;

  // monitor all tasks
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    monitored_tasks_.push_back(tasks_.response.ids[i]);

  // visualize all tasks
  std::vector<unsigned int> ids = pers_task_vis_ids_;
  for (unsigned int i = 0; i < tasks_.response.ids.size(); i++)
    ids.push_back(tasks_.response.ids[i]);

  if (!visualizeStateTasks(ids)) return false;

  return true;
}
//-----------------------------------------------------------------
void DemoGrasping::taskStatusCallback(
    const hqp_controllers_msgs::TaskStatusArrayPtr& msg) {
  boost::mutex::scoped_lock lock(manipulator_tasks_m_, boost::try_to_lock);
  if (!lock) return;

  static struct timeval t_stag;
  struct timeval t;
  gettimeofday(&t, 0);
  static bool stagnation_flag = false;

  if (!stagnation_flag) t_stag = t;

  stagnation_flag = false;
  // double curr = t.tv_sec + 0.000001*t.tv_usec;
  // double stag = t_stag.tv_sec + 0.000001*t_stag.tv_usec;

  // std::cerr<<"t: "<<curr<<std::endl;
  // std::cerr<<"t_stag: "<<stag<<std::endl;

  // std::cerr<<"monitored tasks: ";
  // for(unsigned int i=0; i<monitored_tasks_.size(); i++)
  //   std::cerr<<monitored_tasks_[i]<<" ";

  // std::cerr<<std::endl;
  // std::cerr<<"received tasks: ";
  // std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it2;
  // for(status_it2 = msg->statuses.begin(); status_it2!=msg->statuses.end();
  // ++status_it2)
  // 	std::cerr<<status_it2->id<<" "<<status_it2->name<<" "<<std::endl;

  // std::cerr<<std::endl;

  Eigen::VectorXd t_prog(monitored_tasks_.size());

  // form the maximum norm over all errors

  for (unsigned int i = 0; i < monitored_tasks_.size(); i++) {
    // try to find the monitored task id in the given task status message
    std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it;
    for (status_it = msg->statuses.begin(); status_it != msg->statuses.end();
         ++status_it)
      if (monitored_tasks_[i] == status_it->id) {
        t_prog(i) = status_it->progress;
        break;
      }

    if (status_it == msg->statuses.end()) {
      ROS_WARN("No status feedback for monitored task id %d!",
               monitored_tasks_[i]);
      return;  // just so we don't give a false positive task success
    }
  }

  double e = 0.0;
  double e_diff = INFINITY;

  if (monitored_tasks_.size() > 0) {
    // task error
    e = t_prog.cwiseAbs().maxCoeff();

    // find the task progress difference between iterations
    if (t_prog_prev_.size() > 0)
      e_diff = (t_prog - t_prog_prev_).cwiseAbs().maxCoeff();

    // std::cerr<<"t_prog: "<<t_prog.transpose()<<"e: "<<e<<std::endl;
    // std::cerr<<"t_prog_prev_: "<<t_prog_prev_.transpose()<<"e_diff:
    // "<<e_diff<<std::endl;
    t_prog_prev_ = t_prog;
  }
  //  std::cout<<" t - t_stag: "<<t.tv_sec - t_stag.tv_sec + 0.000001 *
  //  (t.tv_usec - t_stag.tv_usec)<<" task_timeout_tol_:
  //  "<<task_timeout_tol_<<std::endl;
  // std::cout<<"e_diff: "<<e_diff<<" e_diff_tol_: "<<task_diff_tol_<<std::endl;

  if (e <= task_error_tol_) {
    std::cerr << std::endl
              << "STATE CHANGE:" << std::endl
              << "monitored tasks: ";
    for (unsigned int i = 0; i < monitored_tasks_.size(); i++)
      std::cerr << monitored_tasks_[i] << " ";

    std::cerr << std::endl << "task statuses: " << std::endl;
    for (std::vector<hqp_controllers_msgs::TaskStatus>::iterator it =
             msg->statuses.begin();
         it != msg->statuses.end(); ++it)
      std::cerr << "id: " << it->id << " name: " << it->name
                << " progress: " << it->progress << std::endl;

    std::cerr << "e: " << e << std::endl << std::endl;

    // ROS_INFO("Task status switch!");
    task_status_changed_ = true;
    task_success_ = true;
    cond_.notify_one();
  } else if (e_diff <=
             task_diff_tol_)  //(task progresses ain't changing no more)
  {
    stagnation_flag = true;
    std::cerr << "task progress stagnating since:"
              << t.tv_sec - t_stag.tv_sec +
                     0.000001 * (t.tv_usec - t_stag.tv_usec)
              << " s, e_diff is: " << e_diff << std::endl;
    if ((t.tv_sec - t_stag.tv_sec + 0.000001 * (t.tv_usec - t_stag.tv_usec)) >
        task_timeout_tol_) {
      task_status_changed_ = true;
      task_success_ = true;
      ROS_WARN("Task execution timeout!");
      std::cerr << "monitored tasks: ";
      for (unsigned int i = 0; i < monitored_tasks_.size(); i++)
        std::cerr << monitored_tasks_[i] << " ";

      std::cerr << std::endl << "task statuses: " << std::endl;
      for (std::vector<hqp_controllers_msgs::TaskStatus>::iterator it =
               msg->statuses.begin();
           it != msg->statuses.end(); ++it)
        std::cerr << "id: " << it->id << " name: " << it->name
                  << " progress: " << it->progress << std::endl;

      std::cerr << "e: " << e << std::endl << std::endl;
      // std::cerr<<"t: "<<"t - t_stag: "<<t.tv_sec - t_stag.tv_sec + 0.000001 *
      // (t.tv_usec - t_stag.tv_usec)<<" task_timeout_tol_:
      // "<<task_timeout_tol_<<std::endl;
      // std::cerr<<"e_diff: "<<e_diff<<" e_diff_tol_:
      // "<<task_diff_tol_<<std::endl<<std::endl;

      stagnation_flag = false;
      cond_.notify_one();
    }
  }
}
//-----------------------------------------------------------------
void DemoGrasping::jointStateCallback(const sensor_msgs::JointStatePtr& msg) {
  boost::mutex::scoped_lock lock(force_change_m_, boost::try_to_lock);
  if (!lock) return;

  // if(write_jnts_)
  // 	bag_.write("joint_states", ros::Time::now(), *msg);
}
//-----------------------------------------------------------------
void DemoGrasping::imgCallback(const sensor_msgs::ImagePtr& msg) {
  boost::mutex::scoped_lock lock(force_change_m_, boost::try_to_lock);
  if (!lock) return;

  // if(write_img_)
  // 	bag_.write("rgb_image_raw", ros::Time::now(), *msg);
  // write_img_ = false;
}
//-----------------------------------------------------------------
void DemoGrasping::tfCallback(const tf2_msgs::TFMessagePtr& msg) {
  boost::mutex::scoped_lock lock(force_change_m_, boost::try_to_lock);
  if (!lock) return;

  // if(write_tf_)
  // 	bag_.write("tf", ros::Time::now(), *msg);

  // one-shot writing
  // write_tf_ =false;
}
//-----------------------------------------------------------------
void DemoGrasping::clusterCallback(const sensor_msgs::PointCloud2Ptr& msg) {
  boost::mutex::scoped_lock lock(force_change_m_, boost::try_to_lock);
  if (!lock) return;

  // if(write_cluster_)
  // 	bag_.write("result_cloud", ros::Time::now(), *msg);

  // //one-shot writing
  // write_cluster_ =false;
}
//-----------------------------------------------------------------
bool DemoGrasping::loadPersistentTasks() {
  hqp_controllers_msgs::LoadTasks persistent_tasks;
  persistent_tasks.request.task_definitions = "task_definitions";
  if (!load_tasks_clt_.call(persistent_tasks)) return false;

  unsigned int n_tasks = persistent_tasks.response.ids.size();
  ROS_ASSERT(n_tasks >= n_jnts);  // Make sure the joint limits are set

  // visualize (some of) the loaded tasks
  for (unsigned int i = n_jnts; i < n_tasks;
       i++)  // first 7 tasks are supposedly for joint limits
    pers_task_vis_ids_.push_back(persistent_tasks.response.ids[i]);

  if (!visualizeStateTasks(pers_task_vis_ids_)) return false;

  return true;
}
//-----------------------------------------------------------------
bool DemoGrasping::startDemo(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res) {
  std_srvs::Empty srv;

#if 0
#endif
  deactivateHQPControl();
  resetState();
  reset_hqp_control_clt_.call(srv);
  pers_task_vis_ids_.clear();

  if (!loadPersistentTasks()) {
    ROS_ERROR("Could not load persistent tasks!");
    safeShutdown();
    return false;
  }

  if (!with_gazebo_) {
    std_srvs::Empty empty;
    if (!reset_map_clt_.call(empty)) {
      ROS_ERROR("could not clear map");
      ROS_BREAK();
    }

    yumi_hw::YumiGrasp gr;
    gr.request.gripper_id = 1;

    if (!open_gripper_clt_.call(gr)) {
      ROS_ERROR("could not open gripper");
      ROS_BREAK();
    }

    sleep(1);

    gr.request.gripper_id = 2;
    if (!open_gripper_clt_.call(gr)) {
      ROS_ERROR("could not open gripper");
      ROS_BREAK();
    }
    // some time for the sdf to build up a bit
    sleep(2);
  }

  {  // MANIPULATOR SENSING CONFIGURATION
    ROS_INFO("Trying to put the manipulator in sensing configuration.");
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    task_status_changed_ = false;
    task_success_ = false;
    deactivateHQPControl();
    if (!resetState()) {
      ROS_ERROR("Could not reset the state!");
      safeShutdown();
      return false;
    }
    // if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
    // {
    //     safeShutdown();
    //     return false;
    // }

    if (!setJointConfiguration(sensing_config_)) {
      ROS_ERROR("Could not set manipulator sensing state!");
      safeShutdown();
      return false;
    }
    task_error_tol_ = 1e-2;
    activateHQPControl();

    while (!task_status_changed_) cond_.wait(lock);

    if (!task_success_) {
      ROS_ERROR("Could not complete the manipulator sensing state tasks!");
      safeShutdown();
      return false;
    }
    ROS_INFO("Manipulator sensing state tasks executed successfully.");
  }

  {  // GRASP APPROACH
    ROS_INFO("Trying grasp approach.");
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    task_status_changed_ = false;
    task_success_ = false;
    deactivateHQPControl();
    if (!resetState()) {
      ROS_ERROR("Could not reset the state!");
      safeShutdown();
      return false;
    }

    if (!with_gazebo_) {
      if (!getGraspInterval()) {
        ROS_WARN("Could not obtain the grasp intervall!");
        safeReset();
        return false;
      }
    }

    if (!setGraspApproach()) {
      ROS_ERROR("Could not set the grasp approach!");
      safeShutdown();
      return false;
    }
    task_error_tol_ = 1e-4;
    activateHQPControl();

    while (!task_status_changed_) cond_.wait(lock);

    if (!task_success_) {
      ROS_ERROR("Could not complete the grasp approach tasks!");
      safeShutdown();
      return false;
    }

    ROS_INFO("Grasp approach tasks executed successfully.");
  }

  if (!with_gazebo_) {
    yumi_hw::YumiGrasp gr;
    gr.request.gripper_id = (grasp_.e_frame_ == "gripper_l_base") ? 1 : 2;

    if (!close_gripper_clt_.call(gr)) {
      ROS_ERROR("could not close gripper");
      ROS_BREAK();
    }

    sleep(1);

  } else {
    sleep(1);
  }

  {  // OBJECT EXTRACT
    ROS_INFO("Trying object extract.");
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    task_status_changed_ = false;
    task_success_ = false;
    deactivateHQPControl();
    if (!resetState()) {
      ROS_ERROR("Could not reset the state!");
      safeShutdown();
      return false;
    }
    if (!setObjectExtract()) {
      ROS_ERROR("Could not set the object extract!");
      safeShutdown();
      return false;
    }

    task_error_tol_ = 1e-2;
    activateHQPControl();

    while (!task_status_changed_) cond_.wait(lock);

    if (!task_success_) {
      ROS_ERROR("Could not complete the object extract tasks!");
      safeShutdown();
      return false;
    }
    ROS_INFO("Object extract tasks executed successfully.");
  }

  if (!with_gazebo_) {
    yumi_hw::YumiGrasp gr;
    gr.request.gripper_id = 1;

    if (!open_gripper_clt_.call(gr)) {
      ROS_ERROR("could not open gripper");
      ROS_BREAK();
    }

    sleep(1);
    gr.request.gripper_id = 2;
    if (!open_gripper_clt_.call(gr)) {
      ROS_ERROR("could not open gripper");
      ROS_BREAK();
    }
  } else {
    sleep(1);
  }

  {  // MANIPULATOR TRANSFER CONFIGURATION
    ROS_INFO("Trying to put the manipulator in transfer configuration.");

    boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    task_status_changed_ = false;
    task_success_ = false;
    deactivateHQPControl();
    if (!resetState()) {
      ROS_ERROR("Could not reset the state!");
      safeShutdown();
      return false;
    }

    if (!setJointConfiguration(sensing_config_)) {
      ROS_ERROR("Could not set manipulator transfer configuration!");
      safeShutdown();
      return false;
    }
    task_error_tol_ = 1e-2;
    activateHQPControl();

    while (!task_status_changed_) cond_.wait(lock);

    if (!task_success_) {
      ROS_ERROR(
          "Could not complete the manipulator transfer configuration tasks!");
      safeShutdown();
      return false;
    }
    ROS_INFO("Manipulator transfer configuration tasks executed successfully.");
  }

  deactivateHQPControl();
  resetState();
  reset_hqp_control_clt_.call(srv);
  pers_task_vis_ids_.clear();

#if 0
#endif
  ROS_INFO("DEMO FINISHED.");

  return true;
}
//--------------------------------------------------------------------------
}  // end namespace demo_grasping

/////////////////////////////////
//           MAIN              //
/////////////////////////////////

//---------------------------------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_grasping");

  demo_grasping::DemoGrasping demo_grasping;

  ROS_INFO("Demo grasping node ready");
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
//---------------------------------------------------------------------
