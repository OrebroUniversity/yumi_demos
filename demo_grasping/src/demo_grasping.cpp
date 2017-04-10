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
    : n_jnts(14), hiqp_client_("yumi", "hiqp_joint_velocity_controller") {
  // handle to home
  nh_ = ros::NodeHandle("~");
  // global handle
  n_ = ros::NodeHandle();

  // get params
  nh_.param<bool>("with_gazebo", with_gazebo_, false);
  if (with_gazebo_) ROS_INFO("Grasping experiments running in Gazebo.");

  // register general callbacks
  start_demo_srv_ =
      nh_.advertiseService("start_demo", &DemoGrasping::startDemo, this);
  // look_what_i_found_srv_ = nh_.advertiseService(
  //    "dual_grasp_script", &DemoGrasping::lookWhatIFound, this);

  set_gazebo_physics_clt_ = n_.serviceClient<gazebo_msgs::SetPhysicsProperties>(
      "set_physics_properties");
  // hardcode graciously the frame, pose and bbox of object
  nh_.param<std::string>("grasp_req_frame",
                         planGraspMsg.request.header.frame_id, "world");
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

  planGraspMsg.request.object_radius = object_radius;
  planGraspMsg.request.object_height = object_height;
  planGraspMsg.request.objectPose.position.x = px;
  planGraspMsg.request.objectPose.position.y = py;
  planGraspMsg.request.objectPose.position.z = pz;
  planGraspMsg.request.objectPose.orientation.x = ox;
  planGraspMsg.request.objectPose.orientation.y = oy;
  planGraspMsg.request.objectPose.orientation.z = oz;
  planGraspMsg.request.objectPose.orientation.w = ow;
  planGraspMsg.request.approach_frame = "world";

  get_grasp_interval_clt_ =
      n_.serviceClient<grasp_planner::PlanGrasp>("get_grasp_interval");
  get_grasp_interval_clt_.waitForExistence();

  if (!with_gazebo_) {
    close_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("close_gripper");
    open_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("open_gripper");

    reset_map_clt_ = n_.serviceClient<std_srvs::Empty>("reset_map");
    close_gripper_clt_.waitForExistence();
    open_gripper_clt_.waitForExistence();
    reset_map_clt_.waitForExistence();
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

  // PRE-DEFINED JOINT CONFIGURATIONS
  // configs have to be within the safety margins of the joint limits

  sensing_config_ = {-0.42, -1.48, 1.21,  0.75, -0.80, 0.45, 1.21,
                     0.42,  -1.48, -1.21, 0.75, 0.80,  0.45, 1.21};

  // DEFAULT GRASP
  grasp_.obj_frame_ = "world";         // object frame
  grasp_.e_frame_ = "gripper_r_base";  // endeffector frame
  grasp_.e_.setZero();  // endeffector point expressed in the endeffector frame
  grasp_.isSphereGrasp = false;
  grasp_.isDefaultGrasp = true;
}

void DemoGrasping::safeShutdown() {
  hiqp_client_.resetHiQPController();
  ROS_BREAK();  // I must break you ... ros::shutdown() doesn't seem to do the
                // job
}

void DemoGrasping::safeReset() { hiqp_client_.resetHiQPController(); }

bool DemoGrasping::getGraspInterval() {
  // get the grasp intervall
  get_grasp_interval_clt_.waitForExistence();

  grasp_planner::PlanGrasp rPlanGraspMsg = planGraspMsg;
  grasp_planner::PlanGrasp lPlanGraspMsg = planGraspMsg;
  // RIGHT ARM
  rPlanGraspMsg.request.approach_vector = {0.0, 1.0, 0.0};
  rPlanGraspMsg.request.approach_angle = 1.57;
  // LEFT ARM
  lPlanGraspMsg.request.approach_vector = {0.0, -1.0, 0.0};
  lPlanGraspMsg.request.approach_angle = 1.57;

  // ROS_INFO("Request is:");
  // std::cout<<rPlanGraspMsg.request.object_radius <<" "<<
  //   rPlanGraspMsg.request.object_height <<" "<<
  //   rPlanGraspMsg.request.objectPose.position.x <<" "<<
  //   rPlanGraspMsg.request.objectPose.position.y <<" "<<
  //   rPlanGraspMsg.request.objectPose.position.z <<" "<<
  //   rPlanGraspMsg.request.objectPose.orientation.x <<" "<<
  //   rPlanGraspMsg.request.objectPose.orientation.y <<" "<<
  //   rPlanGraspMsg.request.objectPose.orientation.z <<" "<<
  //   rPlanGraspMsg.request.objectPose.orientation.w <<" "<<
  //   rPlanGraspMsg.request.approach_frame <<" "<<
  //   rPlanGraspMsg.request.approach_vector[0] <<" "<<
  //   rPlanGraspMsg.request.approach_vector[1] <<" "<<
  //   rPlanGraspMsg.request.approach_vector[2] <<" "<<
  //   rPlanGraspMsg.request.approach_angle <<"\n";

  if (!get_grasp_interval_clt_.call(rPlanGraspMsg)) {
    ROS_ERROR("Could not call grasp client");
    return false;
  }

  if (!rPlanGraspMsg.response.success) {
    ROS_WARN("Planning was NOT successful! (RIGHT)");
  }

  if (!get_grasp_interval_clt_.call(lPlanGraspMsg)) {
    ROS_ERROR("Could not call grasp client");
    return false;
  }

  if (!lPlanGraspMsg.response.success) {
    ROS_WARN("Planning was NOT successful! (LEFT)");
  }

  if (!lPlanGraspMsg.response.success && !rPlanGraspMsg.response.success) {
    ROS_ERROR("Planning was NOT successful with either arm, ABORT!");
    return false;
  }

  if (rPlanGraspMsg.response.volume > lPlanGraspMsg.response.volume) {
    // grasping with right
    ROS_INFO("Grasping with right");
    planGraspMsg.response = rPlanGraspMsg.response;
    grasp_.e_frame_ = "gripper_r_base";  // endeffector frame
  } else {
    // grasping with left
    ROS_INFO("Grasping with left");
    planGraspMsg.response = lPlanGraspMsg.response;
    std::cout << planGraspMsg.response;
    grasp_.e_frame_ = "gripper_l_base";  // endeffector frame
  }

  ROS_INFO("Processing Grasp Constraints!");
  grasp_.isDefaultGrasp = false;

  ROS_ASSERT(planGraspMsg.response.constraints.size() == 6);
  grasp_.obj_frame_ = planGraspMsg.response.frame_id;

  grasp_.lower = planGraspMsg.response.constraints[0];
  grasp_.upper = planGraspMsg.response.constraints[1];
  grasp_.left = planGraspMsg.response.constraints[2];
  grasp_.right = planGraspMsg.response.constraints[3];
  grasp_.inner = planGraspMsg.response.constraints[4];
  grasp_.outer = planGraspMsg.response.constraints[5];

  ROS_ASSERT(grasp_.lower.type == "plane");
  ROS_ASSERT(grasp_.upper.type == "plane");
  ROS_ASSERT(grasp_.left.type == "plane");
  ROS_ASSERT(grasp_.right.type == "plane");

  ROS_ASSERT(grasp_.lower.parameters.size() == 4);
  ROS_ASSERT(grasp_.upper.parameters.size() == 4);
  ROS_ASSERT(grasp_.left.parameters.size() == 4);
  ROS_ASSERT(grasp_.right.parameters.size() == 4);

  // INNER GRASP CYLINDER OR SPHERE.
  ROS_ASSERT(grasp_.inner.type == "cylinder" || grasp_.inner.type == "sphere");
  ROS_ASSERT(grasp_.outer.type == "cylinder" || grasp_.outer.type == "sphere");

  grasp_.isSphereGrasp = planGraspMsg.response.constraints[4].type == "sphere";

  // Cylinder grasp.
  if (!grasp_.isSphereGrasp) {
    ROS_ASSERT(grasp_.inner.parameters.size() == 8);
    ROS_ASSERT(grasp_.outer.type == "cylinder");
    ROS_ASSERT(grasp_.outer.parameters.size() == 8);
  }

  // This is a sphere grasp.
  else {
    ROS_ASSERT(grasp_.inner.parameters.size() == 4);
    ROS_ASSERT(grasp_.outer.type == "sphere");
    ROS_ASSERT(grasp_.outer.parameters.size() == 4);
  }

  /*
  //set the angle to the max opening allowed
  grasp_.angle = planGraspMsg.response.max_oa;
  //if we are too open, add a little safety margin
  if(grasp_.angle < MIN_OPENING) grasp_.angle = MIN_OPENING;
  //make sure we are never closed more than the allowed angle
  if(grasp_.angle >
  planGraspMsg.response.min_oa-OPENING_SAFETY_MARGIN) grasp_.angle =
  planGraspMsg.response.min_oa-OPENING_SAFETY_MARGIN;
  //if we are too open, add a little safety margin
  if(grasp_.angle < MIN_OPENING) grasp_.angle = MIN_OPENING;
  */

  ROS_INFO("GRIPPER WILL GO TO %f", grasp_.angle);

  // Plane normals need to point in opposite directions to give a closed
  // interval
  // TODO: validate planes like the commented out code below.
  // ROS_ASSERT((grasp_.n1_.transpose() * grasp_.n2_) <= 0.0);
  return true;
}

/*
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
*/
/*-----------------------------------------------------------------
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
*/
//-----------------------------------------------------------------
bool DemoGrasping::doGraspAndLift() {
  if (!with_gazebo_) {
    if (grasp_.isDefaultGrasp) {
      ROS_WARN("Grasp is default grasp!");
      return false;
    }
  }

  hiqp_msgs::Task gripperZToObjectZT, gripper_YToWorldZT;
  hiqp_msgs::Task upperT, lowerT, leftT, rightT, innerT, outerT;

  hiqp_msgs::Primitive eef_point;
  hiqp_msgs::Primitive gripperZ, gripperMinusY, worldZ, cylinderZ;

  // Some assertions to make sure that the grasping constraints are alright.
  // TODO: Write this.

  // Primitive for the End effector point.
  eef_point = hiqp_ros::createPrimitiveMsg(
      "point_eef", "point", grasp_.e_frame_, false, {0, 0, 1, 0.2},
      {grasp_.e_(0), grasp_.e_(1), grasp_.e_(2)});

  // Define tasks

  // LOWER GRASP INTERVAL PLANE
  lowerT = hiqp_ros::createTaskMsg(
      "lower", 2, false, true, true,
      {"TDefGeomProj", "point", "plane",
       eef_point.name + " > " + grasp_.lower.name},
      {"TDynLinear", std::to_string(1.25 * DYNAMICS_GAIN)});

  upperT = hiqp_ros::createTaskMsg(
      "upper", 2, false, true, true,
      {"TDefGeomProj", "point", "plane",
       eef_point.name + " > " + grasp_.upper.name},
      {"TDynLinear", std::to_string(1.25 * DYNAMICS_GAIN)});

  // Left and Right limits only for non-hardcoded grasps
  if (!grasp_.isDefaultGrasp) {
    // LEFT GRASP INTERVAL PLANE
    leftT =
        hiqp_ros::createTaskMsg("left", 2, false, true, true,
                                {"TDefGeomProj", "point", "plane",
                                 eef_point.name + " > " + grasp_.left.name},
                                {"TDynLinear", std::to_string(DYNAMICS_GAIN)});

    // RIGHT GRASP INTERVAL PLANE

    rightT =
        hiqp_ros::createTaskMsg("right", 2, false, true, true,
                                {"TDefGeomProj", "point", "plane",
                                 eef_point.name + " > " + grasp_.right.name},
                                {"TDynLinear", std::to_string(DYNAMICS_GAIN)});
  }

  if (!grasp_.isSphereGrasp) {
    // INNER CONSTRAINT CYLINDER
    // Gripper should be outside the cylinder. So ">".
    innerT = hiqp_ros::createTaskMsg(
        "inner", 2, false, true, true,
        {"TDefGeomProj", "point", "cylinder",
         eef_point.name + " > " + grasp_.inner.name},
        {"TDynLinear", std::to_string(0.75 * DYNAMICS_GAIN)});

    outerT = hiqp_ros::createTaskMsg(
        "outer", 2, false, true, true,
        {"TDefGeomProj", "point", "cylinder",
         eef_point.name + " < " + grasp_.outer.name},
        {"TDynLinear", std::to_string(0.75 * DYNAMICS_GAIN)});

    gripperZ = hiqp_ros::createPrimitiveMsg(
        "eef_z_axis", "line", grasp_.e_frame_, false, {0.0, 0.0, 1.0, 0.2},
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    gripperMinusY = hiqp_ros::createPrimitiveMsg(
        "eef_y_axis", "line", grasp_.e_frame_, false, {0.0, 0.0, 1.0, 0.2},
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0});

    worldZ = hiqp_ros::createPrimitiveMsg("world_z", "line", grasp_.obj_frame_,
                                          false, {0.0, 0.0, 1.0, 0.2},
                                          {0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    cylinderZ = hiqp_ros::createPrimitiveMsg(
        "cylinder_z", "line", grasp_.obj_frame_, false, {0.0, 0.0, 1.0, 0.2},
        {grasp_.inner.parameters[0], grasp_.inner.parameters[1],
         grasp_.inner.parameters[2], grasp_.inner.parameters[3],
         grasp_.inner.parameters[4], grasp_.inner.parameters[5]});

    // Hand parallel to the z axis.

    gripperZToObjectZT = hiqp_ros::createTaskMsg(
        "z_project", 2, false, true, true,
        {"TDefGeomProj", "line", "line",
         cylinderZ.name + " = " + gripperZ.name},
        {"TDynLinear", std::to_string(3.0 * DYNAMICS_GAIN)});

    gripper_YToWorldZT = hiqp_ros::createTaskMsg(
        "y_align", 2, false, true, true,
        {"TDefGeomAlign", "line", "line",
         gripperMinusY.name + " = " + worldZ.name, "0.05"},
        {"TDynLinear", std::to_string(3.0 * DYNAMICS_GAIN)});

  } else {
    // TODO: add constraints for grasp.
    // TODO: add gripper alignment constraints
  }

  // Set the primitives.
  hiqp_client_.setPrimitives(
      {eef_point, grasp_.upper, grasp_.lower, grasp_.left, grasp_.right,
       grasp_.inner, grasp_.outer, gripperMinusY, cylinderZ, worldZ, gripperZ});

  // Set the tasks
  hiqp_client_.setTasks({upperT, lowerT});
  hiqp_client_.setTasks({gripperZToObjectZT, gripper_YToWorldZT});
  hiqp_client_.setTasks({innerT, outerT, leftT, rightT});

  using hiqp_ros::TaskDoneReaction;

  // Wait for completion.
  hiqp_client_.waitForCompletion(
      {upperT.name, lowerT.name, gripperZToObjectZT.name,
       gripper_YToWorldZT.name, innerT.name, outerT.name, leftT.name,
       rightT.name},
      {TaskDoneReaction::REMOVE, TaskDoneReaction::REMOVE,
       TaskDoneReaction::NONE, TaskDoneReaction::NONE, TaskDoneReaction::NONE,
       TaskDoneReaction::NONE, TaskDoneReaction::NONE, TaskDoneReaction::NONE},
      {1e-4, 1e-4, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-4});

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

  // --------------- //
  // --- Extract --- //
  // --------------- //

  hiqp_msgs::Primitive lowerExtractPlane, upperExtractPlane;
  lowerExtractPlane = grasp_.lower;
  lowerExtractPlane.parameters[3] += 0.20;
  lowerExtractPlane.name = "lower_extract_plane";

  upperExtractPlane = grasp_.upper;
  upperExtractPlane.parameters[3] -= 0.23;
  upperExtractPlane.name = "upper_extract_plane";

  // Additional primitives for extract position.
  hiqp_client_.setPrimitives({lowerExtractPlane, upperExtractPlane});

  // Modify the tasks.
  lowerT = hiqp_ros::createTaskMsg(
      "lower", 2, false, true, true,
      {"TDefGeomProj", "point", "plane",
       eef_point.name + " > " + lowerExtractPlane.name},
      {"TDynLinear", std::to_string(1.25 * DYNAMICS_GAIN)});

  upperT = hiqp_ros::createTaskMsg(
      "upper", 2, false, true, true,
      {"TDefGeomProj", "point", "plane",
       eef_point.name + " > " + upperExtractPlane.name},
      {"TDynLinear", std::to_string(1.25 * DYNAMICS_GAIN)});

  // Add these tasks.
  hiqp_client_.setTasks({lowerT, upperT});

  // Wait for completion.
  hiqp_client_.waitForCompletion(
      {upperT.name, lowerT.name, gripperZToObjectZT.name,
       gripper_YToWorldZT.name, innerT.name, outerT.name, leftT.name,
       rightT.name},
      {TaskDoneReaction::REMOVE, TaskDoneReaction::REMOVE,
       TaskDoneReaction::REMOVE, TaskDoneReaction::REMOVE,
       TaskDoneReaction::REMOVE, TaskDoneReaction::REMOVE,
       TaskDoneReaction::REMOVE, TaskDoneReaction::REMOVE},
      {1e-2, 1e-2, 1e-6, 1e-6, 1e-2, -1e-2, 1e-2, 1e-2});

  hiqp_client_.removePrimitives(
      {eef_point.name, grasp_.upper.name, grasp_.lower.name, grasp_.left.name,
       grasp_.right.name, grasp_.inner.name, grasp_.outer.name,
       gripperMinusY.name, cylinderZ.name, worldZ.name, gripperZ.name,
       lowerExtractPlane.name, upperExtractPlane.name});

  return true;
}

//-----------------------------------------------------------------
// bool DemoGrasping::loadPersistentTasks() {
//   hqp_controllers_msgs::LoadTasks persistent_tasks;
//   persistent_tasks.request.task_definitions = "task_definitions";
//   if (!load_tasks_clt_.call(persistent_tasks)) return false;

//   unsigned int n_tasks = persistent_tasks.response.ids.size();
//   ROS_ASSERT(n_tasks >= n_jnts);  // Make sure the joint limits are set

//   // visualize (some of) the loaded tasks
//   for (unsigned int i = n_jnts; i < n_tasks;
//        i++)  // first 7 tasks are supposedly for joint limits
//     pers_task_vis_ids_.push_back(persistent_tasks.response.ids[i]);

//   if (!visualizeStateTasks(pers_task_vis_ids_)) return false;

//   return true;
// }
//-----------------------------------------------------------------
bool DemoGrasping::startDemo(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res) {
  // hiqp_client_.loadPersistentTasks(); TODO: Must implement this.

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

  // MANIPULATOR SENSING CONFIGURATION

  hiqp_client_.setJointAngles(sensing_config_);

  // TODO: Detect Stagnation.

  // GRASP APPROACH
  ROS_INFO("Trying grasp approach.");

  // if (!with_gazebo_) {
  if (!getGraspInterval()) {
    ROS_WARN("Could not obtain the grasp interval!");
    safeReset();
    return false;
  }
  //}

  if (!doGraspAndLift()) {
    ROS_ERROR("Could not set the grasp approach!");
    safeShutdown();
    return false;
  }

  ROS_INFO("Grasp approach tasks executed successfully.");

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

  ROS_INFO("Trying to put the manipulator in transfer configuration.");

  hiqp_client_.setJointAngles(sensing_config_);

  ROS_INFO("DEMO FINISHED.");

  return true;
}
}  // end namespace demo_grasping

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_grasping");

  demo_grasping::DemoGrasping demo_grasping;

  ROS_INFO("Demo grasping node ready");
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
