#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <sys/stat.h>
#include <sys/types.h>

#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include<balancing_grasping/exp_balancing.h>

ExpBalancing::ExpBalancing() {
    
  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  //nh_.param<std::string>("grasp_left_topic",gleft_topic,"/leap_hands/grasp_left");
  nh_.param<std::string>("tf_topic",tf_topic,"/tf");
  nh_.param<std::string>("js_topic",js_topic,"/yumi/joint_states");

  //nh_.param<std::string>("hand_frame_left",hand_frame_left,"teleop_left_frame");
  nh_.param<std::string>("log_directory",log_dir,"log");

  nh_.param<int>("num_pickups",num_pickups,2);
  nh_.param("grasp_threshold",grasp_thresh_tol,0.2); //0 to 1
  nh_.param("joint_task_tol",joint_task_tol,1e-2); //0 to 1
  nh_.param("pre_grasp_task_tol",pre_grasp_task_tol,1e-4); //0 to 1  
  nh_.param("grasp_task_tol", grasp_task_tol,1e-4); //0 to 1
  
  marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("teleop_markers",10);
  //    gripper_pub_ = nh_.advertise<robotiq_85_msgs::GripperCmd> ("grasp_cmd",10);

  //  grasp_left_sub_ = n_.subscribe(gleft_topic, 1, &ExpBalancing::grasp_left_callback, this);
  joint_state_sub_ = n_.subscribe(js_topic, 1, &ExpBalancing::js_callback, this);
  tf_sub_ = n_.subscribe(tf_topic, 1, &ExpBalancing::tf_callback, this);

  start_demo_ = nh_.advertiseService("start_demo", &ExpBalancing::start_demo_callback, this);
  next_task_srv_ = nh_.advertiseService("next_task", &ExpBalancing::next_task_callback, this);
  quit_demo_ = nh_.advertiseService("quit_demo", &ExpBalancing::quit_demo_callback, this);
   
  std::string robot_namespace; 
  nh_.param<std::string>("robot_namespace",robot_namespace,"yumi");
  hiqp_client_ = std::shared_ptr<hiqp_ros::HiQPClient> (new hiqp_ros::HiQPClient(robot_namespace, "hiqp_joint_velocity_controller"));

  tf_pub_timer = nh_.createTimer(ros::Duration(0.1), &ExpBalancing::tf_pub_callback, this);
  marker_pub_timer = nh_.createTimer(ros::Duration(0.1), &ExpBalancing::marker_pub_callback, this);
  quit_demo = true;
  next_task = false;
  publish_markers = false;
  loadGeometricPrimitivesFromParamServer();
  loadTasksFromParamServer("joint_config_tasks", joint_tasks, joint_task_names);
  loadTasksFromParamServer("pre_grasp_tasks", pre_grasp_tasks, pre_grasp_task_names);
  loadTasksFromParamServer("grasp_tasks", grasp_tasks, grasp_task_names);  
  // loadTasksFromParamServer("pick_assisted_tasks", pick_assisted_tasks, pick_assisted_task_names);
  // loadTasksFromParamServer("drop_assisted_tasks", drop_assisted_tasks, drop_assisted_task_names);
  // loadTasksFromParamServer("point_assisted_tasks", point_assisted_tasks, point_assisted_task_names);

  //---- setup TF frames we want to publish ----//
  //
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.35, -0.1, 0.2) );
  tf::Quaternion q;
  q.setRPY(-1.57, 1.57, 0.0);
  transform.setRotation(q);
  target_poses.push_back(tf::StampedTransform(transform, ros::Time::now(), "yumi_base_link", "grasp_r_frame"));
  
  transform.setOrigin( tf::Vector3(0.35, 0.1, 0.2) );
  q.setRPY(1.57, -1.57, 0.0);
  transform.setRotation(q);
  target_poses.push_back(tf::StampedTransform(transform, ros::Time::now(), "yumi_base_link", "grasp_l_frame"));
  
  //  current_target_id=-1;
  std::srand(std::time(0)); // use current time as seed for random generator
  bag_is_open = false;

  //----- markers ------/
  // addCylinderMarker(object_ass, obj_pos, obj_axis, 0.1, obj_frame,0.3, "object", 0.1, 0.9, 0.1);
  // addCylinderMarker(object_ass, bin_pos, bin_axis, 0.1, bin_frame,0.3, "object", 0.1, 0.9, 0.1);

  // addCylinderMarker(object_noass, obj_pos, obj_axis, 0.1, obj_frame,0.3, "object", 0.9, 0.1, 0.1);
  // addCylinderMarker(object_noass, bin_pos, bin_axis, 0.1, bin_frame,0.3, "object", 0.9, 0.1, 0.1);
   
  // std::string gripper_mesh_resource_path; 
  // nh_.param<std::string>("gripper_mesh_resource_path",gripper_mesh_resource_path,
  // 			 "package://amici_teleop/meshes/gripper_full_scaled.stl");

  // visualization_msgs::Marker marker;
  // marker.ns = "gripper_mesh";
  // marker.header.frame_id = "target_frame";
  // marker.header.stamp = ros::Time::now();
  // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // marker.mesh_resource = gripper_mesh_resource_path;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.lifetime = ros::Duration(0.2);
  // marker.id = 0;
  // marker.pose.position.x = 0;
  // marker.pose.position.y = 0;
  // marker.pose.position.z = 0;
  // marker.pose.orientation.x = 0;
  // marker.pose.orientation.y = 0;
  // marker.pose.orientation.z = 0;
  // marker.pose.orientation.w = 1;
  // marker.scale.x = 1;
  // marker.scale.y = 1;
  // marker.scale.z = 1;
  // marker.color.r = 0.9;
  // marker.color.g = 0.1;
  // marker.color.b = 0.1;
  // marker.color.a = 0.1;
  // gripper_target.markers.push_back(marker);
  // marker.color.r = 0.1;
  // marker.color.g = 0.9;
  // marker.color.b = 0.1;
  // marker.color.a = 0.1;
  // gripper_target_ass.markers.push_back(marker);

}

//void ExpBalancing::grasp_left_callback(const std_msgs::Float32::ConstPtr& msg) {
//     robotiq_85_msgs::GripperCmd cmd;
//     cmd.emergency_release = false;
//     cmd.stop = false;       
//     cmd.position = (1-msg->data)*0.085;
//     cmd.position = cmd.position > 0.085 ? 0.085 : (cmd.position < 0 ? 0 : cmd.position);
//     cmd.speed = 0.05;
//     cmd.force =10;
//     gripper_pub_.publish(cmd);
// #if 0
//     if(msg->data > grasp_thresh+grasp_thresh_tol) {
// 	cmd.position = 0.0;
// 	cmd.speed = 0.05;
// 	cmd.force =10;
// 	gripper_pub_.publish(cmd);
//     }
//     if(msg->data < grasp_thresh-grasp_thresh_tol) { 
// 	cmd.position = 0.085;
// 	cmd.speed = 0.05;
// 	cmd.force =10;
// 	gripper_pub_.publish(cmd);
//     }
// #endif
//}

void ExpBalancing::expMainLoop() {

  std::vector<std::string> no_task_names, prev_task_names;
  std::vector<hiqp_ros::TaskDoneReaction> reactions;
  std::vector<double> tolerances;
    
  while(true) {
    //each iteration of this loop is a new set of experiments
    //    current_target_id=-1;
	
    {	
      boost::mutex::scoped_lock lock(bools_mutex);
      while(quit_demo) {
      	cond_.wait(lock);
      }

    }
    next_task = true;
    cond_.notify_one();
    //---------------------------------------------------------------------------------------//	
    //first task: move robot to init joint configuration
    ROS_INFO("TASK SET 1: moving to init joint configuration.");
    {
      reactions.clear();
      tolerances.clear();
      for(int i=0; i<joint_task_names.size(); i++) { 
	reactions.push_back(hiqp_ros::TaskDoneReaction::REMOVE);
	tolerances.push_back(joint_task_tol);
      }
      
      boost::mutex::scoped_lock lock(bools_mutex);
      while(!next_task && !quit_demo) {
	cond_.wait(lock);
      }
      if(quit_demo) {
	//reset variables and so on
	continue;
      }
      if(next_task) {
	//execute next task
	hiqp_client_->setTasks(joint_tasks);
      }
      next_task = false;
    }
	
    hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);
    next_task = true;
    cond_.notify_one();
    //---------------------------------------------------------------------------------------//
    //second task: move to pre-grasp poses
    ROS_INFO("TASK SET 2: moving to pre-grasp poses.");    
    {
      reactions.clear();
      tolerances.clear();
      for(int i=0; i<pre_grasp_task_names.size(); i++) { 
	reactions.push_back(hiqp_ros::TaskDoneReaction::REMOVE);
	tolerances.push_back(pre_grasp_task_tol);
      }

      boost::mutex::scoped_lock lock(bools_mutex);
      while(!next_task && !quit_demo) {
	cond_.wait(lock);
      }
      if(quit_demo) {
	//reset variables and so on
	continue;
      }
      if(next_task) {
	//execute next task
	hiqp_client_->setTasks(pre_grasp_tasks);
      }
      next_task = false;
    }
	
    hiqp_client_->waitForCompletion(pre_grasp_task_names, reactions, tolerances);
    next_task = true;
    cond_.notify_one();    
    //---------------------------------------------------------------------------------------//
    //third task: grasp poses
    ROS_INFO("TASK SET 3: moving to grasp poses.");        
    {
      reactions.clear();
      tolerances.clear();
      for(int i=0; i<grasp_task_names.size(); i++) { 
	reactions.push_back(hiqp_ros::TaskDoneReaction::REMOVE);
	tolerances.push_back(grasp_task_tol);
      }

      boost::mutex::scoped_lock lock(bools_mutex);
      while(!next_task && !quit_demo) {
	cond_.wait(lock);
      }
      if(quit_demo) {
	//reset variables and so on
	continue;
      }
      if(next_task) {
	//execute next task
	hiqp_client_->setTasks(grasp_tasks);
      }
      next_task = false;
    }
	
    hiqp_client_->waitForCompletion(grasp_task_names, reactions, tolerances);
    
    //---------------------------------------------------------------------------------------//
    //second task: pointing task 
    //int rv = std::rand();
    // {
    //   prev_task_names.clear();

    //   //free-form
    //   ROS_INFO("Pointing task, first no assistance");
    //   current_target_id=0;
    //   setTasks(teleop_tasks, no_task_names);
    //   setPubMarkers(gripper_target);
    //   startNextBag("task1_noassist.bag");
      
    //   while(current_target_id < target_poses.size()) { 
    // 	ROS_INFO("Going for pose %lu",current_target_id);
    // 	//wait for completion
    // 	//	if(!waitForPoseAlignment()) break;
    // 	//increment target id
    // 	current_target_id++;
    //   }
    //   hiqp_client_->removeTasks(teleop_task_names);
    //   closeCurrentBag();
    //   stopPubMarkers();
    //   if(current_target_id < target_poses.size()) continue;

    //   //assisted
    //   current_target_id=0;
    //   ROS_INFO("Back to initial configuration");
    //   hiqp_client_->setTasks(joint_tasks);
    //   hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);
    //   setTasks(point_assisted_tasks, no_task_names);
    //   setPubMarkers(gripper_target_ass);
    //   startNextBag("task1_assist.bag");
    //   while(current_target_id < target_poses.size()) { 
    // 	ROS_INFO("Going for pose %lu",current_target_id);
    // 	//wait for completion
    // 	//	if(!waitForPoseAlignment()) break;
    // 	//increment target id
    // 	current_target_id++;
    //   }
    //   closeCurrentBag();
    //   stopPubMarkers();
    //   hiqp_client_->removeTasks(point_assisted_task_names);
    //   if(current_target_id < target_poses.size()) continue;
    // }
    //---------------------------------------------------------------------------------------//	
    //third task: picking task
    // {
    //   ROS_INFO("Back to initial configuration");
    //   hiqp_client_->setTasks(joint_tasks);
    //   hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);

    //   std::string ass_name, noass_name;
    //   std::stringstream ss;
    //   for(int run_number=0; run_number < num_pickups; run_number++) {
	
    // 	ss<<"task2_assist_r"<<run_number<<".bag";
    // 	ss>>ass_name;
    // 	ss.clear();
	    
    // 	ss<<"task2_noassist_r"<<run_number<<".bag";
    // 	ss>>noass_name;
    // 	ss.clear();
	
    // 	//      rv = std::rand();

    // 	ROS_INFO("Pick and drop, first NO assistance");
    // 	//free-form
    // 	if(!setTasks(teleop_tasks, no_task_names)) continue;
    // 	startNextBag(noass_name);
    // 	setPubMarkers(object_noass);
    // 	if(!setTasks(joint_tasks, teleop_task_names)) continue;
    // 	closeCurrentBag();
    // 	stopPubMarkers();
    // 	hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);

    // 	//assisted picking
    // 	if(!setTasks(pick_assisted_tasks, no_task_names)) continue;
    // 	startNextBag(ass_name);
    // 	setPubMarkers(object_ass);
    // 	if(!setTasks(drop_assisted_tasks, pick_assisted_task_names)) continue;
    // 	if(!setTasks(joint_tasks, drop_assisted_task_names)) continue;
    // 	closeCurrentBag();
    // 	stopPubMarkers();
    // 	hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);
    //   }
    // }
    //---------------------------------------------------------------------------------------//	
    //quitting
    {	    
      boost::mutex::scoped_lock lock(bools_mutex);
      //close-up log files
      next_task = false;
      quit_demo = true;
    }
#if 1
    //wait for quitting
    {	    
      boost::mutex::scoped_lock lock(bools_mutex);
      while(!next_task && !quit_demo) {
	cond_.wait(lock);
      }

	//remove previous tasks
      //	hiqp_client_->removeTasks(grasp_task_names);

      //close-up log files
      next_task = false;
    }
#endif
    ROS_INFO("Quit Demo.");
  }

}
bool ExpBalancing::setTasks(std::vector<hiqp_msgs::Task> &next_tasks,
			    std::vector<std::string> &prev_task_names) {
    
  boost::mutex::scoped_lock lock(bools_mutex);
  while(!next_task && !quit_demo) {
    cond_.wait(lock);
  }
  if(prev_task_names.size() > 0) {
    //remove previous tasks
    hiqp_client_->removeTasks(prev_task_names);
  }
  if(quit_demo) {
    //reset variables and so on
    return false;
  }
  std::cerr<<"in setTasks(...), next_task is: "<<next_task<<std::endl;
  if(next_task) {
    //execute next task
    hiqp_client_->setTasks(next_tasks);
  }
  next_task = false;
  return true;
}

// bool ExpBalancing::waitForPoseAlignment() {
    
//   boost::mutex::scoped_lock lock(bools_mutex);
//   while(!next_task && !quit_demo) {
//     cond_.wait(lock);
//   }
//   if(quit_demo) {
//     //reset variables and so on
//     return false;
//   }
//   next_task = false;
//   return true;
// }

bool ExpBalancing::start_demo_callback(std_srvs::Empty::Request  &req,
				       std_srvs::Empty::Response &res ) {


  //HERE setup loggers and demo run id
  setupNewExperiment();
  //set next task
  ROS_INFO("Starting new experiment");
  {
    boost::mutex::scoped_lock lock(bools_mutex, boost::try_to_lock);
    if(!lock) return false;
    quit_demo = false;
    cond_.notify_one();
  }

  return true;
}

bool ExpBalancing::next_task_callback(std_srvs::Empty::Request  &req,
				      std_srvs::Empty::Response &res ) {

  ROS_INFO("Executing next task");
  {
    boost::mutex::scoped_lock lock(bools_mutex, boost::try_to_lock);
    if(!lock) return false;
    next_task = true;
    cond_.notify_one();
  }

  return true;
}

bool ExpBalancing::quit_demo_callback(std_srvs::Empty::Request  &req,
				      std_srvs::Empty::Response &res ) {

  ROS_INFO("Quitting demo");
  {
    boost::mutex::scoped_lock lock(bools_mutex, boost::try_to_lock);
    if(!lock) return false;
    quit_demo = true;
    cond_.notify_one();
  }

  return true;
}

void ExpBalancing::loadGeometricPrimitivesFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_grasping_geometric_primitives;
  if (!nh_.getParam("grasping_geometric_primitives", hiqp_grasping_geometric_primitives)) {
    ROS_WARN_STREAM("No grasping_geometric_primitives parameter "
		    << "found on the parameter server. No geometric primitives "
		    << "were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_grasping_geometric_primitives.size(); ++i) {
      try {
	std::string name = static_cast<std::string>(hiqp_grasping_geometric_primitives[i]["name"]);
	std::string type = static_cast<std::string>(hiqp_grasping_geometric_primitives[i]["type"]);
	std::string frame_id = static_cast<std::string>(hiqp_grasping_geometric_primitives[i]["frame_id"]);
	bool visible =  static_cast<bool>(hiqp_grasping_geometric_primitives[i]["visible"]);

	XmlRpc::XmlRpcValue& color_xml = hiqp_grasping_geometric_primitives[i]["color"];
	XmlRpc::XmlRpcValue& parameters_xml = hiqp_grasping_geometric_primitives[i]["parameters"];

	std::vector<double> color;
	color.push_back(static_cast<double>(color_xml[0]));
	color.push_back(static_cast<double>(color_xml[1]));
	color.push_back(static_cast<double>(color_xml[2]));
	color.push_back(static_cast<double>(color_xml[3]));

	std::vector<double> parameters;
	for (int j = 0; j < parameters_xml.size(); ++j) {
	  parameters.push_back(static_cast<double>(parameters_xml[j]));
	}

	hiqp_client_->setPrimitive(name, type, frame_id, visible, color, parameters);

	if(name.compare("object_vertical_axis")==0) {
	  //parse object position for marker viz.
	  if(parameters.size() != 6) {
	    ROS_ERROR("could not parse object desired position!");
	    continue;
	  }
	  obj_axis<<parameters[0],parameters[1],parameters[2];
	  obj_pos<<parameters[3],parameters[4],parameters[5];
	  obj_frame = frame_id;

	}
	if(name.compare("bin_vertical_axis")==0) {
	  //parse bin position for marker viz.
	  if(parameters.size() != 6) {
	    ROS_ERROR("could not parse bin desired position!");
	    continue;
	  }
	  bin_axis<<parameters[0],parameters[1],parameters[2];
	  bin_pos<<parameters[3],parameters[4],parameters[5];
	  bin_frame = frame_id;
	}
      } catch (const XmlRpc::XmlRpcException& e) {
	ROS_WARN_STREAM("Error while loading "
			<< "teleop_geometric_primitives parameter from the "
			<< "parameter server. XmlRcpException thrown with message: "
			<< e.getMessage());
	parsing_success = false;
	break;
      }
    }

    if (parsing_success)
      ROS_INFO_STREAM("Loaded and initiated geometric primitives from " << ".yaml file successfully!");
  }
}

bool ExpBalancing::loadTasksFromParamServer(std::string task_group_name, std::vector<hiqp_msgs::Task> &tasks_out, std::vector<std::string> &task_names_out) {
    
  XmlRpc::XmlRpcValue hiqp_grasping_tasks;
  bool parsing_success = true;
  if (!nh_.getParam(task_group_name,
		    hiqp_grasping_tasks)) {
    ROS_WARN_STREAM("No "<<task_group_name<<" parameter found on "
		    << "the parameter server. No tasks were loaded!");
    return false;
  } else {
    for (int i = 0; i < hiqp_grasping_tasks.size(); ++i) {
      try {
	hiqp_msgs::Task teleop_task;
	teleop_task.name =
	  static_cast<std::string>(hiqp_grasping_tasks[i]["name"]);

	XmlRpc::XmlRpcValue& def_params_xml =
	  hiqp_grasping_tasks[i]["def_params"];
	for (int j = 0; j < def_params_xml.size(); ++j) {
	  teleop_task.def_params.push_back(static_cast<std::string>(def_params_xml[j]));
	}

	XmlRpc::XmlRpcValue& dyn_params_xml =
	  hiqp_grasping_tasks[i]["dyn_params"];
	for (int j = 0; j < dyn_params_xml.size(); ++j) {
	  teleop_task.dyn_params.push_back(static_cast<std::string>(dyn_params_xml[j]));
	}

	teleop_task.priority =
	  static_cast<int>(hiqp_grasping_tasks[i]["priority"]);
	teleop_task.visible = static_cast<bool>(hiqp_grasping_tasks[i]["visible"]);
	teleop_task.active = static_cast<bool>(hiqp_grasping_tasks[i]["active"]);
	teleop_task.monitored = static_cast<bool>(hiqp_grasping_tasks[i]["monitored"]);

	tasks_out.push_back(teleop_task);
	task_names_out.push_back(teleop_task.name);

      } catch (const XmlRpc::XmlRpcException& e) {
	ROS_WARN_STREAM(
			"Error while loading "
			<< "hiqp_grasping_tasks parameter from the "
			<< "parameter server. XmlRcpException thrown with message: "
			<< e.getMessage());
	parsing_success = false;
	break;
      }
    }

    if (parsing_success) {
      ROS_INFO("Loaded and initiated tasks from .yaml file successfully!");
    }
  }
  return parsing_success;
}

	
void ExpBalancing::tf_pub_callback(const ros::TimerEvent &te) {
  for(unsigned int i=0; i<target_poses.size(); i++){
    target_poses[i].stamp_ = ros::Time::now();
    tb.sendTransform(target_poses[i]);    
  }
}

void ExpBalancing::marker_pub_callback(const ros::TimerEvent &te) {

  if(publish_markers) {
    for(int i=0; i<current_markers.markers.size(); i++) {
      current_markers.markers[i].header.stamp = ros::Time::now();
      current_markers.markers[i].lifetime = ros::Duration(0.2);
    }
    marker_viz_pub_.publish(current_markers);
  }

}
	
/// creates a folder for bags
void ExpBalancing::setupNewExperiment() {
    
  std::stringstream ss;
  ss << log_dir <<"/exp_"<<std::time(0)<<"/";
  ss>>current_log_dir;

  mkdir(current_log_dir.c_str(), S_IFDIR | S_IRWXG | S_IRWXU);

}

/// starts recording the next bag
void ExpBalancing::startNextBag(std::string bagname) {
    
  bag_mutex.lock();
  if(bag_is_open) {
    //close bag
    current_bag.close();
  }
  std::string bb = current_log_dir+bagname;
  current_bag.open(bb, rosbag::bagmode::Write);
  bag_is_open=true;

  bag_mutex.unlock();
}
	
/// stops recording
void ExpBalancing::closeCurrentBag() {
  bag_mutex.lock();
  if(bag_is_open) {
    //close bag
    current_bag.close();
    bag_is_open=false;
  }
  bag_mutex.unlock();
}

void ExpBalancing::js_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  bag_mutex.lock();
  if(bag_is_open) {
    current_bag.write(js_topic, msg->header.stamp, msg);
  }
  bag_mutex.unlock();
}

void ExpBalancing::tf_callback(const tf::tfMessage::ConstPtr& msg) {
  bag_mutex.lock();
  if(bag_is_open) {
    current_bag.write(tf_topic, ros::Time::now(), msg);
  }
  bag_mutex.unlock();
}

//---------------------------------------------------------------------

void ExpBalancing::setPubMarkers(visualization_msgs::MarkerArray &markers) {
  current_markers.markers.clear();
  for(int i=0; i<markers.markers.size(); i++) {
    current_markers.markers.push_back(markers.markers[i]);
  }
  publish_markers = true;
}

void ExpBalancing::stopPubMarkers() {

  publish_markers = false;
}

// void ExpBalancing::addCylinderMarker(visualization_msgs::MarkerArray &markers,
// 				     Eigen::Vector3d p, Eigen::Vector3d v,
// 				     double r, std::string frame_, double h,
// 				     std::string namespc, double rc, double gc,
// 				     double bc) {

//   Eigen::Quaterniond q;

//   // transformation which points z in the cylinder direction
//   q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

//   visualization_msgs::Marker marker;
//   marker.ns = namespc;
//   marker.header.frame_id = frame_;
//   marker.header.stamp = ros::Time::now();
//   marker.type = visualization_msgs::Marker::CYLINDER;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.lifetime = ros::Duration(0.2);
//   marker.id = markers.markers.size();
//   marker.pose.position.x = p(0) + v(0) * 0.5 * h;  // LINE_SCALE;
//   marker.pose.position.y = p(1) + v(1) * 0.5 * h;  // LINE_SCALE;
//   marker.pose.position.z = p(2) + v(2) * 0.5 * h;
//   marker.pose.orientation.x = q.x();
//   marker.pose.orientation.y = q.y();
//   marker.pose.orientation.z = q.z();
//   marker.pose.orientation.w = q.w();
//   marker.scale.x = 2 * r;
//   marker.scale.y = 2 * r;
//   marker.scale.z = h;
//   marker.color.r = rc;
//   marker.color.g = gc;
//   marker.color.b = bc;
//   marker.color.a = 0.1;
//   markers.markers.push_back(marker);
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "exp_blancing");

  ExpBalancing demo_teleop;

  ROS_INFO("Balancing experiments node ready");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  demo_teleop.expMainLoop();
  ros::waitForShutdown();

  return 0;
}
//---------------------------------------------------------------------
    
