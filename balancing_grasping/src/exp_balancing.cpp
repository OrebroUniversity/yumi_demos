#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <sys/stat.h>
#include <sys/types.h>

#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include<balancing_grasping/exp_balancing.h>
#define TF_PUBLISH_PERIOD 0.1

ExpBalancing::ExpBalancing() {

  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  //nh_.param<std::string>("grasp_left_topic",gleft_topic,"/leap_hands/grasp_left");
  nh_.param<std::string>("tf_topic",tf_topic,"/tf");
  nh_.param<std::string>("js_topic",js_topic,"/yumi/joint_states");

  //nh_.param<std::string>("hand_frame_left",hand_frame_left,"teleop_left_frame");
  nh_.param<std::string>("log_directory",log_dir,"log");

  nh_.param<int>("num_pickups",num_pickups,2);
  nh_.param("alpha", alpha,0.2);
  nh_.param("cont_lift", cont_lift,0.005);
  nh_.param("grasp_lift", grasp_lift,0.05);
  nh_.param("grasp_rand", grasp_rand,0.08);         
  nh_.param("grasp_threshold",grasp_thresh_tol,0.2); //0 to 1
  nh_.param("joint_task_tol",joint_task_tol,1e-2); //0 to 1
  nh_.param("pre_grasp_task_tol",pre_grasp_task_tol,1e-4); //0 to 1  
  nh_.param("grasp_task_tol", grasp_task_tol,1e-4); //0 to 1
  nh_.param("grasp_acq_dur", grasp_acq_dur,3.0);
  nh_.param("grasp_lift_dur", grasp_lift_dur,3.0);
  nh_.param("grasp_cont_dur", grasp_cont_dur,2.0);  
  nh_.param("grasp_task_dur", grasp_task_dur,10.0);

  marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("teleop_markers",10);
  //    gripper_pub_ = nh_.advertise<robotiq_85_msgs::GripperCmd> ("grasp_cmd",10);

  //  grasp_left_sub_ = n_.subscribe(gleft_topic, 1, &ExpBalancing::grasp_left_callback, this);
  joint_state_sub_ = n_.subscribe(js_topic, 1, &ExpBalancing::js_callback, this);
  tf_sub_ = n_.subscribe(tf_topic, 1, &ExpBalancing::tf_callback, this);

  start_demo_ = nh_.advertiseService("start_demo", &ExpBalancing::start_demo_callback, this);
  //  next_task_srv_ = nh_.advertiseService("next_task", &ExpBalancing::next_task_callback, this);
  quit_demo_ = nh_.advertiseService("quit_demo", &ExpBalancing::quit_demo_callback, this);
  
  std::string robot_namespace; 
  nh_.param<std::string>("robot_namespace",robot_namespace,"yumi");
  hiqp_client_ = std::shared_ptr<hiqp_ros::HiQPClient> (new hiqp_ros::HiQPClient(robot_namespace, "hiqp_joint_velocity_controller"));

  tf_pub_timer = nh_.createTimer(ros::Duration(TF_PUBLISH_PERIOD), &ExpBalancing::tf_pub_callback, this);
  marker_pub_timer = nh_.createTimer(ros::Duration(TF_PUBLISH_PERIOD), &ExpBalancing::marker_pub_callback, this);
  quit_demo = true;
  //  next_task = false;
  publish_markers = false;
  loadGeometricPrimitivesFromParamServer();
  loadTasksFromParamServer("joint_config_tasks", joint_tasks, joint_task_names);
  loadTasksFromParamServer("pre_grasp_tasks", pre_grasp_tasks, pre_grasp_task_names);
  loadTasksFromParamServer("grasp_tasks", grasp_tasks, grasp_task_names);  
  // loadTasksFromParamServer("pick_assisted_tasks", pick_assisted_tasks, pick_assisted_task_names);
  // loadTasksFromParamServer("drop_assisted_tasks", drop_assisted_tasks, drop_assisted_task_names);
  // loadTasksFromParamServer("point_assisted_tasks", point_assisted_tasks, point_assisted_task_names);

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

  std::vector<std::string> prev_tasks;  
  std::vector<hiqp_ros::TaskDoneReaction> reactions;
  std::vector<double> tolerances;
  tf::Transform L_O_T, R_O_T, target_r_frame, target_l_frame, target_obj_frame, current_r_frame, current_l_frame;

  while(true) {
    //each iteration of this loop is a new set of experiments
    //    current_target_id=-1;
	
    {	
      boost::mutex::scoped_lock lock(bools_mutex);
      while(quit_demo) {
      	cond_.wait(lock);
      }

    }
    //    next_task = true;
    cond_.notify_one();
    hiqp_client_->removeTasks(grasp_task_names);
    #if 1
    //---------------------------------------------------------------------------------------//	
    ROS_INFO("TASK SET 1: Moving to init joint configuration.");
    {
      reactions.clear();
      tolerances.clear();
      for(int i=0; i<joint_task_names.size(); i++) { 
	reactions.push_back(hiqp_ros::TaskDoneReaction::REMOVE);
	tolerances.push_back(joint_task_tol);
      }
      
      // boost::mutex::scoped_lock lock(bools_mutex);
      // while(!quit_demo) {
      // 	cond_.wait(lock);
      // }
      // if(quit_demo) {
      //reset variables and so on
      //	continue;
      //}
      hiqp_client_->setTasks(joint_tasks);
    }
	
    hiqp_client_->waitForCompletion(joint_task_names, reactions, tolerances);
    //cond_.notify_one();
    //---------------------------------------------------------------------------------------//
    ROS_INFO("TASK SET 2: Moving to pre-grasp poses.");    
    {
      reactions.clear();
      tolerances.clear();
      for(int i=0; i<pre_grasp_task_names.size(); i++) { 
	reactions.push_back(hiqp_ros::TaskDoneReaction::REMOVE);
	tolerances.push_back(pre_grasp_task_tol);
      }

      // boost::mutex::scoped_lock lock(bools_mutex);
      // while(!quit_demo) {
      // 	cond_.wait(lock);
      // }
      // if(quit_demo) {
      // 	//reset variables and so on
      // 	continue;
      // }

      hiqp_client_->setTasks(pre_grasp_tasks);

    }
	
    hiqp_client_->waitForCompletion(pre_grasp_task_names, reactions, tolerances);
    //    cond_.notify_one();    
    //---------------------------------------------------------------------------------------//
    ROS_INFO("TASK SET 3: Moving to grasp poses.");        
    {
      current_r_frame=pre_grasp_r_frame;
      current_l_frame=pre_grasp_l_frame;
      target_r_frame=current_r_frame;
      target_l_frame=current_l_frame;
      target_obj_frame=current_obj_frame;
      
      //randomize the pose of the grasp target frames along the world x axis
      tf::Vector3 v=current_r_frame.getOrigin();

      v.setY(-0.1);
      v.setX(v.getX()+randomNumber(-grasp_rand/2, grasp_rand/2));
      target_r_frame.setOrigin(v);
      v=current_l_frame.getOrigin();
      v.setY(0.1);      
      v.setX(v.getX()+randomNumber(-grasp_rand/2, grasp_rand/2));
      target_l_frame.setOrigin(v);

   
      std::list<tf::Transform> poses_r = minJerkTraj(current_r_frame, target_r_frame, grasp_acq_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_l = minJerkTraj(current_l_frame, target_l_frame, grasp_acq_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_obj = minJerkTraj(current_obj_frame, target_obj_frame, grasp_acq_dur, TF_PUBLISH_PERIOD);
      prev_tasks.clear();
      setTasks(grasp_tasks, prev_tasks );
      boost::mutex::scoped_lock lock(tf_pub_mutex);   

      tf_published=false;
      while(!poses_obj.empty() && !quit_demo) {
	tf_pub_cond_.wait(lock);

	target_poses.clear();
	target_poses.push_back(tf::StampedTransform(poses_r.front(),ros::Time::now(),"yumi_base_link","grasp_r_frame"));
	target_poses.push_back(tf::StampedTransform(poses_l.front(),ros::Time::now(),"yumi_base_link","grasp_l_frame"));
	target_poses.push_back(tf::StampedTransform(poses_obj.front(),ros::Time::now(),"yumi_base_link","obj_frame"));	
	poses_r.pop_front();
	poses_l.pop_front();
	poses_obj.pop_front();
      }
      if(quit_demo) {
	continue;
      }
      ROS_INFO("Finished moving to grasp acquisition poses.");
    }

    //---------------------------------------------------------------------------------------//
    ROS_INFO("TASK SET 4: Establishing contact.");        
    {
      current_r_frame=target_r_frame;
      current_l_frame=target_l_frame;
      target_obj_frame=current_obj_frame;
      
      tf::Vector3 v=current_r_frame.getOrigin();
      v.setZ(v.getZ()+cont_lift);
      target_r_frame.setOrigin(v);
      v=current_l_frame.getOrigin();
      v.setZ(v.getZ()+cont_lift);
      target_l_frame.setOrigin(v);

      std::list<tf::Transform> poses_r = minJerkTraj(current_r_frame, target_r_frame, grasp_cont_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_l = minJerkTraj(current_l_frame, target_l_frame, grasp_cont_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_obj = minJerkTraj(current_obj_frame, target_obj_frame, grasp_cont_dur, TF_PUBLISH_PERIOD);
      
      boost::mutex::scoped_lock lock(tf_pub_mutex);   

      while(!poses_obj.empty() && !quit_demo) {
	tf_pub_cond_.wait(lock);
	//	std::cerr<<"pushing new poses, time: "<<ros::Time::now().toSec()<<std::endl;
	target_poses.clear();
	target_poses.push_back(tf::StampedTransform(poses_r.front(),ros::Time::now(),"yumi_base_link","grasp_r_frame"));
	target_poses.push_back(tf::StampedTransform(poses_l.front(),ros::Time::now(),"yumi_base_link","grasp_l_frame"));
	target_poses.push_back(tf::StampedTransform(poses_obj.front(),ros::Time::now(),"yumi_base_link","obj_frame"));		
	poses_r.pop_front();
	poses_l.pop_front();
	poses_obj.pop_front();
      }
      if(quit_demo) {
	continue;
      }
      ROS_INFO("Established contact.");
    }
    //---------------------------------------------------------------------------------------//    
    //Compute the grasp poses in the object frame
    R_O_T=target_obj_frame.inverse()*target_r_frame;
    L_O_T=target_obj_frame.inverse()*target_l_frame;    
    //---------------------------------------------------------------------------------------//
#endif
    ROS_INFO("TASK SET 5: moving to lift poses.");        
    {
      current_r_frame=target_r_frame;
      current_l_frame=target_l_frame;
      target_obj_frame=current_obj_frame;
      
      tf::Vector3 v=current_r_frame.getOrigin();
      v.setZ(v.getZ()+grasp_lift);
      target_r_frame.setOrigin(v);
      v=current_l_frame.getOrigin();
      v.setZ(v.getZ()+grasp_lift);
      target_l_frame.setOrigin(v);
      v=current_obj_frame.getOrigin();
      v.setZ(v.getZ()+grasp_lift);
      target_obj_frame.setOrigin(v);

      std::list<tf::Transform> poses_r = minJerkTraj(current_r_frame, target_r_frame, grasp_lift_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_l = minJerkTraj(current_l_frame, target_l_frame, grasp_lift_dur, TF_PUBLISH_PERIOD);
      std::list<tf::Transform> poses_obj = minJerkTraj(current_obj_frame, target_obj_frame, grasp_lift_dur, TF_PUBLISH_PERIOD);
      
      boost::mutex::scoped_lock lock(tf_pub_mutex);   

      while(!poses_r.empty() && !poses_l.empty() && !quit_demo) {
	tf_pub_cond_.wait(lock);

	target_poses.clear();
	target_poses.push_back(tf::StampedTransform(poses_r.front(),ros::Time::now(),"yumi_base_link","grasp_r_frame"));
	target_poses.push_back(tf::StampedTransform(poses_l.front(),ros::Time::now(),"yumi_base_link","grasp_l_frame"));
	target_poses.push_back(tf::StampedTransform(poses_obj.front(),ros::Time::now(),"yumi_base_link","obj_frame"));
	poses_obj.pop_front();
	poses_r.pop_front();
	poses_l.pop_front();
      }
      if(quit_demo) {
	continue;
      }

      ROS_INFO("Finished moving to lift poses.");
    }

    //---------------------------------------------------------------------------------------//
    ROS_INFO("TASK SET 6: Starting balancing grasping task.");        
    {
      current_obj_frame=target_obj_frame;
      
      //the target frame is tilted by -alpha around the x axis 
      tf::Transform target_obj_frame = current_obj_frame;
      Eigen::Quaterniond q(Eigen::Matrix3d(Eigen::AngleAxisd(-alpha, Eigen::Vector3d::UnitX())));
      target_obj_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

      //split the duration for 1/6 of the tilt motion and 5/6 for the swivel motion      
      std::list<tf::Transform> poses_obj = minJerkTraj(current_obj_frame, target_obj_frame, grasp_task_dur/6, TF_PUBLISH_PERIOD);

      current_obj_frame=target_obj_frame;
      q=Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY())));
      target_obj_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      std::list<tf::Transform> poses_tmp = minJerkTraj(current_obj_frame, target_obj_frame, grasp_task_dur/6, TF_PUBLISH_PERIOD);
      poses_obj.splice(poses_obj.end(), poses_tmp);

      current_obj_frame=target_obj_frame;
      q=Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())));
      target_obj_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      poses_tmp = minJerkTraj(current_obj_frame, target_obj_frame, grasp_task_dur/6, TF_PUBLISH_PERIOD);
      poses_obj.splice(poses_obj.end(), poses_tmp);

      current_obj_frame=target_obj_frame;
      q=Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(-alpha, Eigen::Vector3d::UnitY())));
      target_obj_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      poses_tmp = minJerkTraj(current_obj_frame, target_obj_frame, grasp_task_dur/6, TF_PUBLISH_PERIOD);
      poses_obj.splice(poses_obj.end(), poses_tmp);
      
      current_obj_frame=target_obj_frame;
      tf::Matrix3x3 R; R.setIdentity();
      target_obj_frame.setBasis(R);    
      poses_tmp = minJerkTraj(current_obj_frame, target_obj_frame, grasp_task_dur/6, TF_PUBLISH_PERIOD);   
      poses_obj.splice(poses_obj.end(), poses_tmp);

      //generate the grasp poses from the object poses
      std::list<tf::Transform> poses_r, poses_l;



      
      for (auto it = poses_obj.begin(); it != poses_obj.end(); ++it) {
	poses_r.push_back((*it)*R_O_T);
	poses_l.push_back((*it)*L_O_T);	
      }
	 
      boost::mutex::scoped_lock lock(tf_pub_mutex);   
      while(!poses_obj.empty() && !quit_demo) {
	tf_pub_cond_.wait(lock);
	target_poses.clear();
	target_poses.push_back(tf::StampedTransform(poses_r.front(),ros::Time::now(),"yumi_base_link","grasp_r_frame"));
	target_poses.push_back(tf::StampedTransform(poses_l.front(),ros::Time::now(),"yumi_base_link","grasp_l_frame"));
	target_poses.push_back(tf::StampedTransform(poses_obj.front(),ros::Time::now(),"yumi_base_link","obj_frame"));
	poses_obj.pop_front();
	poses_r.pop_front();
	poses_l.pop_front();

      }
      if(quit_demo) {
	continue;
      }

      ROS_INFO("Finished balancing grasping task.");
    }

    //---------------------------------------------------------------------------------------//	
    //quitting
    {	    
      boost::mutex::scoped_lock lock(bools_mutex);
      //close-up log files
      quit_demo = true;
    }
#if 1
    //wait for quitting
    {	    
      // boost::mutex::scoped_lock lock(bools_mutex);
      // while(!quit_demo) {
      // 	cond_.wait(lock);
      // }

      //remove previous tasks
      //  hiqp_client_->removeTasks(grasp_task_names);

    }
#endif
    ROS_INFO("Quit Demo.");
  }

}
bool ExpBalancing::setTasks(std::vector<hiqp_msgs::Task> &next_tasks,
			    std::vector<std::string> &prev_task_names) {
  boost::mutex::scoped_lock lock(bools_mutex);
  // while(!quit_demo) {
  // std::cerr<<"n set tasks."<<std::endl;    
  //   cond_.wait(lock);
  // }
  if(prev_task_names.size() > 0) {
    //remove previous tasks
    hiqp_client_->removeTasks(prev_task_names);
  }
  if(quit_demo) {
    //reset variables and so on
    return false;
  }

  hiqp_client_->setTasks(next_tasks);

  return true;
}

bool ExpBalancing::loadPreGraspPoses(){
  XmlRpc::XmlRpcValue hiqp_grasping_geometric_primitives;
  if (!nh_.getParam("grasping_geometric_primitives", hiqp_grasping_geometric_primitives)) {
    ROS_WARN_STREAM("No grasping_geometric_primitives parameter "
		    << "found on the parameter server. Cannot identify pre-grasp pose!");
    return false;
  } else {
    for (int i = 0; i < hiqp_grasping_geometric_primitives.size(); ++i) {
      std::string frame_id = static_cast<std::string>(hiqp_grasping_geometric_primitives[i]["frame_id"]);
      std::string name = static_cast<std::string>(hiqp_grasping_geometric_primitives[i]["name"]);
      XmlRpc::XmlRpcValue& parameters_xml = hiqp_grasping_geometric_primitives[i]["parameters"];
      std::vector<double> parameters;
      for (int j = 0; j < parameters_xml.size(); ++j) parameters.push_back(static_cast<double>(parameters_xml[j]));
	  
      ROS_ASSERT(parameters.size()==6);
      tf::Vector3 v(parameters[0], parameters[1], parameters[2]);
      Eigen::Quaterniond q(Eigen::Matrix3d(Eigen::AngleAxisd(parameters[3], Eigen::Vector3d::UnitX()) *
					   Eigen::AngleAxisd(parameters[4], Eigen::Vector3d::UnitY()) *
					   Eigen::AngleAxisd(parameters[5], Eigen::Vector3d::UnitZ())));
	
      if(strcmp(name.c_str(),"pre_grasp_r_frame") == 0){
	pre_grasp_r_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
	pre_grasp_r_frame.setOrigin(v);
	ROS_INFO("Parametrized pre_grasp_r_frame");
      }
      if(strcmp(name.c_str(),"pre_grasp_l_frame") == 0){
	pre_grasp_l_frame.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
	pre_grasp_l_frame.setOrigin(v);
	ROS_INFO("Parametrized pre_grasp_l_frame");	  
      }
	
    }
  }
  return true;
}

bool ExpBalancing::start_demo_callback(std_srvs::Empty::Request  &req,
				       std_srvs::Empty::Response &res ) {
  initializeDemo();

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

// bool ExpBalancing::next_task_callback(std_srvs::Empty::Request  &req,
// 				      std_srvs::Empty::Response &res ) {

//   ROS_INFO("Executing next task");
//   {
//     boost::mutex::scoped_lock lock(bools_mutex, boost::try_to_lock);
//     if(!lock) return false;
//     next_task = true;
//     cond_.notify_one();
//   }

//   return true;
// }

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
  {

    boost::mutex::scoped_lock lock(tf_pub_mutex, boost::try_to_lock);
    if(lock){
      for(unsigned int i=0; i<target_poses.size(); i++){
	target_poses[i].stamp_ = ros::Time::now();
	tb.sendTransform(target_poses[i]);

	//DEBUG =========================================
	// std::cerr<<"Publishing target pose"<<std::endl;
	// std::cerr<<"frame_id_: "<<target_poses[i].frame_id_<<std::endl;
	// std::cerr<<"child_frame_id_: "<<target_poses[i].child_frame_id_<<std::endl;
	
	// std::cerr<<"origin: "<<target_poses[i].getOrigin().getX()<<" "<<target_poses[i].getOrigin().getY()<<" "<<target_poses[i].getOrigin().getZ()<<std::endl<<std::endl;
	//DEBUG END =========================================
      }

      tf_pub_cond_.notify_one();
    }
  }

  // return true;
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

void ExpBalancing::initializeDemo(){
  if(!loadPreGraspPoses())
    ROS_ERROR("Incorrect pre-grasp pose setup!");

  current_obj_frame.setIdentity();
  //assume that the object frame is centered in between the pre-grasp frames in y, has the same x value as either pre-grasp frame and a z value of pre-grasp + vertical contact lift travel
  tf::Vector3 v(pre_grasp_r_frame.getOrigin().getX(),(pre_grasp_l_frame.getOrigin().getY()-pre_grasp_l_frame.getOrigin().getY())/2, pre_grasp_r_frame.getOrigin().getZ()+cont_lift);
  current_obj_frame.setOrigin(v);
  
  //---- setup TF frames we want to publish ----//
  //
  target_poses.push_back(tf::StampedTransform(pre_grasp_r_frame, ros::Time::now(), "yumi_base_link", "grasp_r_frame"));
  target_poses.push_back(tf::StampedTransform(pre_grasp_l_frame, ros::Time::now(), "yumi_base_link", "grasp_l_frame"));
  target_poses.push_back(tf::StampedTransform(current_obj_frame, ros::Time::now(), "yumi_base_link", "obj_frame"));
}

std::list<tf::Transform> ExpBalancing::minJerkTraj(const tf::Transform& start_pose, const tf::Transform& end_pose, double T, double dt){
  std::list<tf::Transform> poses;
  ROS_ASSERT(dt > 0.0 && T > dt);

  tf::Vector3 vi=start_pose.getOrigin();
  tf::Vector3 ve=end_pose.getOrigin();
 
  Eigen::Quaterniond qi(start_pose.getRotation().getW(), start_pose.getRotation().getX(),start_pose.getRotation().getY(), start_pose.getRotation().getZ());
  Eigen::Quaterniond qe(end_pose.getRotation().getW(), end_pose.getRotation().getX(),end_pose.getRotation().getY(), end_pose.getRotation().getZ());    
  double t=0;
  for(unsigned int i=0; i<ceil(T/dt)+1; i++){
    tf::Vector3 v=vi+(ve-vi)*(10*pow(t/T,3)-15*pow(t/T,4)+6*pow(t/T,5));
    double ip=(10*pow(t/T,3)-15*pow(t/T,4)+6*pow(t/T,5));
    Eigen::Quaterniond q = qi.slerp(ip, qe);
    poses.push_back(tf::Transform(tf::Quaternion(q.x(),q.y(),q.z(),q.w()),v));
    t+=dt;
  }
  return poses;
}

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
    
