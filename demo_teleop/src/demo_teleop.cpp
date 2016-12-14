#include <ros/ros.h>
#include <yumi_hw/YumiGrasp.h>
#include <geometry_msgs/PoseStamped.h>
#include <hiqp_msgs/SetTask.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/AddGeometricPrimitive.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream

class DemoTeleop {
    private:
	ros::NodeHandle nh_;
	ros::NodeHandle n_;

	ros::Subscriber pose_left_sub_;
	ros::Subscriber pose_right_sub_;
	
	ros::Subscriber grasp_left_sub_;
	ros::Subscriber grasp_right_sub_;

	ros::Publisher marker_viz_pub_;

	///Clients to other nodes
	ros::ServiceClient set_controller_task_;
	ros::ServiceClient remove_controller_task_;
	ros::ServiceClient add_controller_primitive_;
	
	ros::ServiceClient close_gripper_clt_;
	ros::ServiceClient open_gripper_clt_;
	ros::ServiceClient reset_map_clt_;
	ros::ServiceClient convert_and_publish_map_;

	ros::ServiceServer start_demo_;
	ros::ServiceServer start_demo_right_;
	
	ros::ServiceServer regain_start_pose_;
	ros::ServiceServer switch_align_;
	ros::ServiceServer quit_demo_;

	tf::TransformListener tl;

	boost::mutex newpose_m;
	boost::mutex bools_mutex;
	boost::condition_variable cond_;

	std::string pleft_topic, pright_topic, gleft_topic, gright_topic;
	std::string frame_left, frame_right;

	Eigen::Affine3d leftInFrame, rightInFrame;
	double trans_thresh, rot_thresh, grasp_thresh, time_last_left_pose, time_last_right_pose, fresh_thresh, jnt_task_dynamics, teleop_task_dynamics;
	bool new_pose_, runOnline, leftClosed, rightClosed;
	bool regain_start, quit_demo, align_on;

	double grasp_thresh_tol;
	std::vector<double> teleop_sensing, teleop_init;

    private:
	///functions
	double getDoubleTime() {
	    struct timeval time;
	    gettimeofday(&time,NULL);
	    return time.tv_sec + time.tv_usec * 1e-6; 
	}
    public:
	DemoTeleop() {
	    nh_ = ros::NodeHandle("~");
	    n_ = ros::NodeHandle();
	    
	    nh_.param<std::string>("pose_left_topic",pleft_topic,"/leap_hands/pose_left");
	    nh_.param<std::string>("pose_rigth_topic",pright_topic,"/leap_hands/pose_right");
	    nh_.param<std::string>("grasp_left_topic",gleft_topic,"/leap_hands/grasp_left");
	    nh_.param<std::string>("grasp_rigth_topic",gright_topic,"/leap_hands/grasp_right");
	    
	    nh_.param<std::string>("frame_left",frame_left,"gripper_l_base");
	    nh_.param<std::string>("frame_right",frame_right,"gripper_r_base");
	    
	    nh_.param("translation_threshold",trans_thresh,0.05); //m
	    nh_.param("rotation_threshold",rot_thresh,0.25); //rad
	    nh_.param("grasp_threshold",grasp_thresh,0.7); //0 to 1
	    nh_.param("pose_freshness_threshold",fresh_thresh,0.5); //seconds
	    nh_.param("run_online",runOnline,false); //0 to 1
	    
	    nh_.param("jnt_task_dynamics",jnt_task_dynamics,1.0); //seconds
	    nh_.param("teleop_task_dynamics",teleop_task_dynamics,8.0); //seconds

	    nh_.getParam("jnts_teleop_sensing", teleop_sensing);
	    ROS_ASSERT_MSG(teleop_sensing.size() == 14, 
		    "Please set a valid configuration for sensing pose. You provided %lu values, instead of 14",teleop_sensing.size());
	    nh_.getParam("jnts_teleop_init", teleop_init);
	    ROS_ASSERT_MSG(teleop_init.size() == 14, 
		    "Please set a valid configuration for sensing pose. You provided %lu values, instead of 14",teleop_init.size());
	    
	    marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("tracking_state_markers",10);

	    pose_left_sub_ = n_.subscribe(pleft_topic, 1, &DemoTeleop::pose_left_callback, this);
	    pose_right_sub_ = n_.subscribe(pright_topic, 1, &DemoTeleop::pose_right_callback, this);
	    grasp_left_sub_ = n_.subscribe(gleft_topic, 1, &DemoTeleop::grasp_left_callback, this);
	    grasp_right_sub_ = n_.subscribe(gright_topic, 1, &DemoTeleop::grasp_right_callback, this);

	    start_demo_ = nh_.advertiseService("start_demo", &DemoTeleop::start_demo_callback, this);
	    start_demo_right_ = nh_.advertiseService("start_demo_right", &DemoTeleop::start_demo_right_callback, this);
	    regain_start_pose_ = nh_.advertiseService("regain_start", &DemoTeleop::regain_start_callback, this);
	    switch_align_ = nh_.advertiseService("change_alignment", &DemoTeleop::switch_align_callback, this);
	    quit_demo_ = nh_.advertiseService("quit_demo", &DemoTeleop::quit_demo_callback, this);

	    new_pose_ = false;
	    time_last_left_pose = 0;
	    time_last_right_pose = 0;
	    //these should probably be read from joint states
	    leftClosed = false;
	    rightClosed = false;
	    align_on = true;
	    grasp_thresh_tol=0.2;

	    regain_start=false;
	    quit_demo=false;

	    set_controller_task_ = n_.serviceClient<hiqp_msgs::SetTask>("set_task");
	    set_controller_task_.waitForExistence();
	
	    remove_controller_task_ = n_.serviceClient<hiqp_msgs::RemoveTask>("remove_task");
	    remove_controller_task_.waitForExistence();
	
	    add_controller_primitive_ = n_.serviceClient<hiqp_msgs::AddGeometricPrimitive>("add_primitive");
	    add_controller_primitive_.waitForExistence();
		
	    if(runOnline) {

		close_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("close_gripper");
		close_gripper_clt_.waitForExistence();

		open_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("open_gripper");
		open_gripper_clt_.waitForExistence();
#if 0	
		reset_map_clt_ = n_.serviceClient<std_srvs::Empty>("reset_map");
		reset_map_clt_.waitForExistence();

		convert_and_publish_map_ = n_.serviceClient<std_srvs::Empty>("map_to_edt");
		convert_and_publish_map_.waitForExistence();
#endif
	    }
	}

	void pose_left_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	    std::string base_frame = msg->header.frame_id;

	    tf::StampedTransform base2frame_tf;
	    Eigen::Affine3d base2frame, msg2base;
	    try {
		tl.waitForTransform(frame_left, base_frame, msg->header.stamp, ros::Duration(0.25) );
		tl.lookupTransform(frame_left, base_frame, msg->header.stamp, base2frame_tf);
	    } catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return;
	    }
	    tf::transformTFToEigen(base2frame_tf,base2frame);
	    
	    tf::poseMsgToEigen(msg->pose, msg2base);

	    {
		boost::mutex::scoped_lock lock(newpose_m, boost::try_to_lock);
		if(!lock) return;
		leftInFrame = msg2base*base2frame;
		new_pose_ = true;
		time_last_left_pose = getDoubleTime();
		cond_.notify_one();
	    }

	}

	void pose_right_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	    std::string base_frame = msg->header.frame_id;

	    tf::StampedTransform base2frame_tf;
	    Eigen::Affine3d base2frame, msg2base;
	    try {
		tl.waitForTransform(frame_right, base_frame, msg->header.stamp, ros::Duration(0.25) );
		tl.lookupTransform(frame_right, base_frame, msg->header.stamp, base2frame_tf);
	    } catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return;
	    }
	    tf::transformTFToEigen(base2frame_tf,base2frame);
	    
	    tf::poseMsgToEigen(msg->pose, msg2base);

	    {
		boost::mutex::scoped_lock lock(newpose_m, boost::try_to_lock);
		if(!lock) return;
		rightInFrame = msg2base*base2frame;
		new_pose_ = true;
		time_last_right_pose = getDoubleTime();
		cond_.notify_one();
	    }

	}

	void grasp_left_callback(const std_msgs::Float32::ConstPtr& msg) {
	   
	    if(msg->data > grasp_thresh+grasp_thresh_tol && !leftClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =1;
		if(runOnline) {
		    if(!close_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not close gripper");
			ROS_BREAK();
		    }
		} else {
		    ROS_INFO("left gripper should close");
		}
		sleep(1);
		leftClosed = true;
	    }
	    if(msg->data < grasp_thresh-grasp_thresh_tol && leftClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =1;
		if(runOnline) {
		    if(!open_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not open gripper");
			ROS_BREAK();
		    }
		} else {
		    ROS_INFO("left gripper should open");
		}
		sleep(1);
		leftClosed = false;
	    }

	}

	void grasp_right_callback(const std_msgs::Float32::ConstPtr& msg) {
	   
	    if(msg->data > grasp_thresh+grasp_thresh_tol && !rightClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =2;
		if(runOnline) {
		    if(!close_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not close gripper");
			ROS_BREAK();
		    }
		} else {
		   ROS_INFO("right gripper should close");
		}
		sleep(1);
		rightClosed = true;
	    }
	    if(msg->data < grasp_thresh-grasp_thresh_tol && rightClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =2;
		if(runOnline) {
		    if(!open_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not open gripper");
			ROS_BREAK();
		    }
		} else {
		    ROS_INFO("right gripper should open");
		}
		sleep(1);
		rightClosed = false;
	    }

	}

	bool waitForSync (double trans_thresh_loc, double rot_thresh_loc, double fresh_thresh_loc, bool isSyncedAlready, bool checkLeft=true, bool checkRight=true) {

	    ///set up visualization markers
	    visualization_msgs::MarkerArray marker_array;
	    visualization_msgs::Marker leftSphere, rightSphere;
	    leftSphere.header.frame_id = frame_left;
	    rightSphere.header.frame_id = frame_right;
	    leftSphere.ns = "candy_left";
	    rightSphere.ns = "candy_right";
	    leftSphere.header.stamp = ros::Time::now();
	    rightSphere.header.stamp = ros::Time::now();
	    leftSphere.id = 0;
	    rightSphere.id = 1;
	    leftSphere.type = visualization_msgs::Marker::SPHERE;
	    rightSphere.type = visualization_msgs::Marker::SPHERE;
	    leftSphere.scale.x = leftSphere.scale.y = leftSphere.scale.z = 0.15;
	    rightSphere.scale.x = rightSphere.scale.y = rightSphere.scale.z = 0.15;
	    leftSphere.action = rightSphere.action = visualization_msgs::Marker::ADD;
	    leftSphere.color.b = 0.1;
	    rightSphere.color.b = 0.1;
	    leftSphere.color.a = 0.6;
	    rightSphere.color.a = 0.6;
	    leftSphere.pose.position.x = leftSphere.pose.position.y = leftSphere.pose.position.z = 0;
	    rightSphere.pose.position.x = rightSphere.pose.position.y = rightSphere.pose.position.z = 0;

	    bool synced = false;
	    int secs = fresh_thresh_loc*1.5;
	    secs = secs < 1 ? 1 : secs;
	    boost::posix_time::time_duration timeout (0,0,secs,0); //max 3 seconds and then wake up
	    while(!synced)
	    {
		boost::mutex::scoped_lock lock(newpose_m);
		double t_start = getDoubleTime();
		    
		cond_.timed_wait(lock, timeout);
		if(!new_pose_) {
		    if(getDoubleTime() - t_start > fresh_thresh_loc) {
			//no new pose in a while
			synced = false;
			//publish markers...
			leftSphere.color.r = 0.9;
			leftSphere.color.g = 0.1;
			rightSphere.color.r = 0.9;
			rightSphere.color.g = 0.1;
			marker_array.markers.clear();
			marker_array.markers.push_back(leftSphere);
			marker_array.markers.push_back(rightSphere);
			marker_viz_pub_.publish(marker_array);
		    }
		    if(isSyncedAlready) synced = !synced;
		    continue; 
		}
		new_pose_ = false;

		bools_mutex.lock();
		bool quit = quit_demo || regain_start;
		bool check_alignment = align_on;
		bools_mutex.unlock();

		if(quit) return false;
	
		Eigen::AngleAxisd min_rot_left(leftInFrame.rotation());
		Eigen::AngleAxisd min_rot_right(rightInFrame.rotation());

		double t_err_l = leftInFrame.translation().norm();
		double r_err_l = min_rot_left.angle();
		double t_err_r = rightInFrame.translation().norm();
		double r_err_r = min_rot_right.angle();
		bool fresh_left = (getDoubleTime() - time_last_left_pose) < fresh_thresh_loc;
		bool fresh_right = (getDoubleTime() - time_last_right_pose) < fresh_thresh_loc;

		bool left_ok = (t_err_l < trans_thresh_loc) && ((r_err_l < rot_thresh_loc) || !check_alignment);
	        bool right_ok =(t_err_r < trans_thresh_loc) && ((r_err_r < rot_thresh_loc) || !check_alignment);	
		synced = (left_ok || !checkLeft) && (right_ok || !checkRight);
		synced = synced && (fresh_left || !checkLeft)  && (fresh_right || !checkRight); // make sure we are not running on empty

		//if we are monitoring a synced task, we exit once we loose sync
		if(isSyncedAlready) synced = !synced; 

		/// visualization stuff ////
		if((left_ok && fresh_left) || !checkLeft) {
		    //set left marker to green
		    leftSphere.color.r = 0.1;
		    leftSphere.color.g = 0.9;
		} else {
		    leftSphere.color.r = 0.9;
		    leftSphere.color.g = 0.1;
		}
		if((right_ok && fresh_right) || !checkRight) {
		    //set right marker to green
		    rightSphere.color.r = 0.1;
		    rightSphere.color.g = 0.9;
		} else {
		    rightSphere.color.r = 0.9;
		    rightSphere.color.g = 0.1;
		}
		marker_array.markers.clear();
		marker_array.markers.push_back(leftSphere);
		marker_array.markers.push_back(rightSphere);

		marker_viz_pub_.publish(marker_array);
	    }
	    return true;

	}

	bool start_demo_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    //-------------------------------------------------------------------------//
	    //set joint configuration out of sensor view
	    {
		hiqp_msgs::SetTask task;
		task.request.name = "teleop_sensing_config";
		task.request.priority = 3;
		task.request.visible = 1;
		task.request.active = 1;
		task.request.def_params.push_back("TDefFullPose");
		for(int i=0; i<teleop_sensing.size(); ++i) {
		    std::stringstream strm;
		    strm<<teleop_sensing[i];
		    task.request.def_params.push_back(strm.str());
		}
		task.request.dyn_params.push_back("TDynFirstOrder");
		std::stringstream strm;
		strm<<jnt_task_dynamics;
		task.request.dyn_params.push_back(strm.str());

		if(!set_controller_task_.call(task))
		{
		    ROS_ERROR("could not set task %s",task.request.name.c_str());
		    ROS_BREAK();
		}
		sleep(10);
	    }
	    //wait to achieve
#if 0
	    //-------------------------------------------------------------------------//
	    // reset map
	    {
		if(runOnline) {
		    std_srvs::Empty empty;
		    if(!reset_map_clt_.call(empty))
		    {
			ROS_ERROR("could not clear map");
			ROS_BREAK();
		    }
		}
	    }

	    //-------------------------------------------------------------------------//
	    //wait for 2 seconds, compute edt and publish
	    {
		sleep(2);
		if(runOnline) {
		    std_srvs::Empty empty;
		    if(!convert_and_publish_map_.call(empty))
		    {
			ROS_ERROR("could not compute edt map");
			ROS_BREAK();
		    }
		}
	    }
#endif
	    //-------------------------------------------------------------------------//
	    //move to teleop joint configuration
	    //-------------------------------------------------------------------------//
	    {
		//remove previous task
		hiqp_msgs::RemoveTask rtask;
		rtask.request.task_name = "teleop_sensing_config";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}

		hiqp_msgs::SetTask task;
		task.request.name = "teleop_init_config";
		task.request.priority = 3;
		task.request.visible = 1;
		task.request.active = 1;
		task.request.def_params.push_back("TDefFullPose");
		for(int i=0; i<teleop_sensing.size(); ++i) {
		    std::stringstream strm;
		    strm<<teleop_init[i];
		    task.request.def_params.push_back(strm.str());
		}
		task.request.dyn_params.push_back("TDynFirstOrder");
		std::stringstream strm;
		strm<<jnt_task_dynamics;
		task.request.dyn_params.push_back(strm.str());

		if(!set_controller_task_.call(task))
		{
		    ROS_ERROR("could not set task %s",task.request.name.c_str());
		    ROS_BREAK();
		}
		sleep(3);
	    }

	    //-------------------------------------------------------------------------//
	    {
		//add geometric primitives	
		double col[] = {0, 0, 0, 1};
		double params[] = {0, 0, 0, 1, 0, 0, 0};
		double params2[] = {0, 0, 0, 0, 0, 0};
		
		hiqp_msgs::AddGeometricPrimitive prim;
		prim.request.name = "teleop_left_frame";
		prim.request.type = "frame";
		prim.request.frame_id = "yumi_body";
		prim.request.visible = true;
		prim.request.color = std::vector<double>(col, col+sizeof(col)/sizeof(double));
		prim.request.parameters= std::vector<double>(params, params+sizeof(params)/sizeof(double));

		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}
		
		prim.request.name = "teleop_right_frame";
		prim.request.frame_id = "yumi_body";
		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}

		prim.request.name = "teleop_gripper_left_frame";
		prim.request.frame_id = "gripper_l_base";
		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}

		prim.request.name = "teleop_gripper_right_frame";
		prim.request.frame_id = "gripper_r_base";
		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}
		
		//Lines
		hiqp_msgs::AddGeometricPrimitive prim2;
		prim2.request.color = std::vector<double>(col, col+sizeof(col)/sizeof(double));
		prim2.request.parameters= std::vector<double>(params2, params2+sizeof(params2)/sizeof(double));

		prim2.request.name = "object_vertical_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "yumi_body";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 0;
		prim2.request.parameters[2] = 1;
		prim2.request.parameters[3] = 0.45;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0.1;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add primitive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		prim2.request.name = "gripper_approach_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "gripper_r_base";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 0;
		prim2.request.parameters[2] = 1;
		prim2.request.parameters[3] = 0;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add prim2itive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		prim2.request.name = "gripper_vertical_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "gripper_r_base";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 1;
		prim2.request.parameters[2] = 0;
		prim2.request.parameters[3] = 0;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add prim2itive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		//remove previous task
		hiqp_msgs::RemoveTask rtask;
		rtask.request.task_name = "teleop_init_config";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}

	    }
	    // enter teleop loop //
	    //define tasks	
	    std::stringstream strm;
	    hiqp_msgs::RemoveTask rtask;
	    
	    hiqp_msgs::SetTask task_proj_left;
	    task_proj_left.request.name = "teleop_left_frame";
	    task_proj_left.request.priority = 2;
	    task_proj_left.request.visible = 1;
	    task_proj_left.request.active = 1;
	    task_proj_left.request.def_params.push_back("TDefGeomProj");
	    task_proj_left.request.def_params.push_back("frame");
	    task_proj_left.request.def_params.push_back("frame");
	    task_proj_left.request.def_params.push_back("teleop_left_frame = teleop_gripper_left_frame");
	    task_proj_left.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<teleop_task_dynamics;
	    task_proj_left.request.dyn_params.push_back(strm.str());
	   
	    strm.clear(); 
	    hiqp_msgs::SetTask task_proj_right;
	    task_proj_right.request.name = "teleop_right_frame";
	    task_proj_right.request.priority = 2;
	    task_proj_right.request.visible = 1;
	    task_proj_right.request.active = 1;
	    task_proj_right.request.def_params.push_back("TDefGeomProj");
	    task_proj_right.request.def_params.push_back("frame");
	    task_proj_right.request.def_params.push_back("frame");
	    task_proj_right.request.def_params.push_back("teleop_right_frame = teleop_gripper_right_frame");
	    task_proj_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<teleop_task_dynamics;
	    task_proj_right.request.dyn_params.push_back(strm.str());

	    strm.clear(); 
	    hiqp_msgs::SetTask task_align_left;
	    task_align_left.request.name = "teleop_left_align";
	    task_align_left.request.priority = 3;
	    task_align_left.request.visible = 1;
	    task_align_left.request.active = 1;
	    task_align_left.request.def_params.push_back("TDefGeomAlign");
	    task_align_left.request.def_params.push_back("frame");
	    task_align_left.request.def_params.push_back("frame");
	    task_align_left.request.def_params.push_back("teleop_left_frame = teleop_gripper_left_frame");
	    task_align_left.request.def_params.push_back("0");
	    task_align_left.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_align_left.request.dyn_params.push_back(strm.str());

	    strm.clear(); 
	    hiqp_msgs::SetTask task_align_right;
	    task_align_right.request.name = "teleop_right_align";
	    task_align_right.request.priority = 3;
	    task_align_right.request.visible = 1;
	    task_align_right.request.active = 1;
	    task_align_right.request.def_params.push_back("TDefGeomAlign");
	    task_align_right.request.def_params.push_back("frame");
	    task_align_right.request.def_params.push_back("frame");
	    task_align_right.request.def_params.push_back("teleop_right_frame = teleop_gripper_right_frame");
	    task_align_right.request.def_params.push_back("0");
	    task_align_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_align_right.request.dyn_params.push_back(strm.str());

	    strm.clear(); 
	    hiqp_msgs::SetTask task_auto_align_vertical_right;
	    task_auto_align_vertical_right.request.name = "gripper_right_align";
	    task_auto_align_vertical_right.request.priority = 3;
	    task_auto_align_vertical_right.request.visible = 1;
	    task_auto_align_vertical_right.request.active = 1;
	    task_auto_align_vertical_right.request.def_params.push_back("TDefGeomAlign");
	    task_auto_align_vertical_right.request.def_params.push_back("line");
	    task_auto_align_vertical_right.request.def_params.push_back("line");
	    task_auto_align_vertical_right.request.def_params.push_back("object_vertical_axis = gripper_vertical_axis");
	    task_auto_align_vertical_right.request.def_params.push_back("0");
	    task_auto_align_vertical_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_auto_align_vertical_right.request.dyn_params.push_back(strm.str());
	    
	    strm.clear(); 
	    hiqp_msgs::SetTask task_auto_align_approach_right;
	    task_auto_align_approach_right.request.name = "gripper_right_approach";
	    task_auto_align_approach_right.request.priority = 3;
	    task_auto_align_approach_right.request.visible = 1;
	    task_auto_align_approach_right.request.active = 1;
	    task_auto_align_approach_right.request.def_params.push_back("TDefGeomProj");
	    task_auto_align_approach_right.request.def_params.push_back("line");
	    task_auto_align_approach_right.request.def_params.push_back("line");
	    task_auto_align_approach_right.request.def_params.push_back("gripper_approach_axis = object_vertical_axis");
	    task_auto_align_approach_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_auto_align_approach_right.request.dyn_params.push_back(strm.str());

	    bools_mutex.lock();
	    bool quit = quit_demo;
	    bools_mutex.unlock();
	    while(!quit) {

		//wait for operator to align markers
		bools_mutex.lock();
		bool restart = regain_start;
		bool check_alignment = align_on;
		regain_start = false;
		bools_mutex.unlock();
		if(restart) {
		    ROS_INFO("Setting initial pose again");
		    hiqp_msgs::SetTask task;
		    task.request.name = "teleop_init_config";
		    task.request.priority = 3;
		    task.request.visible = 1;
		    task.request.active = 1;
		    task.request.def_params.push_back("TDefFullPose");
		    for(int i=0; i<teleop_sensing.size(); ++i) {
			std::stringstream strm;
			strm<<teleop_init[i];
			task.request.def_params.push_back(strm.str());
		    }
		    task.request.dyn_params.push_back("TDynFirstOrder");
		    std::stringstream strm;
		    strm<<jnt_task_dynamics;
		    task.request.dyn_params.push_back(strm.str());

		    if(!set_controller_task_.call(task))
		    {
			ROS_ERROR("could not set task %s",task.request.name.c_str());
			ROS_BREAK();
		    }
		    sleep(3);
		    //remove previous task
		    hiqp_msgs::RemoveTask rtask;
		    rtask.request.task_name = "teleop_init_config";
		    if(!remove_controller_task_.call(rtask))
		    {
			ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
			ROS_BREAK();
		    }
		}

		ROS_INFO("waiting for sync...");
		if(!waitForSync(trans_thresh, rot_thresh, fresh_thresh, false)) { 
		    bools_mutex.lock();
		    quit = quit_demo;
		    bools_mutex.unlock();
		    continue; 
		}
		//-------------------------------------------------------------------------//
		//synced, enable task
		ROS_INFO("Synced, enabling teleop task");
		task_proj_left.request.active = 1;
		if(!set_controller_task_.call(task_proj_left))
		{
		    ROS_ERROR("could not set task %s",task_proj_left.request.name.c_str());
		    ROS_BREAK();
		}
		
		task_proj_right.request.active = 1;
		if(!set_controller_task_.call(task_proj_right))
		{
		    ROS_ERROR("could not set task %s",task_proj_right.request.name.c_str());
		    ROS_BREAK();
		}
/*
*/		
		task_align_left.request.active = 1;
		if(!set_controller_task_.call(task_align_left))
		{
		    ROS_ERROR("could not set task %s",task_align_left.request.name.c_str());
		    ROS_BREAK();
		}
		if(check_alignment) {
		    task_align_right.request.active = 1;
		    if(!set_controller_task_.call(task_align_right))
		    {
			ROS_ERROR("could not set task %s",task_align_right.request.name.c_str());
			ROS_BREAK();
		    }
		} else {
		    task_auto_align_vertical_right.request.active = 1;
		    if(!set_controller_task_.call(task_auto_align_vertical_right))
		    {
			ROS_ERROR("could not set task %s",task_auto_align_vertical_right.request.name.c_str());
			ROS_BREAK();
		    }
		    task_auto_align_approach_right.request.active = 1;
		    if(!set_controller_task_.call(task_auto_align_approach_right))
		    {
			ROS_ERROR("could not set task %s",task_auto_align_approach_right.request.name.c_str());
			ROS_BREAK();
		    }

		}
		/*
		*/
		
		//-------------------------------------------------------------------------//
		//monitor sync status
		if(!waitForSync(5*trans_thresh, 5*rot_thresh, 2*fresh_thresh, true)) { 
		    bools_mutex.lock();
		    quit = quit_demo;
		    bools_mutex.unlock();
		    continue; 
		}
		
		
		//-------------------------------------------------------------------------//
		//disable task
		rtask.request.task_name = "teleop_left_frame";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}
		rtask.request.task_name = "teleop_right_frame";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}

		rtask.request.task_name = "teleop_left_align";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}
		if(check_alignment) {
		    rtask.request.task_name = "teleop_right_align";
		    if(!remove_controller_task_.call(rtask))
		    {
			ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
			ROS_BREAK();
		    }
		} else {
		    rtask.request.task_name = "gripper_right_align";
		    if(!remove_controller_task_.call(rtask))
		    {
			ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
			ROS_BREAK();
		    }
		    rtask.request.task_name = "gripper_right_approach";
		    if(!remove_controller_task_.call(rtask))
		    {
			ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
			ROS_BREAK();
		    }

		}

		/*
		*/
		//-------------------------------------------------------------------------//
	    }

	    return true;
	}

	bool switch_align_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    bools_mutex.lock();
	    align_on = !align_on;
	    bools_mutex.unlock();
	    return true;
	}
	bool regain_start_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    bools_mutex.lock();
	    regain_start = true;
	    bools_mutex.unlock();
	    return true;
	}

	bool quit_demo_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    bools_mutex.lock();
	    quit_demo = true;
	    bools_mutex.unlock();
	    return true;
	}

	bool start_demo_right_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    //-------------------------------------------------------------------------//
	    //-------------------------------------------------------------------------//
	    //move to teleop joint configuration
	    //-------------------------------------------------------------------------//
	    {
		//remove previous task

		hiqp_msgs::SetTask task;
		task.request.name = "teleop_init_config";
		task.request.priority = 3;
		task.request.visible = 1;
		task.request.active = 1;
		task.request.def_params.push_back("TDefFullPose");
		for(int i=0; i<teleop_sensing.size(); ++i) {
		    std::stringstream strm;
		    strm<<teleop_init[i];
		    task.request.def_params.push_back(strm.str());
		}
		task.request.dyn_params.push_back("TDynFirstOrder");
		std::stringstream strm;
		strm<<jnt_task_dynamics;
		task.request.dyn_params.push_back(strm.str());

		if(!set_controller_task_.call(task))
		{
		    ROS_ERROR("could not set task %s",task.request.name.c_str());
		    ROS_BREAK();
		}
		sleep(3);
	    }

	    //-------------------------------------------------------------------------//
	    {
		//add geometric primitives	
		double col[] = {0, 0, 0, 1};
		double params[] = {0, 0, 0, 1, 0, 0, 0};
		double params2[] = {0, 0, 0, 0, 0, 0};
		
		hiqp_msgs::AddGeometricPrimitive prim;
		prim.request.type = "frame";
		prim.request.visible = true;
		prim.request.color = std::vector<double>(col, col+sizeof(col)/sizeof(double));
		prim.request.parameters= std::vector<double>(params, params+sizeof(params)/sizeof(double));
		
		prim.request.name = "teleop_right_frame";
		prim.request.frame_id = "yumi_body";
		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}

		prim.request.name = "teleop_gripper_right_frame";
		prim.request.frame_id = "gripper_r_base";
		if(!add_controller_primitive_.call(prim))
		{
		    ROS_ERROR("could not add primitive %s",prim.request.name.c_str());
		    ROS_BREAK();
		}
		
		//Lines
		hiqp_msgs::AddGeometricPrimitive prim2;
		prim2.request.color = std::vector<double>(col, col+sizeof(col)/sizeof(double));
		prim2.request.parameters= std::vector<double>(params2, params2+sizeof(params2)/sizeof(double));

		prim2.request.name = "object_vertical_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "yumi_body";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 0;
		prim2.request.parameters[2] = 1;
		prim2.request.parameters[3] = 0.45;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0.1;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add primitive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		prim2.request.name = "gripper_approach_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "gripper_r_base";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 0;
		prim2.request.parameters[2] = 1;
		prim2.request.parameters[3] = 0;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add prim2itive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		prim2.request.name = "gripper_vertical_axis";
		prim2.request.type = "line";
		prim2.request.frame_id = "gripper_r_base";
		prim2.request.visible = false;
		prim2.request.parameters[0] = 0;
		prim2.request.parameters[1] = 1;
		prim2.request.parameters[2] = 0;
		prim2.request.parameters[3] = 0;
		prim2.request.parameters[4] = 0;
		prim2.request.parameters[5] = 0;
		if(!add_controller_primitive_.call(prim2))
		{
		    ROS_ERROR("could not add prim2itive %s",prim2.request.name.c_str());
		    ROS_BREAK();
		}
		
		//remove previous task
		hiqp_msgs::RemoveTask rtask;
		rtask.request.task_name = "teleop_init_config";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}

	    }
	    // enter teleop loop //
	    //define tasks	
	    std::stringstream strm;
	    hiqp_msgs::RemoveTask rtask;
	    
	    strm.clear(); 
	    hiqp_msgs::SetTask task_proj_right;
	    task_proj_right.request.name = "teleop_right_frame";
	    task_proj_right.request.priority = 2;
	    task_proj_right.request.visible = 1;
	    task_proj_right.request.active = 1;
	    task_proj_right.request.def_params.push_back("TDefGeomProj");
	    task_proj_right.request.def_params.push_back("frame");
	    task_proj_right.request.def_params.push_back("frame");
	    task_proj_right.request.def_params.push_back("teleop_right_frame = teleop_gripper_right_frame");
	    task_proj_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<teleop_task_dynamics;
	    task_proj_right.request.dyn_params.push_back(strm.str());

	    strm.clear(); 
	    hiqp_msgs::SetTask task_auto_align_vertical_right;
	    task_auto_align_vertical_right.request.name = "gripper_right_align";
	    task_auto_align_vertical_right.request.priority = 3;
	    task_auto_align_vertical_right.request.visible = 1;
	    task_auto_align_vertical_right.request.active = 1;
	    task_auto_align_vertical_right.request.def_params.push_back("TDefGeomAlign");
	    task_auto_align_vertical_right.request.def_params.push_back("line");
	    task_auto_align_vertical_right.request.def_params.push_back("line");
	    task_auto_align_vertical_right.request.def_params.push_back("object_vertical_axis = gripper_vertical_axis");
	    task_auto_align_vertical_right.request.def_params.push_back("0");
	    task_auto_align_vertical_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_auto_align_vertical_right.request.dyn_params.push_back(strm.str());
	    
	    strm.clear(); 
	    hiqp_msgs::SetTask task_auto_align_approach_right;
	    task_auto_align_approach_right.request.name = "gripper_right_approach";
	    task_auto_align_approach_right.request.priority = 3;
	    task_auto_align_approach_right.request.visible = 1;
	    task_auto_align_approach_right.request.active = 1;
	    task_auto_align_approach_right.request.def_params.push_back("TDefGeomProj");
	    task_auto_align_approach_right.request.def_params.push_back("line");
	    task_auto_align_approach_right.request.def_params.push_back("line");
	    task_auto_align_approach_right.request.def_params.push_back("gripper_approach_axis = object_vertical_axis");
	    task_auto_align_approach_right.request.dyn_params.push_back("TDynFirstOrder");
	    strm<<jnt_task_dynamics; //FIXME ?
	    task_auto_align_approach_right.request.dyn_params.push_back(strm.str());

	    bools_mutex.lock();
	    bool quit = quit_demo;
	    bools_mutex.unlock();
	    while(!quit) {

		//wait for operator to align markers
		bools_mutex.lock();
		bool restart = regain_start;
		align_on = false;
		bool check_alignment = align_on;
		regain_start = false;
		bools_mutex.unlock();
		if(restart) {
		    ROS_INFO("Setting initial pose again");
		    hiqp_msgs::SetTask task;
		    task.request.name = "teleop_init_config";
		    task.request.priority = 3;
		    task.request.visible = 1;
		    task.request.active = 1;
		    task.request.def_params.push_back("TDefFullPose");
		    for(int i=0; i<teleop_sensing.size(); ++i) {
			std::stringstream strm;
			strm<<teleop_init[i];
			task.request.def_params.push_back(strm.str());
		    }
		    task.request.dyn_params.push_back("TDynFirstOrder");
		    std::stringstream strm;
		    strm<<jnt_task_dynamics;
		    task.request.dyn_params.push_back(strm.str());

		    if(!set_controller_task_.call(task))
		    {
			ROS_ERROR("could not set task %s",task.request.name.c_str());
			ROS_BREAK();
		    }
		    sleep(3);
		    //remove previous task
		    hiqp_msgs::RemoveTask rtask;
		    rtask.request.task_name = "teleop_init_config";
		    if(!remove_controller_task_.call(rtask))
		    {
			ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
			ROS_BREAK();
		    }
		}

		ROS_INFO("waiting for sync...");
		if(!waitForSync(trans_thresh, rot_thresh, fresh_thresh, false, false, true)) { 
		    bools_mutex.lock();
		    quit = quit_demo;
		    bools_mutex.unlock();
		    continue; 
		}
		//-------------------------------------------------------------------------//
		//synced, enable task
		ROS_INFO("Synced, enabling teleop task");
		
		task_proj_right.request.active = 1;
		if(!set_controller_task_.call(task_proj_right))
		{
		    ROS_ERROR("could not set task %s",task_proj_right.request.name.c_str());
		    ROS_BREAK();
		}
		
		task_auto_align_vertical_right.request.active = 1;
		if(!set_controller_task_.call(task_auto_align_vertical_right))
		{
		    ROS_ERROR("could not set task %s",task_auto_align_vertical_right.request.name.c_str());
		    ROS_BREAK();
		}
		task_auto_align_approach_right.request.active = 1;
		if(!set_controller_task_.call(task_auto_align_approach_right))
		{
		    ROS_ERROR("could not set task %s",task_auto_align_approach_right.request.name.c_str());
		    ROS_BREAK();
		}

		/*
		*/
		
		//-------------------------------------------------------------------------//
		//monitor sync status
		if(!waitForSync(5*trans_thresh, 5*rot_thresh, 2*fresh_thresh, true, false, true)) { 
		    bools_mutex.lock();
		    quit = quit_demo;
		    bools_mutex.unlock();
		    continue; 
		}
		
		
		//-------------------------------------------------------------------------//
		//disable task
		rtask.request.task_name = "teleop_right_frame";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}
		rtask.request.task_name = "gripper_right_align";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}
		rtask.request.task_name = "gripper_right_approach";
		if(!remove_controller_task_.call(rtask))
		{
		    ROS_ERROR("could not remove task %s",rtask.request.task_name.c_str());
		    ROS_BREAK();
		}

		/*
		*/
		//-------------------------------------------------------------------------//
	    }

	    return true;
	    return true;
	}


};
//---------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_teleop");

    DemoTeleop demo_teleop;

    ROS_INFO("Demo teleop node ready");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
//---------------------------------------------------------------------
