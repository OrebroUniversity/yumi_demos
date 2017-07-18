#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream


#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include<demo_teleop/demo_teleop.h>

DemoTeleop::DemoTeleop(): hiqp_client_("yumi", "hiqp_joint_velocity_controller") {
    
    nh_ = ros::NodeHandle("~");
    n_ = ros::NodeHandle();

    nh_.param<std::string>("grasp_left_topic",gleft_topic,"/leap_hands/grasp_left");
    nh_.param<std::string>("grasp_rigth_topic",gright_topic,"/leap_hands/grasp_right");

    nh_.param<std::string>("frame_left",frame_left,"gripper_l_base");
    nh_.param<std::string>("frame_right",frame_right,"gripper_r_base");
    nh_.param<std::string>("hand_frame_left",hand_frame_left,"teleop_left_frame");
    nh_.param<std::string>("hand_frame_right",hand_frame_right,"teleop_right_frame");

    nh_.param("grasp_threshold",grasp_thresh,0.7); //0 to 1
    nh_.param("run_online",runOnline,false); //0 to 1

    nh_.getParam("jnts_teleop_sensing", teleop_sensing);
    ROS_ASSERT_MSG(teleop_sensing.size() == 14, 
	    "Please set a valid configuration for sensing pose. You provided %lu values, instead of 14",teleop_sensing.size());
    nh_.getParam("jnts_teleop_init", teleop_init);
    ROS_ASSERT_MSG(teleop_init.size() == 14, 
	    "Please set a valid configuration for sensing pose. You provided %lu values, instead of 14",teleop_init.size());

    marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("tracking_state_markers",10);

    grasp_left_sub_ = n_.subscribe(gleft_topic, 1, &DemoTeleop::grasp_left_callback, this);
    grasp_right_sub_ = n_.subscribe(gright_topic, 1, &DemoTeleop::grasp_right_callback, this);

    start_demo_ = nh_.advertiseService("start_demo", &DemoTeleop::start_demo_callback, this);
    regain_start_pose_ = nh_.advertiseService("regain_start", &DemoTeleop::regain_start_callback, this);
    quit_demo_ = nh_.advertiseService("quit_demo", &DemoTeleop::quit_demo_callback, this);

    //these should probably be read from joint states
    leftClosed = false;
    rightClosed = false;
    grasp_thresh_tol=0.2;

    loadGeometricPrimitivesFromParamServer();
    loadTasksFromParamServer();

    if(runOnline) {
	close_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("close_gripper");
	close_gripper_clt_.waitForExistence();

	open_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("open_gripper");
	open_gripper_clt_.waitForExistence();

	reset_map_clt_ = n_.serviceClient<std_srvs::Empty>("reset_map");
	reset_map_clt_.waitForExistence();

	convert_and_publish_map_ = n_.serviceClient<std_srvs::Empty>("map_to_edt");
	convert_and_publish_map_.waitForExistence();
    }
}

#if 0
	void pose_left_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	    std::string base_frame = msg->header.frame_id;

	    tf::StampedTransform base2frame_tf;
	    Eigen::Affine3d base2frame, msg2base;
	    try {
		tl.waitForTransform(frame_left, hand_frame_left, ros::Time(0), ros::Duration(0.25) );
		tl.lookupTransform(frame_left, hand_frame_left,  ros::Time(0), base2frame_tf);
	    } catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return;
	    }
	    tf::transformTFToEigen(base2frame_tf,leftInFrame);
	    

	}
#endif

void DemoTeleop::grasp_left_callback(const std_msgs::Float32::ConstPtr& msg) {
#if 0 
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
#endif
}

void DemoTeleop::grasp_right_callback(const std_msgs::Float32::ConstPtr& msg) {

#if 0	    
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
#endif	    

}


bool DemoTeleop::start_demo_callback(std_srvs::Empty::Request  &req,
	std_srvs::Empty::Response &res ) {

    //-------------------------------------------------------------------------//
    //set joint configuration out of sensor view
    {
	if(!hiqp_client_.setJointAngles(teleop_sensing))
	{
	    ROS_ERROR("could not set task");
	    ROS_BREAK();
	}
    }
    //wait to achieve
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
    //-------------------------------------------------------------------------//
    //move to teleop joint configuration
    //-------------------------------------------------------------------------//
    {
	if(!hiqp_client_.setJointAngles(teleop_init))
	{
	    ROS_ERROR("could not set task ");
	    ROS_BREAK();
	}
	sleep(2);
    }

    //-------------------------------------------------------------------------//
    hiqp_client_.setTasks(teleop_tasks);


    return true;
}

bool DemoTeleop::regain_start_callback(std_srvs::Empty::Request  &req,
	std_srvs::Empty::Response &res ) {

    ROS_INFO("Setting initial pose again");
    hiqp_client_.removeTasks(teleop_task_names);
    if(!hiqp_client_.setJointAngles(teleop_init))
    {
	ROS_ERROR("could not set task");
	ROS_BREAK();
    }
    sleep(2);
    hiqp_client_.setTasks(teleop_tasks);
    return true;
}

bool DemoTeleop::quit_demo_callback(std_srvs::Empty::Request  &req,
	std_srvs::Empty::Response &res ) {

    hiqp_client_.removeTasks(teleop_task_names);
    return true;
}

void DemoTeleop::loadGeometricPrimitivesFromParamServer() {
    XmlRpc::XmlRpcValue hiqp_teleop_geometric_primitives;
    if (!nh_.getParam(
		"teleop_geometric_primitives",
		hiqp_teleop_geometric_primitives)) {
	ROS_WARN_STREAM("No teleop_geometric_primitives parameter "
		<< "found on the parameter server. No geometric primitives "
		<< "were loaded!");
    } else {
	bool parsing_success = true;
	for (int i = 0; i < hiqp_teleop_geometric_primitives.size(); ++i) {
	    try {
		std::string name = static_cast<std::string>(
			hiqp_teleop_geometric_primitives[i]["name"]);
		std::string type = static_cast<std::string>(
			hiqp_teleop_geometric_primitives[i]["type"]);
		std::string frame_id = static_cast<std::string>(
			hiqp_teleop_geometric_primitives[i]["frame_id"]);
		bool visible =
		    static_cast<bool>(hiqp_teleop_geometric_primitives[i]["visible"]);

		XmlRpc::XmlRpcValue& color_xml =
		    hiqp_teleop_geometric_primitives[i]["color"];
		XmlRpc::XmlRpcValue& parameters_xml =
		    hiqp_teleop_geometric_primitives[i]["parameters"];

		std::vector<double> color;
		color.push_back(static_cast<double>(color_xml[0]));
		color.push_back(static_cast<double>(color_xml[1]));
		color.push_back(static_cast<double>(color_xml[2]));
		color.push_back(static_cast<double>(color_xml[3]));

		std::vector<double> parameters;
		for (int j = 0; j < parameters_xml.size(); ++j) {
		    parameters.push_back(static_cast<double>(parameters_xml[j]));
		}

		hiqp_client_.setPrimitive(name, type, frame_id, visible, color, parameters);
	    } catch (const XmlRpc::XmlRpcException& e) {
		ROS_WARN_STREAM(
			"Error while loading "
			<< "teleop_geometric_primitives parameter from the "
			<< "parameter server. XmlRcpException thrown with message: "
			<< e.getMessage());
		parsing_success = false;
		break;
	    }
	}

	if (parsing_success)
	    ROS_INFO_STREAM("Loaded and initiated geometric primitives from "
		    << ".yaml file successfully!");
    }
}

void DemoTeleop::loadTasksFromParamServer() {
    XmlRpc::XmlRpcValue hiqp_teleop_tasks;
    if (!nh_.getParam("teleop_tasks",
		hiqp_teleop_tasks)) {
	ROS_WARN_STREAM("No teleop_tasks parameter found on "
		<< "the parameter server. No tasks were loaded!");
    } else {
	bool parsing_success = true;
	for (int i = 0; i < hiqp_teleop_tasks.size(); ++i) {
	    try {
		hiqp_msgs::Task teleop_task;
		teleop_task.name =
		    static_cast<std::string>(hiqp_teleop_tasks[i]["name"]);

		XmlRpc::XmlRpcValue& def_params_xml =
		    hiqp_teleop_tasks[i]["def_params"];
		for (int j = 0; j < def_params_xml.size(); ++j) {
		    teleop_task.def_params.push_back(static_cast<std::string>(def_params_xml[j]));
		}

		XmlRpc::XmlRpcValue& dyn_params_xml =
		    hiqp_teleop_tasks[i]["dyn_params"];
		for (int j = 0; j < dyn_params_xml.size(); ++j) {
		    teleop_task.dyn_params.push_back(static_cast<std::string>(dyn_params_xml[j]));
		}

		teleop_task.priority =
		    static_cast<int>(hiqp_teleop_tasks[i]["priority"]);
		teleop_task.visible = static_cast<bool>(hiqp_teleop_tasks[i]["visible"]);
		teleop_task.active = static_cast<bool>(hiqp_teleop_tasks[i]["active"]);
		teleop_task.monitored = static_cast<bool>(hiqp_teleop_tasks[i]["monitored"]);

		teleop_tasks.push_back(teleop_task);
		teleop_task_names.push_back(teleop_task.name);

	    } catch (const XmlRpc::XmlRpcException& e) {
		ROS_WARN_STREAM(
			"Error while loading "
			<< "hiqp_teleop_tasks parameter from the "
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
}


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
    
