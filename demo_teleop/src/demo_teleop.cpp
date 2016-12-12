#include <ros/ros.h>
#include <yumi_hw/YumiGrasp.h>
#include <geometry_msgs/PoseStamped.h>
#include <hiqp_msgs/SetTask.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
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
	ros::ServiceClient set_joint_task_;
	ros::ServiceClient close_gripper_clt_;
	ros::ServiceClient open_gripper_clt_;
	ros::ServiceClient clear_map_;
	ros::ServiceClient convert_map_;
	ros::ServiceClient publish_map_;
	ros::ServiceClient activate_controller_;

	ros::ServiceServer start_demo_;
	ros::ServiceServer start_demo_right_;

	tf::TransformListener tl;

	boost::mutex newpose_m;
	boost::condition_variable cond_;

	std::string pleft_topic, pright_topic, gleft_topic, gright_topic;
	std::string frame_left, frame_right;

	Eigen::Affine3d leftInFrame, rightInFrame;
	double trans_thresh, rot_thresh, grasp_thresh;
	bool new_pose_, first_left, first_right, runOnline, leftClosed, rightClosed;
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
	    nh_.param("run_online",runOnline,false); //0 to 1
	    
	    marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("tracking_state_markers",10);

	    pose_left_sub_ = n_.subscribe(pleft_topic, 1, &DemoTeleop::pose_left_callback, this);
	    pose_right_sub_ = n_.subscribe(pright_topic, 1, &DemoTeleop::pose_right_callback, this);
	    grasp_left_sub_ = n_.subscribe(gleft_topic, 1, &DemoTeleop::grasp_left_callback, this);
	    grasp_right_sub_ = n_.subscribe(gright_topic, 1, &DemoTeleop::grasp_right_callback, this);

	    start_demo_ = nh_.advertiseService("start_demo", &DemoTeleop::start_demo_callback, this);
	    start_demo_right_ = nh_.advertiseService("start_demo_right", &DemoTeleop::start_demo_right_callback, this);

	    new_pose_ = false;
	    first_left = false;
	    first_right = false;
	    //these should probably be read from joint states
	    leftClosed = false;
	    rightClosed = false;
	
	    if(runOnline) {
		close_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("close_gripper");
		close_gripper_clt_.waitForExistence();
	
		open_gripper_clt_ = n_.serviceClient<yumi_hw::YumiGrasp>("open_gripper");
		open_gripper_clt_.waitForExistence();
	    }
	}

	void pose_left_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	    std::string base_frame = msg->header.frame_id;

	    tf::StampedTransform base2frame_tf;
	    Eigen::Affine3d base2frame, msg2base;
	    try {
		tl.waitForTransform(frame_left, base_frame, msg->header.stamp, ros::Duration(0.15) );
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
		first_left = true;
		cond_.notify_one();
	    }

	}

	void pose_right_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	    std::string base_frame = msg->header.frame_id;

	    tf::StampedTransform base2frame_tf;
	    Eigen::Affine3d base2frame, msg2base;
	    try {
		tl.waitForTransform(frame_right, base_frame, msg->header.stamp, ros::Duration(0.15) );
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
		first_right = true;
		cond_.notify_one();
	    }

	}

	void grasp_left_callback(const std_msgs::Float32::ConstPtr& msg) {
	   
	    if(msg->data > grasp_thresh && !leftClosed) { 
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
	    if(msg->data < grasp_thresh && leftClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =1;
		if(runOnline) {
		    if(!open_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not close gripper");
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
	   
	    if(msg->data > grasp_thresh && !rightClosed) { 
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
	    if(msg->data < grasp_thresh && rightClosed) { 
		yumi_hw::YumiGrasp gr;
		gr.request.gripper_id =2;
		if(runOnline) {
		    if(!open_gripper_clt_.call(gr))
		    {
			ROS_ERROR("could not close gripper");
			ROS_BREAK();
		    }
		} else {
		    ROS_INFO("right gripper should open");
		}
		sleep(1);
		rightClosed = false;
	    }

	}

	bool start_demo_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

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
	    while(!synced)
	    {
		boost::mutex::scoped_lock lock(newpose_m, boost::try_to_lock);
		while(!new_pose_)
		    cond_.wait(lock);
	
		new_pose_ = false;
		
		double t_err_l = leftInFrame.translation().norm();
		double r_err_l = leftInFrame.rotation().eulerAngles(0,1,2).norm();
		double t_err_r = rightInFrame.translation().norm();
		double r_err_r = rightInFrame.rotation().eulerAngles(0,1,2).norm();
		
		synced = t_err_l < trans_thresh && r_err_l < rot_thresh && t_err_r < trans_thresh && r_err_r < rot_thresh;
		synced = synced && first_left && first_right; // make sure we are not running on empty

		/// visualization stuff ////
		if(t_err_l < trans_thresh && r_err_l < rot_thresh && first_left) {
		    //set left marker to green
		    leftSphere.color.r = 0.1;
		    leftSphere.color.g = 0.9;
		} else {
		    leftSphere.color.r = 0.9;
		    leftSphere.color.g = 0.1;
		}
		if(t_err_r < trans_thresh && r_err_r < rot_thresh && first_right) {
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

	bool start_demo_right_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

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
