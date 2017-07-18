#ifndef DEMO_TELEOP_H
#define DEMO_TELEOP_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <hiqp_ros/hiqp_client.h>

#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>

class DemoTeleop {
    private:
	ros::NodeHandle nh_;
	ros::NodeHandle n_;

	ros::Subscriber grasp_left_sub_;
	ros::Subscriber grasp_right_sub_;

	ros::Publisher marker_viz_pub_;

	///Clients to other nodes
	hiqp_ros::HiQPClient hiqp_client_;
	
	ros::ServiceClient close_gripper_clt_;
	ros::ServiceClient open_gripper_clt_;
	ros::ServiceClient reset_map_clt_;
	ros::ServiceClient convert_and_publish_map_;

	ros::ServiceServer start_demo_;
	ros::ServiceServer regain_start_pose_;
	ros::ServiceServer switch_align_;
	ros::ServiceServer quit_demo_;

	tf::TransformListener tl;

	boost::mutex bools_mutex;

	std::string gleft_topic, gright_topic;
	std::string frame_left, frame_right;
	std::string hand_frame_left, hand_frame_right;

	//Eigen::Affine3d leftInFrame, rightInFrame;
	//double trans_thresh, rot_thresh, time_last_left_pose, time_last_right_pose, fresh_thresh, jnt_task_dynamics, teleop_task_dynamics;
        double grasp_thresh;
	bool runOnline, leftClosed, rightClosed;
	bool regain_start, quit_demo, align_on;

	double grasp_thresh_tol;
	std::vector<double> teleop_sensing, teleop_init;
	std::vector<hiqp_msgs::Task> teleop_tasks;
	std::vector<std::string> teleop_task_names;

    private:
	///functions
	double getDoubleTime() {
	    struct timeval time;
	    gettimeofday(&time,NULL);
	    return time.tv_sec + time.tv_usec * 1e-6; 
	}

	void loadGeometricPrimitivesFromParamServer();
	void loadTasksFromParamServer();



    public:
	DemoTeleop();

	void grasp_left_callback(const std_msgs::Float32::ConstPtr& msg);
	void grasp_right_callback(const std_msgs::Float32::ConstPtr& msg);

	//bool waitForSync (double trans_thresh_loc, double rot_thresh_loc, double fresh_thresh_loc, bool isSyncedAlready, bool checkLeft=true, bool checkRight=true);

	bool start_demo_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res );
	bool regain_start_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res );
	bool quit_demo_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res );


};

#endif
