#ifndef EXP_BALANCING_H
#define EXP_BALANCING_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <hiqp_ros/hiqp_client.h>

#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tfMessage.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <rosbag/bag.h>

class ExpBalancing {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;

  //  ros::Subscriber grasp_left_sub_;

  ros::Subscriber joint_state_sub_;
  ros::Subscriber tf_sub_;

  ros::Publisher marker_viz_pub_;
  //ros::Publisher gripper_pub_;	

  ///Clients to other nodes
  std::shared_ptr<hiqp_ros::HiQPClient> hiqp_client_;
	
  ros::ServiceServer start_demo_;
  ros::ServiceServer next_task_srv_;
  ros::ServiceServer switch_align_;
  ros::ServiceServer quit_demo_;

  tf::TransformListener tl;
  tf::TransformBroadcaster tb;

  ros::Timer tf_pub_timer, marker_pub_timer;

  boost::mutex bools_mutex, bag_mutex;
  boost::condition_variable cond_;

  std::string js_topic, tf_topic;
  std::string log_dir;

  double grasp_thresh, joint_task_tol, pre_grasp_task_tol, grasp_task_tol;
  int num_pickups;

  bool next_task, quit_demo;

  double grasp_thresh_tol;
  std::vector<hiqp_msgs::Task> pre_grasp_tasks, grasp_tasks, joint_tasks, teleop_tasks,
    pick_assisted_tasks, drop_assisted_tasks, point_assisted_tasks;
  std::vector<std::string> pre_grasp_task_names, grasp_task_names, joint_task_names, teleop_task_names,
    pick_assisted_task_names, drop_assisted_task_names, point_assisted_task_names;

  std::vector<tf::StampedTransform> target_poses;
  //  size_t current_target_id;

  bool publish_markers;
  visualization_msgs::MarkerArray gripper_target, gripper_target_ass, object_ass, object_noass, current_markers;
  Eigen::Vector3d obj_pos, obj_axis, bin_pos, bin_axis;
  std::string obj_frame, bin_frame;
  ///for bagging
  rosbag::Bag current_bag;
  std::string current_log_dir;
  bool bag_is_open;
 private:
  ///functions
  double getDoubleTime() {
    struct timeval time;
    gettimeofday(&time,NULL);
    return time.tv_sec + time.tv_usec * 1e-6; 
  }

  void loadGeometricPrimitivesFromParamServer();
  bool loadTasksFromParamServer(std::string task_group_name, 
				std::vector<hiqp_msgs::Task> &tasks_out, std::vector<std::string> &task_names_out);
	
  void grasp_left_callback(const std_msgs::Float32::ConstPtr& msg);
  void js_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void tf_callback(const tf::tfMessage::ConstPtr& msg);

  bool start_demo_callback(std_srvs::Empty::Request  &req,
			   std_srvs::Empty::Response &res );
  bool next_task_callback(std_srvs::Empty::Request  &req,
			  std_srvs::Empty::Response &res );
  bool quit_demo_callback(std_srvs::Empty::Request  &req,
			  std_srvs::Empty::Response &res );

  void tf_pub_callback(const ros::TimerEvent &te);
  void marker_pub_callback(const ros::TimerEvent &te);


  /// helpers for tasks
  bool setTasks(std::vector<hiqp_msgs::Task> &next_tasks,
		std::vector<std::string> &prev_task_names);
  //  bool waitForPoseAlignment();
	
  /// creates a folder for bags
  void setupNewExperiment();
  /// starts recording the next bag
  void startNextBag(std::string bagname);
  /// stops recording
  void closeCurrentBag();

  //helper for markers
  /* void addCylinderMarker(visualization_msgs::MarkerArray &markers, */
  /* 			 Eigen::Vector3d p, Eigen::Vector3d v, */
  /* 			 double r, std::string frame_, double h, */
  /* 			 std::string namespc, double rc, double g, */
  /* 			 double b); */
  
  void setPubMarkers(visualization_msgs::MarkerArray &markers);
  void stopPubMarkers();
 public:
  ExpBalancing();
  void expMainLoop();


};

#endif