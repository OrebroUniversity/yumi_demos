#ifndef DEMO_GRASPING_H
#define DEMO_GRASPING_H

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <controller_manager_msgs/SwitchController.h>
#include <grasp_planner/PlanGrasp.h>
#include <hiqp_ros/hiqp_client.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Core>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace demo_grasping {
//-----------------------------------------------------------
//#define HQP_GRIPPER_JOINT 1

//#define PILE_GRASPING 1

#define DYNAMICS_GAIN 0.5
#define ALIGNMENT_ANGLE 0.05

#define SAFETY_HEIGHT 0.34
#define BEER_RADIUS 0.55
#define BEER_HEIGHT -0.03
#define EXTRACT_OFFSET 0.1
#define MIN_OPENING 0.1
#define OPENING_SAFETY_MARGIN 0.2
//-----------------------------------------------------------
///**To simplify, a grasp intervall is given as two concentric cylinders,
/// described by axis v and a point p on the axis (referenced in a static
/// obj_frame), and two planes. The controller will try to bring endeffector
/// point e, expressed in frame e_frame, inside the intervall described by the
/// two cylinders and the planes (i.e., inside the shell formed by the cylinders
/// and in between the planes described by n^Tx - d = 0)*/
struct GraspInterval {
  hiqp_msgs::Primitive upper;
  hiqp_msgs::Primitive lower;
  hiqp_msgs::Primitive left;
  hiqp_msgs::Primitive right;
  hiqp_msgs::Primitive inner;
  hiqp_msgs::Primitive outer;

  std::string obj_frame_;  // object frame
  std::string e_frame_;    // endeffector frame
  Eigen::Vector3d e_;      // endeffector point expressed in e_frame_
  float angle;
  bool isSphereGrasp, isDefaultGrasp;
  // PILE GRASPING STUFF
  //  Eigen::Vector3d p_;  // pile attack point
  // Eigen::Vector3d a_;  // approach axis
};

/*
struct PlaceInterval {
  std::string place_frame_;
  std::string e_frame_;  // endeffector frame

  Eigen::Vector3d e_;  // endeffector point expressed in e_frame_

  Eigen::Vector3d v_;  // place cylinder axis
  Eigen::Vector3d p_;  // place cylinder reference point
  double r_;           // place cylinder radius

  Eigen::Vector3d n_;  // place plane normal
  double d_;           // place plane offsets d !> 0

  std::vector<double> joints_;  // pre-place joint values
};
*/

class DemoGrasping {
 public:
  DemoGrasping();

 private:
  unsigned int n_jnts;
  std::vector<std::string> link_frame_names;

  ros::NodeHandle nh_;
  ros::NodeHandle n_;

  hiqp_ros::HiQPClient hiqp_client_;
  ros::ServiceClient start_client_ =
      n_.serviceClient<std_srvs::Empty>("/gplanner/start_tracker");
  ros::ServiceClient stop_client_ =
      n_.serviceClient<std_srvs::Empty>("/gplanner/stop_tracker");

  // double task_error_tol_;
  // double task_diff_tol_;
  // double task_timeout_tol_;
  // bool task_status_changed_;
  // bool task_success_;
  bool with_gazebo_;  ///<indicate whether the node is run in simulation
  // std::vector<unsigned int> pers_task_vis_ids_;

  //**Grasp definition - this should be modified to grasp different objects */
  GraspInterval grasp_;
  grasp_planner::PlanGrasp planGraspMsg;
  // std::vector<PlaceInterval> place_zones_;  ///< placement zones for the
  // object
  Eigen::VectorXd t_prog_prev_;

  /// Clients to other nodes
  ros::ServiceClient get_grasp_interval_clt_;
  ros::ServiceClient set_gazebo_physics_clt_;

  ros::ServiceClient close_gripper_clt_;
  ros::ServiceClient open_gripper_clt_;

  ros::ServiceClient reset_map_clt_;

  /// Servers
  ros::ServiceServer start_demo_srv_;
  // ros::ServiceServer look_what_i_found_srv_;

  //** Manipulator joint configuration while moving the forklift */
  std::vector<double> transfer_config_;
  //** Manipulator joint configuration prior to reach-to-grasp */
  std::vector<double> sensing_config_;

  //**First deactivates the HQP control scheme (the controller will output zero
  // velocity commands afterwards) and then calls a ros::shutdown */
  void safeShutdown();
  //**like shutdown but we can run again */
  void safeReset();

  void startTracker();
  void stopTracker();

  bool doGraspAndLift();
  // bool setGraspApproach();
  // bool setObjectExtract();
  // bool setGripperExtract(PlaceInterval const& place);
  // bool setObjectPlace(PlaceInterval const& place);
  // bool loadPersistentTasks();
  bool getGraspInterval();

  tf::TransformListener transform_listener_;
  ros::Publisher pose_pub_;

  bool startDemo(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  // bool lookWhatIFound(std_srvs::Empty::Request& req,
  //                   std_srvs::Empty::Response& res);
};

}  // end namespace hqp controllers

#endif
