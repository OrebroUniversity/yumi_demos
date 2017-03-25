#include <ros/ros.h>
#include <std_srvs/Empty.h>
// #include <hqp_controllers_msgs/TaskGeometry.h>
// #include <hqp_controllers_msgs/FindCanTask.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

// #include<pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/kdtree/kdtree.h>

// #include <tf/transform_listener.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <eigen_conversions/eigen_msg.h>
// #include <tf_conversions/tf_eigen.h>

// #include <Eigen/Core>
//#include <boost/thread/mutex.hpp>

class TeleopRemapper {

private:
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_;

  ros::Subscriber link_states_sub_;
  ros::Subscriber grasp_left_sub_;
  ros::Subscriber grasp_right_sub_;
  ros::Publisher grasp_left_pub_;
  ros::Publisher grasp_right_pub_;
	ros::ServiceServer reset_obj_srv_;
        ros::ServiceClient set_object_state_clt_;
  // ros::ServiceServer find_can_srv_;
  // tf::TransformListener tl;
  tf::TransformBroadcaster object_pose_bc_;
  // pcl::PointCloud<pcl::PointXYZ> my_cloud;
  // std::string pcloud_topic;
  // std::string pcloud_frame_name;
  // std::string world_frame;
  // std::string palm_frame;
  // double expected_pallet_height;
  // double height_cutoff;
  // double eps;
  // double angle_thresh;
  // double eval_thresh;
  // double max_dist;
  // double max_x;
  // double cpoint_zheight;
  // int min_number_pts;
  // //boost::mutex::cloud_mutex;
  // double grow_cylinder_m, inner2outer, grow_plane_m;

  // double dist_factor;
  // Eigen::Affine3d world2palm;
public:

  TeleopRemapper() {
    nh_private_ = ros::NodeHandle("~");
    nh_ = ros::NodeHandle();
    std::string link_states_topic, grasp_left_topic, grasp_right_topic, grasp_left_command, grasp_right_command;

    nh_private_.param<std::string>("link_states_topic", link_states_topic,"/gazebo/link_states");
    nh_private_.param<std::string>("grasp_left_topic", grasp_left_topic,"/leap_hands/grasp_left");
    nh_private_.param<std::string>("grasp_right_topic", grasp_right_topic,"/leap_hands/grasp_right");
    nh_private_.param<std::string>("grasp_left_command", grasp_left_command,"/yumi/yumi_gripper_l_effort_position_controller/command");
    nh_private_.param<std::string>("grasp_right_command", grasp_right_command,"/yumi/yumi_gripper_r_effort_position_controller/command");


    // nh_.param("pallet_height", expected_pallet_height ,0.1);
    // nh_.param("max_dist", max_dist ,2.0);
    // nh_.param("max_x", max_x ,1.3);
    // nh_.param("dist_factor", dist_factor ,0.02);
	    
    // nh_.param("min_pts_cluster",min_number_pts,250);
    // nh_.param("pallet_height_tolerance",eps,0.03); 
    // nh_.param("plane_angle_tolerance",angle_thresh,10*M_PI/180); 
    // nh_.param("objects_max_height",height_cutoff,0.3);
    // nh_.param("cylinder_evals_thresh",eval_thresh,0.8);
    // nh_.param("grow_cylinder_m",grow_cylinder_m,0.25);
    // nh_.param("cyl2cyl_m",inner2outer,0.2);
    // nh_.param("grow_plane_m",grow_plane_m,0.05);
    // nh_.param("cpoint_zheight",cpoint_zheight,0.18);
	    

    // find_can_srv_ = nh_.advertiseService("find_cans", &CanFinderNode::find_cans, this);
    link_states_sub_ = nh_.subscribe(link_states_topic, 1, &TeleopRemapper::linkStatesCallback, this);
    grasp_left_sub_ = nh_.subscribe(grasp_left_topic, 1, &TeleopRemapper::graspLeftCallback, this);
    grasp_right_sub_ = nh_.subscribe(grasp_right_topic, 1, &TeleopRemapper::graspRightCallback, this);
    grasp_left_pub_ = nh_.advertise<std_msgs::Float64>(grasp_left_command, 10); 
    grasp_right_pub_ = nh_.advertise<std_msgs::Float64>(grasp_right_command, 10); 

	    reset_obj_srv_ = nh_.advertiseService("reset_object", &TeleopRemapper::resetObjectCallback, this);

	    set_object_state_clt_ = nh_.serviceClient<gazebo_msgs::SetLinkState>("gazebo/set_link_state");
	    set_object_state_clt_.waitForExistence();

  }
  //===================================================================================
	bool resetObjectCallback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	  gazebo_msgs::SetLinkState msg;
	  msg.request.link_state.link_name="object::object";
          msg.request.link_state.pose.position.x=0.45;
          msg.request.link_state.pose.position.y=0.0;
          msg.request.link_state.pose.position.z=0.5; //make the object fall down just to make sure it doesn't intersect with the ground plane
          msg.request.link_state.pose.orientation.x=0.0;
          msg.request.link_state.pose.orientation.y=0.0;
          msg.request.link_state.pose.orientation.z=0.0;
          msg.request.link_state.pose.orientation.w=1.0;
          msg.request.link_state.twist.linear.x=0.0;
          msg.request.link_state.twist.linear.z=0.0;
          msg.request.link_state.twist.linear.y=0.0;
          msg.request.link_state.twist.angular.x=0.0;
          msg.request.link_state.twist.angular.y=0.0;
          msg.request.link_state.twist.angular.z=0.0;

	  if(set_object_state_clt_.call(msg))
	  return true;
	  else
	    return false;
	}
  //===================================================================================
//! Broadcasts the pose of the target object to TF -> used for visualization as a robot model in Rviz
  void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
    //std::cerr<<"link states callback"<<std::endl;

    tf::Transform transform;
    std::vector<std::string> names=msg->name;
    std::vector<std::string>::iterator it=std::find(names.begin(), names.end(), "object::object");

    if (it == names.end())
      {
	ROS_WARN("TeleopRemapper::linkStatesCallback(...): Cannot find object in Gazebo!");
	return;
      }
    else
      {
	int index= std::distance(names.begin(), it);
	tf::Quaternion q;
	tf::Point p;
	quaternionMsgToTF(msg->pose[index].orientation, q);
	transform.setRotation(q);
	pointMsgToTF(msg->pose[index].position, p);
	transform.setOrigin(p);
      }

    object_pose_bc_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "object"));
  }
  //===================================================================================
  void graspLeftCallback(const std_msgs::Float32::ConstPtr& msg) {
    //std::cerr<<"grasp left callback"<<std::endl;
    std_msgs::Float64 pos;
    pos.data=0.025-0.025*msg->data; //invert open/close motion and scale to 0.25 (max yumi gripper opening)
    grasp_left_pub_.publish(pos);
  }
  //===================================================================================
  void graspRightCallback(const std_msgs::Float32::ConstPtr& msg) {
    //std::cerr<<"grasp right callback"<<std::endl;
    std_msgs::Float64 pos;
    pos.data=0.025-0.025*msg->data;
    grasp_right_pub_.publish(pos);
  }
  //===================================================================================
};


int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "teleop_remapper");
  TeleopRemapper teleop_remapper;
  // ros::AsyncSpinner spinner(4); // Use 4 threads
  // spinner.start();
  // ros::waitForShutdown();

  ros::spin();
  return 0;
}

