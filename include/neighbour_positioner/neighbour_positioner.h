#include <ros/ros.h>

#include <rcomponent/rcomponent.h>
#include <robotnik_msgs/State.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <costmap_prohibition_layer/UpdateZones.h>

class NeighbourPositioner : public rcomponent::RComponent
{
public:
  NeighbourPositioner(ros::NodeHandle h);
  virtual ~NeighbourPositioner();

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  int rosSetup();
  //! Shutdowns all the ROS' stuff
  int rosShutdown();
  //! Reads data a publish several info into different topics
  void rosPublish();
  //! Reads params from params server
  void rosReadParams();

  //! Actions performed on standby state
  void standbyState();
  //! Actions performed on ready state
  void readyState();
  //! Actions performed on the emergency state
  void emergencyState();
  //! Actions performed on Failure state
  void failureState();

protected:
  // Specific node stuff
  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  string local_position_topic_, neighbours_topic_;
  ros::Publisher local_position_pub_;
  ros::Subscriber neighbours_sub_;

  nav_msgs::Odometry last_odom_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform current_transform_, last_transform_;

  ros::Time last_tf_time_;

  string fixed_frame_, base_frame_;

  string prohibition_layer_srv_;
  ros::ServiceClient prohibition_layer_client_;

  geometry_msgs::TransformStamped current_neighbour_transform_, last_neighbour_transform_;
  ros::Time last_time_neighbour_;

  void neighboursCb(const geometry_msgs::TransformStamped::ConstPtr &msg);
};
