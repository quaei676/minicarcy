
 /*********************************************************************/

#ifndef MINICAR_CONTROL_NODE_H__
#define MINICAR_CONTROL_NODE_H__

#include <ros/ros.h>
#include <ros/console.h>


// ROS messages:
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Dense>  /* for matrix multiplication */
#include <math.h>       /* sin, sqrt */



namespace minicar
{
class ControlNode
{
public:
  ControlNode(ros::NodeHandle& nh);

  void update();

  double update_rate_;
  double update_time_;

protected:
  // helper
  double convertDistanceToAngle(double dis);
  std::vector<double> convertVectorToAngle(std::vector<double> input_vec);

  void calc2DMotorCommands();
  void calc2MotorCommands_withoutOdometry();
  void calc2MotorCommands_withOdometry();
  std::vector<double> toEulerAngle(double x, double y, double z, double w);

  std::vector<double> convertToAngleVel(std::vector<double> input_vec);
  double linearVelToAngleVel(double linear_vel);

  // UI callbacks

  // ROS API callbacks
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);


  // class members
  //Control2D control2D_;

  // helper variables:
  sensor_msgs::Imu previous_imu_msg_;
  bool imu_updated_=true;
  sensor_msgs::JointState previous_joint_state_msg_;
  bool joint_state_updated_;
  std::vector<double> imu_phi_, imu_dphi_;
  std::vector<double> joints_position_;
  std::vector<double> joints_velocity_;
  std::vector<double> realV_;
  geometry_msgs::Vector3 desired_velocity_;

  // parameters from the parameter server
  std::vector<double> gains_2D_Kxz_;
  std::vector<double> gains_2D_Kyz_;
  double Kr_;
  double Tv_;

  std::string controller_type_;
  std::string motors_controller_type_;
  double alpha_;
  double theta_;
  double Ka_;
  double k1_;
  double k2_;
  double k3_;
  double k6_;
  double gy_;
  double t_; 
  double Mb_;
  double Mw_;
  double ixx_;
  double iyy_;
  double izz_;
  double cz_ ;
  double R_ ;
  double Ks1_ ;
  double Ks2_ ;
  int balancing_time_;



   // subscriber
  ros::Subscriber imu_sub_;
  ros::Subscriber joints_sub_;

  //publisher:
  ros::Publisher joint_commands_1_pub_;
  ros::Publisher joint_commands_2_pub_;
  ros::Publisher rpy_pub_;
  ros::Publisher calc_minicar_odom_pub_;
  ros::Publisher desired_velocity_pub_;    // calculated torques in NM!

  // action server


  //for control:

  std::vector<double> omwga_car_last_;
  std::vector<double> imu_phi_last_;
  std::vector<double> positionxyz;
  std::vector<double> positionxyz_last_;
};

} // end namespace
#endif
