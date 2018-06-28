

#include <mc_control_node.hpp>

namespace minicarcy
{
  //helper functions:
  double ControlNode::convertDistanceToAngle(double dis)
  {
     // std::cout<<dis<<" m"<<std::endl;
      double a =  (double) 360/(2*3.14159265359*0.06) *dis;

      //std::cout<<a<<" grad"<<std::endl;
      double b = (double) a/180 * 3.14159265359;
      //stdcout<<b<<" rad"<<std::endl;


      double c = a/360;
      double d = a-(c*360);

      //std::cout<<d<<" grad angepasst"<<std::endl;

      double e = (double) d/180 * 3.14159265359;
      std::cout<<e<<" rad angepasst"<<std::endl;

      // return valueebetwenn 0 and 2 pi!
      return e;
  }


  std::vector<double> ControlNode::toEulerAngle(double x, double y, double z, double w)
  {
    // roll (x-axis rotation)
    double sinr = +2.0 * (w*x +y*z);
    double cosr = +1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = 2.0*(z*w+y*x);
    double cosp = +1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(sinp, cosp);


    // yaw (z-axis rotation)
    double  pitch= asin(2*w*y-2*z*x);
    return {roll, pitch, yaw};
  }

 

  // this node is updated every 10ms.
  // this calculation takes less than 1ms.
//yooooooooooooooooooo start
  void ControlNode::calc2MotorCommands_withOdometry()
  {
    double t1_=10;
    if(t_<=40)
        t_=t_+update_time_;
    if (t_<=10)
        t1_=t_;
    double I=0.145;
    double theta_dot_=0.035*(-joints_velocity_.at(1)+joints_velocity_.at(0))/I;
    theta_=theta_+theta_dot_*update_time_;
    double v_=-0.035*(joints_velocity_.at(1)+joints_velocity_.at(0))/2;
    double v_x = cos(theta_)*v_ ;
    double v_y = sin(theta_)*v_ ;
    positionxyz.at(0) =positionxyz_last_.at(0) +  v_x*update_time_;
    positionxyz.at(1) =positionxyz_last_.at(1) +  v_y*update_time_;
    double position_reference_=t_;
    double x_tilde_=0.1*t_-positionxyz.at(0);
    double y_tilde_=0.1*t_-positionxyz.at(1);
    double theta_tilde_=3.1415926153589/4-theta_; 
    double v_r_=0.1;
    double omega_r_=0;
    double alpha_=imu_phi_.at(1);
    double alpha_dot_=imu_dphi_.at(1);
    double e1=cos(theta_)*x_tilde_+sin(theta_)*y_tilde_;
    double e2=-sin(theta_)*x_tilde_+cos(theta_)*y_tilde_;
    double e3=theta_tilde_;
    double e_ba3_=e3+alpha_*e2;
    double phi_1_=-Ka_*alpha_;
    double phi_2_=k1_*e1+v_r_*cos(e3);
    double phi_3_=(k2_*e_ba3_+0.2*(sin(e3)/e_ba3_)*e2+alpha_*v_r_*sin(e3)+omega_r_)/(1.0+k3_*e1);
    double D_alpha_=pow(Mb_,2)*pow(cos(alpha_),2)*pow(cz_ ,2.)*pow(R_,2.)+((-pow(Mb_,2.)-2.*Mw_*Mb_)*
           pow(cz_,2.)-2.*iyy_*Mw_-iyy_*Mb_)*pow(R_,2.);
    double G_alpha_=(-Mb_*pow(cz_,2)+izz_-ixx_)*pow(R_,2)*pow(cos(alpha_),2)+(Mb_*pow(cz_,2)+ixx_
           +2*pow(0.065,2)*Mw_)*pow(R_,2);
    double H_ba_=Mb_*pow(R_,2)*izz_-Mw_*pow(R_,2)*ixx_-Mb_*pow(cz_,2)*Mw_*pow(R_,2)-Mb_*pow(R_,2)
           *ixx_/2+Mw_*pow(R_,2)*izz_;
    double S1_=(imu_dphi_.at(1)-phi_1_)+(v_-phi_2_);
    double g21=(Mb_*pow(R_,2)+2*Mw_*pow(R_,2)+Mb_*cos(alpha_)*cz_*R_)/D_alpha_;
    double g22=-R_/D_alpha_*(Mb_*cos(alpha_)*cz_*R_+iyy_+Mb_*pow(cz_,2));
    double C_theta_=-(1/(g22+g21))*(Ks1_*S1_/abs(S1_)+Ks2_*S1_);
    double S_eta_=theta_dot_-phi_3_;
    double C_y_=S_eta_/abs(S_eta_)*(-k6_-gy_);
    geometry_msgs::PointStamped minicarcy_odom;
    minicarcy_odom.header.stamp = ros::Time::now();
    minicarcy_odom.header.frame_id = "minicarcy_odom_phi_x,y_dot";
    minicarcy_odom.point.x = positionxyz.at(0) ;
    minicarcy_odom.point.y = positionxyz.at(1) ;
    minicarcy_odom.point.z = theta_;
    calc_minicarcy_odom_pub_.publish(minicarcy_odom);


  
    double VR=0,VL=0;

	VR = //0.5*C_theta_-0.5*C_y_;
             1.0*(gains_2D_Kxz_[1]*imu_phi_.at(1)+gains_2D_Kxz_[3]*imu_dphi_.at(1)+gains_2D_Kxz_[0]*((positionxyz.at(0)-0.1*0))       
               +gains_2D_Kxz_[2]*(joints_velocity_.at(1)));
	VL = //0.5*C_theta_+0.5*C_y_;
             1.0*(gains_2D_Kxz_[1]*imu_phi_.at(1)+gains_2D_Kxz_[3]*imu_dphi_.at(1)+gains_2D_Kxz_[0]*((positionxyz.at(0)-0.1*0))       
               +gains_2D_Kxz_[2]*(joints_velocity_.at(0)));
    //ROS_INFO("%f,%f",VR,VL);
    positionxyz_last_.at(0) = positionxyz.at(0) ;
    positionxyz_last_.at(1) = positionxyz.at(1) ;
    realV_ = {VR,VL}; // wheels turn the other way round!
  }

  

  //constructor is called only once:
  ControlNode::ControlNode(ros::NodeHandle& nh)
  {
    //get Params:
    motors_controller_type_ = nh.param("control_constants/motors_controller_type", std::string("EffortJointInterface"));

    // get 2D control gains:
    gains_2D_Kxz_=nh.param("control_constants/gains_2D_Kxz",(std::vector<double> ) {1.0,2.0,3.0,4.0,5.0,6.0});
    gains_2D_Kyz_=nh.param("control_constants/gains_2D_Kyz", (std::vector<double> ) {1.0,2.0,3.0,4.0,5.0,6.0});


    update_rate_ = nh.param("control_constants/update_rate", 10.0);
    Ka_ = nh.param("control_constants/Ka",(double) 10.0);
    k1_ = nh.param("control_constants/k1",(double) 10.0);
    k2_ = nh.param("control_constants/k2",(double) 10.0);
    k3_ = nh.param("control_constants/k3",(double) 10.0);
    k6_ = nh.param("control_constants/k6",(double) 10.0);

    Mb_ = nh.param("control_constants/Mb",(double) 10.0);
    Mw_ = nh.param("control_constants/Mw",(double) 10.0);
    ixx_ = nh.param("control_constants/ixx",(double) 10.0);
    iyy_ = nh.param("control_constants/iyy",(double) 10.0);
    izz_ = nh.param("control_constants/izz",(double) 10.0);
    cz_ = nh.param("control_constants/cz",(double) 10.0);
    R_ = nh.param("control_constants/Rw",(double) 10.0);
    Ks1_ = nh.param("control_constants/Ks1",(double) 10.0);
    Ks2_ = nh.param("control_constants/Ks2",(double) 10.0);
    gy_ = nh.param("control_constants/gy",(double) 10.0);
    update_time_ = 1/update_rate_;

    //Subscriber:
    imu_sub_ = nh.subscribe("/minicarcy/sensor/imu", 5, &ControlNode::imuCallback,this);
    joints_sub_ = nh.subscribe("/minicarcy/joints/joint_states", 5, &ControlNode::jointsCallback,this);

    //Publisher:
    if(motors_controller_type_=="PositionJointInterface")
      motors_controller_type_="position";
    if(motors_controller_type_=="VelocityJointInterface")
      motors_controller_type_="velocity";
    if(motors_controller_type_=="EffortJointInterface")
      motors_controller_type_="effort";
    joint_commands_1_pub_ = nh.advertise<std_msgs::Float64>("/minicarcy/joints/rwheel_"+motors_controller_type_+"_controller/command", 5);
    joint_commands_2_pub_ = nh.advertise<std_msgs::Float64>("/minicarcy/joints/lwheel_"+motors_controller_type_+"_controller/command", 5);

    //Publish rpy angles:
    rpy_pub_ =  nh.advertise<geometry_msgs::PointStamped>("rpy_angles", 5);
    calc_minicarcy_odom_pub_ = nh.advertise<geometry_msgs::PointStamped>("minicarcy", 5);
    //Publish desired torques:
    desired_velocity_pub_ = nh.advertise<geometry_msgs::Vector3>("desired_velocity", 5);

    //action server:

    // init vectors:
    imu_phi_.push_back(0.0);
    imu_phi_.push_back(0.0);
    imu_phi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    realV_.push_back(0.0);
    realV_.push_back(0.0);

    // for control:
    t_=0;
    theta_=0;
    imu_phi_last_ = {0,0,0};
    positionxyz={0,0};
    positionxyz_last_.push_back(0.0);
    positionxyz_last_.push_back(0.0);
    joints_velocity_.push_back(0.0);
    joints_velocity_.push_back(0.0);
  }

  // ROS API Callbacks: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
  // runs with 200Hz actually new messages come in every 6ms.
  void ControlNode::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
  {
    previous_imu_msg_ = *imu_msg;
    imu_phi_ = toEulerAngle(previous_imu_msg_.orientation.x, previous_imu_msg_.orientation.y, previous_imu_msg_.orientation.z, previous_imu_msg_.orientation.w); //rad

    // TODO: z-Achse müsste auch noch gedreht werden!
    imu_dphi_.at(0)=previous_imu_msg_.angular_velocity.x; // rad/sec
    imu_dphi_.at(1)=previous_imu_msg_.angular_velocity.y;
    imu_dphi_.at(2)=previous_imu_msg_.angular_velocity.z;
    balancing_time_ =previous_imu_msg_.header.stamp.toSec();
  }

  void ControlNode::jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
  {
    previous_joint_state_msg_ = *joint_state_msg;
    joints_velocity_.at(0) = previous_joint_state_msg_.velocity[0]; // wheel1
    joints_velocity_.at(1) = previous_joint_state_msg_.velocity[1]; // wheel2

	//ROS_INFO("%f ",previous_joint_state_msg_.velocity[2]);
  }

  //update (publish messages...) every 10ms. This is only updated every 100 ms! but why?!
  void ControlNode::update()
  {
    calc2MotorCommands_withOdometry();

    // note that the wheel torque is limited to ca. 4.1Nm.
    std_msgs::Float64 realVR;
    std_msgs::Float64 realVL;


    realVR.data=realV_.at(0);
    realVL.data=realV_.at(1);

    joint_commands_1_pub_.publish(realVR);
    joint_commands_2_pub_.publish(realVL);

    // publish desired Torques:
    desired_velocity_.x =realV_.at(0);
    desired_velocity_.y =realV_.at(1);
    desired_velocity_pub_.publish(desired_velocity_);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::PointStamped rpy;
    rpy.header.stamp = ros::Time::now();
    rpy.header.frame_id = "rpy_grad_angles";
    rpy.point.x = imu_phi_.at(0)*180/3.14159265359;
    rpy.point.y = imu_phi_.at(1)*180/3.14159265359;
    rpy.point.z = imu_phi_.at(2)*180/3.14159265359;
    rpy_pub_.publish(rpy);
    if(abs(rpy.point.x)>10 || abs(rpy.point.y)>10)
    {
      ROS_WARN_ONCE("x || y = higher than 10° Grad!"); // shows the simulation time!
    }
  }

}  // end namespace ballbot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "minicarcy_control");
  ROS_INFO("yooooooooooooooostart minicarcy_control_node:");

  ros::NodeHandle nh;

  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  minicarcy::ControlNode node(nh);
  ros::Rate loop_rate(nh.param("/minicarcy/control_constants/update_rate", 100.0));

  // TODO wait here to start publishing nodes information
  // until first non zero value from gazebo is received!
  while (ros::ok())
  {
    node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
