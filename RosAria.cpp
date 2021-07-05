#include <stdio.h>
#include <math.h>
#include <Aria.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "geometry_msgs/msg/transformstamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "LaserPublisher.h"
#include "rosaria/msg/bumper_state.hpp"

//tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/point_cloud_conversion.hpp>






//#include "nav_msgs/msg/odometry.hpp"
#include <sstream>

using namespace std::chrono_literals;
/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library. 

    RosAriaNode will use ARIA to connect to a robot controller (configure via
    ~port parameter), either direct serial connection or over the network.  It 
    runs ARIA's robot communications cycle in a background thread, and
    as part of that cycle (a sensor interpretation task which calls RosAriaNode::publish()),
    it  publishes various topics with newly received robot
    data.  It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.

    For more information about ARIA see
    http://robots.mobilerobots.com/wiki/Aria.

    RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
    information, tutorials and documentation.
*/

//this->get_parameter("my_parameter", parameter_string_);
//https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html

class RosAriaNode : public rclcpp::Node
{
    public:
    RosAriaNode() : Node("RosAria"), myPublishCB(this, &RosAriaNode::publish)
    {
    Aria::init();

    TicksMM = -1;
    DriftFactor= -99999;
    RevCount = -1;

    timer_ = this->create_wall_timer(1ms, std::bind(&RosAriaNode::publish, this));

    conf_TicksMM = 0;
    conf_DriftFactor = -99999;
    conf_RevCount = 0;

    sonar_enabled = false;
    serial_baud = 0;
    debug_aria = false;
    publish_aria_lasers = false;
    laserConnector = NULL;
    robot = NULL;
    conn = NULL;
    serial_port = "/dev/ttyS0";
    aria_log_filename = "Aria.log";

    veltime = this->now();
    sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("sonar", 50);
      //std::bind(&RosAriaNode::sonarConnectCb, this),  NOTE thht SubsriberStatusCallback is not yet supported in ROS2, FIND SOLUTION
      //std::bind(&RosAriaNode::sonarConnectCb, this));
    sonar_pointcloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_pointcloud2", 50);
      //std::bind(&RosAriaNode::sonarConnectCb, this),
      //std::bind(&RosAriaNode::sonarConnectCb, this));


    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("pose", 1000);
    bumpers_pub = this->create_publisher<rosaria::msg::BumperState>("bumper", 1000);
    voltage_pub = this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1000);
    state_of_charge_pub = this->create_publisher<std_msgs::msg::Float32>("battery_state_of_charge", 100);
    recharge_state_pub = this->create_publisher<std_msgs::msg::Int8>("battery_recharge_state", 5);
    motors_state_pub = this->create_publisher<std_msgs::msg::Bool>("motors_state", 5);
    publish_sonar = true; 
    publish_sonar_pointcloud2 = true;
    Setup();
    }

    virtual ~RosAriaNode();


    public:
    




    /////////////////////////////////need tp be arranged////////////////////////////////////
    void dynamic_reconfigureCB(uint32_t level);
    void cmdvel_watchdog();
    void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    //void sonarConnectCb();
    int Setup();
    void readParameters();
    void publish();

    protected:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr sonar_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pointcloud2_pub;
    rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

    bool publish_sonar; 
    bool publish_sonar_pointcloud2;

    bool sonar_enabled; 
    int serial_baud;
    bool debug_aria;
    bool publish_aria_lasers;

    std::string serial_port;
    std::string aria_log_filename;

    rosaria::msg::BumperState bumpers;

    int TicksMM, DriftFactor, RevCount;
    double trans_accel, trans_decel, lat_accel, lat_decel;
    double rot_accel, rot_decel;
    int conf_TicksMM, conf_DriftFactor, conf_RevCount;
    rclcpp::Time veltime;
    //rclcpp::Duration cmdvel_timeout;

    ArRobot *robot;
    ArLaserConnector *laserConnector;
    ArRobotConnector *conn;

    ArFunctorC<RosAriaNode> myPublishCB;

    bool enable_motors_cb(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response);
    bool disable_motors_cb(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response);

	
//publish() specific variables
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
	rclcpp::Publisher<rosaria::msg::BumperState>::SharedPtr bumpers_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub;
	
	std_msgs::msg::Int8 recharge_state;
	std_msgs::msg::Bool motors_state;
	bool published_motors_state = false;

	ArPose pos;
	nav_msgs::msg::Odometry position;

	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	std::string frame_id_odom = "odom";
	std::string frame_id_base_link = "base_link";
	std::string frame_id_bumper = "bumpers";
	std::string frame_id_sonar = "sonar";
	
};

//tested
RosAriaNode::~RosAriaNode()
    {
    // disable motors and sonar.
    robot->disableMotors();
    robot->disableSonar();

    robot->stopRunning();
    robot->waitForRunExit();
    Aria::shutdown();
    }

//tested
void RosAriaNode::dynamic_reconfigureCB(uint32_t level)
{
  robot->lock();
  if(TicksMM != conf_TicksMM && conf_TicksMM > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, conf_TicksMM);
    TicksMM = conf_TicksMM;
    robot->comInt(93, TicksMM);
  }

  if(DriftFactor != conf_DriftFactor && conf_DriftFactor != -99999)
  {
    RCLCPP_INFO(this->get_logger(), "Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, conf_DriftFactor);
    DriftFactor = conf_DriftFactor;
    robot->comInt(89, DriftFactor);
  }

  if(RevCount != conf_RevCount && conf_RevCount > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, conf_RevCount);
    RevCount = conf_RevCount;
    robot->comInt(88, RevCount);
  }

  //
  // Acceleration Parameters
  //
  int value;
  value = trans_accel * 1000;
  if(value != robot->getTransAccel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting TransAccel from Dynamic Reconfigure: %d", value);
    robot->setTransAccel(value);
  }

  value = trans_decel * 1000;
  if(value != robot->getTransDecel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting TransDecel from Dynamic Reconfigure: %d", value);
    robot->setTransDecel(value);
  }

  value = lat_accel * 1000;
  if(value != robot->getLatAccel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting LatAccel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatAccel() > 0 )
      robot->setLatAccel(value);
  }

  value = lat_decel * 1000;
  if(value != robot->getLatDecel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting LatDecel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatDecel() > 0 )
      robot->setLatDecel(value);
  }

  value = rot_accel * 180/M_PI;
  if(value != robot->getRotAccel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting RotAccel from Dynamic Reconfigure: %d", value);
    robot->setRotAccel(value);
  }

  value = rot_decel * 180/M_PI;
  if(value != robot->getRotDecel() && value > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting RotDecel from Dynamic Reconfigure: %d", value);
    robot->setRotDecel(value);
  }
  robot->unlock();
}

//tested
bool RosAriaNode::enable_motors_cb(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response)
{
    RCLCPP_INFO(this->get_logger(), "RosAria: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        RCLCPP_WARN_ONCE(this->get_logger(), "RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");

    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

//tested
bool RosAriaNode::disable_motors_cb(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response)
{
    RCLCPP_INFO(this->get_logger(), "RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void RosAriaNode::cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  veltime = this->now();
  RCLCPP_INFO(this->get_logger(),"new speed: [%0.2f,%0.2f](%0.3f)",msg->linear.x*1e3,msg->angular.z,veltime.nanoseconds()/1e9);

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  RCLCPP_INFO(this->get_logger(),"RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.nanoseconds()/1e9,
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1e3, (double) msg->angular.z * 180/M_PI);
}

void RosAriaNode::cmdvel_watchdog()
{
  // stop robot if no cmd_vel message was received for 0.6 seconds
  if (this->now() - veltime > rclcpp::Duration(0.6,0))
  {
    robot->lock();
    robot->setVel(0.0);
    if(robot->hasLatVel())
      robot->setLatVel(0.0);
    robot->setRotVel(0.0);
    robot->unlock();
  }
}

// void RosAriaNode::sonarConnectCb()
// {
//   publish_sonar = (sonar_pub->get_subscription_count() > 0);
//   publish_sonar_pointcloud2 = (sonar_pointcloud2_pub->get_subscription_count() > 0);
//   robot->lock();
//   if (publish_sonar || publish_sonar_pointcloud2)
//   {
//     robot->enableSonar();
//     sonar_enabled = false;
//   }
//   else if(!publish_sonar && !publish_sonar_pointcloud2)
//   {
//     robot->disableSonar();
//     sonar_enabled = true;
//   }
//   robot->unlock();
// }

int RosAriaNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort %s", serial_port.c_str()); // pass robot's serial port to Aria
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }


  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

  if(publish_aria_lasers)
    laserConnector = new ArLaserConnector(argparser, robot, conn);

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  //readParameters();
  // not needed for now. ALways uses default parameter

  // Setup Parameter Minimums and maximums
  // rosaria::RosAriaConfig dynConf_min;
  // rosaria::RosAriaConfig dynConf_max;
  
  // dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  // dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  // dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  // dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  // dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

  // dynConf_min.trans_accel = 0;
  // dynConf_min.trans_decel = 0;
  // dynConf_min.lat_accel = 0;
  // dynConf_min.lat_decel = 0;
  // dynConf_min.rot_accel = 0;
  // dynConf_min.rot_decel = 0;
  
  // dynConf_min.TicksMM     = 0;
  // dynConf_max.TicksMM     = 200;
  // dynConf_min.DriftFactor = -99999;
  // dynConf_max.DriftFactor = 32767;
  // dynConf_min.RevCount    = 0;
  // dynConf_max.RevCount    = 65535;

  // dynamic_reconfigure_server->setConfigMax(dynConf_max);
  // dynamic_reconfigure_server->setConfigMin(dynConf_min);
  
  
  // rosaria::RosAriaConfig dynConf_default;
  trans_accel = robot->getTransAccel() / 1000;
  trans_decel = robot->getTransDecel() / 1000;
  lat_accel   = robot->getLatAccel() / 1000;
  lat_decel   = robot->getLatDecel() / 1000;
  rot_accel   = robot->getRotAccel() * M_PI/180;
  rot_decel   = robot->getRotDecel() * M_PI/180;




  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);

  // connect to lasers and create publishers
  if(publish_aria_lasers)
  {
    RCLCPP_INFO(this->get_logger(), "rosaria: Connecting to laser(s) configured in ARIA parameter file(s)...");
    if (!laserConnector->connectLasers())
    {
      RCLCPP_INFO(this->get_logger(), "rosaria: Error connecting to laser(s)...");
      return 1;
    }

    robot->lock();
    const std::map<int, ArLaser*> *lasers = robot->getLaserMap();
    RCLCPP_INFO(this->get_logger(), "rosaria: there are %lu connected lasers", lasers->size());
    for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
    {
      ArLaser *l = i->second;
      int ln = i->first;
      std::string tfname("laser");
      if(lasers->size() > 1 || ln > 1) // no number if only one laser which is also laser 1
        tfname += ln; 
      tfname += "_frame";
      RCLCPP_INFO(this->get_logger(),"rosaria: Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
      //This should be changed when LaserPublisher is done 
      new LaserPublisher(l, this, true, tfname);
    }
    robot->unlock();
    RCLCPP_INFO(this->get_logger(), "rosaria: Done creating laser publishers");
  }
    
  // subscribe to command topics
  cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", 1, std::bind(&RosAriaNode::cmdvel_cb, this, std::placeholders::_1 ));

  // register a watchdog for cmd_vel timeout
  double cmdvel_timeout_param = 0.6;

  //cmdvel_timeout = rclcpp::Duration(cmdvel_timeout_param,0);
  if (cmdvel_timeout_param > 0.0)
  {
    cmdvel_watchdog_timer = this->create_wall_timer(100ms, std::bind(&RosAriaNode::cmdvel_watchdog, this) );
  }
  RCLCPP_INFO(this->get_logger(), "rosaria: Setup complete");

  robot->lock();
  robot->enableSonar();
//  sonar_enabled = false;
  robot->unlock();

  return 0;
}



void RosAriaNode::readParameters()
{
  // Robot Parameters. If a parameter was given and is nonzero, set it now.
  // Otherwise, get default value for this robot (from getOrigRobotConfig()).
  // Parameter values are stored in member variables for possible later use by the user with dynamic reconfigure.
  robot->lock(); //TODO check all robot pointer functions
  
  //ros::NodeHandle n_("~");
  auto node = rclcpp::Node::make_shared("readParameters");
  
  rclcpp::Parameter TicksMM_param;
  rclcpp::Parameter DriftFactor_param;
  rclcpp::Parameter RevCount_param;
  
  if (node->get_parameter("TicksMM", TicksMM_param) && TicksMM_param.as_int() > 0)
  {
    RCLCPP_INFO(node->get_logger(), "Setting robot TicksMM from ROS Parameter: %d", TicksMM_param.as_int());
    robot->comInt(93, TicksMM_param.as_int());
    TicksMM = TicksMM_param.as_int();
  }
  else
  {
    TicksMM = robot->getOrigRobotConfig()->getTicksMM(); 
    RCLCPP_INFO(node->get_logger(), "This robot's TicksMM parameter: %d", TicksMM);
    //n_.setParam( "TicksMM", TicksMM);
  }
  
  if (node->get_parameter("DriftFactor", DriftFactor_param) && DriftFactor_param.as_int() != -99999)
  {
    RCLCPP_INFO(node->get_logger(), "Setting robot DriftFactor from ROS Parameter: %d", DriftFactor_param.as_int());
    robot->comInt(89, DriftFactor_param.as_int());
    DriftFactor = DriftFactor_param.as_int();
  }
  else
  {
    DriftFactor = robot->getOrigRobotConfig()->getDriftFactor(); 
    RCLCPP_INFO(node->get_logger(), "This robot's DriftFactor parameter: %d", DriftFactor);
    //n_.setParam( "DriftFactor", DriftFactor);
  }
  
  if (node->get_parameter("RevCount", RevCount_param) && RevCount_param.as_int() > 0)
  {
    RCLCPP_INFO(node->get_logger(), "Setting robot RevCount from ROS Parameter: %d", RevCount_param.as_int());
    robot->comInt(88, RevCount_param.as_int());
    RevCount = RevCount_param.as_int();
  }
  else
  {
    RevCount = robot->getOrigRobotConfig()->getRevCount(); 
    RCLCPP_INFO(node->get_logger(), "This robot's RevCount parameter: %d", RevCount);
    //n_.setParam( "RevCount", RevCount);
  }
  
  robot->unlock();
}

void RosAriaNode::publish()
{

  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  
  tf2::Quaternion q;
  q.setRPY(0,0,pos.getTh()*M_PI/180);  
  //tf2::convert(tf2::Transform(q, tf2::Vector3(pos.getX()/1000, pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  //TODO, line 582 gives error  
  position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = this->now();
  pose_pub->publish(position);

  RCLCPP_DEBUG(this->get_logger(), "RosAria: publish: (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
    position.header.stamp, //was toSec() earlier
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double)position.twist.twist.linear.x,
    (double)position.twist.twist.linear.y,
    (double)position.twist.twist.angular.z
  );

  // publishing transform odom->base_link
  geometry_msgs::msg::TransformStamped odom_trans;

  odom_trans.header.stamp = this->now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  
  tf2::Quaternion x;
  x.setRPY(0,0,pos.getTh()*M_PI/180);
  odom_trans.transform.rotation = tf2::toMsg(x);

  odom_broadcaster->sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = this->now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  RCLCPP_DEBUG(this->get_logger(), "RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  RCLCPP_DEBUG(this->get_logger(),"RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  

  bumpers_pub->publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::msg::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub->publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::msg::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub->publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    RCLCPP_INFO(this->get_logger(), "RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub->publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	RCLCPP_INFO(this->get_logger(), "RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub->publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    sensor_msgs::msg::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
  

    std::stringstream sonar_debug_info; // Log debugging info
    sonar_debug_info << "Sonar readings: ";

    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        RCLCPP_WARN(this->get_logger(), "RosAria: Did not receive a sonar reading.");
        continue;
      }
    
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
    
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::msg::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), sonar_debug_info.str());
    
    // publish topic(s)

    if(publish_sonar_pointcloud2)
    {
      sensor_msgs::msg::PointCloud2 cloud2;
      
      if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2)) //TODO check if conversion accessible in ros2
      {
        RCLCPP_WARN(this->get_logger(), "Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
      }
      else
      {
        sonar_pointcloud2_pub->publish(cloud2);
      }
    }//RCLCPP_WARN(this->get_logger(), "Conversion from PointCloud to PointCloud2 not available. Not publishing this time.");
    //TODO fix if function above, commented out because of sensor_msgs::msg::convertPointCloudToPointCloud2(cloud, cloud2)

    if(publish_sonar)
    {
      sonar_pub->publish(cloud);
    }
  } // end if sonar_enabled

}






int main( int argc, char** argv )
{

   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<RosAriaNode>());
   rclcpp::shutdown();
   return 0;

}

