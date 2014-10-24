#include <ros/ros.h>
#include <controllers_manager/Transition.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <iostream>
#include <quadrotor_msgs/PositionCommand.h>
#include <trajectory.h>
using namespace std;

#define SAFETY_ON

enum controller_state
{
  INIT,
  TAKEOFF,
  HOVER,
  HOME,
  LINE_TRACKER,
  LINE_TRACKER_YAW,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  LAND,
  PREP_TRAJ,
  TRAJ,
  NONE,
};

// Buttons
static int traj_button = 34; // Cycle
static int estop_button = 26;  // Stop
static int play_button = 27;  // Play
static int motors_on_button = 28;  // Rec
static int line_tracker_button = 29;  // Track L
static int velocity_tracker_button = 30; // Track R
static int hover_button = 31; // Marker Set
static int line_tracker_yaw_button = 24; // Rewind 

// Variables and parameters
double xoff, yoff, zoff, yaw_off;
geometry_msgs::Point goal;

// Stuff for trajectory
#include <string>
traj_type traj;
ros::Time traj_start_time; 
double traj_time;
static ros::Publisher pub_goal_trajectory_;
quadrotor_msgs::PositionCommand traj_goal;
static const std::string trajectory_tracker_str("trajectory_tracker/TrajectoryTracker");
void updateTrajGoal();
static std::string traj_filename;

// States
static enum controller_state state_ = INIT;

// Publishers & services
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_info_bool_;
static ros::ServiceClient srv_transition_;

// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Vector3 vel_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_ = false;
static bool imu_info_ = false;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker("line_tracker/LineTracker");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");

// Function Declarations
void hover_in_place();
void go_to(const quadrotor_msgs::FlatOutputs goal);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    // Disable motors
    ROS_WARN("Disarming motors...");
    std_msgs::Bool motors_cmd;
    motors_cmd.data = false;
    pub_motors_.publish(motors_cmd);
  }
  
  if(state_ == INIT) 
  {
    if (!have_odom_)
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }
  
    // Motors on (Rec) 
    if(msg->buttons[motors_on_button])
    {
      ROS_INFO("Sending enable motors command");
      std_msgs::Bool motors_cmd;
      motors_cmd.data = true;
      pub_motors_.publish(motors_cmd);
    }
    
    // Take off (Play)
    if(msg->buttons[play_button])
    {
      state_ = TAKEOFF;
      ROS_INFO("Initiating launch sequence...");

      goal.x = pos_.x;
      goal.y = pos_.y;
      goal.z = pos_.z;
      pub_goal_distance_.publish(goal);
      usleep(100000);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_distance;
      srv_transition_.call(transition_cmd);
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to enable motors and Play to Take off.");
  }
  else
  {
    // This is executed every time the midi controller changes
    switch(state_)
    {    
      case VELOCITY_TRACKER:
        {
          quadrotor_msgs::FlatOutputs vel_goal;
          vel_goal.x = msg->axes[0] * fabs(msg->axes[0]) / 2;
          vel_goal.y = msg->axes[1] * fabs(msg->axes[1]) / 2;
          vel_goal.z = msg->axes[2] * fabs(msg->axes[2]) / 2;
          vel_goal.yaw = msg->axes[3] * fabs(msg->axes[3]) / 2;
         
          pub_goal_velocity_.publish(vel_goal);
          ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", vel_goal.x, vel_goal.y, vel_goal.z, vel_goal.yaw);
        }
        break;
   
      default:
        break;    
    }

    // Hover
    if(msg->buttons[hover_button])  // Marker Set
    {
      hover_in_place(); 
    }
    // Line Tracker
    else if(msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER;
      ROS_INFO("Engaging controller: LINE_TRACKER");
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = msg->axes[2] + .9 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      quadrotor_msgs::FlatOutputs goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      goal.yaw = M_PI * msg->axes[3] + yaw_off;
      go_to(goal);
    }
    // Velocity Tracker
    else if(msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are 
      // already in the velocity tracker controller since it 
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER");

      quadrotor_msgs::FlatOutputs vel_goal;
      vel_goal.x = 0;
      vel_goal.y = 0;
      vel_goal.z = 0;
      vel_goal.yaw = 0;
      pub_goal_velocity_.publish(vel_goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    else if(msg->buttons[traj_button] && state_ == HOVER)
    {
      // traj[t_idx][flat_out][deriv]
      //
      // Load the trajectory
      int flag = loadTraj(traj_filename.c_str(), traj);

      // If there are any errors
      if (flag != 0)
      {
        hover_in_place();
        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...", traj_filename.c_str(), flag);
      }
      else
      {
        state_ = PREP_TRAJ;
        ROS_INFO("Loading Trajectory.  state_ == PREP_TRAJ;");
        
        goal.x = traj[0][0][0] + xoff;
        goal.y = traj[0][1][0] + yoff;
        goal.z = traj[0][2][0] + zoff;

        pub_goal_min_jerk_.publish(goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = line_tracker;
        srv_transition_.call(transition_cmd);
      }
    }
    else if(msg->buttons[play_button] && state_ == PREP_TRAJ)
    {
      // If we are ready to start the trajectory
      if ( sqrt( pow(traj_goal.position.x + xoff - pos_.x, 2)
               + pow(traj_goal.position.y + yoff - pos_.y, 2)
               + pow(traj_goal.position.z + zoff - pos_.z, 2) ) > .03 ||
           sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) > 0.05)
      {
        ROS_INFO("Starting Trajectory");

        state_ = TRAJ;

        // Publish the trajectory signal
        std_msgs::Bool traj_on_signal;
        traj_on_signal.data = true;
        pub_info_bool_.publish(traj_on_signal);
        
        traj_start_time = ros::Time::now();
        
        updateTrajGoal();
                
        pub_goal_trajectory_.publish(traj_goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = trajectory_tracker_str;
        srv_transition_.call(transition_cmd);
      }
      else
      {
        ROS_WARN("Not ready to start trajectory.");
      }
    }
  }
}

void updateTrajGoal()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - traj_start_time;
  traj_time = delta_time.toSec();

  unsigned long i = traj_time * 1000;

  if (i > traj.size()-1)
  {
    ROS_INFO("Trajectory completed.");
    hover_in_place();
  }
  else
  {
    traj_goal.position.x = traj[i][0][0] + xoff;
    traj_goal.position.y = traj[i][1][0] + yoff;
    traj_goal.position.z = traj[i][2][0] + zoff;
    traj_goal.yaw = traj[i][3][0] + yaw_off;
    
    traj_goal.velocity.x = traj[i][0][1];
    traj_goal.velocity.y = traj[i][1][1];
    traj_goal.velocity.z = traj[i][2][1];
    traj_goal.yaw_dot = traj[i][3][1];

    traj_goal.acceleration.x = traj[i][0][2];
    traj_goal.acceleration.y = traj[i][1][2];
    traj_goal.acceleration.z = traj[i][2][2];
    // traj_goal.yaw_ddot = traj[i][3][2];
  }
}

static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_q_ = msg->orientation;
  imu_info_ = true;
}

void hover_in_place()
{
  state_ = HOVER;
  ROS_INFO("Hovering in place...");

  // Publish the trajectory signal
  std_msgs::Bool traj_on_signal;
  traj_on_signal.data = false;
  pub_info_bool_.publish(traj_on_signal);

  goal.x = pos_.x;
  goal.y = pos_.y;
  goal.z = pos_.z;
  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance; 
  srv_transition_.call(transition_cmd);
}

void go_to(const quadrotor_msgs::FlatOutputs goal)
{
  state_ = LINE_TRACKER_YAW;
  ROS_INFO("Engaging controller: LINE_TRACKER_YAW");
  pub_goal_yaw_.publish(goal);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_yaw;
  srv_transition_.call(transition_cmd);
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;
  ori_ = msg->pose.pose.orientation;

  // If we are currently executing a trajectory, update the setpoint
  if (state_ == TRAJ)
  {
    updateTrajGoal();
    pub_goal_trajectory_.publish(traj_goal);
  }

  // For the safety check
  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  static tf::Matrix3x3 R; 
  R.setEulerYPR(0, pitch, roll);
  R.getRotation(q);
  q.normalize();

  #ifdef SAFETY_ON
    // Position and attitude Safety Catch
    if (state_ == VELOCITY_TRACKER && (abs(pos_.x) > 2.2 || abs(pos_.y) > 1.8|| pos_.z > 3.5 || q.getAngle() > M_PI/10)) // || pos_.z < 0.2
    {
      ROS_WARN("Safety Catch initiated from VELOCITY_TRACKER...");
      hover_in_place();
    }
  #endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n;

  // Position offsets for this robot
  n.param("state_control/offsets/x", xoff, 0.0);
  n.param("state_control/offsets/y", yoff, 0.0);
  n.param("state_control/offsets/z", zoff, 0.0);
  n.param("state_control/offsets/yaw", yaw_off, 0.0); 
  ROS_INFO("Using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  n.param("state_control/traj_filename", traj_filename, string("traj.csv"));

  // Publishers
  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/velocity_tracker/vel_cmd", 1);
  pub_goal_yaw_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/line_tracker_yaw/goal", 1);
  pub_info_bool_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("quad_decode_msg/imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  // Trajectory publisher
  pub_goal_trajectory_ = n.advertise<quadrotor_msgs::PositionCommand>("controllers_manager/trajectory_tracker/goal", 1);

  // Disabling the motors to be safe
  ROS_INFO("Disabling motors for launch");
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);
  usleep(100000);

  ros::spin();

  return 0;
}
