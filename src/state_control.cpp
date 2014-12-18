// Regular Includes
#include <math.h>
#include <iostream>

// ROS Related Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

// Custom Includes
#include <controllers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

// Local Includes
// #include "state_control.h"
#include "nano_kontrol2.h"
#include "trajectory.h"

using namespace std;

#define RED "\e[91m"
#define GREEN "\e[92m"
#define YELLOW "\e[93m"
#define BLUE "\e[94m"
#define MAGENTA "\e[95m"
#define CYAN "\e[96m"
#define RESET "\e[0m"

// State machine
enum controller_state
{
  ESTOP,
  INIT,
  TAKEOFF,
  HOVER,
  HOME,
  LINE_TRACKER,
  LINE_TRACKER_DISTANCE,
  LINE_TRACKER_YAW,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  LAND,
  PREP_TRAJ,
  TRAJ,
  NONE,
};
static enum controller_state state_ = INIT;

// Variables and parameters
double xoff, yoff, zoff, yaw_off;

// =======================
// Stuff for trajectory
// =======================
traj_type traj;
ros::Time traj_start_time;
double traj_time;
static ros::Publisher pub_goal_trajectory_;
quadrotor_msgs::PositionCommand traj_goal;
static const std::string null_tracker_str("null_tracker/NullTracker");
void updateTrajGoal();
static std::string traj_filename;
static int traj_num_ = 0;
static bool play_button_pressed = false;
// =======================

// Publishers & services
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_traj_signal_;
static ros::Publisher pub_so3_command_;
static ros::Publisher pub_traj_num_;
static ros::ServiceClient srv_transition_;

// Quadrotor Pose
static geometry_msgs::Point goal, pos_, home_;
static geometry_msgs::Vector3 vel_;
static tf::Quaternion imu_q_, odom_q_;
static bool have_odom_(false), imu_info_(false), motors_on_(false);

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker("line_tracker/LineTrackerMinJerk");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");

// Function Prototypes
void hover_in_place();
void go_to(const quadrotor_msgs::FlatOutputs &goal);
void go_to(const geometry_msgs::Point &goal);
double norm(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
void motors_on(const bool flag);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    motors_on(false);

    state_ = ESTOP;
  }

  play_button_pressed = msg->buttons[play_button];

  if (state_ == ESTOP)
    return;

  if(state_ == INIT)
  {
    if (!have_odom_)
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }

    // Motors on (Rec)
    if(msg->buttons[motors_on_button])
      motors_on(true);

    // Take off (Play)
    if(msg->buttons[play_button])
    {
      if (!motors_on_)
        ROS_INFO("You must turn on the motors first...");
      else
      {
        state_ = TAKEOFF;
        ROS_INFO("Initiating launch sequence...");

        home_ = pos_;
        goal = pos_;
        goal.z += 0.1;

        go_to(goal);
      }
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to turn on the motors and Play to Take off.");
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
    else if(msg->buttons[7])
    {
      state_ = HOME;
      go_to(home_);
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
      if (flag == 0)
      {
        state_ = PREP_TRAJ;
        ROS_INFO("Successfully loaded trajectory: %s.", traj_filename.c_str());

        quadrotor_msgs::FlatOutputs goal;
        goal.x = traj[0][0][0] + xoff;
        goal.y = traj[0][1][0] + yoff;
        goal.z = traj[0][2][0] + zoff;
        goal.yaw = traj[0][3][0] + yaw_off;

        go_to(goal);
      }
      else
      {
        hover_in_place();
        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...", traj_filename.c_str(), flag);
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

    // Publish the trajectory signal
    std_msgs::Bool traj_on_signal;
    traj_on_signal.data = false;
    pub_traj_signal_.publish(traj_on_signal);

    // Get ready to run the next trajectory
    state_ = PREP_TRAJ;

    quadrotor_msgs::FlatOutputs goal;
    goal.x = traj[0][0][0] + xoff;
    goal.y = traj[0][1][0] + yoff;
    goal.z = traj[0][2][0] + zoff;
    goal.yaw = traj[0][3][0] + yaw_off;

    go_to(goal);
  }
  else
  {
    traj_goal.header.stamp = current_time;
    // traj_goal.header.frame_id = ????

    traj_goal.position.x = traj[i][0][0] + xoff;
    traj_goal.position.y = traj[i][1][0] + yoff;
    traj_goal.position.z = traj[i][2][0] + zoff;

    traj_goal.velocity.x = traj[i][0][1];
    traj_goal.velocity.y = traj[i][1][1];
    traj_goal.velocity.z = traj[i][2][1];

    traj_goal.acceleration.x = traj[i][0][2];
    traj_goal.acceleration.y = traj[i][1][2];
    traj_goal.acceleration.z = traj[i][2][2];

    traj_goal.jerk.x = traj[i][0][3];
    traj_goal.jerk.y = traj[i][1][3];
    traj_goal.jerk.z = traj[i][2][3];

    traj_goal.yaw = traj[i][3][0] + yaw_off;
    traj_goal.yaw_dot = traj[i][3][1];

    traj_goal.kx[0] = traj[i][4][0];
    traj_goal.kx[1] = traj[i][4][1];
    traj_goal.kx[2] = traj[i][4][2];
    traj_goal.kv[0] = traj[i][4][3];
    traj_goal.kv[1] = traj[i][4][4];
    traj_goal.kv[2] = traj[i][4][5];
  }
}

static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  geometry_msgs::Quaternion q;
  q = msg->orientation;
  tf::quaternionMsgToTF(q, imu_q_);

  imu_info_ = true;
}

void hover_in_place()
{
  state_ = HOVER;
  ROS_INFO("Hovering in place...");

  // Publish the trajectory signal
  std_msgs::Bool traj_on_signal;
  traj_on_signal.data = false;
  pub_traj_signal_.publish(traj_on_signal);

  goal.x = pos_.x;
  goal.y = pos_.y;
  goal.z = pos_.z;
  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
}

void go_to(const quadrotor_msgs::FlatOutputs &goal)
{
  pub_goal_yaw_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_yaw;
  srv_transition_.call(transition_cmd);
}

void go_to(const geometry_msgs::Point &goal)
{
  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;

  geometry_msgs::Quaternion odom_q;
  odom_q = msg->pose.pose.orientation;
  tf::quaternionMsgToTF(odom_q, odom_q_);

  // If we are currently executing a trajectory, update the setpoint
  if (state_ == TRAJ)
  {
    updateTrajGoal();
    pub_goal_trajectory_.publish(traj_goal);

    std_msgs::Int16 traj_num_msg;
    traj_num_msg.data = traj_num_;
    pub_traj_num_.publish(traj_num_msg);
  }

  // For the safety check
  tf::Quaternion q = odom_q_;
  static double roll, pitch, yaw;
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

  if(state_ == PREP_TRAJ)
  {
    // Updates traj goal to allow for correct initalization of the trajectory
    traj_start_time = ros::Time::now();
    updateTrajGoal();

    // If we are ready to start the trajectory
    if (sqrt( pow(traj_goal.position.x - pos_.x, 2)
             + pow(traj_goal.position.y - pos_.y, 2)
             + pow(traj_goal.position.z - pos_.z, 2) ) < 0.1 &&
         sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) < 0.1)
    {
      state_ = TRAJ;
      traj_num_++;

      std::cout << GREEN << "Starting trajectory " << traj_num_ << RESET << std::endl;

      // Publish the trajectory signal
      std_msgs::Bool traj_on_signal;
      traj_on_signal.data = true;
      pub_traj_signal_.publish(traj_on_signal);

      traj_start_time = ros::Time::now();

      updateTrajGoal();

      pub_goal_trajectory_.publish(traj_goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = null_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    else
    {
      ROS_WARN_THROTTLE(2, "Not ready to start trajectory.");
      // cout << "rdes - r = {"
      //      << traj_goal.position.x + xoff - pos_.x
      //      << ", " << traj_goal.position.y + yoff - pos_.y
      //      << ", " << traj_goal.position.z + zoff - pos_.z
      //      << "} " <<
      //    sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) << endl;
    }
  }

  if (state_ == HOME && (norm(pos_, home_) < 0.1 || play_button_pressed))
  {
    motors_on(false);
    cout << RED << "Stopping motors..." << RESET << endl;
    state_ = INIT;
  }
  else if (state_ == HOME)
    ROS_INFO_THROTTLE(1, "Going home...");
}

double norm(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  return std::sqrt(
      std::pow(a.x - b.x, 2)
    + std::pow(a.y - b.y, 2)
    + std::pow(a.z - b.z, 2));
}

void motors_on(const bool flag)
{
  motors_on_ = flag;

  std::string message = flag ? "Turning motors on..." : "Turning motors off...";
  cout << YELLOW << message.c_str() << RESET << endl;
  std_msgs::Bool motors_cmd;
  motors_cmd.data = flag;
  pub_motors_.publish(motors_cmd);

  // Switch to null_tracker so that the trackers do not publish so3_commands
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);

  // Create and publish the so3_command
  quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
  cmd->aux.enable_motors = flag;
  pub_so3_command_.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n;

  // Position offsets for this robot
  n.param("offsets/x", xoff, 0.0);
  n.param("offsets/y", yoff, 0.0);
  n.param("offsets/z", zoff, 0.0);
  n.param("offsets/yaw", yaw_off, 0.0);
  ROS_INFO("Using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  n.param("state_control/traj_filename", traj_filename, string(""));

  // Publishers
  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_min_jerk/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/velocity_tracker/vel_cmd", 1);
  pub_goal_yaw_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/line_tracker_yaw/goal", 1);
  pub_traj_signal_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  pub_traj_num_ = n.advertise<std_msgs::Int16>("traj_num", 1);
  pub_so3_command_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("quad_decode_msg/imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  // Trajectory publisher
  pub_goal_trajectory_ = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 1);

  motors_on(false);

  ros::spin();

  return 0;
}
