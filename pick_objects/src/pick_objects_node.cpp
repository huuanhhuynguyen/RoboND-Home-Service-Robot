#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

double goal[2] = {0, 0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void get_goal(const visualization_msgs::Marker::ConstPtr& msg)
{
  goal[0] = msg->pose.position.x;
  goal[1] = msg->pose.position.y;
  ROS_INFO("Update goal %f %f", goal[0], goal[1]);
}

void send_goal_to_move_base(MoveBaseClient & ac, double x, double y, double orient)
{
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orient;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal ...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, reached the goal!");
  else
    ROS_INFO("The base failed to reach the goal for some reason.");
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  ros::Subscriber marker_sub = n.subscribe("visualization_marker", 10, get_goal);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spinOnce();

  send_goal_to_move_base(ac, goal[0], goal[1], 1.0);

  sleep(3); // simulate the picking up

  ros::spinOnce();

  send_goal_to_move_base(ac, goal[0], goal[1], 1.0);

  return 0;
}