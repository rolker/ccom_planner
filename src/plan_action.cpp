#include <ccom_planner/plan_action.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <project11_nav_msgs/msg/robot_state.hpp>
#include <tf2_ros/buffer.h>
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/context.h>
#include <ccom_planner/dubins_astar.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ccom_planner::PlanAction>("CCOMPlanner");
}

namespace ccom_planner
{

PlanAction::PlanAction(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList PlanAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose", "{start_pose}", "Robot starting position and orientation"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "{goal_pose}", "Robot goal position and orientation"),
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_path", "{navigation_path}", "Planned path to follow"),
    BT::InputPort<double>("turn_radius", "{robot_turn_radius}", ""),
    BT::InputPort<double>("speed", "{robot_default_speed}", ""),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer", "{tf_buffer}", "Transform buffer"),
    BT::InputPort<std::shared_ptr<project11_navigation::Context> >("context", "{context}", "Navigation context")
  };
}

BT::NodeStatus PlanAction::onStart()
{
  auto start_pose = getInput<geometry_msgs::msg::PoseStamped>("start_pose");
  if(!start_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [start_pose]: ", start_pose.error() );    
  }

  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [goal_pose]: ", goal_pose.error() );    
  }

  auto radius = getInput<double>("turn_radius");
  if(!radius)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [turn_radius]: ", radius.error() );    
  }

  auto speed = getInput<double>("speed");
  if(!radius)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [speed]: ", speed.error() );    
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  auto context = getInput<std::shared_ptr<project11_navigation::Context> >("context");
  if(!context)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [context]: ", context.error() );
  }

  auto turn_radius = radius.value();

  auto start = start_pose.value(); 
  auto goal = goal_pose.value();

  std::string map_frame = context.value()->environment().mapFrame();

  if(start.header.frame_id != map_frame)
  {
    try
    {
      context.value()->tfBuffer()->transform(start, start, map_frame);
      start.header.frame_id = map_frame;
    }
    catch(const std::exception& e)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Error transforming start_pose to map frame. " << e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  if(goal.header.frame_id != map_frame)
  {
    try
    {
      context.value()->tfBuffer()->transform(goal, goal, map_frame);
      goal.header.frame_id = map_frame;
    }
    catch(const std::exception& e)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Error transforming goal_pose to map frame. " << e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  project11_nav_msgs::msg::RobotState start_state, goal_state;
  start_state.pose = start.pose;
  goal_state.pose = goal.pose;

  planner_ = std::make_shared<DubinsAStar>(start_state, goal_state, context.value(), turn_radius, speed.value());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanAction::onRunning()
{
  auto start_pose = getInput<geometry_msgs::msg::PoseStamped>("start_pose");
  if(!start_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [start_pose]: ", start_pose.error() );    
  }


  auto context = getInput<std::shared_ptr<project11_navigation::Context> >("context");
  if(!context)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [context]: ", context.error() );
  }

  std::string map_frame = context.value()->environment().mapFrame();

  std_msgs::msg::Header start_header;
  start_header.frame_id = map_frame;
  start_header.stamp = start_pose.value().header.stamp;

  std::vector<geometry_msgs::msg::PoseStamped> potential_plan;
  if(planner_->getPlan(potential_plan, start_header))
  {
    setOutput("navigation_path", std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >(potential_plan));
    return BT::NodeStatus::SUCCESS;  
  }
  else
      setOutput("potential_path", std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >(potential_plan));

  return BT::NodeStatus::RUNNING;
}

void PlanAction::onHalted()
{
  planner_.reset();
}

} // namespace ccom_planner

