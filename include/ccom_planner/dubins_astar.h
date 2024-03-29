#ifndef CCOM_PLANNER_DUBINS_ASTAR_H
#define CCOM_PLANNER_DUBINS_ASTAR_H

#include <memory>
#include <queue>
#include <map>
#include <future>
#include <project11_navigation/context.h>
#include <project11_navigation/environment.h>
#include "project11/utils.h"
#include "project11_nav_msgs/RobotState.h"

extern "C" {
#include "dubins_curves/dubins.h"
};

// loosly based on 
// https://en.wikipedia.org/wiki/A*_search_algorithm

// TODO look at D* lite as alternative
// http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

namespace ccom_planner
{

// Combines a state with a pointer to the previous node
// and cost components (g, h)
struct Node
{
  Node(const project11_nav_msgs::RobotState& s, double heuristic, double distance_so_far = 0.0, double cost_so_far = 0.0, std::shared_ptr<Node> from = std::shared_ptr<Node>()): state(s), cummulative_distance(distance_so_far), h(heuristic), g(cost_so_far), previous_node(from)
  {
  }

  project11_nav_msgs::RobotState state;
  double cummulative_distance; // distance of path from start
  double g; // cost of path from start
  double h; // heuristic estimate of cost to goal

  std::shared_ptr<Node> previous_node;

  // total predicted cost froms start to goal
  double f() const 
  {
    return g+h;
  }

  bool operator< (const Node & other) const
  {
    return f() < other.f();
  }

  bool operator> (const Node & other) const
  {
    return f() > other.f();
  }

  using Ptr = std::shared_ptr<Node>;
};

// Used with priority queue to compare Node's f values
struct NodePointerCompare
{
  bool operator()(const Node::Ptr& lhs, const Node::Ptr& rhs) const
  {
    return *lhs > *rhs;
  }
};

// used to discretize the nodes
struct NodeIndex
{
  // x and y indicies in units of step size
  int xi;
  int yi;

  // how many yaw steps from 0
  int yawi;

  // used for ordering the NodeIndexs to use in a map
  bool operator<(const NodeIndex& other) const
  {
    if(xi == other.xi)
      if(yi == other.yi)
          return yawi < other.yawi;
      else
        return yi < other.yi;
    return xi < other.xi;
  }

  bool operator!=(const NodeIndex& other) const
  {
    return operator<(other) || other<*this;
  }

  bool operator==(const NodeIndex& other) const
  {
    return !operator!=(other);
  }

  bool samePosition(const NodeIndex& other) const
  {
    return xi == other.xi && yi == other.yi;
  }
};

// Converts a chain of Nodes into PoseStampeds
void unwrap(Node::Ptr plan, std::vector<geometry_msgs::PoseStamped> &poses, const std_msgs::Header& start_header);

class DubinsAStar
{
public:
  DubinsAStar(project11_nav_msgs::RobotState start, project11_nav_msgs::RobotState goal, project11_navigation::Context::Ptr context);
  ~DubinsAStar();

  // If planner is done, return true
  // If done and a plan was found, the plan is added to the plan argument.
  // If a plan was not found the plan argument is not updated.
  // If the planner is not done, return false and populate the plan argument
  // with the best candidate so far.
  bool getPlan(std::vector<geometry_msgs::PoseStamped> &plan, const std_msgs::Header& start_header);
private:
  // Execute the search, returning the last node if a plan was found.
  Node::Ptr plan();

  // Turn a state in continuous space to something we can use in a grid
  NodeIndex indexOf(const project11_nav_msgs::RobotState& state) const;

  // Calculates the Dubins path between states. Returns true is succesful.
  bool dubins(const project11_nav_msgs::RobotState & from, const project11_nav_msgs::RobotState & to, DubinsPath & path) const;

  // Estimate the cost between states
  double heuristic(const project11_nav_msgs::RobotState & from, const project11_nav_msgs::RobotState & to) const;

  // Generates potential next states from a given state.
  std::vector<Node::Ptr> generateNeighbors(Node::Ptr from) const;

  // Returns a cost from 0.0 to 1.0 if a state not blocked and on
  // the costmap. Returns -1.0 if blocked or off the map
  double getCost(const project11_nav_msgs::RobotState& to_state, const project11_nav_msgs::RobotState& from_state);

  // Returns true if i is one of the goal indecies
  bool isGoal(const NodeIndex& i) const;

  double step_size_; // meters
  double turn_radius_; // meters
  double yaw_step_; // yaw step size for search

  // Deltas relative to a state for a step in a given direction
  struct SearchStep
  {
    SearchStep(double dyaw, double step_size):delta_yaw(dyaw)
    {
      if(delta_yaw != 0.0)
      {
        double radius_of_curvature = step_size/delta_yaw;
        dx = std::abs(radius_of_curvature*sin(delta_yaw));
        dy = std::copysign(radius_of_curvature*(1.0-cos(delta_yaw)), delta_yaw);
      }
      else
      {
        dx = step_size;
        dy = 0.0;
      }
    }

    double delta_yaw;
    double dx;
    double dy;
  };

  std::vector<SearchStep> search_steps_; // Valid relative directions for searching

  double speed_; // Speed at cost 0;

  project11_navigation::Environment::Snapshot environment_snapshot_;

  // A* nodes that can still be expanded
  std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodePointerCompare> open_set_;
  std::mutex open_set_mutex_;

  // Visited node, to avoid looping and retracing paths
  std::map<NodeIndex, bool> visited_nodes_;

  project11_nav_msgs::RobotState start_;
  std::vector<project11_nav_msgs::RobotState> goals_;
  std::vector<NodeIndex> goal_indexes_;

  std::future<Node::Ptr> plan_ready_;
  Node::Ptr plan_;

  bool abort_flag_=false;
  std::mutex abort_flag_mutex_;
};

}; // namespace ccom_planner

// std::ostream& operator<<(std::ostream&, const ccom_planner::State&);
// std::ostream& operator<<(std::ostream&, const ccom_planner::Node&);

#endif
