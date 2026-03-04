/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include "nav2_costmap_2d/cost_values.hpp"
#include <tf2/utils.h>

inline static bool same_point(
  const geometry_msgs::msg::Point & one,
  const geometry_msgs::msg::Point & two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
: Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
  , goal_active_(false)
  , waiting_for_goal_response_(false)
  , exploration_finished_(false)
  , goal_sent_time_(this->now())
{
  double timeout;
  double min_frontier_size;
  bool auto_start;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("auto_start", true);
  this->declare_parameter<int>("no_frontier_retry", 3);
  this->declare_parameter<std::string>("battery_topic", "/battery_state");
  this->declare_parameter<double>("energy_per_meter", 0.1);
  this->declare_parameter<double>("battery_safety_margin", 0.1);
  this->declare_parameter<double>("battery_penalty_scale", 0.0);


  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("auto_start", auto_start);
  this->get_parameter("no_frontier_retry", no_frontier_retry_);
  this->get_parameter("battery_topic", battery_topic_);
  this->get_parameter("energy_per_meter", energy_per_meter_);
  this->get_parameter("battery_safety_margin", battery_safety_margin_);
  this->get_parameter("battery_penalty_scale", battery_penalty_scale_);

  progress_timeout_ = timeout;
  move_base_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(
    costmap_client_.getCostmap(),
    potential_scale_, gain_scale_,
    orientation_scale_,
    min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "explore/"
      "frontier"
      "s",
      10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "explore/resume", 10,
    std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10,
    std::bind(&Explore::batteryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
        map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_, "Couldn't find transform from %s to %s: %s",
        map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
    [this]() {makePlan();});

  if (auto_start) {
    RCLCPP_INFO(logger_, "Exploration starting automatically.");
    makePlan();
  } else {
    RCLCPP_INFO(logger_, "Exploration waiting for start signal.");
    stop(false);
  }

  // Create services
  start_exploration_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start",
    std::bind(
      &Explore::startExplorationServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  stop_exploration_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/stop",
    std::bind(
      &Explore::stopExplorationServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  return_to_initial_pose_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/return_to_initial_pose",
    std::bind(
      &Explore::returnToInitialPoseServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));
}

Explore::~Explore()
{
  stop();
}

bool Explore::startExplorationServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  resume();
  response->success = true;
  response->message = "Exploration resumed.";
  return true;
}

bool Explore::stopExplorationServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  stop(false);   // Stop, but don't consider exploration finished
  response->success = true;
  response->message = "Exploration stopped.";
  return true;
}

bool Explore::returnToInitialPoseServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  stop(false);   // Stop current exploration first
  returnToInitialPose();
  response->success = true;
  response->message = "Returning to initial pose.";
  return true;
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  latest_battery_state_ = *msg;
}

void Explore::visualizeFrontiers(
  const std::vector<frontier_exploration::Frontier> & frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker> & markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::DELETE;
  m.id = 0; // ID 0 is conventionally used for DELETEALL if no other ID is specified.
  markers.push_back(m);

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 1; // Start ID from 1 to avoid conflict with DELETEALL (ID 0)
  for (auto & frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // The DELETEALL covers old markers, so individual DELETE is no longer strictly needed but kept for completeness
  // In this case, DELETEALL clears everything.
  // The id loop for deleting specific old markers is not required if DELETEALL is used.
  // Original delete loop:
  // m.action = visualization_msgs::msg::Marker::DELETE;
  // for (; id < last_markers_count_; ++id) {
  //   m.id = int(id);
  //   markers.push_back(m);
  // }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (exploration_finished_) {
    return;
  }

  // If a goal is currently being pursued or waiting for response, do not send a new one.
  if (goal_active_ || waiting_for_goal_response_) {
    // Check if the active goal has timed out waiting for progress
    if ((this->now() - last_progress_) > tf2::durationFromSec(progress_timeout_)) {
      RCLCPP_WARN(
        logger_,
        "Active navigation goal has timed out waiting for progress (%f seconds). Assuming stuck and moving on.",
        progress_timeout_);
      if (goal_active_ && navigation_goal_handle_) {
        move_base_client_->async_cancel_goal(navigation_goal_handle_);
      }
      goal_active_ = false;
      waiting_for_goal_response_ = false;
    } else {
      // Check if current goal is still traversable
      nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
      unsigned int mx, my;
      if (costmap->worldToMap(prev_goal_.x, prev_goal_.y, mx, my)) {
        unsigned char cost = costmap->getCost(mx, my);
        if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          RCLCPP_DEBUG(
            logger_,
            "Current goal is still traversable. Deferring new plan generation.");
          return;
        } else {
          RCLCPP_INFO(logger_, "Current goal is blocked (cost: %d). Re-planning.", cost);
          if (goal_active_ && navigation_goal_handle_) {
            move_base_client_->async_cancel_goal(navigation_goal_handle_);
          }
          goal_active_ = false;
          waiting_for_goal_response_ = false;
        }
      } else {
        RCLCPP_DEBUG(
          logger_,
          "A navigation goal is currently active or pending. Deferring new plan generation.");
        return;
      }
    }
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  double robot_yaw = tf2::getYaw(pose.orientation);
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(
    pose.position, robot_yaw, latest_battery_state_,
    initial_pose_.position, energy_per_meter_,
    battery_safety_margin_, battery_penalty_scale_);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  // --- Exploration Completion Check 1: No frontiers found at all ---
  if (frontiers.empty()) {
    if (no_frontier_count_ < no_frontier_retry_) {
      no_frontier_count_++;
      RCLCPP_INFO(
        logger_, "No frontiers found, retry %d/%d", no_frontier_count_,
        no_frontier_retry_);
      return;
    }
    handleExplorationCompletion();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // --- Find non-blacklisted, not too close, and high-quality frontier ---
  auto frontier =
    std::find_if_not(
    frontiers.begin(), frontiers.end(),
    [this, &pose](const frontier_exploration::Frontier & f) {
      // 1. Check if blacklisted
      if (goalOnBlacklist(f.centroid)) {return true;}

      // 2. Check if too close to current position
      double dx = f.centroid.x - pose.position.x;
      double dy = f.centroid.y - pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < 0.75 * 0.75) {
        return true;                                             // Ignore frontiers closer than 0.75m

      }
      // 3. Quality Check: Ensure frontier leads into a significant unexplored area.
      // We check the density of unknown space around the centroid.
      nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
      unsigned int cx, cy;
      if (costmap->worldToMap(f.centroid.x, f.centroid.y, cx, cy)) {
        int unknown_cells = 0;
        int total_cells = 0;
        int range_px = 6;                    // approx 0.3m check radius
        for (int x = (int)cx - range_px; x <= (int)cx + range_px; ++x) {
          for (int y = (int)cy - range_px; y <= (int)cy + range_px; ++y) {
            if (x >= 0 && x < (int)costmap->getSizeInCellsX() &&
            y >= 0 && y < (int)costmap->getSizeInCellsY())
            {
              total_cells++;
              if (costmap->getCharMap()[costmap->getIndex(
                x,
                y)] == nav2_costmap_2d::NO_INFORMATION)
              {
                unknown_cells++;
              }
            }
          }
        }
        // If the immediate area is mostly already explored (e.g. < 20% unknown),
        // it's likely a small hole or noise. Skip it.
        if (total_cells > 0 && (double)unknown_cells / total_cells < 0.20) {
          RCLCPP_DEBUG(
            logger_, "Skipping frontier at (%f, %f) - low unknown density (%f)",
            f.centroid.x, f.centroid.y, (double)unknown_cells / total_cells);
          return true;
        }
      }

      return false;
    });

  // --- Exploration Completion Check 2: All found frontiers are blacklisted ---
  if (frontier == frontiers.end()) {
    if (no_frontier_count_ < no_frontier_retry_) {
      no_frontier_count_++;
      RCLCPP_INFO(
        logger_, "No valid frontiers found, retry %d/%d", no_frontier_count_,
        no_frontier_retry_);
      return;
    }
    handleExplorationCompletion();
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // --- Goal Hysteresis and Local Refinement Logic ---
  if (goal_active_) {
    double dx = target_position.x - prev_goal_.x;
    double dy = target_position.y - prev_goal_.y;
    double dist_to_old_goal = sqrt(dx * dx + dy * dy);

    if (dist_to_old_goal > 3.0) {
      // The new 'best' frontier is far away. Stay committed to the current one
      // to avoid backtracking and erratic behavior.
      RCLCPP_DEBUG(
        logger_,
        "New best frontier is too far (%f m) from current goal. Staying committed.",
        dist_to_old_goal);
      return;
    }

    if (dist_to_old_goal < 0.5) {
      // The correction is too small to justify a Nav2 preemption.
      return;
    }
    RCLCPP_INFO(logger_, "Refining current goal position (correction: %f m).", dist_to_old_goal);
  }

  // We are about to send a goal, reset progress trackers
  prev_goal_ = target_position;
  prev_distance_ = frontier->min_distance;
  last_progress_ = this->now();
  no_frontier_count_ = 0; // Reset counter because we found a valid frontier

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  // Calculate orientation to face the frontier centroid from the robot's current position
  double yaw = atan2(target_position.y - pose.position.y, target_position.x - pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Explore::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Explore::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&Explore::resultCallback, this, _1, target_position);

  waiting_for_goal_response_ = true;
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::handleExplorationCompletion()
{
  if (exploration_finished_) {
    return;
  }
  exploration_finished_ = true;

  RCLCPP_WARN(logger_, "Exploration finished: no more frontiers found or all traversed/tried out.");

  // 1. Halt exploration (cancel current goal, stop timer)
  RCLCPP_INFO(logger_, "Halting exploration due to completion.");
  move_base_client_->async_cancel_all_goals(); // Cancel any active navigation goal
  exploring_timer_->cancel(); // Stop the main planning loop
  goal_active_ = false; // Ensure goal active state is reset

  // 2. Return to original pose, if configured
  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Initiating return to initial pose as exploration finished.");
    returnToInitialPose(); // This function already handles sending the goal
  } else {
    RCLCPP_INFO(logger_, "Robot will remain at current position as return_to_init is false.");
  }
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const NavigationGoalHandle::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
      } else {
        RCLCPP_WARN(logger_, "Failed to return to initial pose: %d", static_cast<int>(result.code));
      }
    };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point & goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D * costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto & frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
      y_diff < tolerace * costmap2d->getResolution())
    {
      return true;
    }
  }
  return false;
}

void Explore::goalResponseCallback(const NavigationGoalHandle::SharedPtr & goal_handle)
{
  waiting_for_goal_response_ = false;
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Goal was rejected by action server.");
    goal_active_ = false; // Goal rejected, so no active goal
  } else {
    RCLCPP_INFO(logger_, "Goal accepted by action server, waiting for result.");
    goal_active_ = true; // Goal accepted, now active
    navigation_goal_handle_ = goal_handle;
    goal_sent_time_ = this->now(); // Record the time the goal was accepted
    last_progress_ = this->now(); // Reset progress tracker
  }
}

void Explore::feedbackCallback(
  NavigationGoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  if (feedback->distance_remaining < prev_distance_) {
    last_progress_ = this->now();
    prev_distance_ = feedback->distance_remaining;
  }
}

void Explore::resultCallback(
  const NavigationGoalHandle::WrappedResult & result,
  const geometry_msgs::msg::Point & frontier_goal)
{
  goal_active_ = false; // Reset goal_active_ as the goal has reached a terminal state
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  exploration_finished_ = false;
  no_frontier_count_ = 0;
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  exploration_finished_ = finished_exploring;
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  waiting_for_goal_response_ = false;

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  exploration_finished_ = false;
  resuming_ = true;
  no_frontier_count_ = 0;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
    std::make_shared<explore::Explore>());    // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
