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
  , last_trajectory_yaw_(0.0)
  , has_last_trajectory_(false)
  , goal_active_(false)
  , waiting_for_goal_response_(false)
  , exploration_finished_(false)
  , stopped_(true)
  , goal_sent_time_(this->now())

{
  double timeout;
  double min_frontier_size;
  bool auto_start;
  this->declare_parameter<double>("planner_frequency", 1.0);
  this->declare_parameter<double>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<double>("potential_scale", 1.0);
  this->declare_parameter<double>("orientation_scale", 10.0);
  this->declare_parameter<double>("gain_scale", 1.0);
  this->declare_parameter<double>("min_frontier_size", 0.5);
  this->declare_parameter<double>("min_frontier_clearance", 0.1);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("auto_start", true);
  this->declare_parameter<int>("no_frontier_retry", 3);
  this->declare_parameter<std::string>("battery_topic", "/battery_state");
  this->declare_parameter<double>("energy_per_meter", 0.1);
  this->declare_parameter<double>("battery_safety_margin", 0.1);
  this->declare_parameter<double>("battery_penalty_scale", 0.0);
  this->declare_parameter<double>("min_goal_distance_for_replanning", 0.75); // New parameter
  this->declare_parameter<double>("min_battery_percentage", 0.1);
  this->declare_parameter<std::string>("radiation_topic", "rad");
  this->declare_parameter<std::string>("heatmap_topic", "/rad_heatmap/raw_mean_map");
  this->declare_parameter<double>("max_radiation_dose", 100.0);
  this->declare_parameter<double>("critical_dose_mrem_hr", 2.0);
  this->declare_parameter<double>("acute_dose_rate_mrem_hr", 10.0);
  this->declare_parameter<double>("dose_budget_scale", 10.0);
  this->declare_parameter<double>("safe_dose_threshold_mrem_hr", 0.5);
  this->declare_parameter<double>("radiation_scale", 1.0);
  this->declare_parameter<double>("radiation_gradient_scale", 1.0);
  this->declare_parameter<int>("max_verification_attempts", 3);
  this->declare_parameter<double>("max_battery_used", 0.9);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("min_frontier_clearance", min_frontier_clearance_);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("auto_start", auto_start);
  this->get_parameter("no_frontier_retry", no_frontier_retry_);
  this->get_parameter("battery_topic", battery_topic_);
  this->get_parameter("energy_per_meter", energy_per_meter_);
  this->get_parameter("battery_safety_margin", battery_safety_margin_);
  this->get_parameter("battery_penalty_scale", battery_penalty_scale_);
  this->get_parameter("min_goal_distance_for_replanning", min_goal_distance_for_replanning_); // Get new parameter
  this->get_parameter("min_battery_percentage", min_battery_percentage_);
  this->get_parameter("radiation_topic", radiation_topic_);
  this->get_parameter("heatmap_topic", heatmap_topic_);
  this->get_parameter("max_radiation_dose", max_radiation_dose_);
  this->get_parameter("critical_dose_mrem_hr", critical_dose_mrem_hr_);
  this->get_parameter("acute_dose_rate_mrem_hr", acute_dose_rate_mrem_hr_);
  this->get_parameter("dose_budget_scale", dose_budget_scale_);
  this->get_parameter("safe_dose_threshold_mrem_hr", safe_dose_threshold_mrem_hr_);
  this->get_parameter("radiation_scale", radiation_scale_);
  this->get_parameter("radiation_gradient_scale", radiation_gradient_scale_);
  this->get_parameter("max_verification_attempts", max_verification_attempts_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("max_battery_used", max_battery_used_);

  // Initialize new member variables
  last_sent_goal_position_.x = initial_pose_.position.x;
  last_sent_goal_position_.y = initial_pose_.position.y;
  last_sent_goal_position_.z = initial_pose_.position.z;
  last_goal_successful_ = true; // Assume initial state is successful completion of previous (non-existent) goal
  last_progress_ = this->now();

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
    explored_area_publisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "explore/"
      "explored_area",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  }

  exploration_finished_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "explore/is_finished",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "explore/resume", 10,
    std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10,
    std::bind(&Explore::batteryCallback, this, std::placeholders::_1));

  radiation_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    radiation_topic_, 10,
    std::bind(&Explore::radiationCallback, this, std::placeholders::_1));
  last_radiation_update_ = this->now();

  heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    heatmap_topic_, 10,
    std::bind(&Explore::heatmapCallback, this, std::placeholders::_1));

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

  exploring_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration::from_seconds(1.0 / planner_frequency_),
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

  radiation_seeking_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_radiation_seeking",
    std::bind(
      &Explore::setRadiationSeekingCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  mode_publisher_ = this->create_publisher<std_msgs::msg::String>("~/mode", 10);
  std_msgs::msg::String initial_mode;
  initial_mode.data = "spatial_exploration";
  mode_publisher_->publish(initial_mode);
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
  bool battery_low = latest_battery_state_.percentage < min_battery_percentage_;
  bool battery_used_up = (1.0 - latest_battery_state_.percentage) > max_battery_used_;

  if (!exploration_finished_ && (battery_low || battery_used_up)) {
    if (battery_low) {
      RCLCPP_WARN(
        logger_, "Battery level %.2f below threshold %.2f. Returning to origin.",
        latest_battery_state_.percentage, min_battery_percentage_);
    } else {
      RCLCPP_WARN(
        logger_, "Battery used %.2f exceeds threshold %.2f. Returning to origin.",
        1.0 - latest_battery_state_.percentage, max_battery_used_);
    }
    handleExplorationCompletion(true);
  }
}

void Explore::radiationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {return;}
  if (exploration_finished_) {return;}

  double mR_hr = msg->data[1];
  current_radiation_rate_ = mR_hr; // Store current rate
  rclcpp::Time now = this->now();
  double dt = (now - last_radiation_update_).seconds();
  last_radiation_update_ = now;

  if (dt > 0.0 && dt < 10.0) { // Sanity check for dt
    // Dose (mR) = rate (mR/hr) * time (hr)
    cumulative_dose_ += mR_hr * (dt / 3600.0);
  }

  // Trigger evacuation only on acute dose rates — genuinely dangerous readings
  // that would consume a dangerous fraction of the remaining budget in seconds.
  // Moderate radiation (> critical_dose_mrem_hr_ but < acute) is handled by
  // the frontier cost function: the robot may traverse such areas if the
  // cumulative dose budget permits.
  if (current_radiation_rate_ > acute_dose_rate_mrem_hr_) {
    if (!is_recovering_ && goal_active_) {
      RCLCPP_WARN(
        logger_,
        "ACUTE radiation (%.2f mR/hr) at (%.2f, %.2f). Blacklisting and evacuating.",
        current_radiation_rate_, last_sent_goal_position_.x, last_sent_goal_position_.y);
      frontier_blacklist_.push_back(last_sent_goal_position_);
    }
    is_recovering_ = true;
    has_queued_target_ = false;
  }

  if (!exploration_finished_ && cumulative_dose_ > max_radiation_dose_) {
    RCLCPP_WARN(
      logger_, "Cumulative radiation dose %.2f mR exceeded threshold %.2f mR. Returning to origin.",
      cumulative_dose_, max_radiation_dose_);
    handleExplorationCompletion(true);
  }
}

void Explore::heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    latest_heatmap_ = cv_ptr->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception in explore node: %s", e.what());
  }
}

void Explore::updateExploredArea()
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
  if (!costmap) {return;}

  unsigned int map_w = costmap->getSizeInCellsX();
  unsigned int map_h = costmap->getSizeInCellsY();

  if (explored_grid_.empty() ||
    explored_grid_.cols != (int)map_w ||
    explored_grid_.rows != (int)map_h)
  {
    explored_grid_ = cv::Mat(map_h, map_w, CV_8UC1, cv::Scalar(0));
    explored_count_ = 0;
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    return;
  }
  auto pose = pose_stamped.pose;
  unsigned int robot_mx, robot_my;
  if (!costmap->worldToMap(pose.position.x, pose.position.y, robot_mx, robot_my)) {
    return;
  }

  // Raycasting to update explored area.
  // Uses 180 angles (2° steps) and incremental integer cell stepping to avoid
  // calling worldToMap() ~200 times per ray — a ~20x reduction vs. the original.
  constexpr int N_ANGLES = 180;
  constexpr double ANGLE_STEP = 2.0 * M_PI / N_ANGLES;
  double range = 10.0;
  double res = costmap->getResolution();
  int n_steps = static_cast<int>(range / res);

  // Precompute trig table once per call
  double cos_t[N_ANGLES], sin_t[N_ANGLES];
  for (int i = 0; i < N_ANGLES; i++) {
    cos_t[i] = std::cos(i * ANGLE_STEP);
    sin_t[i] = std::sin(i * ANGLE_STEP);
  }

  const unsigned char * map_data = costmap->getCharMap();
  double start_cx = static_cast<double>(robot_mx) + 0.5;
  double start_cy = static_cast<double>(robot_my) + 0.5;

  for (int ai = 0; ai < N_ANGLES; ai++) {
    double dcx = cos_t[ai];
    double dcy = sin_t[ai];
    double fcx = start_cx;
    double fcy = start_cy;

    for (int k = 0; k < n_steps; k++) {
      fcx += dcx;
      fcy += dcy;
      int rmx = static_cast<int>(fcx);
      int rmy = static_cast<int>(fcy);
      if (rmx < 0 || rmx >= (int)map_w || rmy < 0 || rmy >= (int)map_h) {
        break;
      }
      uchar & cell = explored_grid_.at<uchar>(rmy, rmx);
      if (map_data[rmy * map_w + rmx] >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        if (cell == 0) { cell = 255; explored_count_++; }
        break;
      }
      if (cell == 0) { cell = 255; explored_count_++; }
    }
  }

  double area = explored_count_ * res * res;
  RCLCPP_INFO_THROTTLE(
    logger_, *this->get_clock(), 5000,
    "Explored Area: %.2f m^2 (%d cells)", area, explored_count_);

  // Publish the explored area map for visualization
  if (visualize_ && explored_area_publisher_) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = costmap_client_.getGlobalFrameID();
    msg.info.resolution = res;
    msg.info.width = costmap->getSizeInCellsX();
    msg.info.height = costmap->getSizeInCellsY();
    msg.info.origin.position.x = costmap->getOriginX();
    msg.info.origin.position.y = costmap->getOriginY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(msg.info.width * msg.info.height);

    for (size_t i = 0; i < msg.data.size(); ++i) {
      // Map CV_8UC1 (0 or 255) to OccupancyGrid (0 or 100)
      msg.data[i] = (explored_grid_.data[i] > 0) ? 100 : 0;
    }
    explored_area_publisher_->publish(msg);
  }
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
  double max_gain = 0.0;
  for (const auto & f : frontiers) {
    if (f.info_gain > max_gain) {max_gain = f.info_gain;}
  }

  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.id = 0;
  markers.push_back(m);

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto & frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position = geometry_msgs::msg::Point();
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      // Color gradient based on info_gain: Blue (0) to Green (max)
      if (max_gain > 0) {
        double ratio = frontier.info_gain / max_gain;
        m.color.r = 0;
        m.color.g = ratio;
        m.color.b = 1.0 - ratio;
        m.color.a = 1.0;
      } else {
        m.color = blue;
      }
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

  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (exploration_finished_ || stopped_) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    RCLCPP_WARN(logger_, "Failed to get robot pose for planning.");
    return;
  }
  auto pose = pose_stamped.pose;

  // --- Stage 0: Danger Check (Radiation) ---
  // If in recovery, fully yield to local_recovery_node. Stop exploration
  // so both nodes don't contend for navigation goals simultaneously.
  // local_recovery_node will call /explore_node/start once safe.
  if (is_recovering_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *this->get_clock(), 5000,
      "Radiation critical (%.2f mR/hr). Yielding to recovery node.",
      current_radiation_rate_);
    stop(false);
    return;
  }

  if (current_radiation_rate_ > acute_dose_rate_mrem_hr_) {
    RCLCPP_WARN(
      logger_,
      "Robot in ACUTE radiation zone (%.2f mR/hr). Stopping exploration, yielding to recovery node.",
      current_radiation_rate_);
    is_recovering_ = true;
    stop(false);
    return;
  }

  // --- Stage 1: Check current navigation goal status ---
  if (waiting_for_goal_response_) {
    RCLCPP_DEBUG(logger_, "Waiting for goal response. Deferring new plan.");
    return;
  }

  bool is_current_goal_invalid = false;
  if (goal_active_) {
    // 1. Check for timeout
    if ((this->now() - last_progress_) > tf2::durationFromSec(progress_timeout_)) {
      RCLCPP_WARN(logger_, "Active goal timed out. Re-planning.");
      is_current_goal_invalid = true;
    }
    // 2. Check if current goal is now blocked OR already explored (no longer unknown)
    nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
    unsigned int mx, my;
    if (costmap->worldToMap(last_sent_goal_position_.x, last_sent_goal_position_.y, mx, my)) {
      unsigned char cost = costmap->getCost(mx, my);
      // If it's an obstacle (>= 253)
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        RCLCPP_INFO(logger_, "Current goal blocked. Blacklisting.");
        frontier_blacklist_.push_back(last_sent_goal_position_);
        is_current_goal_invalid = true;
      } else if (cost != nav2_costmap_2d::NO_INFORMATION) {
        // If it's already been explored, we only cancel if we are reasonably close to it.
        // This prevents 'target switching' loops where SLAM clears the goal area just before arrival.
        double dx = last_sent_goal_position_.x - pose.position.x;
        double dy = last_sent_goal_position_.y - pose.position.y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_goal_distance_for_replanning_ * min_goal_distance_for_replanning_) {
          RCLCPP_INFO(
            logger_,
            "Current goal area already explored by sensors and reached. Switching targets.");
          // Blacklist the reached goal to prevent immediate re-targeting of the same spot
          frontier_blacklist_.push_back(last_sent_goal_position_);
          is_current_goal_invalid = true;
        }
      }
    }

    if (is_current_goal_invalid) {
      move_base_client_->async_cancel_goal(navigation_goal_handle_);
      goal_active_ = false;
      // We fall through to allow the search to immediately find a replacement.
    }
  }

  // --- Stage 2: Find the best new frontier ---
  updateExploredArea();
  double robot_yaw = tf2::getYaw(pose.orientation);
  double search_yaw = robot_yaw;

  double remaining_dose_budget = std::max(0.0, max_radiation_dose_ - cumulative_dose_);
  auto frontiers = search_.searchFrom(
    pose.position, search_yaw, latest_battery_state_,
    initial_pose_.position, energy_per_meter_,
    battery_safety_margin_, battery_penalty_scale_,
    min_frontier_clearance_, latest_heatmap_,
    radiation_scale_, radiation_gradient_scale_,
    critical_dose_mrem_hr_, explored_grid_,
    goal_active_ ? last_sent_goal_position_ : geometry_msgs::msg::Point(),
    remaining_dose_budget, dose_budget_scale_);

  // In radiation-seeking mode, re-sort frontiers to prefer those nearest to
  // high-radiation areas, turning the normal cost penalty into a reward.
  if (radiation_seeking_mode_ && !latest_heatmap_.empty() && !frontiers.empty()) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
    std::stable_sort(
      frontiers.begin(), frontiers.end(),
      [&](const frontier_exploration::Frontier & a,
        const frontier_exploration::Frontier & b) {
        auto sample = [&](const geometry_msgs::msg::Point & pt) -> float {
          unsigned int mx, my;
          if (!costmap->worldToMap(pt.x, pt.y, mx, my)) {return 0.0f;}
          int row = (latest_heatmap_.rows - 1) - static_cast<int>(my);
          if (row < 0 || row >= latest_heatmap_.rows ||
            mx >= static_cast<unsigned int>(latest_heatmap_.cols))
          {return 0.0f;}
          return latest_heatmap_.at<float>(row, static_cast<int>(mx));
        };
        return sample(a.centroid) > sample(b.centroid);
      });
  }

  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Filter frontiers: find the best one
  auto frontier_it = frontiers.begin();
  geometry_msgs::msg::Point best_target;
  bool found_target = false;

  // Count infeasible frontiers for diagnostics
  int n_inf_cost = 0;
  for (const auto & f : frontiers) {
    if (std::isinf(f.cost)) { n_inf_cost++; }
  }
  if (n_inf_cost > 0 && n_inf_cost == static_cast<int>(frontiers.size())) {
    RCLCPP_WARN(logger_,
      "All %d frontiers have infinite cost (infeasible). "
      "Check battery feasibility params: energy_per_meter=%.3f, "
      "battery_safety_margin=%.3f, battery_pct=%.3f. "
      "If battery constraints are not needed set energy_per_meter=0.",
      n_inf_cost, energy_per_meter_, battery_safety_margin_,
      latest_battery_state_.percentage);
  }

  while (frontier_it != frontiers.end()) {
    const auto & f = *frontier_it;

    // 0. Never navigate to an infeasible frontier — doing so causes immediate
    //    blacklisting and replanning churn with no forward progress.
    if (std::isinf(f.cost)) {
      frontier_it++;
      continue;
    }

    // 1. Check if centroid is blacklisted
    if (goalOnBlacklist(f.centroid)) {
      frontier_it++;
      continue;
    }

    // 2. Check if we are already pursuing this frontier
    if (goal_active_ && (same_point(f.centroid, last_sent_goal_position_) ||
                         same_point(f.middle, last_sent_goal_position_))) {
      frontier_it++;
      continue;
    }

    // 3. Distance check: ignore frontiers too close to the robot
    double dx = f.centroid.x - pose.position.x;
    double dy = f.centroid.y - pose.position.y;
    if (dx * dx + dy * dy < min_goal_distance_for_replanning_ * min_goal_distance_for_replanning_) {
      frontier_it++;
      continue;
    }

    // 4. Generate viewpoint and check IF IT is blacklisted
    best_target = findSafeViewpoint(f);
    if (goalOnBlacklist(best_target)) {
      frontier_it++;
      continue;
    }

    found_target = true;
    // Log why this frontier was chosen so backtracking can be diagnosed in log.out
    {
      const auto & f = *frontier_it;
      double ang = std::atan2(f.centroid.y - pose.position.y, f.centroid.x - pose.position.x) - search_yaw;
      while (ang > M_PI) { ang -= 2.0 * M_PI; }
      while (ang < -M_PI) { ang += 2.0 * M_PI; }
      RCLCPP_INFO(logger_,
        "Selected frontier: centroid=(%.2f, %.2f) dist=%.2fm angle=%.1fdeg cost=%.2f (robot yaw=%.1fdeg)",
        f.centroid.x, f.centroid.y, f.min_distance,
        ang * 180.0 / M_PI, f.cost, search_yaw * 180.0 / M_PI);
    }
    break;
  }

  // Handle no valid frontiers
  if (!found_target) {
    if (!goal_active_) {
      no_frontier_count_++;
      if (no_frontier_count_ >= no_frontier_retry_) {
        no_frontier_count_ = 0;
        startVerification();
      }
    }
    has_queued_target_ = false;
    return;
  }

  // Final check for stopped_ flag before sending goal to prevent race with service calls
  if (stopped_) {
    RCLCPP_INFO(logger_, "Exploration stopped just before sending goal. Aborting goal.");
    return;
  }

  // If a goal is active and valid, we just queue the next target.
  if (goal_active_) {
    queued_target_ = best_target;
    has_queued_target_ = true;
    RCLCPP_DEBUG(
      logger_, "Queued next target (%.2f, %.2f) while moving.", best_target.x,
      best_target.y);
    return;
  }

  // If we get here, no goal is active (or it was just invalidated).
  // Check the queue first for zero-delay transition, otherwise use the best target found.
  if (has_queued_target_ && !goalOnBlacklist(queued_target_)) {
    sendGoal(queued_target_);
    has_queued_target_ = false;
  } else {
    sendGoal(best_target);
  }
}

void Explore::sendGoal(const geometry_msgs::msg::Point & target)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    return;
  }
  auto pose = pose_stamped.pose;
  RCLCPP_INFO(logger_, "Sending navigation goal to (%.2f, %.2f).", target.x, target.y);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target;
  double yaw = atan2(target.y - pose.position.y, target.x - pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&Explore::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback = std::bind(&Explore::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&Explore::resultCallback, this, _1, target);

  waiting_for_goal_response_ = true;
  move_base_client_->async_send_goal(goal, send_goal_options);

  last_sent_goal_position_ = target;
  goal_start_pose_ = pose;
  last_progress_ = this->now();
  prev_distance_ = std::numeric_limits<double>::infinity();
  no_frontier_count_ = 0;
  last_goal_successful_ = false;
}

void Explore::handleExplorationCompletion(bool force_return)
{
  if (exploration_finished_) {
    return;
  }
  exploration_finished_ = true;
  std_msgs::msg::Bool finished_msg;
  finished_msg.data = true;
  exploration_finished_publisher_->publish(finished_msg);

  RCLCPP_WARN(logger_, "Exploration finished: no more frontiers found or all traversed/tried out.");

  // 1. Halt exploration (cancel current goal, stop timer)
  RCLCPP_INFO(logger_, "Halting exploration due to completion.");
  if (goal_active_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
  }
  exploring_timer_->cancel(); // Stop the main planning loop
  goal_active_ = false; // Ensure goal active state is reset
  has_last_trajectory_ = false;

  // 2. Return to original pose, if configured or forced
  if (return_to_init_ || force_return) {
    RCLCPP_INFO(logger_, "Initiating return to initial pose.");
    returnToInitialPose(); // This function already handles sending the goal
  } else {
    RCLCPP_INFO(logger_, "Robot will remain at current position.");
  }
}

void Explore::startVerification()
{
  if (verification_attempts_ >= max_verification_attempts_) {
    RCLCPP_WARN(
      logger_,
      "Verification phase complete after %d attempts. No new frontiers found. Believing completion.",
      verification_attempts_);
    handleExplorationCompletion(true);
    return;
  }

  is_verifying_ = true;
  verification_attempts_++;

  RCLCPP_INFO(
    logger_,
    "Starting verification attempt %d/%d to ensure no regions were missed. Explored %d frontiers so far.",
    verification_attempts_, max_verification_attempts_, frontiers_explored_count_);

  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    return;
  }
  auto pose = pose_stamped.pose;
  geometry_msgs::msg::Point verification_goal;
  bool found_goal = false;

  // Systematically check different directions (N, S, E, W, NE, NW, SE, SW)
  double angles[] =
  {0.0, M_PI, M_PI / 2.0, -M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0, -M_PI / 4.0,
    -3.0 * M_PI / 4.0};
  int start_angle_idx = (verification_attempts_ - 1) % 8;
  double search_distance = 4.0 + 4.0 * ((verification_attempts_ - 1) / 2); // Increase distance every 2 attempts

  for (int i = 0; i < 8; ++i) {
    int idx = (start_angle_idx + i) % 8;
    double angle = angles[idx];
    verification_goal.x = pose.position.x + search_distance * cos(angle);
    verification_goal.y = pose.position.y + search_distance * sin(angle);

    unsigned int mx, my;
    if (costmap_client_.getCostmap()->worldToMap(
        verification_goal.x, verification_goal.y, mx,
        my))
    {
      unsigned char cost = costmap_client_.getCostmap()->getCost(mx, my);
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        found_goal = true;
        break;
      }
    }
  }

  if (found_goal) {
    RCLCPP_INFO(
      logger_, "Verification: Traversing to region at (%.2f, %.2f) to check for missing frontiers.",
      verification_goal.x, verification_goal.y);
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = verification_goal;
    goal.pose.pose.orientation = pose.orientation;
    goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.pose.header.stamp = this->now();

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &Explore::resultCallback, this, _1,
      verification_goal);
    send_goal_options.goal_response_callback = std::bind(&Explore::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Explore::feedbackCallback, this, _1, _2);

    waiting_for_goal_response_ = true;
    move_base_client_->async_send_goal(goal, send_goal_options);

    last_sent_goal_position_ = verification_goal;
    goal_start_pose_ = pose;
    last_progress_ = this->now();
    prev_distance_ = std::numeric_limits<double>::infinity();
  } else {
    RCLCPP_WARN(
      logger_,
      "Verification: Could not find a valid goal in attempt %d. Trying next attempt.",
      verification_attempts_);
    startVerification(); // Recursive call to next attempt
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

  send_goal_options.goal_response_callback = std::bind(&Explore::goalResponseCallback, this, _1);

  send_goal_options.result_callback =
    [this](const NavigationGoalHandle::WrappedResult & result) {
      goal_active_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
      } else {
        RCLCPP_WARN(logger_, "Failed to return to initial pose: %d", static_cast<int>(result.code));
      }
    };

  waiting_for_goal_response_ = true;
  move_base_client_->async_send_goal(goal, send_goal_options);
  last_sent_goal_position_ = initial_pose_.position;
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

  if (feedback->number_of_recoveries > last_recovery_count_) {
    RCLCPP_INFO(
      logger_, "Nav2 recovery behavior detected (count: %d). Resetting progress timeout.",
      feedback->number_of_recoveries);
    last_progress_ = this->now();
    last_recovery_count_ = feedback->number_of_recoveries;
  }
}

void Explore::resultCallback(
  const NavigationGoalHandle::WrappedResult & result,
  const geometry_msgs::msg::Point & frontier_goal)
{
  // Ignore results from old/superseded goals
  if (navigation_goal_handle_ && result.goal_id != navigation_goal_handle_->get_goal_id()) {
    RCLCPP_DEBUG(logger_, "Result received for old or superseded goal. Ignoring.");
    return;
  }

  goal_active_ = false; // Reset goal_active_ for the CURRENT goal

  // Update trajectory information based on the movement since the goal was started
  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    return;
  }
  auto pose = pose_stamped.pose;
  double dx = pose.position.x - goal_start_pose_.position.x;
  double dy = pose.position.y - goal_start_pose_.position.y;
  double dist_sq = dx * dx + dy * dy;
  if (dist_sq > 0.5 * 0.5) { // Minimum travel distance to trust the yaw
    last_trajectory_yaw_ = std::atan2(dy, dx);
    has_last_trajectory_ = true;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        logger_, "Navigation goal to (%.2f, %.2f) SUCCEEDED.", frontier_goal.x,
        frontier_goal.y);
      last_goal_successful_ = true;
      if (!is_verifying_) {
        frontiers_explored_count_++;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if (waiting_for_goal_response_) {
        RCLCPP_INFO(
          logger_,
          "Navigation goal to (%.2f, %.2f) PREEMPTED by new goal. Not blacklisting.",
          frontier_goal.x, frontier_goal.y);
      } else {
        RCLCPP_WARN(
          logger_, "Navigation goal to (%.2f, %.2f) ABORTED. Adding to blacklist.",
          frontier_goal.x, frontier_goal.y);
        frontier_blacklist_.push_back(frontier_goal);
      }
      last_goal_successful_ = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(
        logger_, "Navigation goal to (%.2f, %.2f) CANCELED.", frontier_goal.x,
        frontier_goal.y);
      last_goal_successful_ = false;
      break;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }

  // Force a fresh frontier search from the current (arrived) position.
  // The queued target was computed while the robot was still moving — discard it.
  has_queued_target_ = false;
  if (!waiting_for_goal_response_ && !stopped_) {
    makePlan();
  }
}

bool Explore::setRadiationSeekingCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  radiation_seeking_mode_ = request->data;
  std_msgs::msg::String mode_msg;
  mode_msg.data = radiation_seeking_mode_ ? "radiation_seeking" : "spatial_exploration";
  mode_publisher_->publish(mode_msg);
  response->success = true;
  response->message = radiation_seeking_mode_ ?
    "Radiation-seeking mode enabled: prioritizing high-radiation frontiers." :
    "Spatial exploration mode enabled: prioritizing coverage.";
  RCLCPP_INFO(logger_, "%s", response->message.c_str());
  return true;
}

void Explore::start()
{
  exploration_finished_ = false;
  stopped_ = false;
  no_frontier_count_ = 0;
  has_last_trajectory_ = false;
  goal_active_ = false;
  waiting_for_goal_response_ = false;
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  exploration_finished_ = finished_exploring;
  stopped_ = true;
  RCLCPP_INFO(logger_, "Exploration stopped.");
  if (goal_active_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
  }
  exploring_timer_->cancel();
  waiting_for_goal_response_ = false;
  goal_active_ = false;
  has_last_trajectory_ = false;

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  exploration_finished_ = false;
  stopped_ = false;
  is_recovering_ = false;  // local_recovery_node confirmed safe before calling start
  resuming_ = true;
  no_frontier_count_ = 0;
  has_last_trajectory_ = false;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

geometry_msgs::msg::Point Explore::findSafeViewpoint(
  const frontier_exploration::Frontier & frontier)
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_client_.getCostmap();
  unsigned int mx, my;
  if (!costmap->worldToMap(frontier.middle.x, frontier.middle.y, mx, my)) {
    return frontier.middle;
  }

  // Search in a small radius around the frontier point for a cell that is:
  // 1. FREE_SPACE (cost 0 or very low)
  // 2. Not adjacent to lethal/inscribed obstacles

  double resolution = costmap->getResolution();
  double search_radius = 1.0; // Search within 1 meter
  int cell_radius = static_cast<int>(search_radius / resolution);

  geometry_msgs::msg::Point safe_point = frontier.middle;
  unsigned char min_cost = 255;
  double min_dist_to_target = std::numeric_limits<double>::infinity();

  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!costmap_client_.getRobotPose(pose_stamped)) {
    return frontier.middle;
  }
  auto robot_pose = pose_stamped.pose;

  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      int cx = static_cast<int>(mx) + dx;
      int cy = static_cast<int>(my) + dy;

      if (cx < 0 || cx >= static_cast<int>(costmap->getSizeInCellsX()) ||
        cy < 0 || cy >= static_cast<int>(costmap->getSizeInCellsY())) {continue;}

      unsigned char cost = costmap->getCost(cx, cy);

      // We want a point that is FREE and away from obstacles.
      // Lower cost is better. Cost 0 is ideal (FREE_SPACE).
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost < 50) {
        // Ensure we are not on the very last edge of the map to prevent planner boundary errors
        if (cx < 2 || cx >= static_cast<int>(costmap->getSizeInCellsX()) - 2 ||
            cy < 2 || cy >= static_cast<int>(costmap->getSizeInCellsY()) - 2) {
          continue;
        }

        double wx, wy;
        costmap->mapToWorld(cx, cy, wx, wy);

        // Favor points that are closer to the frontier middle (the target)
        // while still being safe and navigable.
        double dx_f = wx - frontier.middle.x;
        double dy_f = wy - frontier.middle.y;
        double dist_to_target = dx_f * dx_f + dy_f * dy_f;

        // Prefer lower cost first, then closer to frontier middle
        if (cost < min_cost) {
          min_cost = cost;
          min_dist_to_target = dist_to_target;
          safe_point.x = wx;
          safe_point.y = wy;
        } else if (cost == min_cost && dist_to_target < min_dist_to_target) {
          min_dist_to_target = dist_to_target;
          safe_point.x = wx;
          safe_point.y = wy;
        }
      }
    }
  }

  return safe_point;
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
    std::make_shared<explore::Explore>());    // std::move(std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
