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
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
  class Explore: public rclcpp::Node
  {
public:
    Explore();
    ~Explore();

    void start();
    void stop(bool finished_exploring = false);
    void resume();

    using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle < nav2_msgs::action::NavigateToPose >;

private:
    void handleRecovery();
    /**
     * @brief  Make a global plan
     */
    void makePlan();

    // /**
    //  * @brief  Publish a frontiers as markers
    //  */
    void visualizeFrontiers(
      const std::vector < frontier_exploration::Frontier > & frontiers);

    bool goalOnBlacklist(const geometry_msgs::msg::Point & goal);

    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    // void
    // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
    // future);
    void goalResponseCallback(const NavigationGoalHandle::SharedPtr & goal_handle);
    void feedbackCallback(
      NavigationGoalHandle::SharedPtr,
      const std::shared_ptr < const nav2_msgs::action::NavigateToPose::Feedback > feedback);
    void resultCallback(
      const NavigationGoalHandle::WrappedResult & result,
      const geometry_msgs::msg::Point & frontier_goal);

    rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr
    marker_array_publisher_;
    rclcpp::Publisher < nav_msgs::msg::OccupancyGrid > ::SharedPtr
    explored_area_publisher_;
    rclcpp::Publisher < std_msgs::msg::Bool > ::SharedPtr
    exploration_finished_publisher_;
    rclcpp::Logger logger_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    Costmap2DClient costmap_client_;
    rclcpp_action::Client < nav2_msgs::action::NavigateToPose > ::SharedPtr
    move_base_client_;
    frontier_exploration::FrontierSearch search_;
    rclcpp::TimerBase::SharedPtr exploring_timer_;
    // rclcpp::TimerBase::SharedPtr oneshot_;

    rclcpp::Subscription < std_msgs::msg::Bool > ::SharedPtr resume_subscription_;
    void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription < sensor_msgs::msg::BatteryState > ::SharedPtr battery_sub_;
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

    rclcpp::Subscription < std_msgs::msg::Float64MultiArray > ::SharedPtr radiation_sub_;
    void radiationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Subscription < sensor_msgs::msg::Image > ::SharedPtr heatmap_sub_;
    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::vector < geometry_msgs::msg::Point > frontier_blacklist_;
    geometry_msgs::msg::Point prev_goal_;
    double prev_distance_;
    int last_recovery_count_ = 0;
    rclcpp::Time last_progress_;
    size_t last_markers_count_;

    geometry_msgs::msg::Pose initial_pose_;
    geometry_msgs::msg::Pose goal_start_pose_;
    double last_trajectory_yaw_;
    bool has_last_trajectory_ = false;
    sensor_msgs::msg::BatteryState latest_battery_state_;
    double cumulative_dose_ = 0.0;
    rclcpp::Time last_radiation_update_;
    int frontiers_explored_count_ = 0;
    bool is_verifying_ = false;
    int verification_attempts_ = 0;
    void returnToInitialPose(void);

    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr start_exploration_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr stop_exploration_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr return_to_initial_pose_service_;
    rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr radiation_seeking_service_;
    rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr mode_publisher_;

    bool startExplorationServiceCallback(
      const std::shared_ptr < std_srvs::srv::Trigger::Request > request,
      std::shared_ptr < std_srvs::srv::Trigger::Response > response);
    bool stopExplorationServiceCallback(
      const std::shared_ptr < std_srvs::srv::Trigger::Request > request,
      std::shared_ptr < std_srvs::srv::Trigger::Response > response);
    bool returnToInitialPoseServiceCallback(
      const std::shared_ptr < std_srvs::srv::Trigger::Request > request,
      std::shared_ptr < std_srvs::srv::Trigger::Response > response);
    bool setRadiationSeekingCallback(
      const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
      std::shared_ptr < std_srvs::srv::SetBool::Response > response);

    // parameters
    double planner_frequency_;
    double potential_scale_, orientation_scale_, gain_scale_;
    double progress_timeout_;
    double min_frontier_clearance_;
    bool visualize_;
    bool return_to_init_;
    int no_frontier_retry_;
    std::string robot_base_frame_;
    bool resuming_ = false;
    bool goal_active_ = false;
    bool waiting_for_goal_response_ = false;
    bool exploration_finished_ = false;
    bool stopped_ = true;
    int no_frontier_count_ = 0;
    rclcpp::Time goal_sent_time_;

    // Added for more robust goal management
    geometry_msgs::msg::Point last_sent_goal_position_;
    geometry_msgs::msg::Point queued_target_;
    bool has_queued_target_ = false;
    bool last_goal_successful_ = false;
    double min_goal_distance_for_replanning_;

    void sendGoal(const geometry_msgs::msg::Point & target);

    // battery-aware parameters
    std::string battery_topic_;
    double energy_per_meter_;
    double battery_safety_margin_;
    double battery_penalty_scale_;
    double min_battery_percentage_;
    double max_battery_used_;

    // radiation parameters
    std::string radiation_topic_;
    std::string heatmap_topic_;
    double max_radiation_dose_;
    double radiation_scale_;
    double radiation_gradient_scale_;
    double critical_dose_mrem_hr_;
    double safe_dose_threshold_mrem_hr_;
    double current_radiation_rate_ = 0.0;
    bool is_recovering_ = false;
    bool radiation_seeking_mode_ = false;
    cv::Mat latest_heatmap_;

    // Explored area tracking
    cv::Mat explored_grid_;
    void updateExploredArea();

    // verification parameters
    int max_verification_attempts_;

    void handleExplorationCompletion(bool force_return = false);
    void startVerification();
    geometry_msgs::msg::Point findSafeViewpoint(const frontier_exploration::Frontier & frontier);
  };
}  // namespace explore

#endif
