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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

    rclcpp::Subscription < sensor_msgs::msg::Image > ::SharedPtr heatmap_std_sub_;
    void heatmapStdCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription < geometry_msgs::msg::PointStamped > ::SharedPtr stable_source_sub_;
    void stableSourceCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // stuck_escape_node publishes here once its own backup+Spin recovery
    // sequence completes. A goal that fails (timeout or costmap-blocked)
    // shortly after one of these is not evidence the frontier itself is bad
    // -- it may just be the same physical jam that made the robot stuck in
    // the first place, now resolved, with the goal simply not having had a
    // fair, uninterrupted attempt yet. See makePlan()'s stuck-grace check.
    rclcpp::Subscription < std_msgs::msg::Float64 > ::SharedPtr stuck_escape_event_sub_;
    void stuckEscapeEventCallback(const std_msgs::msg::Float64::SharedPtr msg);
    rclcpp::Time last_stuck_recovery_time_{0, 0, RCL_ROS_TIME};
    double stuck_grace_window_s_;
    // Reset in sendGoal() for every new goal; sending an early-return "one
    // grace pass and done" per goal avoids the grace check masking a
    // genuinely bad frontier forever if stuck-escapes keep recurring at the
    // same point.
    bool current_goal_given_stuck_grace_ = false;
    // Global (not per-goal) cap on total grace passes for the whole mission.
    // Without this, a robot physically wedged in one trap that keeps
    // targeting a slightly different nearby frontier candidate each time it
    // frees itself gets a fresh per-goal grace pass on every single one of
    // those candidates, so no individual point ever accumulates the two
    // failures needed to actually get blacklisted -- observed directly: 17
    // grace passes and zero blacklist events in one trial, 48 stuck events,
    // 5m of net progress, full wall-clock cap. This cap ensures the
    // mechanism can rescue a handful of genuinely unlucky one-off frontiers
    // without being able to indefinitely forestall blacklisting an entire
    // trap region.
    static constexpr int kMaxStuckGraceTotal = 3;
    int total_stuck_grace_count_ = 0;

    std::vector < geometry_msgs::msg::Point > frontier_blacklist_;
    geometry_msgs::msg::Point prev_goal_;
    double prev_distance_;
    int last_recovery_count_ = 0;
    rclcpp::Time last_progress_;
    size_t last_markers_count_;

    // Set by feedbackCallback when Nav2's recovery count increases; consumed
    // by makePlan() on its next tick, which cancels the current frontier goal
    // and sends a retreat-to-safety goal instead of letting explore_lite keep
    // replanning frontiers on top of whatever made Nav2 recover in the first
    // place.
    bool recalibration_requested_ = false;
    // True from when a retreat-to-safety goal is sent until its result comes
    // back; while true, makePlan() does not evaluate or replace frontiers.
    bool recalibrating_ = false;
    // True only for the currently in-flight goal if it's a recalibration
    // goal, so resultCallback can skip frontier-specific bookkeeping for it.
    bool current_goal_is_recalibration_ = false;

    geometry_msgs::msg::Pose initial_pose_;
    geometry_msgs::msg::Pose goal_start_pose_;
    double last_trajectory_yaw_;
    bool has_last_trajectory_ = false;
    sensor_msgs::msg::BatteryState latest_battery_state_;
    double cumulative_dose_ = 0.0;
    rclcpp::Time last_radiation_update_;

    // Dose-budget-aware relaxation of acute_dose_rate_mrem_hr_, using the
    // identical formula as local_recovery_node and rad_safety_monitor_node.
    // This node MUST relax whenever they do: its acute check is a "stop and
    // yield to the recovery node" handoff, and is_recovering_ is only ever
    // cleared by local_recovery_node calling /explore_node/start. If this
    // node stops at the fixed threshold while the recovery nodes' relaxed
    // triggers sit above the current dose rate, nobody ever comes -- the
    // robot idles in the dead band accumulating dose until the mission
    // budget expires (observed directly across the k>0 pilot arms: trials
    // parked with zero stuck events and near-zero distance). k=0 reproduces
    // the fixed threshold exactly.
    double dose_budget_relaxation_k_ = 0.0;
    double dose_budget_ceiling_mult_ = 2.0;
    double effectiveAcuteDoseThreshold();
    // Shared relaxation multiplier: authority fraction when fresh, own
    // integration otherwise (see the definition for the contract argument).
    double currentBudgetMultiplier();
    double authority_fraction_ = -1.0;
    rclcpp::Time authority_fraction_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription < std_msgs::msg::Float64 > ::SharedPtr budget_authority_sub_;
    rclcpp::Publisher < std_msgs::msg::Float64 > ::SharedPtr multiplier_pub_;
    rclcpp::TimerBase::SharedPtr multiplier_timer_;

    // Transit-dose frontier gating: the base fraction of the remaining dose
    // budget a single frontier visit may cost (see frontier_search's
    // frontierCost), relaxed by the same k/ceiling family as the thresholds
    // above so k acts as the budget-spend knob at the targeting layer too.
    double frontier_dose_spend_fraction_ = 0.05;
    double effectiveFrontierSpendFraction();

    // "Every remaining frontier is currently gated" is not the same map
    // state as "no frontiers remain": the GP heatmap keeps refining as
    // samples arrive, so a temporarily all-gated frontier set can become
    // partially eligible again. Hold completion while in that state, up to
    // this timeout, instead of declaring the mission complete on the spot
    // (observed directly: a hot spawn gated every frontier and the mission
    // reported success at 0.0m traveled).
    double all_gated_completion_timeout_s_ = 180.0;
    rclcpp::Time all_gated_since_{0, 0, RCL_ROS_TIME};
    int frontiers_explored_count_ = 0;
    bool is_verifying_ = false;
    int verification_attempts_ = 0;
    void returnToInitialPose(void);

    // Retry state for returnToInitialPose(): a cancellation (e.g. from
    // stuck_escape_node clearing a physical obstruction) is not necessarily
    // a bad path, only an interrupted attempt. Without a retry, the robot
    // ends up freed but idle forever once exploration_finished_ has already
    // stopped the main planning timer that would otherwise resend a goal.
    static constexpr int kMaxReturnToPoseRetries = 10;
    static constexpr double kReturnToPoseRetryDelaySec = 3.0;
    // A CANCELED result is stuck_escape_node preempting the goal to run its
    // ~10s backup-and-spin; it says nothing about whether home is reachable,
    // so it must not consume the retry budget. The retry also has to wait
    // out the escape maneuver, or the re-sent goal races it and drives the
    // robot straight back into the same jam. The cancel cap bounds a robot
    // that is permanently wedged (the trial wallclock cap is the final
    // backstop).
    static constexpr int kMaxReturnToPoseCancels = 40;
    static constexpr double kReturnAfterCancelDelaySec = 12.0;
    int return_to_pose_retry_count_ = 0;
    int return_to_pose_cancel_count_ = 0;
    rclcpp::TimerBase::SharedPtr return_to_pose_retry_timer_;

    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr start_exploration_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr stop_exploration_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr return_to_initial_pose_service_;
    rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr radiation_seeking_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr toggle_mode_service_;
    rclcpp::Service < std_srvs::srv::Trigger > ::SharedPtr reset_service_;
    bool resetServiceCallback(
      const std::shared_ptr < std_srvs::srv::Trigger::Request > request,
      std::shared_ptr < std_srvs::srv::Trigger::Response > response);
    bool toggleModeServiceCallback(
      const std::shared_ptr < std_srvs::srv::Trigger::Request > request,
      std::shared_ptr < std_srvs::srv::Trigger::Response > response);
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
    double obstacle_penalty_scale_;
    double frontier_clustering_distance_;
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

    // is_recalibration marks a goal sent to retreat the robot to nearby safe
    // space after a Nav2 recovery, rather than a normal frontier goal -- see
    // resultCallback, which skips frontier bookkeeping (blacklisting,
    // frontiers_explored_count_) for these.
    void sendGoal(const geometry_msgs::msg::Point & target, bool is_recalibration = false);

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
    std::string heatmap_std_topic_;
    double max_radiation_dose_;
    double radiation_scale_;
    double radiation_gradient_scale_;
    double radiation_seek_scale_;
    bool radiation_seek_adaptive_;     // scale seek_bonus by a stall deficit instead of a constant
    int radiation_survey_dirs_;        // boundary-survey ray count (default 8)
    double radiation_survey_time_cap_s_;  // boundary-survey time budget (0.0 = disabled)
    double critical_dose_mrem_hr_;     // cost-function penalty threshold
    double acute_dose_rate_mrem_hr_;   // hard stop: triggers evacuation + blacklist
    double dose_budget_scale_;
    double safe_dose_threshold_mrem_hr_;
    double current_radiation_rate_ = 0.0;
    bool is_recovering_ = false;

    // Effective seek scale for the current planning tick: either the raw
    // constant radiation_seek_scale_ (radiation_seek_adaptive_ == false) or
    // that constant scaled by a stall deficit computed directly from recent
    // net displacement (see recent_positions_), near-zero during healthy
    // exploration and ramping toward 1.0 as the robot genuinely fails to
    // get anywhere. Deliberately independent of last_progress_/
    // progress_timeout_ -- an earlier version used that timer and was found
    // (seek-adaptive smoke test, 2026-07-21) to be an unreliable stall
    // proxy, since many unrelated code paths reset it (goal-sent, Nav2
    // recovery, stuck-escape grace), so it could read "healthy" during a
    // mission that was macroscopically going nowhere.
    double effectiveRadiationSeekScale();

    // Rolling (time, position) history used by effectiveRadiationSeekScale().
    // Updated once per makePlan() tick, pruned to stall_window_s_.
    std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Point>> recent_positions_;
    double stall_window_s_;
    double stall_min_displacement_m_;

    // Wall-clock start of the current boundary-survey (RADIATION mode
    // Phase 2) episode, set once per fresh entry; used by
    // getRadiationFrontiers() to force-exit the survey once
    // radiation_survey_time_cap_s_ elapses, regardless of how many of the
    // radiation_survey_dirs_ rays have been visited.
    rclcpp::Time boundary_phase_start_time_;

    enum class ExplorationMode { SPATIAL, RADIATION };
    ExplorationMode exploration_mode_ = ExplorationMode::SPATIAL;

    // Set to true once the source peak is reached (gradient negligible) so the
    // boundary survey phase persists until the full zone is mapped.  Cleared
    // whenever RADIATION mode is re-entered from scratch.
    bool radiation_in_boundary_phase_ = false;

    // Rolling background estimator — mirrors the heatmap node's logic so the
    // explore node uses the same Poisson-aware dynamic threshold rather than a
    // fixed parameter that the spatially-varying background can easily exceed.
    static constexpr size_t RAD_BG_BUFFER_SIZE = 50;
    static constexpr double RAD_ELEVATION_SIGMA = 2.5;
    std::deque<double> rad_bg_buffer_;
    double rad_bg_mean_ = 0.0;
    double rad_bg_std_  = 1.0;   // conservative until buffer fills
    void updateRadBgStats();

    // Entry hysteresis: require this many consecutive above-threshold readings
    // before switching TO radiation mode.
    int radiation_mode_entry_hysteresis_ = 5;
    int radiation_mode_above_count_ = 0;

    // Exit hysteresis: use a rolling window of recent readings.
    // Exit RADIATION mode when the window mean drops below the dynamic threshold,
    // rather than requiring N *consecutive* below readings (which resets on every
    // spike and never accumulates in a noisy transition zone).
    int radiation_mode_hysteresis_ = 10;
    std::deque<double> recent_rad_buffer_;

    std::vector<frontier_exploration::Frontier> generateRadiationFrontiers();

    cv::Mat latest_heatmap_;
    cv::Mat latest_heatmap_std_;

    // Explored area tracking
    cv::Mat explored_grid_;
    int explored_count_ = 0;
    void updateExploredArea();

    // verification parameters
    int max_verification_attempts_;

    void handleExplorationCompletion(bool force_return = false);
    void startVerification();
    // Searches a small radius around `center` for a low-cost, non-inscribed-
    // obstacle cell to use as a goal -- used both to back a frontier goal off
    // a wall/shelf, and (centered on the robot's own pose) to find nearby
    // free space to retreat to after a Nav2 recovery. found_safe is set to
    // false if no cell within the search radius cleared the check, in which
    // case the returned point is not safe to send as a goal.
    geometry_msgs::msg::Point findSafeViewpoint(
      const geometry_msgs::msg::Point & center, bool & found_safe);
  };
}  // namespace explore

#endif
