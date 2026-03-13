#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <opencv2/opencv.hpp>

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
  struct Frontier
  {
    std::uint32_t size;
    double min_distance;
    double cost;
    double info_gain;
    geometry_msgs::msg::Point initial;
    geometry_msgs::msg::Point centroid;
    geometry_msgs::msg::Point middle;
    std::vector < geometry_msgs::msg::Point > points;
  };

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
  class FrontierSearch
  {
public:
    FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {
    }                                                                  // Default constructor for the logger

    /**
     * @brief Constructor for search task
     * @param costmap Reference to costmap data to search.
     */
    FrontierSearch(
      nav2_costmap_2d::Costmap2D * costmap, double potential_scale,
      double gain_scale, double orientation_scale,
      double min_frontier_size, rclcpp::Logger logger);

    /**
     * @brief Runs search implementation, outward from the start position
     * @param position Initial position to search from
     * @return List of frontiers, if any
     */
    std::vector < Frontier > searchFrom(
      geometry_msgs::msg::Point position,
      double robot_yaw,
      const sensor_msgs::msg::BatteryState & battery_state,
      const geometry_msgs::msg::Point & home_pos,
      double energy_per_meter,
      double battery_safety_margin,
      double battery_penalty_scale,
      double min_frontier_clearance = 0.0,
      const cv::Mat & heatmap = cv::Mat(),
      double radiation_scale = 0.0,
      double radiation_gradient_scale = 0.0,
      const cv::Mat & explored_grid = cv::Mat());

protected:
    /**
     * @brief Starting from an initial cell, build a frontier from valid adjacent
     * cells
     * @param initial_cell Index of cell to start frontier building
     * @param reference Reference index to calculate position from
     * @param frontier_flag Flag vector indicating which cells are already marked
     * as frontiers
     * @return new frontier
     */
    Frontier buildNewFrontier(
      unsigned int initial_cell, unsigned int reference,
      std::vector < bool > & frontier_flag,
      double min_frontier_clearance);

    /**
     * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
     * for a new frontier.
     * @param idx Index of candidate cell
     * @param frontier_flag Flag vector indicating which cells are already marked
     * as frontiers
     * @return true if the cell is frontier cell
     */
    bool isNewFrontierCell(
      unsigned int idx,
      const std::vector < bool > & frontier_flag,
      double min_frontier_clearance);

    /**
     * @brief computes frontier cost
     * @details cost function is defined by potential_scale and gain_scale
     *
     * @param frontier frontier for which compute the cost
     * @return cost of the frontier
     */
    double frontierCost(
      Frontier & frontier,
      double robot_yaw,
      const sensor_msgs::msg::BatteryState & battery_state,
      const geometry_msgs::msg::Point & robot_pos,
      const geometry_msgs::msg::Point & home_pos,
      double energy_per_meter,
      double battery_safety_margin,
      double battery_penalty_scale,
      const cv::Mat & heatmap = cv::Mat(),
      double radiation_scale = 0.0,
      double radiation_gradient_scale = 0.0,
      const cv::Mat & explored_grid = cv::Mat());

private:
    nav2_costmap_2d::Costmap2D * costmap_;
    unsigned char * map_;
    unsigned int size_x_, size_y_;
    double potential_scale_, gain_scale_, orientation_scale_;
    double min_frontier_size_;
    rclcpp::Logger logger_;
  };
}  // namespace frontier_exploration
#endif
