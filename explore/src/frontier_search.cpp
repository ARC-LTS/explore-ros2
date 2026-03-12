#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(
  nav2_costmap_2d::Costmap2D * costmap,
  nav2_costmap_2d::Costmap2D * radiation_costmap,
  double potential_scale, double gain_scale,
  double orientation_scale,
  double min_frontier_size, rclcpp::Logger logger)
: costmap_(costmap)
  , radiation_costmap_(radiation_costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , orientation_scale_(orientation_scale)
  , min_frontier_size_(min_frontier_size)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(
  geometry_msgs::msg::Point position,
  double robot_yaw,
  const sensor_msgs::msg::BatteryState & battery_state,
  const geometry_msgs::msg::Point & home_pos,
  double energy_per_meter,
  double battery_safety_margin,
  double battery_penalty_scale,
  const nav_msgs::msg::OccupancyGrid & explored_map,
  double min_frontier_clearance,
  double radiation_weight,
  bool has_rad_gradient,
  geometry_msgs::msg::Point rad_gradient_vec)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(
      logger_,
      "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(costmap_->getMutex()));
  
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> rad_lock(
    *(radiation_costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  rad_map_ = radiation_costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest traversable cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (map_[pos] < LETHAL_OBSTACLE) {
    bfs.push(pos);
  } else if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find traversable cell to start search from robot position");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells
      if (map_[nbr] < LETHAL_OBSTACLE && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isSpatialFrontierCell(nbr, frontier_flag, min_frontier_clearance)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, explored_map, min_frontier_clearance, false);
        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      } else if (isRadiationFrontierCell(nbr, frontier_flag, explored_map, min_frontier_clearance)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, explored_map, min_frontier_clearance, true);
        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto & frontier : frontier_list) {
    frontier.cost = frontierCost(
      frontier, robot_yaw, battery_state, position, home_pos,
      energy_per_meter, battery_safety_margin, battery_penalty_scale, 
      explored_map, radiation_weight, has_rad_gradient, rad_gradient_vec);
  }
  std::sort(
    frontier_list.begin(), frontier_list.end(),
    [](const Frontier & f1, const Frontier & f2) {return f1.cost < f2.cost;});

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(
  unsigned int initial_cell,
  unsigned int reference,
  std::vector<bool> & frontier_flag,
  const nav_msgs::msg::OccupancyGrid & explored_map,
  double min_frontier_clearance,
  bool is_radiation)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 0;
  output.min_distance = std::numeric_limits<double>::infinity();
  output.is_radiation = is_radiation;

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // initialize size and centroid with the initial cell
  output.size = 1;
  output.points.push_back(output.initial);
  output.centroid.x = output.initial.x;
  output.centroid.y = output.initial.y;

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  output.min_distance = sqrt(
    pow((double(reference_x) - double(output.initial.x)), 2.0) +
    pow((double(reference_y) - double(output.initial.y)), 2.0));
  output.middle.x = output.initial.x;
  output.middle.y = output.initial.y;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell of the SAME TYPE
      bool valid = is_radiation ? 
                   isRadiationFrontierCell(nbr, frontier_flag, explored_map, min_frontier_clearance) :
                   isSpatialFrontierCell(nbr, frontier_flag, min_frontier_clearance);

      if (valid) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        // Check for NaN or Inf values in world coordinates
        if (std::isnan(wx) || std::isinf(wx) || std::isnan(wy) || std::isinf(wy)) {
          continue;
        }

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);
        output.size++;
        output.centroid.x += wx;
        output.centroid.y += wy;

        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }
        bfs.push(nbr);
      }
    }
  }

  if (output.size > 0) {
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
  }
  return output;
}

bool FrontierSearch::isSpatialFrontierCell(
  unsigned int idx,
  const std::vector<bool> & frontier_flag,
  double min_frontier_clearance)
{
  // 1. Spatial Requirement: SLAM map must be NO_INFORMATION
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // 2. Accessibility: Must have at least one traversable neighbor
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] < LETHAL_OBSTACLE) {
        if (min_frontier_clearance <= 0 || map_[nbr] < 100) {
            return true;
        }
    }
  }
  return false;
}

bool FrontierSearch::isRadiationFrontierCell(
  unsigned int idx,
  const std::vector<bool> & frontier_flag,
  const nav_msgs::msg::OccupancyGrid & explored_map,
  double min_frontier_clearance)
{
  // 1. Known Space Requirement: SLAM map must NOT be NO_INFORMATION
  if (map_[idx] == NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // 2. Radiation Interest: Unknown radiation OR High radiation
  bool is_rad_unknown = (rad_map_[idx] == NO_INFORMATION);
  bool is_high_rad = (rad_map_[idx] > 50 && rad_map_[idx] < LETHAL_OBSTACLE);
  
  if (!is_rad_unknown && !is_high_rad) {
      return false;
  }

  // 3. Physical Coverage: Must NOT have been covered by robot footprint already
  if (idx < explored_map.data.size() && explored_map.data[idx] == 100) {
      return false;
  }

  // 4. Accessibility: Must have at least one traversable neighbor
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] < LETHAL_OBSTACLE) {
        if (min_frontier_clearance <= 0 || map_[nbr] < 100) {
            return true;
        }
    }
  }
  return false;
}

bool FrontierSearch::isNewFrontierCell(
  unsigned int idx,
  const std::vector<bool> & frontier_flag,
  const nav_msgs::msg::OccupancyGrid & explored_map,
  double min_frontier_clearance)
{
  // Defunct, but kept for signature compatibility if needed
  return isSpatialFrontierCell(idx, frontier_flag, min_frontier_clearance) ||
         isRadiationFrontierCell(idx, frontier_flag, explored_map, min_frontier_clearance);
}

double FrontierSearch::frontierCost(
  const Frontier & frontier,
  double robot_yaw,
  const sensor_msgs::msg::BatteryState & battery_state,
  const geometry_msgs::msg::Point & robot_pos,
  const geometry_msgs::msg::Point & home_pos,
  double energy_per_meter,
  double battery_safety_margin,
  double battery_penalty_scale,
  const nav_msgs::msg::OccupancyGrid & explored_map,
  double radiation_weight,
  bool has_rad_gradient,
  geometry_msgs::msg::Point rad_gradient_vec)
{
  (void)explored_map;

  // 1. Base Costs
  double dist_to_frontier = frontier.min_distance;
  double dist_to_home = std::sqrt(std::pow(frontier.centroid.x - home_pos.x, 2) + 
                                  std::pow(frontier.centroid.y - home_pos.y, 2));
  
  double energy_required = (dist_to_frontier + dist_to_home) * energy_per_meter;
  if (energy_required > battery_state.percentage - battery_safety_margin) {
    return std::numeric_limits<double>::infinity();
  }

  // 2. Multipliers and Bonuses
  double rad_multiplier = 1.0;
  double spatial_bonus = 0.0;

  if (frontier.is_radiation) {
      // RADIATION FRONTIER LOGIC
      double rad_priority = 0.0;
      unsigned int mx, my;
      if (radiation_costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, mx, my)) {
          unsigned char rad_cost = rad_map_[radiation_costmap_->getIndex(mx, my)];
          if (rad_cost != NO_INFORMATION && rad_cost > 0) {
              rad_priority = std::min(1.0, static_cast<double>(rad_cost) / 150.0);
          }
      }

      // ONLY RADIATION FRONTIERS USE THE GRADIENT FOR DIRECT BIAS
      if (has_rad_gradient) {
          double dx = frontier.centroid.x - robot_pos.x;
          double dy = frontier.centroid.y - robot_pos.y;
          double dist = std::sqrt(dx*dx + dy*dy);
          if (dist > 0.1) {
              double dot = (dx/dist * rad_gradient_vec.x + dy/dist * rad_gradient_vec.y);
              if (dot > 0) {
                  rad_priority = std::max(rad_priority, dot);
              }
          }
      }
      rad_multiplier = 1.0 - (0.5 * rad_priority * (radiation_weight / 10.0));
      rad_multiplier = std::max(0.1, rad_multiplier);
  } else {
      // SPATIAL FRONTIER LOGIC
      // No gradient navigation for spatial frontiers, just physical mapping
      spatial_bonus = 5.0; // Preference for mapping new physical space
  }

  // 3. Final Calculation
  double base_cost = potential_scale_ * dist_to_frontier;
  double gain = gain_scale_ * (frontier.size * costmap_->getResolution() + spatial_bonus);
  
  double battery_penalty = 0;
  if (battery_penalty_scale > 0) {
      double home_factor = (battery_state.percentage < 0.5) ? 1.0 : 0.2;
      battery_penalty = battery_penalty_scale * (dist_to_frontier + dist_to_home * home_factor) * energy_per_meter;
  }

  double angle_to_frontier = std::atan2(frontier.centroid.y - robot_pos.y, frontier.centroid.x - robot_pos.x);
  double angle_diff = std::abs(angle_to_frontier - robot_yaw);
  while (angle_diff > M_PI) {angle_diff -= 2.0 * M_PI;}
  angle_diff = std::abs(angle_diff);

  double orientation_penalty = 0;
  if (orientation_scale_ > 0 && angle_diff > M_PI / 3.0) {
      orientation_penalty = orientation_scale_ * std::pow(angle_diff / (M_PI/3.0), 2);
  }

  return (base_cost * rad_multiplier) - gain + orientation_penalty + battery_penalty;
}
}  // namespace frontier_exploration
