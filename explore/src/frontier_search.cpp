#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <queue>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(
  nav2_costmap_2d::Costmap2D * costmap,
  double potential_scale, double gain_scale,
  double orientation_scale,
  double min_frontier_size, rclcpp::Logger logger)
: costmap_(costmap)
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
  double min_frontier_clearance,
  const cv::Mat & heatmap,
  double radiation_scale,
  double radiation_gradient_scale,
  double critical_dose_mrem_hr,
  const cv::Mat & explored_grid,
  const geometry_msgs::msg::Point & current_goal)
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

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);
  std::vector<double> dist_map(size_x_ * size_y_, std::numeric_limits<double>::infinity());

  // initialize priority queue for Dijkstra
  typedef std::pair<double, unsigned int> Cell;
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;

  // find closest traversable cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  unsigned int start_idx;
  if (map_[pos] < LETHAL_OBSTACLE) {
    start_idx = pos;
  } else if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    start_idx = clear;
  } else {
    start_idx = pos;
    RCLCPP_WARN(
      logger_,
      "[FrontierSearch] Could not find traversable cell to start search from robot position");
  }

  dist_map[start_idx] = 0.0;
  pq.push({0.0, start_idx});

  double res = costmap_->getResolution();
  double diag = res * 1.41421356;

  while (!pq.empty()) {
    Cell current = pq.top();
    pq.pop();
    unsigned int idx = current.second;
    double d = current.first;

    if (d > dist_map[idx]) {continue;}
    if (visited_flag[idx]) {continue;}
    visited_flag[idx] = true;

    // iterate over 8-connected neighbourhood for Dijkstra
    for (unsigned nbr : nhood8(idx, *costmap_)) {
      // add to queue all free, unvisited cells
      if (map_[nbr] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        unsigned int nx, ny, ix, iy;
        costmap_->indexToCells(nbr, nx, ny);
        costmap_->indexToCells(idx, ix, iy);
        double step = (nx != ix && ny != iy) ? diag : res;
        double new_dist = d + step;

        if (new_dist < dist_map[nbr]) {
          dist_map[nbr] = new_dist;
          pq.push({new_dist, nbr});
        }
      } else if (isNewFrontierCell(nbr, frontier_flag, min_frontier_clearance)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(
          nbr, frontier_flag, min_frontier_clearance,
          dist_map);
        if (new_frontier.size * res >= min_frontier_size_) {
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
      heatmap, radiation_scale, radiation_gradient_scale,
      critical_dose_mrem_hr, explored_grid, current_goal);
  }
  std::sort(
    frontier_list.begin(), frontier_list.end(),
    [](const Frontier & f1, const Frontier & f2) {return f1.cost < f2.cost;});

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(
  unsigned int initial_cell,
  std::vector<bool> & frontier_flag,
  double min_frontier_clearance,
  const std::vector<double> & dist_map)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 0; // Initialize size to 0, it will be incremented for valid points.
  output.min_distance = std::numeric_limits<double>::infinity();

  double res = costmap_->getResolution();
  double diag = res * 1.41421356;

  auto get_path_dist = [&](unsigned int cell_idx) -> double {
      double min_d = std::numeric_limits<double>::infinity();
      unsigned int cx, cy;
      costmap_->indexToCells(cell_idx, cx, cy);
      for (unsigned int n : nhood8(cell_idx, *costmap_)) {
        if (dist_map[n] < std::numeric_limits<double>::infinity()) {
          unsigned int nx, ny;
          costmap_->indexToCells(n, nx, ny);
          double step = (nx != cx && ny != cy) ? diag : res;
          min_d = std::min(min_d, dist_map[n] + step);
        }
      }
      return min_d;
    };

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // initialize size and centroid with the initial cell
  output.size = 1;
  output.points.push_back(output.initial);
  output.centroid.x = output.initial.x;
  output.centroid.y = output.initial.y;
  output.min_distance = get_path_dist(initial_cell);
  output.middle.x = output.initial.x;
  output.middle.y = output.initial.y;

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag, min_frontier_clearance)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        // Check for NaN or Inf values in world coordinates
        if (std::isnan(wx) || std::isinf(wx) || std::isnan(wy) || std::isinf(wy)) {
          RCLCPP_ERROR(
            logger_,
            "[FrontierSearch] Invalid world coordinates (NaN/Inf) detected for cell (%d, %d). Skipping this point.", mx,
            my);
          continue;   // Skip this point, do not add it to the frontier
        }

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, using path distance
        double distance = get_path_dist(nbr);
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  if (output.size > 0) { // Avoid division by zero
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
  } else {
    RCLCPP_WARN(logger_, "[FrontierSearch] Frontier with size 0 created. Centroid will be (0,0).");
    // If a frontier is size 0, it shouldn't be added to the list in searchFrom anyway
    // But for safety, ensure min_distance is infinity if size is 0
    output.min_distance = std::numeric_limits<double>::infinity();
  }
  return output;
}

bool FrontierSearch::isNewFrontierCell(
  unsigned int idx,
  const std::vector<bool> & frontier_flag,
  double min_frontier_clearance)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free and has enough clearance
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (min_frontier_clearance <= 0) {
        return true;
      }
      // If clearance is requested, we check if the neighbor's cost is low enough.
      // A cost of 0 (FREE_SPACE) is the safest.
      // As a heuristic, we'll use a threshold of 50 to ensure some distance from obstacles while being more tolerant.
      if (map_[nbr] < 50) {
        return true;
      }
    }
  }

  return false;
}

double FrontierSearch::frontierCost(
  Frontier & frontier,
  double robot_yaw,
  const sensor_msgs::msg::BatteryState & battery_state,
  const geometry_msgs::msg::Point & robot_pos,
  const geometry_msgs::msg::Point & home_pos,
  double energy_per_meter,
  double battery_safety_margin,
  double battery_penalty_scale,
  const cv::Mat & heatmap,
  double radiation_scale,
  double radiation_gradient_scale,
  double critical_dose_mrem_hr,
  const cv::Mat & explored_grid,
  const geometry_msgs::msg::Point & current_goal)
{
  frontier.info_gain = 0.0;
  // Feasibility Check
  double dist_to_frontier = frontier.min_distance;
  double dist_to_home =
    std::sqrt(
    std::pow(
      frontier.centroid.x - home_pos.x,
      2) + std::pow(frontier.centroid.y - home_pos.y, 2));
  double total_dist = dist_to_frontier + dist_to_home;
  double energy_cost = total_dist * energy_per_meter;

  if (energy_cost > battery_state.percentage - battery_safety_margin) {
    return std::numeric_limits<double>::infinity();
  }

  // Original Cost
  // min_distance is already in meters, removed resolution multiplication.
  double original_cost = (potential_scale_ * frontier.min_distance) -
    (gain_scale_ * frontier.size * costmap_->getResolution());

  // Raycasting-based Information Gain (Request 2)
  double info_gain_cost = 0;
  if (!explored_grid.empty()) {
    int unknown_cells_visible = 0;
    double angle_step = 2.0 * M_PI / 36.0; // 10 degree steps for efficiency
    double range = 5.0; // Visibility range from frontier
    double res = costmap_->getResolution();

    for (double a = 0; a < 2.0 * M_PI; a += angle_step) {
      for (double r = 0; r < range; r += res) {
        double wx = frontier.centroid.x + r * cos(a);
        double wy = frontier.centroid.y + r * sin(a);
        unsigned int rmx, rmy;
        if (costmap_->worldToMap(wx, wy, rmx, rmy)) {
          if (costmap_->getCost(rmx, rmy) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            break;
          }
          if (explored_grid.at<uchar>(rmy, rmx) == 0) {
            unknown_cells_visible++;
          }
        } else {
          break;
        }
      }
    }
    // Store gain for visualization
    frontier.info_gain = unknown_cells_visible * res * res;
    // Subtract from cost (higher gain = lower cost)
    info_gain_cost = -gain_scale_ * unknown_cells_visible * res;
  }

  // Radiation-informed Cost (Request 1)
  double rad_cost = 0;
  if (!heatmap.empty()) {
    unsigned int mx, my;
    if (costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, mx, my)) {
      int row = (heatmap.rows - 1) - my;
      if (mx < (unsigned int)heatmap.cols && row >= 0 && row < heatmap.rows) {
        float radiation = heatmap.at<float>(row, mx);
        if (radiation >= critical_dose_mrem_hr) {return std::numeric_limits<double>::infinity();}
      }
    }
    if (costmap_->worldToMap(frontier.middle.x, frontier.middle.y, mx, my)) {
      int row = (heatmap.rows - 1) - my;
      if (mx < (unsigned int)heatmap.cols && row >= 0 && row < heatmap.rows) {
        float radiation = heatmap.at<float>(row, mx);
        if (radiation >= critical_dose_mrem_hr) {return std::numeric_limits<double>::infinity();}
      }
    }

    if (costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, mx, my)) {
      int h_x = mx;
      int h_y = my;

      if (h_x >= 0 && h_x < heatmap.cols && h_y >= 0 && h_y < heatmap.rows) {
        // Map h_y (bottom-up) to row index (top-down)
        int row = (heatmap.rows - 1) - h_y;
        float radiation = heatmap.at<float>(row, h_x);

        // If radiation exceeds critical threshold, this frontier is lethal/invalid
        if (radiation >= critical_dose_mrem_hr) {
          return std::numeric_limits<double>::infinity();
        }

        // Non-linear aggressive penalty (Request 1 refinement)
        // If radiation is high, the penalty should grow faster than the gain can offset.
        // Using a squared penalty after a warning threshold.
        double warning_thresh = 0.25; // Physical threshold for "elevated"
        if (radiation > warning_thresh) {
          rad_cost = radiation_scale * std::pow(radiation / warning_thresh, 2.0);
        } else {
          rad_cost = radiation_scale * radiation;
        }

        // Gradient (Request 1)
        if (radiation_gradient_scale != 0 && h_x > 0 && h_x < heatmap.cols - 1 && h_y > 0 &&
          h_y < heatmap.rows - 1)
        {
          int row = (heatmap.rows - 1) - h_y;
          float grad_x = (heatmap.at<float>(row, h_x + 1) - heatmap.at<float>(row, h_x - 1)) / 2.0;
          // Note: In heatmap (top-down), row decreases as y (bottom-up) increases.
          // d(rad)/dy = d(rad)/d(row) * d(row)/dy = d(rad)/d(row) * (-1)
          // d(rad)/d(row) is approx (rad(row+1) - rad(row-1)) / 2
          // So d(rad)/dy is (rad(row-1) - rad(row+1)) / 2
          float grad_y = (heatmap.at<float>(row - 1, h_x) - heatmap.at<float>(row + 1, h_x)) / 2.0;

          // Vector from robot to frontier
          double dx = frontier.centroid.x - robot_pos.x;
          double dy = frontier.centroid.y - robot_pos.y;
          double d_len = std::sqrt(dx * dx + dy * dy);
          if (d_len > 0.001) {
            dx /= d_len;
            dy /= d_len;
            // Dot product: preference for moving with/against gradient
            rad_cost += radiation_gradient_scale * (grad_x * dx + grad_y * dy);
          }
        }
      }
    }
  }

  // Orientation Cost (Trajectory/Orientation Awareness)
  // Sharply penalize frontiers that are "behind" the robot to prevent backtracking.
  double angle_to_frontier = std::atan2(
    frontier.centroid.y - robot_pos.y,
    frontier.centroid.x - robot_pos.x);
  double angle_diff = std::abs(angle_to_frontier - robot_yaw);
  // Normalize angle_diff to [0, PI]
  while (angle_diff > M_PI) {angle_diff -= 2.0 * M_PI;}
  angle_diff = std::abs(angle_diff);

  // Use a quadratic penalty for backtracking beyond a certain threshold (e.g., 60 degrees)
  double orientation_penalty = 0;
  if (orientation_scale_ > 0) {
    if (angle_diff > M_PI / 3.0) { // > 60 degrees
      // Quadratic penalty: grows quickly as we look further behind
      double normalized_diff = angle_diff / (M_PI / 3.0);
      orientation_penalty = orientation_scale_ * std::pow(normalized_diff, 2);
    } else {
      // Light linear penalty for small deviations
      orientation_penalty = (orientation_scale_ / 2.0) * (angle_diff / (M_PI / 3.0));
    }
  }

  // Battery Penalty (Safety Awareness)
  // Only apply a strong penalty if battery is significantly depleted or if we are very far from home.
  double battery_penalty = 0;
  if (battery_penalty_scale > 0) {
    // Reduce the influence of dist_to_home unless necessary to avoid "magnetic" pull to start point
    double home_factor = (battery_state.percentage < 0.5) ? 1.0 : 0.2;
    double adjusted_energy_cost = (dist_to_frontier + dist_to_home * home_factor) *
      energy_per_meter;
    battery_penalty = battery_penalty_scale * adjusted_energy_cost;
  }

  // Hysteresis Bonus: Give a small discount to the current goal area to avoid oscillations
  double hysteresis_bonus = 0.0;
  if (current_goal.x != 0 || current_goal.y != 0) {
    double dx = frontier.centroid.x - current_goal.x;
    double dy = frontier.centroid.y - current_goal.y;
    double dist_to_current_sq = dx * dx + dy * dy;
    if (dist_to_current_sq < 1.0 * 1.0) { // 1.0m radius
      hysteresis_bonus = -0.1 * (1.0 - std::sqrt(dist_to_current_sq));
    }
  }

  return original_cost + info_gain_cost + rad_cost + orientation_penalty + battery_penalty +
         hysteresis_bonus;
}
}  // namespace frontier_exploration
