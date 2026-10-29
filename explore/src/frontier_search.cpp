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
  double min_frontier_size, double obstacle_penalty_scale,
  double frontier_clustering_distance,
  rclcpp::Logger logger)
: costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , orientation_scale_(orientation_scale)
  , min_frontier_size_(min_frontier_size)
  , obstacle_penalty_scale_(obstacle_penalty_scale)
  , frontier_clustering_distance_(frontier_clustering_distance)
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
  double lethal_point_ceiling_mrem_hr,
  const cv::Mat & explored_grid,
  const geometry_msgs::msg::Point & current_goal,
  double remaining_dose_budget,
  double dose_budget_scale,
  double frontier_spend_fraction,
  const cv::Mat & heatmap_std,
  double radiation_seek_scale,
  const std::vector<geometry_msgs::msg::Point> & recent_trap_locations,
  double trap_avoid_radius_m)
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
  // Per-cell initial Dijkstra step direction (unit vector from robot toward each cell).
  // Propagated outward so every reachable cell knows which first step leads to it.
  std::vector<float> init_dx(size_x_ * size_y_, 0.0f);
  std::vector<float> init_dy(size_x_ * size_y_, 0.0f);

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

  unsigned int start_cx, start_cy;
  costmap_->indexToCells(start_idx, start_cx, start_cy);

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
          // Seed direction from robot on first step; propagate outward unchanged.
          if (idx == start_idx) {
            float ddx = static_cast<float>((int)nx - (int)start_cx);
            float ddy = static_cast<float>((int)ny - (int)start_cy);
            float len = std::sqrt(ddx * ddx + ddy * ddy);
            init_dx[nbr] = (len > 0.0f) ? ddx / len : 0.0f;
            init_dy[nbr] = (len > 0.0f) ? ddy / len : 0.0f;
          } else {
            init_dx[nbr] = init_dx[idx];
            init_dy[nbr] = init_dy[idx];
          }
          pq.push({new_dist, nbr});
        }
      } else if (isNewFrontierCell(nbr, frontier_flag, min_frontier_clearance)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(
          nbr, frontier_flag, min_frontier_clearance,
          dist_map, init_dx, init_dy);
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
      lethal_point_ceiling_mrem_hr, explored_grid, current_goal,
      remaining_dose_budget, dose_budget_scale, frontier_spend_fraction,
      heatmap_std, radiation_seek_scale,
      recent_trap_locations, trap_avoid_radius_m);
  }

  // --- Frontier Clustering ("Mean Frontier Candidate") ---
  if (frontier_clustering_distance_ > 0.0 && !frontier_list.empty()) {
    std::vector<Frontier> clustered_list;
    // Sort by size to use larger frontiers as cluster seeds
    std::sort(frontier_list.begin(), frontier_list.end(),
              [](const Frontier& a, const Frontier& b) { return a.size > b.size; });

    for (const auto& f : frontier_list) {
      bool merged = false;
      for (auto& cluster : clustered_list) {
        double dx = f.centroid.x - cluster.centroid.x;
        double dy = f.centroid.y - cluster.centroid.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < frontier_clustering_distance_) {
          // Merge f into cluster
          double total_size = static_cast<double>(cluster.size + f.size);
          // Weighted average for centroid
          cluster.centroid.x = (cluster.centroid.x * cluster.size + f.centroid.x * f.size) / total_size;
          cluster.centroid.y = (cluster.centroid.y * cluster.size + f.centroid.y * f.size) / total_size;
          
          cluster.size += f.size;
          cluster.info_gain += f.info_gain;
          
          // Use the 'middle' point of the best (lowest cost) frontier as the target
          if (f.cost < cluster.cost) {
              cluster.middle = f.middle;
              cluster.initial = f.initial;
              cluster.path_heading_x = f.path_heading_x;
              cluster.path_heading_y = f.path_heading_y;
              cluster.min_distance = f.min_distance;
              cluster.cost = f.cost; // Temporarily update cost
          }
          
          // Add points to cluster (optional, but good for visualization)
          cluster.points.insert(cluster.points.end(), f.points.begin(), f.points.end());
          
          merged = true;
          break;
        }
      }
      if (!merged) {
        clustered_list.push_back(f);
      }
    }
    
    // Final pass: Recalculate cost for the merged cluster
    for (auto & cluster : clustered_list) {
        cluster.cost = frontierCost(
          cluster, robot_yaw, battery_state, position, home_pos,
          energy_per_meter, battery_safety_margin, battery_penalty_scale,
          heatmap, radiation_scale, radiation_gradient_scale,
          lethal_point_ceiling_mrem_hr, explored_grid, current_goal,
          remaining_dose_budget, dose_budget_scale, frontier_spend_fraction,
          heatmap_std, radiation_seek_scale,
          recent_trap_locations, trap_avoid_radius_m);
    }
    frontier_list = clustered_list;
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
  const std::vector<double> & dist_map,
  const std::vector<float> & init_dx,
  const std::vector<float> & init_dy)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 0; // Initialize size to 0, it will be incremented for valid points.
  output.min_distance = std::numeric_limits<double>::infinity();
  output.path_heading_x = 0.0;
  output.path_heading_y = 0.0;

  double res = costmap_->getResolution();
  double diag = res * 1.41421356;

  // Returns path distance and sets out_hdx/out_hdy to the initial Dijkstra step
  // direction (unit vector) that leads toward cell_idx from the robot start.
  auto get_path_dist = [&](unsigned int cell_idx, double & out_hdx, double & out_hdy) -> double {
      double min_d = std::numeric_limits<double>::infinity();
      unsigned int cx, cy;
      costmap_->indexToCells(cell_idx, cx, cy);
      for (unsigned int n : nhood8(cell_idx, *costmap_)) {
        if (dist_map[n] < std::numeric_limits<double>::infinity()) {
          unsigned int nx, ny;
          costmap_->indexToCells(n, nx, ny);
          double step = (nx != cx && ny != cy) ? diag : res;
          double d = dist_map[n] + step;
          if (d < min_d) {
            min_d = d;
            out_hdx = static_cast<double>(init_dx[n]);
            out_hdy = static_cast<double>(init_dy[n]);
          }
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
  output.min_distance = get_path_dist(initial_cell, output.path_heading_x, output.path_heading_y);
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
        double ph_dx = 0.0, ph_dy = 0.0;
        double distance = get_path_dist(nbr, ph_dx, ph_dy);
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
          output.path_heading_x = ph_dx;
          output.path_heading_y = ph_dy;
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
    // Goal point: the frontier cell nearest the centroid, i.e. mid-span. The
    // BFS above left `middle` at the cell with the shortest path to the robot,
    // which is by construction the frontier's end nearest the robot -- where
    // the frontier meets a wall -- so the viewpoint hugged the inflation band
    // and the robot wedged there (2026-10-29). min_distance/path_heading still
    // come from the nearest cell; only the target point moves.
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto & pt : output.points) {
      double ddx = pt.x - output.centroid.x;
      double ddy = pt.y - output.centroid.y;
      double d2 = ddx * ddx + ddy * ddy;
      if (d2 < best_d2) {
        best_d2 = d2;
        output.middle.x = pt.x;
        output.middle.y = pt.y;
      }
    }
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
      // As a heuristic, we'll use a threshold of 20 to ensure some distance from obstacles while being more tolerant.
      if (map_[nbr] < 20) {
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
  double lethal_point_ceiling_mrem_hr,
  const cv::Mat & explored_grid,
  const geometry_msgs::msg::Point & current_goal,
  double remaining_dose_budget,
  double dose_budget_scale,
  double frontier_spend_fraction,
  const cv::Mat & heatmap_std,
  double radiation_seek_scale,
  const std::vector<geometry_msgs::msg::Point> & recent_trap_locations,
  double trap_avoid_radius_m)
{
  frontier.info_gain = 0.0;
  // Battery feasibility check.
  // Only applied when:
  //   - energy_per_meter > 0 (battery modelling is enabled)
  //   - battery percentage is a valid, known value (sensor_msgs/BatteryState uses
  //     -1.0 for "unknown"; 0.0 may be an uninitialized default before the first
  //     message arrives, so we also skip the check in that case)
  //
  // NOTE on scaling: battery_state.percentage follows the ROS standard (0.0–1.0).
  // energy_per_meter must be set accordingly — e.g. 0.001 means 0.1 % per meter.
  // If your battery driver publishes 0–100, divide energy_per_meter by 100.
  double dist_to_frontier = frontier.min_distance;
  double dist_to_home =
    std::sqrt(
    std::pow(
      frontier.centroid.x - home_pos.x,
      2) + std::pow(frontier.centroid.y - home_pos.y, 2));
  double total_dist = dist_to_frontier + dist_to_home;
  double energy_cost = total_dist * energy_per_meter;

  const float batt_pct = battery_state.percentage;
  const bool batt_valid = (batt_pct > 0.0f);  // skip if unknown (-1) or uninitialized (0)
  if (energy_per_meter > 0.0 && batt_valid &&
    energy_cost > static_cast<double>(batt_pct) - battery_safety_margin)
  {
    return std::numeric_limits<double>::infinity();
  }

  // Original Cost
  // min_distance is already in meters, removed resolution multiplication.
  // Size-based gain is capped at 25% of distance cost so frontier size can't
  // override the distance preference.
  double size_gain = gain_scale_ * frontier.size * costmap_->getResolution();
  double max_size_gain = potential_scale_ * frontier.min_distance * 0.25;
  size_gain = std::min(size_gain, max_size_gain);
  double original_cost = (potential_scale_ * frontier.min_distance) - size_gain;

  // Obstacle Penalty
  // Penalize frontiers that are in high-cost areas (e.g. inflation buffer)
  // to favor free-space frontiers unless no others are available.
  if (obstacle_penalty_scale_ > 0.0) {
    unsigned int mx, my;
    if (costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, mx, my)) {
      unsigned char cost = map_[my * size_x_ + mx];
      if (cost > 0 && cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Linear penalty based on costmap value (0-252)
        original_cost += obstacle_penalty_scale_ * cost;
      }
    }
  }

  // Raycasting-based Information Gain (Request 2)
  // Uses incremental integer cell stepping to avoid worldToMap() on every inner
  // loop iteration — replaces ~3600 FP divisions with simple integer arithmetic.
  double info_gain_cost = 0;
  if (!explored_grid.empty() && gain_scale_ != 0.0) {
    int unknown_cells_visible = 0;
    constexpr int N_RAYS = 36;
    constexpr double ANGLE_STEP = 2.0 * M_PI / N_RAYS;
    double range = 5.0;
    double res = costmap_->getResolution();
    int n_steps = static_cast<int>(range / res);

    unsigned int cx_start, cy_start;
    if (costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, cx_start, cy_start)) {
      double fstart_x = static_cast<double>(cx_start) + 0.5;
      double fstart_y = static_cast<double>(cy_start) + 0.5;

      for (int ai = 0; ai < N_RAYS; ai++) {
        double a = ai * ANGLE_STEP;
        double dca = std::cos(a);
        double dsa = std::sin(a);
        double fcx = fstart_x;
        double fcy = fstart_y;

        for (int k = 0; k < n_steps; k++) {
          fcx += dca;
          fcy += dsa;
          int rmx = static_cast<int>(fcx);
          int rmy = static_cast<int>(fcy);
          if (rmx < 0 || rmx >= (int)size_x_ || rmy < 0 || rmy >= (int)size_y_) {break;}
          if (map_[rmy * size_x_ + rmx] >= nav2_costmap_2d::LETHAL_OBSTACLE) {break;}
          if (explored_grid.at<uchar>(rmy, rmx) == 0) {
            unknown_cells_visible++;
          }
        }
      }
    }
    frontier.info_gain = unknown_cells_visible * res * res;
    info_gain_cost = -gain_scale_ * unknown_cells_visible * res;

    // Cap: info gain can offset at most 50% of this frontier's own distance cost.
    // This ensures distance always dominates — a far high-gain frontier can never
    // beat a closer frontier purely on information value.
    double max_info_benefit = potential_scale_ * frontier.min_distance * 0.5;
    info_gain_cost = std::max(info_gain_cost, -max_info_benefit);
  }

  // Radiation-informed Cost (Request 1)
  // Compute worldToMap for centroid once and reuse for both the critical-dose
  // check and the radiation cost calculation below.
  // Point-lethality checks below use lethal_point_ceiling_mrem_hr, an
  // acute-level ceiling (the caller passes its effective acute-dose yield
  // threshold), NOT the old critical_dose_mrem_hr (2.0). Gating destinations
  // at 2.0 mrem/hr was incoherent with the mission's budget framing: against
  // a 100 mrem budget, 2.0 mrem/hr is 0.03% of budget per minute -- the robot
  // could dwell there for 50 hours before exhausting it -- yet the point-gate
  // foreclosed such frontiers, and everything behind them, permanently
  // (observed directly: a ~16 mrem/hr spawn painted every frontier past the
  // gate and the mission declared itself complete at 0.0m traveled).
  // Affordability against the actual budget is enforced by the transit-dose
  // gate further down; the point ceiling only rejects destinations that
  // would immediately re-trigger the acute yield/recovery cascade on arrival.
  double radiation_at_centroid = 0.0;
  double rad_cost = 0;
  double seek_bonus = 0.0;
  if (!heatmap.empty()) {
    unsigned int mx, my;
    // Check lethal point ceiling at centroid (computed once, reused below)
    bool centroid_in_map = costmap_->worldToMap(
      frontier.centroid.x, frontier.centroid.y, mx, my);
    if (centroid_in_map) {
      int row = (heatmap.rows - 1) - my;
      if (mx < (unsigned int)heatmap.cols && row >= 0 && row < heatmap.rows) {
        float radiation = heatmap.at<float>(row, mx);
        if (radiation >= lethal_point_ceiling_mrem_hr) {return std::numeric_limits<double>::infinity();}
      }
    }
    {
      unsigned int mmx, mmy;
      if (costmap_->worldToMap(frontier.middle.x, frontier.middle.y, mmx, mmy)) {
        int row = (heatmap.rows - 1) - mmy;
        if (mmx < (unsigned int)heatmap.cols && row >= 0 && row < heatmap.rows) {
          float radiation = heatmap.at<float>(row, mmx);
          if (radiation >= lethal_point_ceiling_mrem_hr) {return std::numeric_limits<double>::infinity();}
        }
      }
    }

    if (centroid_in_map) {
      int h_x = mx;
      int h_y = my;

      if (h_x >= 0 && h_x < heatmap.cols && h_y >= 0 && h_y < heatmap.rows) {
        // Map h_y (bottom-up) to row index (top-down)
        int row = (heatmap.rows - 1) - h_y;
        float radiation = heatmap.at<float>(row, h_x);
        radiation_at_centroid = static_cast<double>(radiation);

        if (radiation >= lethal_point_ceiling_mrem_hr) {
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

        // Radiation-seeking bonus (information-gain framing, Request: make
        // frontier selection actually radiation-driven rather than only
        // radiation-averse). Rewards frontiers that are BOTH elevated above
        // background AND still uncertain in the GP's own estimate, so the
        // robot is drawn to linger near an unconfirmed source long enough to
        // actually localize it, rather than brushing past it once on the way
        // to the next spatial frontier. Deliberately fades to ~0 once a
        // region is either back at background (elevation term vanishes,
        // nothing there) or the GP has converged on it (std term vanishes,
        // already characterized) -- it cannot indefinitely trap the robot
        // orbiting an already-found source, and it never touches the
        // lethal-point or dose-budget hard gates above, which remain the
        // final word on safety regardless of this term. rad_cost's own
        // quadratic-above-threshold growth naturally outpaces this bonus's
        // linear growth as radiation climbs, so the net effect tapers off
        // well before the hard ceiling -- seek moderate, uncharacterized
        // signal; let the existing avoidance machinery still win as it gets
        // genuinely dangerous.
        if (radiation_seek_scale != 0.0 && !heatmap_std.empty() &&
          h_x >= 0 && h_x < heatmap_std.cols && h_y >= 0 && h_y < heatmap_std.rows)
        {
          float std_at_centroid = heatmap_std.at<float>(row, h_x);
          double elevation = std::max(0.0, radiation_at_centroid - warning_thresh);
          seek_bonus = -radiation_seek_scale * elevation * static_cast<double>(std_at_centroid);
          // Cap at 25% of this frontier's own distance cost (mirrors
          // size_gain's cap): can reorder preference among already-safe,
          // already-affordable frontiers, but a far, merely-curious
          // frontier can never beat a close one purely on radiological
          // interest.
          double max_seek_benefit = potential_scale_ * frontier.min_distance * 0.25;
          seek_bonus = std::max(seek_bonus, -max_seek_benefit);

          // Deny the bonus entirely near a recent stuck-escape location.
          // Observed directly (c06_r04_adapt10v3, 2026-07-21): amplifying
          // attraction toward an elevated-but-uncertain frontier while the
          // robot is stalled can reinforce a goal on the far side of a
          // physical obstacle it has already failed to get past repeatedly,
          // rather than letting the stuck-escape/blacklist machinery route
          // around it -- the feedback loop that wedged the robot for the
          // back half of that mission (31 stuck-escape events over ~800s,
          // then fully immobile for the rest of it). trap_avoid_radius_m
          // == 0.0 (default) disables this and reproduces prior behavior.
          if (trap_avoid_radius_m > 0.0) {
            for (const auto & trap : recent_trap_locations) {
              double tdx = frontier.centroid.x - trap.x;
              double tdy = frontier.centroid.y - trap.y;
              if (tdx * tdx + tdy * tdy < trap_avoid_radius_m * trap_avoid_radius_m) {
                seek_bonus = 0.0;
                break;
              }
            }
          }
        }
      }
    }
  }

  // Dose-budget feasibility and efficiency cost.
  // The affordability question that actually matters for coverage: what
  // dose does *visiting* this frontier cost, and is that an acceptable
  // spend of what remains? Round trip (out and back), since the mission
  // must still come home within budget. frontier_spend_fraction caps how
  // much of the remaining budget a single frontier visit may consume; the
  // caller scales it with the dose-budget-relaxation coefficient k, making
  // k the "how freely may budget be spent per target" knob rather than
  // only a retreat-trigger nudge.
  double dose_budget_cost = 0.0;
  if (remaining_dose_budget < std::numeric_limits<double>::infinity()) {
    constexpr double robot_speed_mps = 0.5;  // conservative estimate
    double round_trip_time_hr = (2.0 * frontier.min_distance / robot_speed_mps) / 3600.0;
    double expected_transit_dose = radiation_at_centroid * round_trip_time_hr;
    if (expected_transit_dose > frontier_spend_fraction * remaining_dose_budget) {
      return std::numeric_limits<double>::infinity();
    }
    if (remaining_dose_budget > 0.0 && expected_transit_dose > 0.0) {
      dose_budget_cost = dose_budget_scale * (expected_transit_dose / remaining_dose_budget);
    }
  }

  // Orientation Cost (Trajectory/Orientation Awareness)
  // Two-tier priority: every frontier in the forward hemisphere (within ±90° of heading)
  // ranks ahead of every rear-hemisphere frontier, regardless of distance.
  // Within each tier, the base cost (distance, gain, etc.) determines ranking.
  // Rear frontiers are only selected when no forward frontier is reachable.
  // Determine the orientation to this frontier using the actual initial Dijkstra step
  // direction, which reflects the true first move the robot must make regardless of
  // where the frontier's centroid or middle point happen to lie geometrically.
  // This correctly handles warehouse aisles and any environment where obstacles force
  // the path to initially diverge from the straight-line direction to the frontier.
  // Falls back to direction-to-middle (closest accessible point) if path heading is
  // unavailable (e.g. frontier adjacent to robot start).
  double angle_to_frontier;
  if (std::abs(frontier.path_heading_x) > 1e-6 || std::abs(frontier.path_heading_y) > 1e-6) {
    angle_to_frontier = std::atan2(frontier.path_heading_y, frontier.path_heading_x);
  } else {
    angle_to_frontier = std::atan2(
      frontier.middle.y - robot_pos.y,
      frontier.middle.x - robot_pos.x);
  }
  double angle_diff = angle_to_frontier - robot_yaw;
  // Normalize angle_diff to [-PI, PI]
  while (angle_diff > M_PI) {angle_diff -= 2.0 * M_PI;}
  while (angle_diff < -M_PI) {angle_diff += 2.0 * M_PI;}
  angle_diff = std::abs(angle_diff);  // now in [0, PI]

  double orientation_penalty = 0;
  if (angle_diff > M_PI / 2.0) {
    // Rear hemisphere (>90° from heading): hard tier offset guarantees every forward
    // frontier ranks first. 1e6 >> any realistic base cost (potential_scale * world_size).
    orientation_penalty = 1e6;
  } else if (orientation_scale_ > 0) {
    // Forward hemisphere: light smooth penalty to mildly prefer straight-ahead.
    double t = (1.0 - std::cos(angle_diff)) / 2.0;  // 0 at 0°, 0.5 at 90°
    orientation_penalty = orientation_scale_ * t * t * 4.0;
  }

  // Battery Penalty (Safety Awareness)
  // Only apply when battery modelling is enabled and we have a valid reading.
  double battery_penalty = 0;
  if (battery_penalty_scale > 0 && energy_per_meter > 0.0 && batt_valid) {
    // Reduce the influence of dist_to_home unless necessary to avoid "magnetic" pull to start point
    double home_factor = (batt_pct < 0.5f) ? 1.0 : 0.2;
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

  return original_cost + info_gain_cost + rad_cost + seek_bonus + dose_budget_cost +
         orientation_penalty + battery_penalty + hysteresis_bonus;
}
}  // namespace frontier_exploration
