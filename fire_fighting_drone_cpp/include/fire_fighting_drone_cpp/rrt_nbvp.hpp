#ifndef FIRE_FIGHTING_DRONE_CPP__RRT_NBVP_HPP_
#define FIRE_FIGHTING_DRONE_CPP__RRT_NBVP_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace fire_fighting_drone_cpp
{

/// A single node of the RRT exploration tree.
struct RrtNode
{
  octomap::point3d position;
  double yaw{0.0};
  int parent{-1};
  double gain{0.0};           // Information gain attached to this viewpoint.
  double cost_from_root{0.0}; // Euclidean path length accumulated from the root.
  double utility{0.0};        // Accumulated, distance-discounted information gain.
};

/// Parameters controlling the RRT-based Next-Best-View Planner (NBVP).
struct RrtNbvpParams
{
  octomap::point3d bound_min{-5.0F, -15.0F, 0.3F};
  octomap::point3d bound_max{5.0F, 15.0F, 5.0F};
  int max_iterations{300};
  double extension_range{1.5};      // Max RRT edge length (m).
  double collision_check_resolution{0.1}; // Step used while checking an edge (m).
  double sensor_range{5.0};         // Max ray-cast distance for gain (m).
  double horizontal_fov{1.2217};    // ~70 deg, radians.
  double vertical_fov{0.7854};      // ~45 deg, radians.
  int horizontal_rays{5};
  int vertical_rays{3};
  int yaw_samples{12};              // Candidate headings evaluated per node.
  double lambda{0.5};               // Distance-discount factor for utility.
  double gain_threshold{5.0};       // Below this, exploration is considered complete.
  double min_node_separation{0.3};  // Reject samples too close to an existing node.
};

/// Result of a single planning call: the best branch found, root -> best node.
struct RrtNbvpResult
{
  std::vector<RrtNode> path; // path.front() == root, path.back() == best viewpoint.
  double best_utility{0.0};
  double best_gain{0.0};
  bool exploration_complete{false};
};

/// Grows an RRT rooted at the current drone pose over the current OctoMap and
/// returns the branch that maximizes the (distance discounted) information
/// gain, following the receding-horizon NBVP formulation of Bircher et al.
/// (2016), "Receding Horizon Next-Best-View Planner for 3D Exploration".
class RrtNbvp
{
public:
  explicit RrtNbvp(RrtNbvpParams params)
  : params_(params), rng_(std::random_device{}())
  {}

  RrtNbvpResult plan(
    const std::shared_ptr<octomap::OcTree> & octree,
    const octomap::point3d & root_position,
    double root_yaw)
  {
    octree_ = octree;

    std::vector<RrtNode> tree;
    RrtNode root;
    root.position = root_position;
    root.yaw = root_yaw;
    root.parent = -1;
    root.gain = 0.0;
    root.cost_from_root = 0.0;
    root.utility = 0.0;
    tree.push_back(root);

    int best_index = 0;
    double best_utility = 0.0;

    for (int i = 0; i < params_.max_iterations; ++i) {
      const octomap::point3d sample = sampleRandomPoint();
      const int nearest_index = findNearest(tree, sample);
      const octomap::point3d new_position =
        steer(tree[nearest_index].position, sample, params_.extension_range);

      if (!withinBounds(new_position)) {
        continue;
      }
      if (tooCloseToExistingNode(tree, new_position)) {
        continue;
      }
      if (!collisionFree(tree[nearest_index].position, new_position)) {
        continue;
      }

      double best_yaw = 0.0;
      const double gain = computeGain(new_position, best_yaw);
      const double edge_cost = (new_position - tree[nearest_index].position).norm();

      RrtNode node;
      node.position = new_position;
      node.yaw = best_yaw;
      node.parent = nearest_index;
      node.gain = gain;
      node.cost_from_root = tree[nearest_index].cost_from_root + edge_cost;
      node.utility = tree[nearest_index].utility + gain * std::exp(-params_.lambda * node.cost_from_root);

      tree.push_back(node);
      const int new_index = static_cast<int>(tree.size()) - 1;
      if (node.utility > best_utility) {
        best_utility = node.utility;
        best_index = new_index;
      }
    }

    RrtNbvpResult result;
    result.best_utility = best_utility;
    result.best_gain = tree[best_index].gain;
    result.exploration_complete = (best_index == 0) || (tree[best_index].gain < params_.gain_threshold);

    // Backtrack from the best node to the root, then reverse to get root->best.
    std::vector<RrtNode> path;
    for (int idx = best_index; idx != -1; idx = tree[idx].parent) {
      path.push_back(tree[idx]);
    }
    std::reverse(path.begin(), path.end());
    result.path = std::move(path);
    return result;
  }

private:
  octomap::point3d sampleRandomPoint()
  {
    std::uniform_real_distribution<double> dx(params_.bound_min.x(), params_.bound_max.x());
    std::uniform_real_distribution<double> dy(params_.bound_min.y(), params_.bound_max.y());
    std::uniform_real_distribution<double> dz(params_.bound_min.z(), params_.bound_max.z());
    return {static_cast<float>(dx(rng_)), static_cast<float>(dy(rng_)), static_cast<float>(dz(rng_))};
  }

  static int findNearest(const std::vector<RrtNode> & tree, const octomap::point3d & sample)
  {
    int nearest = 0;
    double nearest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree.size(); ++i) {
      const double d = (tree[i].position - sample).norm();
      if (d < nearest_dist) {
        nearest_dist = d;
        nearest = static_cast<int>(i);
      }
    }
    return nearest;
  }

  static octomap::point3d steer(
    const octomap::point3d & from, const octomap::point3d & to, double max_extension)
  {
    const octomap::point3d delta = to - from;
    const double dist = delta.norm();
    if (dist <= max_extension || dist < 1e-6) {
      return to;
    }
    return from + (delta * (max_extension / dist));
  }

  bool withinBounds(const octomap::point3d & p) const
  {
    return p.x() >= params_.bound_min.x() && p.x() <= params_.bound_max.x() &&
           p.y() >= params_.bound_min.y() && p.y() <= params_.bound_max.y() &&
           p.z() >= params_.bound_min.z() && p.z() <= params_.bound_max.z();
  }

  bool tooCloseToExistingNode(const std::vector<RrtNode> & tree, const octomap::point3d & p) const
  {
    for (const auto & node : tree) {
      if ((node.position - p).norm() < params_.min_node_separation) {
        return true;
      }
    }
    return false;
  }

  /// An edge is collision free if no *known occupied* voxel lies along it.
  /// Extending into unknown space is allowed (and desired) by design, since
  /// that is precisely what exploration needs to do.
  bool collisionFree(const octomap::point3d & from, const octomap::point3d & to) const
  {
    const octomap::point3d delta = to - from;
    const double dist = delta.norm();
    if (dist < 1e-6) {
      return true;
    }
    const octomap::point3d step = delta * (params_.collision_check_resolution / dist);
    const int n_steps = static_cast<int>(dist / params_.collision_check_resolution);
    octomap::point3d p = from;
    for (int i = 0; i <= n_steps; ++i) {
      const octomap::OcTreeNode * node = octree_->search(p);
      if (node != nullptr && octree_->isNodeOccupied(node)) {
        return false;
      }
      p += step;
    }
    return true;
  }

  /// Estimates the information gain of placing the sensor at `position`,
  /// searching over `yaw_samples` candidate headings and, for each, casting a
  /// grid of rays through the sensor FOV, counting unknown voxels observed
  /// before a known-occupied voxel (or max sensor range) is hit. Returns the
  /// gain of the best heading and writes it to `best_yaw`.
  double computeGain(const octomap::point3d & position, double & best_yaw) const
  {
    double best_gain = 0.0;
    best_yaw = 0.0;

    const double yaw_step = 2.0 * M_PI / params_.yaw_samples;
    const double h_step = (params_.horizontal_rays > 1)
      ? params_.horizontal_fov / (params_.horizontal_rays - 1) : 0.0;
    const double v_step = (params_.vertical_rays > 1)
      ? params_.vertical_fov / (params_.vertical_rays - 1) : 0.0;

    for (int yi = 0; yi < params_.yaw_samples; ++yi) {
      const double yaw = yi * yaw_step;
      double gain = 0.0;

      for (int hi = 0; hi < params_.horizontal_rays; ++hi) {
        const double d_yaw = -params_.horizontal_fov / 2.0 + hi * h_step;
        for (int vi = 0; vi < params_.vertical_rays; ++vi) {
          const double d_pitch = -params_.vertical_fov / 2.0 + vi * v_step;
          const double ray_yaw = yaw + d_yaw;
          const double ray_pitch = d_pitch;
          const octomap::point3d direction(
            static_cast<float>(std::cos(ray_pitch) * std::cos(ray_yaw)),
            static_cast<float>(std::cos(ray_pitch) * std::sin(ray_yaw)),
            static_cast<float>(std::sin(ray_pitch)));
          const octomap::point3d end = position + direction * static_cast<float>(params_.sensor_range);

          std::vector<octomap::point3d> ray;
          octree_->computeRay(position, end, ray);
          for (const auto & p : ray) {
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node == nullptr) {
              gain += 1.0; // Unknown voxel observed: counts toward information gain.
              continue;
            }
            if (octree_->isNodeOccupied(node)) {
              break; // Ray blocked by a known obstacle.
            }
          }
        }
      }

      if (gain > best_gain) {
        best_gain = gain;
        best_yaw = yaw;
      }
    }

    return best_gain;
  }

  RrtNbvpParams params_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::mt19937 rng_;
};

}  // namespace fire_fighting_drone_cpp

#endif  // FIRE_FIGHTING_DRONE_CPP__RRT_NBVP_HPP_
