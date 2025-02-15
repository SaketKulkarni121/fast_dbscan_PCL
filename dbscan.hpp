#pragma once

#include <cassert>
#include <cstddef>
#include <span>
#include <vector>
#include <cstdlib>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/dbscan.h>

// Forward declaration of dbscan function
auto dbscan(const std::span<const pcl::PointXYZ>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
