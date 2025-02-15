#include "dbscan.hpp"
#include <cstddef>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <algorithm>

// "dataset to kd-tree" adaptor class:
inline auto get_pt(const pcl::PointXYZ& p, std::size_t dim)
{
    if(dim == 0) return p.x;
    if(dim == 1) return p.y;
    if(dim == 2) return p.z;
}

template<typename Point>
struct adaptor
{
    const std::span<const Point>& points;

    adaptor(const std::span<const Point>& points) : points(points) { }

    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
    {
        return get_pt(points[idx], dim);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

    auto const* elem_ptr(const std::size_t idx) const
    {
        return &points[idx].x;
    }
};

auto sort_clusters(std::vector<std::vector<size_t>>& clusters)
{
    for (auto& cluster : clusters)
    {
        std::sort(cluster.begin(), cluster.end());
    }
}

auto dbscan(const std::span<const pcl::PointXYZ>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>
{
    // Use the PCL KdTree
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;

    // Convert std::span to a pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(data.size());
    std::memcpy(cloud.points.data(), data.data(), sizeof(pcl::PointXYZ) * data.size());

    kd_tree.setInputCloud(cloud.makeShared());

    std::vector<bool> visited(data.size(), false);
    std::vector<std::vector<size_t>> clusters;

    for (size_t i = 0; i < data.size(); i++)
    {
        if (visited[i]) continue;

        std::vector<int> neighbors;
        std::vector<float> distances;

        // Find neighbors within eps radius
        kd_tree.radiusSearch(cloud.points[i], eps, neighbors, distances);

        if (neighbors.size() < static_cast<size_t>(min_pts)) continue;

        // Mark the current point as visited
        visited[i] = true;

        std::vector<size_t> cluster = {i};

        // Add all neighbors to the cluster
        for (size_t idx = 0; idx < neighbors.size(); idx++)
        {
            if (!visited[neighbors[idx]])
            {
                visited[neighbors[idx]] = true;
                cluster.push_back(neighbors[idx]);

                // Recursively add all the neighbors of the neighbors
                std::vector<int> sub_neighbors;
                std::vector<float> sub_distances;
                kd_tree.radiusSearch(cloud.points[neighbors[idx]], eps, sub_neighbors, sub_distances);

                if (sub_neighbors.size() >= static_cast<size_t>(min_pts))
                {
                    neighbors.insert(neighbors.end(), sub_neighbors.begin(), sub_neighbors.end());
                }
            }
        }

        clusters.push_back(std::move(cluster));
    }

    sort_clusters(clusters);

    return clusters;
}

int main(int argc, char** argv)
{
    // Usage check
    if (argc != 4)
    {
        std::cerr << "usage: example <tsv file> <epsilon> <min points>\n";
        return 1;
    }

    auto epsilon = std::stof(argv[2]);
    auto min_pts = std::stoi(argv[3]);

    // Read input data (implement your read_values function here)
    auto [values, dim] = read_values(argv[1]);

    // Check if the data is 3D
    if (dim == 3)
    {
        // Call the DBSCAN function with PCL-based KD-Tree
        auto clusters = dbscan(values, epsilon, min_pts);

        // Print the results
        for (const auto& cluster : clusters)
        {
            std::cout << "Cluster: ";
            for (const auto& idx : cluster)
            {
                std::cout << idx << " ";
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cerr << "Error: Only 3D data is supported.\n";
        return 1;
    }

    return 0;
}
