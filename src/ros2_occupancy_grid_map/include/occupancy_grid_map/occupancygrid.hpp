#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace PerceptionNS
{
  // Enumeration defining the state of a cell in the occupancy grid
  enum CellState : size_t
  {
    FREE,     // Cell is free
    OCCUPIED, // Cell is occupied
    UNKNOWN   // Cell state is unknown
  };

  // Enumeration defining the index of a cell state
  enum Index : size_t
  {
    FREE_cell,    // Index of a free cell
    OCCUPIED_cell // Index of an occupied cell
  };

  // Struct to represent information about a 3D bin
  struct BinInfo3D
  {
    BinInfo3D(
        const double _range = 0.0, const double _wx = 0.0, const double _wy = 0.0,
        const double _wz = 0.0, const double _projection_length = 0.0,
        const double _projected_wx = 0.0, const double _projected_wy = 0.0)
        : range(_range),
          wx(_wx),
          wy(_wy),
          wz(_wz),
          projection_length(_projection_length),
          projected_wx(_projected_wx),
          projected_wy(_projected_wy)
    {
    }
    double range;             // Range
    double wx;                // World X-coordinate
    double wy;                // World Y-coordinate
    double wz;                // World Z-coordinate
    double projection_length; // Projection length
    double projected_wx;      // Projected world X-coordinate
    double projected_wy;      // Projected world Y-coordinate
  };
  double origin_z_, robot_z_;

  // Class for ground segmentor node
  class GroundSegmentor : public rclcpp::Node
  {
  public:
    explicit GroundSegmentor(const rclcpp::NodeOptions &options); // Constructor
    ~GroundSegmentor();

  private:
    struct CloudWithOrigin
    {
      sensor_msgs::msg::PointCloud2::SharedPtr cloud;
      double origin_x{0.0};
      double origin_y{0.0};
    };

    struct CloudPair
    {
      CloudWithOrigin raw;
      CloudWithOrigin obstacle;
    };

    // Callback entry point for synchronized point clouds (push to queue)
    void enqueue_pointcloud_pair(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_pcl_msg_1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_pcl_msg_2);

    void single_pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_pcl_msg);

    // Heavy processing function (formerly synchro callback)
  void process_pointcloud_pair(const CloudPair &pair);

    void processing_loop();
  CloudWithOrigin transformPointCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilterIncomingCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;

    // Helper function for Bresenham's line algorithm
    inline void bresenham2D(
        nav_msgs::msg::OccupancyGrid &map, unsigned int abs_da, unsigned int abs_db, int error_b,
        int offset_a, int offset_b, unsigned int offset, CellState state, unsigned int max_length);

    // Helper function for ray tracing a line
    inline void raytraceLine(nav_msgs::msg::OccupancyGrid &map, unsigned int x0, unsigned int y0, unsigned int x1,
                             unsigned int y1, int size_x, CellState state, unsigned int max_length = UINT_MAX);

    // Function to convert world coordinates to map coordinates
    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const;

    // Function to perform ray tracing
    void raytrace(
        nav_msgs::msg::OccupancyGrid &map, double source_x, double source_y, double target_x, double target_y,
        int size_x, CellState state);

    // Utility function to convert degrees to radians
    constexpr double deg2rad(const double deg) { return deg * M_PI / 180.0; }

    // Utility function to convert radians to degrees
    constexpr double rad2deg(const double rad) { return rad * 180.0 / M_PI; }

    // Member variables
    std::string target_frame_;   // Target frame
    std::vector<double> length_; // Length of the grid
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>
        SyncPolicy;

    typedef message_filters::Synchronizer<SyncPolicy> Sync; // Synchronizer for point clouds

    std::shared_ptr<Sync> sync_; // Shared pointer to Synchronizer

    // TF variables
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool state_ = false;
    float cell_size_, cell_size_recp_, bottomright_x_, bottomright_y_, obstacle_separation_threshold_;
    int cell_num_x_, cell_num_y_, grid_size_, origin_index_x_, origin_index_y_, origin_idx_;

    double min_angle_, max_angle_, angle_increment_, bins_numb_, projection_dz_threshold_, v_ratio_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub1_, pointcloud_sub2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr single_pointcloud_sub_;

    bool use_dual_input_; 
    bool incremental_update_;
    double min_height_filter_;
    double max_height_filter_;
    double voxel_leaf_size_;
    bool project_to_2d_;
    int max_queue_size_;

    std::deque<CloudPair> cloud_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> processing_running_{false};
    std::thread processing_thread_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_; // Publisher for occupancy grid

    nav_msgs::msg::OccupancyGrid grid_msg_; // Occupancy grid message
    std::vector<signed char> grid_template_;
  };
}
