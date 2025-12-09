#include "orbslam3_dense_ros2/ros2_slam_publisher.h"
#include <chrono>
#include <sensor_msgs/image_encodings.hpp>


namespace ORB_SLAM3 {

ROS2SlamPublisher::ROS2SlamPublisher(SlamDataProcess* slam_data_processor)
    : slam_data_processor_(slam_data_processor)
    , finish_requested_(false)
    , finished_(true)
    , publish_rate_hz_(10.0)
    , map_pointcloud_rate_hz_(5.0)
    , camera_frame_("camera")
    , ground_frame_("map")
    , vehicle_frame_("vehicle")
    , target_map_frame_("map")
    , z_min_filter_(-0.5)
    , z_max_filter_(1.5)
    , voxel_leaf_size_(0.1)
    , project_to_2d_(true)
    , enable_outlier_removal_(false)
    , outlier_mean_k_(30)
    , outlier_stddev_mul_(1.0)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("orbslam3_ros2_publisher");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    LoadROSParameters();
}

ROS2SlamPublisher::~ROS2SlamPublisher()
{
    RequestFinish();
    
    if (pose_thread_.joinable()) pose_thread_.join();
    if (pointcloud_thread_.joinable()) pointcloud_thread_.join();
    if (pointcloud_mapping_thread_.joinable()) pointcloud_mapping_thread_.join();
    if (frame_thread_.joinable()) frame_thread_.join();
    if (trajectory_thread_.joinable()) trajectory_thread_.join();
}

void ROS2SlamPublisher::LoadROSParameters()
{
    // Declare and get ROS2 parameters
    node_->declare_parameter<double>("publish_rate_hz", 10.0);
    node_->declare_parameter<double>("map_pointcloud_rate_hz", 5.0);
    node_->declare_parameter<std::string>("camera_frame", "camera_link");
    node_->declare_parameter<std::string>("ground_frame", "odom");
    node_->declare_parameter<std::string>("vehicle_frame", "base_footprint");
    node_->declare_parameter<std::string>("target_map_frame", "map");
    node_->declare_parameter<double>("pointcloud_z_min", -0.5);
    node_->declare_parameter<double>("pointcloud_z_max", 1.5);
    node_->declare_parameter<double>("pointcloud_voxel_leaf", 0.1);
    node_->declare_parameter<bool>("pointcloud_project_to_2d", true);
    node_->declare_parameter<bool>("pointcloud_enable_outlier_removal", false);
    node_->declare_parameter<int>("pointcloud_outlier_mean_k", 30);
    node_->declare_parameter<double>("pointcloud_outlier_stddev_mul", 1.0);
    
    publish_rate_hz_ = node_->get_parameter("publish_rate_hz").as_double();
    map_pointcloud_rate_hz_ = node_->get_parameter("map_pointcloud_rate_hz").as_double();
    camera_frame_ = node_->get_parameter("camera_frame").as_string();
    ground_frame_ = node_->get_parameter("ground_frame").as_string();
    vehicle_frame_ = node_->get_parameter("vehicle_frame").as_string();
    target_map_frame_ = node_->get_parameter("target_map_frame").as_string();
    z_min_filter_ = node_->get_parameter("pointcloud_z_min").as_double();
    z_max_filter_ = node_->get_parameter("pointcloud_z_max").as_double();
    voxel_leaf_size_ = node_->get_parameter("pointcloud_voxel_leaf").as_double();
    project_to_2d_ = node_->get_parameter("pointcloud_project_to_2d").as_bool();
    enable_outlier_removal_ = node_->get_parameter("pointcloud_enable_outlier_removal").as_bool();
    outlier_mean_k_ = node_->get_parameter("pointcloud_outlier_mean_k").as_int();
    outlier_stddev_mul_ = node_->get_parameter("pointcloud_outlier_stddev_mul").as_double();
    
    RCLCPP_INFO(node_->get_logger(), "ROS2 Publisher initialized with rate: %.1f Hz", publish_rate_hz_);
}

void ROS2SlamPublisher::Initialize()
{
    // Initialize publishers
    camera_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    vehicle_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose", 10);
    camera_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
    vehicle_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
    all_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_all", 10);
    ref_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_ref", 10);
    map_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cloudPointMapping", 10);
    
    // Initialize image transport and TF broadcaster
    image_transport_ = std::make_shared<image_transport::ImageTransport>(node_);
    frame_pub_ = image_transport_->advertise("current_frame", 1);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);


    ROS2SlamMapToSlamMapStaticTransform();

    RCLCPP_INFO(node_->get_logger(), "ROS2 Publishers initialized");
}

void ROS2SlamPublisher::Run()
{
    finished_ = false;
    
    Initialize();
    
    // Start publishing threads
    pose_thread_ = std::thread(&ROS2SlamPublisher::PublishPoseData, this);
    pointcloud_thread_ = std::thread(&ROS2SlamPublisher::PublishPointCloudData, this);
    pointcloud_mapping_thread_ = std::thread(&ROS2SlamPublisher::PublishPointCloudMappingData, this);
    frame_thread_ = std::thread(&ROS2SlamPublisher::PublishFrameData, this);
    trajectory_thread_ = std::thread(&ROS2SlamPublisher::PublishTrajectoryData, this);
    
    RCLCPP_INFO(node_->get_logger(), "ROS2 Publisher threads started");
}

void ROS2SlamPublisher::ROS2SlamMapToSlamMapStaticTransform()
{
    map_to_slam_map_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node_->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ground_frame_;

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.065;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    map_to_slam_map_broadcaster_->sendTransform(transformStamped);
}

void ROS2SlamPublisher::PublishPoseData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pose_data = slam_data_processor_->GetCurrentPoseData();
        
        if (pose_data.has_new_pose)
        {
            // Publish camera pose
            auto camera_pose_msg = EigenMatrixToPoseStamped(pose_data.cam_pose_to_ground, ground_frame_);
            camera_pose_pub_->publish(camera_pose_msg);
            
            // Publish vehicle pose
            auto vehicle_pose_msg = EigenMatrixToPoseStamped(pose_data.vehicle_pose_to_ground, ground_frame_);
            vehicle_pose_pub_->publish(vehicle_pose_msg);
            
            // Publish TF transform
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = node_->now();
            transform_stamped.header.frame_id = ground_frame_;
            transform_stamped.child_frame_id = vehicle_frame_;

            transform_stamped.transform.translation.x = vehicle_pose_msg.pose.position.x;
            transform_stamped.transform.translation.y = vehicle_pose_msg.pose.position.y;
            transform_stamped.transform.translation.z = vehicle_pose_msg.pose.position.z;
            transform_stamped.transform.rotation = vehicle_pose_msg.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform_stamped);
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishPointCloudMappingData()
{
    auto effective_rate = map_pointcloud_rate_hz_ > 0.0 ? map_pointcloud_rate_hz_ : publish_rate_hz_;
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / effective_rate));
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pc_data = slam_data_processor_->GetCurrentPointCloudMappingData();

        if (pc_data.has_new_map)
        {
            if (pc_data.map_points && pc_data.map_points->points.size() > 0)
            {
                auto filtered = FilterPointCloud(pc_data.map_points);
                if (!filtered || filtered->empty())
                {
                    std::this_thread::sleep_for(period);
                    continue;
                }

                auto intensity_cloud = ConvertToIntensityCloud(filtered);
                if (!intensity_cloud || intensity_cloud->empty())
                {
                    std::this_thread::sleep_for(period);
                    continue;
                }

                auto transformed_cloud = TransformToTargetFrame(intensity_cloud, ground_frame_);
                if (!transformed_cloud)
                {
                    std::this_thread::sleep_for(period);
                    continue;
                }

                sensor_msgs::msg::PointCloud2 map_points_msg;
                pcl::toROSMsg(*transformed_cloud, map_points_msg);
                map_points_msg.header.frame_id = target_map_frame_;
                map_points_msg.header.stamp = node_->now();
                map_points_pub_->publish(map_points_msg);
            }
        }

        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishPointCloudData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(2000.0 / publish_rate_hz_)); // Slower rate for point clouds
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pc_data = slam_data_processor_->GetCurrentPointCloudData();
        
        if (pc_data.has_new_cloud)
        {
            if (pc_data.all_points && pc_data.all_points->points.size() > 0)
            {
                auto all_points_msg = PCLToROS(pc_data.all_points, ground_frame_);
                all_points_pub_->publish(all_points_msg);
            }
            
            if (pc_data.ref_points && pc_data.ref_points->points.size() > 0)
            {
                auto ref_points_msg = PCLToROS(pc_data.ref_points, ground_frame_);
                ref_points_pub_->publish(ref_points_msg);
            }
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishFrameData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto frame_data = slam_data_processor_->GetCurrentFrameData();

        if (frame_data.has_new_frame && !frame_data.current_frame.empty())
        {
            cv_bridge::CvImage cv_image;
            cv_image.header.frame_id = camera_frame_;
            cv_image.header.stamp = node_->now();
            cv_image.encoding = "bgr8";
            cv_image.image = frame_data.current_frame;
            
            sensor_msgs::msg::Image ros_image;
            cv_image.toImageMsg(ros_image);
            frame_pub_.publish(ros_image);
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishTrajectoryData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(5000.0 / publish_rate_hz_)); // Even slower for trajectories
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto traj_data = slam_data_processor_->GetCurrentTrajectoryData();
        
        if (traj_data.has_new_trajectory)
        {
            if (!traj_data.camera_trajectory.empty())
            {
                auto camera_path = TrajectoryToPath(traj_data.camera_trajectory, ground_frame_);
                camera_path_pub_->publish(camera_path);
            }
            
            if (!traj_data.vehicle_trajectory.empty())
            {
                auto vehicle_path = TrajectoryToPath(traj_data.vehicle_trajectory, ground_frame_);
                vehicle_path_pub_->publish(vehicle_path);
            }
        }
        
        std::this_thread::sleep_for(period);
    }
}

geometry_msgs::msg::PoseStamped ROS2SlamPublisher::EigenMatrixToPoseStamped(const Eigen::Matrix4f& matrix, 
                                                                            const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    
    pose_msg.pose.position.x = matrix(0, 3);
    pose_msg.pose.position.y = matrix(1, 3);
    pose_msg.pose.position.z = matrix(2, 3);
    
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    pose_msg.header.frame_id = frame_id;
    pose_msg.header.stamp = node_->now();
    
    return pose_msg;
}

sensor_msgs::msg::PointCloud2 ROS2SlamPublisher::PCLToROS(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
                                                          const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_id;
    ros_cloud.header.stamp = node_->now();
    return ros_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ROS2SlamPublisher::FilterPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) const
{
    if (!input_cloud)
    {
        return nullptr;
    }

    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(z_min_filter_), static_cast<float>(z_max_filter_));
    pass.filter(*filtered_cloud);

    if (voxel_leaf_size_ > 0.0 && !filtered_cloud->empty())
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(filtered_cloud);
        float leaf = static_cast<float>(voxel_leaf_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        voxel.filter(*downsampled);
        filtered_cloud = downsampled;
    }

    if (enable_outlier_removal_ && !filtered_cloud->empty())
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(std::max(outlier_mean_k_, 1));
        sor.setStddevMulThresh(outlier_stddev_mul_);
        auto denoised = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        sor.filter(*denoised);
        filtered_cloud = denoised;
    }

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ROS2SlamPublisher::ConvertToIntensityCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) const
{
    if (!input_cloud)
    {
        return nullptr;
    }

    auto intensity_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    intensity_cloud->reserve(input_cloud->points.size());

    for (const auto& point : input_cloud->points)
    {
        pcl::PointXYZI intensity_point;
        intensity_point.x = point.x;
        intensity_point.y = point.y;
        intensity_point.z = project_to_2d_ ? 0.0f : point.z;
        intensity_point.intensity = (static_cast<float>(point.r) + static_cast<float>(point.g) + static_cast<float>(point.b)) /
            (3.0f * 255.0f);
        intensity_cloud->points.push_back(intensity_point);
    }

    return intensity_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ROS2SlamPublisher::TransformToTargetFrame(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& source_frame)
{
    if (!cloud)
    {
        return nullptr;
    }

    if (target_map_frame_ == source_frame || !tf_buffer_)
    {
        return cloud;
    }

    try
    {
        auto transform_stamped = tf_buffer_->lookupTransform(
            target_map_frame_, source_frame, tf2::TimePointZero);

        Eigen::Isometry3d iso = tf2::transformToEigen(transform_stamped);
        Eigen::Matrix4f tf_matrix = iso.matrix().cast<float>();
        auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*cloud, *transformed_cloud, tf_matrix);
        return transformed_cloud;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Unable to look up transform %s -> %s: %s",
            source_frame.c_str(), target_map_frame_.c_str(), ex.what());
    }

    return nullptr;
}

nav_msgs::msg::Path ROS2SlamPublisher::TrajectoryToPath(const std::vector<Eigen::Matrix4f>& trajectory,
                                                       const std::string& frame_id)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = node_->now();
    
    for (const auto& pose_matrix : trajectory)
    {
        auto pose_stamped = EigenMatrixToPoseStamped(pose_matrix, frame_id);
        path_msg.poses.push_back(pose_stamped);
    }
    
    return path_msg;
}

void ROS2SlamPublisher::RequestFinish()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    finish_requested_ = true;
}

bool ROS2SlamPublisher::IsFinished()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    return finished_;
}

} // namespace ORB_SLAM3