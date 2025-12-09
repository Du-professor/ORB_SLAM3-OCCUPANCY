# Occupancy Grid Mapping ROS 2 Node

`occupancy_grid_map` turns the filtered ORB-SLAM3 point cloud into a lightweight 2D occupancy grid that downstream navigation stacks can consume. The node runs as a regular rclcpp component (single executable) and only needs a single point cloud input.

## Key features

- **Single-topic pipeline** – consumes the filtered SLAM cloud published by `orbslam3_dense_ros2` (default: `/slam/filtered_cloud`).
- **TF-aware projection** – automatically transforms every cloud into the requested `target_frame` using a built-in `tf2_ros::Buffer`.
- **Log-odds grid** – maintains a probability buffer with configurable hit/miss increments, occupancy thresholds, and optional decay back to “unknown”.
- **Dynamic throttling** – point clouds are queued on reception and processed in batches at the configured publish frequency, preventing callback spikes.
- **Built-in downsampling** – optional voxel grid filtering keeps CPU usage predictable even with dense SLAM maps.

## Parameters (excerpt)

| Name | Description | Default |
| --- | --- | --- |
| `input_cloud_topic` | Topic name of the filtered SLAM cloud. | `/slam/filtered_cloud` |
| `target_frame` | Frame to express the grid and points in. | `odom` |
| `map_length_x`, `map_length_y` | Physical size of the grid (meters). | `40.0` |
| `resolution` | Cell size (meters). | `0.1` |
| `min_z`, `max_z` | Vertical crop range. | `[-1.5, 2.0]` |
| `min_range`, `max_range` | Radial crop range in XY plane. | `[0.2, 30.0]` |
| `log_odds_hit`, `log_odds_miss` | Increment/decrement applied per measurement. | `0.85 / 0.4` |
| `occupied_threshold`, `free_threshold` | Convert log-odds to OccupancyGrid values. | `0.7 / 0.3` |
| `decay_rate` | Optional log-odds decay per second back toward unknown. | `0.0` |
| `voxel_leaf_size` | VoxelGrid leaf size used before projecting. | `0.1` |

All parameters can be overridden via the YAML file (`config/occupancy_grid_map.param.yaml`) or directly in a launch description.

## Building

```bash
colcon build --packages-select occupancy_grid_map
```

## Running

```bash
ros2 launch occupancy_grid_map occupancygrid.launch.py \
  config:=/path/to/occupancy_grid_map.param.yaml
```

By default the node listens to `/slam/filtered_cloud` and publishes `nav_msgs/OccupancyGrid` on `slam/occupancy_grid`. Update the YAML file (or pass parameters inline) to integrate with a different frame or topic layout.

