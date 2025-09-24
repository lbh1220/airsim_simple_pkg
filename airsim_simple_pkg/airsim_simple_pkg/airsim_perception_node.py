#!/usr/bin/env python3

"""
AirSim Perception Node

A ROS2 node that integrates with AirSim to provide:
1. High-frequency drone pose publishing via odometry topic
2. Low-frequency point cloud publishing with FOV-based filtering
3. Map management through AirSimMapManager
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import airsim

# ROS2 message types
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import std_msgs.msg

# TF2 for coordinate transformations
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Import the map manager
from .map_manager import AirSimMapManager


class AirSimPerceptionNode(Node):
    """
    ROS2 node for AirSim perception and mapping integration
    """
    
    def __init__(self):
        super().__init__('airsim_perception_node')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize AirSim client
        self._initialize_airsim_client()
        
        # Initialize map manager
        self._initialize_map_manager()
        
        # Initialize ROS2 publishers
        self._initialize_publishers()
        
        # Initialize timers for different publishing rates
        self._initialize_timers()
        
        # TF broadcaster for coordinate frame transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.last_pose = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x,y,z,w]
        
        self.get_logger().info('AirSim Perception Node initialized successfully')
        self.get_logger().info(f'Point cloud output frame mode: {self.pointcloud_output_frame}')
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values"""
        
        # AirSim connection parameters
        self.declare_parameter('airsim.drone_name', 'Drone1')
        
        # Publishing frequencies
        self.declare_parameter('frequencies.pose_publish_rate', 50.0)
        self.declare_parameter('frequencies.pointcloud_publish_rate', 5.0)
        
        # Perception parameters
        self.declare_parameter('perception.horizontal_fov', 90.0)
        # self.declare_parameter('perception.vertical_fov', 60.0) # cancel vertical fov, need to calculate ray casting for vertical fov
        self.declare_parameter('perception.detection_range', 100.0)
        self.declare_parameter('perception.pointcloud_output_frame', 'local')
        
        # Map parameters
        self.declare_parameter('simulation.center_point_in_airsim', [0.0, 0.0, 0.0])
        self.declare_parameter('simulation.area_bounds.xmin', -1000.0)
        self.declare_parameter('simulation.area_bounds.xmax', 1000.0)
        self.declare_parameter('simulation.area_bounds.ymin', -1000.0)
        self.declare_parameter('simulation.area_bounds.ymax', 1000.0)
        self.declare_parameter('simulation.area_bounds.zmin', -1000.0)
        self.declare_parameter('simulation.area_bounds.zmax', 1000.0)
        self.declare_parameter('simulation.area_bounds.resolution', 5.0)
        self.declare_parameter('simulation.area_bounds.grid_size', 1.0)
        self.declare_parameter('simulation.map_dir', 'maps')
        self.declare_parameter('simulation.map_file_name', 'map_cloud.npy')
        
        # Topic names
        self.declare_parameter('topics.odom_topic', '/airsim/odom')
        self.declare_parameter('topics.pointcloud_topic', '/airsim/pointcloud')
        
        # Frame IDs
        self.declare_parameter('frames.world_frame', 'world')
        self.declare_parameter('frames.base_frame', 'base_link')
    
    def _get_parameters(self):
        """Get all parameter values"""
        
        # AirSim settings
        self.drone_name = self.get_parameter('airsim.drone_name').get_parameter_value().string_value
        
        # Frequencies
        self.pose_rate = self.get_parameter('frequencies.pose_publish_rate').get_parameter_value().double_value
        self.pointcloud_rate = self.get_parameter('frequencies.pointcloud_publish_rate').get_parameter_value().double_value
        
        # Perception parameters
        self.h_fov = math.radians(self.get_parameter('perception.horizontal_fov').get_parameter_value().double_value)
        # self.v_fov = math.radians(self.get_parameter('perception.vertical_fov').get_parameter_value().double_value)
        # self.get_logger().info(f'h_fov: {self.h_fov}, v_fov: {self.v_fov}')
        self.detection_range = self.get_parameter('perception.detection_range').get_parameter_value().double_value
        self.pointcloud_output_frame = self.get_parameter('perception.pointcloud_output_frame').get_parameter_value().string_value
        
        # Topic names
        self.odom_topic = self.get_parameter('topics.odom_topic').get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter('topics.pointcloud_topic').get_parameter_value().string_value
        
        # Frame IDs
        self.world_frame = self.get_parameter('frames.world_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('frames.base_frame').get_parameter_value().string_value

        
        # Build configuration dictionary for map manager
        self.map_config = {
            'simulation': {
                'center_point_in_airsim': self.get_parameter('simulation.center_point_in_airsim').get_parameter_value().double_array_value,
                'area_bounds': {
                    'xmin': self.get_parameter('simulation.area_bounds.xmin').get_parameter_value().double_value,
                    'xmax': self.get_parameter('simulation.area_bounds.xmax').get_parameter_value().double_value,
                    'ymin': self.get_parameter('simulation.area_bounds.ymin').get_parameter_value().double_value,
                    'ymax': self.get_parameter('simulation.area_bounds.ymax').get_parameter_value().double_value,
                    'zmin': self.get_parameter('simulation.area_bounds.zmin').get_parameter_value().double_value,
                    'zmax': self.get_parameter('simulation.area_bounds.zmax').get_parameter_value().double_value,
                    'resolution': self.get_parameter('simulation.area_bounds.resolution').get_parameter_value().double_value,
                    'grid_size': self.get_parameter('simulation.area_bounds.grid_size').get_parameter_value().double_value,
                },
                'map_dir': self.get_parameter('simulation.map_dir').get_parameter_value().string_value,
                'map_file_name': self.get_parameter('simulation.map_file_name').get_parameter_value().string_value,
            }
        }
    
    def _initialize_airsim_client(self):
        """Initialize connection to AirSim"""
        try:
            self.airsim_client = airsim.MultirotorClient()
            self.airsim_client.confirmConnection()
            self.airsim_client.enableApiControl(True, self.drone_name)
            self.get_logger().info(f'Controlling drone: {self.drone_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to AirSim: {str(e)}')
            raise
    
    def _initialize_map_manager(self):
        """Initialize the map manager"""
        try:
            self.map_manager = AirSimMapManager(self.map_config, client=self.airsim_client)
            # Try to load existing map, if not available it will be created later
            self.map_manager.read_map_from_local()
            self.get_logger().info('Map manager initialized successfully')
            x_points = self.map_manager.cloud_data[:, 0]
            y_points = self.map_manager.cloud_data[:, 1]
            z_points = self.map_manager.cloud_data[:, 2]
            self.get_logger().info(f'x_points: {x_points.min()}, {x_points.max()}')
            self.get_logger().info(f'y_points: {y_points.min()}, {y_points.max()}')
            self.get_logger().info(f'z_points: {z_points.min()}, {z_points.max()}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize map manager: {str(e)}')
            raise
    
    def _initialize_publishers(self):
        """Initialize ROS2 publishers"""
        
        # Odometry publisher
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )
        
        # Point cloud publisher
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.pointcloud_topic,
            10
        )
        
        self.get_logger().info('Publishers initialized')
    
    def _initialize_timers(self):
        """Initialize timers for different publishing frequencies"""
        
        # High frequency timer for pose publishing
        pose_timer_period = 1.0 / self.pose_rate
        self.pose_timer = self.create_timer(pose_timer_period, self._pose_timer_callback)
        
        # Low frequency timer for point cloud publishing
        pointcloud_timer_period = 1.0 / self.pointcloud_rate
        self.pointcloud_timer = self.create_timer(pointcloud_timer_period, self._pointcloud_timer_callback)
        
        self.get_logger().info(f'Timers initialized - Pose: {self.pose_rate}Hz, PointCloud: {self.pointcloud_rate}Hz')
    
    def _pose_timer_callback(self):
        """High frequency callback for publishing drone pose as odometry"""
        try:
            # Get current pose from AirSim
            kinematics_estimated = self.airsim_client.getMultirotorState(self.drone_name).kinematics_estimated
            
            # Get AirSim pose in NED coordinate system
            # Keep using NED coordinates as required by the workspace
            position = kinematics_estimated.position
            orientation = kinematics_estimated.orientation
            
            # Use AirSim NED coordinates directly: x=North, y=East, z=Down
            ned_position = [position.x_val, position.y_val, position.z_val]
            
            # Use AirSim quaternion directly (NED frame)
            ned_orientation = [orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val]
            
            # Update state variables
            self.current_position = np.array(ned_position)
            self.current_orientation = np.array(ned_orientation)
            
            # Create and publish odometry message
            odom_msg = Odometry()
            odom_msg.header = Header()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.world_frame
            odom_msg.child_frame_id = self.base_frame
            
            # Set pose (NED coordinates)
            odom_msg.pose.pose.position.x = ned_position[0]
            odom_msg.pose.pose.position.y = ned_position[1]
            odom_msg.pose.pose.position.z = ned_position[2]
            
            odom_msg.pose.pose.orientation.x = ned_orientation[0]
            odom_msg.pose.pose.orientation.y = ned_orientation[1]
            odom_msg.pose.pose.orientation.z = ned_orientation[2]
            odom_msg.pose.pose.orientation.w = ned_orientation[3]
            
            # Get velocity information (already from kinematics_estimated above)
            velocity_data = kinematics_estimated.linear_velocity
            angular_velocity_data = kinematics_estimated.angular_velocity
            
            # Use AirSim NED velocity directly: x=North, y=East, z=Down
            ned_linear_velocity = [velocity_data.x_val, velocity_data.y_val, velocity_data.z_val]
            ned_angular_velocity = [angular_velocity_data.x_val, angular_velocity_data.y_val, angular_velocity_data.z_val]
            
            odom_msg.twist.twist.linear.x = ned_linear_velocity[0]
            odom_msg.twist.twist.linear.y = ned_linear_velocity[1]
            odom_msg.twist.twist.linear.z = ned_linear_velocity[2]
            
            odom_msg.twist.twist.angular.x = ned_angular_velocity[0]
            odom_msg.twist.twist.angular.y = ned_angular_velocity[1]
            odom_msg.twist.twist.angular.z = ned_angular_velocity[2]
            
            # Publish odometry
            self.odom_publisher.publish(odom_msg)
            
            # Debug logging
            self.get_logger().debug(f'Published odom: pos=({ned_position[0]:.2f}, {ned_position[1]:.2f}, {ned_position[2]:.2f})')
            
            # Publish TF transform
            self._publish_tf_transform(ned_position, ned_orientation)
            
        except Exception as e:
            self.get_logger().error(f'Error in pose timer callback: {str(e)}')
    
    def _pointcloud_timer_callback(self):
        """Low frequency callback for publishing filtered point cloud"""
        try:
            if self.map_manager.cloud_data is None:
                # Try to load map data if not available
                if self.map_manager.read_map_from_local() is None:
                    self.get_logger().warn('No map data available, attempting to get from AirSim...')
                    # self.map_manager.get_map_from_airsim()
                    # if self.map_manager.cloud_data is None:
                    #     self.get_logger().error('Failed to get map data')
                    #     return
            
            # Get current drone pose
            if self.current_position is None:
                self.get_logger().warn('No current position available for point cloud filtering')
                return
            
            # Filter point cloud based on FOV and detection range
            filtered_points = self._filter_pointcloud_by_fov()
            
            if len(filtered_points) == 0:
                self.get_logger().debug('No points in current FOV')
                return
            
            # Create and publish point cloud message
            pointcloud_msg = self._create_pointcloud_message(filtered_points)
            self.pointcloud_publisher.publish(pointcloud_msg)
            
            self.get_logger().debug(f'Published point cloud with {len(filtered_points)} points')
            self.get_logger().debug(f"first point: {filtered_points[0]}")
            self.get_logger().info(f'Point cloud published: {len(filtered_points)} points in current FOV')
            
        except Exception as e:
            self.get_logger().error(f'Error in point cloud timer callback: {str(e)}')
    
    def _filter_pointcloud_by_fov(self):
        """
        Filter point cloud based on current drone position, full orientation, and FOV parameters
        Uses map data that is already in NED coordinates (consistent with AirSim)
        Returns filtered points in either vehicle local coordinates or world coordinates
        depending on the pointcloud_output_frame parameter
        """
        
        # Get drone position and orientation (NED coordinates)
        drone_pos = self.current_position
        drone_quat = self.current_orientation
        
        # Use map_manager point cloud directly (already in NED coordinates)
        all_points = self.map_manager.cloud_data
        
        # Early filtering by distance to reduce computation load
        # Calculate distances in world frame first (cheaper than full transformation)
        world_distances = np.linalg.norm(all_points - drone_pos, axis=1)
        distance_mask = world_distances <= self.detection_range
        
        # Only process points within detection range
        if not np.any(distance_mask):
            return np.array([]).reshape(0, 3)
        
        nearby_points = all_points[distance_mask]
        
        # Get full rotation matrix from quaternion (includes pitch, roll, yaw)
        rotation_matrix = self._quaternion_to_rotation_matrix(drone_quat)
        
        # Transform points to vehicle local frame
        # 1. Translate to drone position
        relative_points = nearby_points - drone_pos
        
        # 2. Rotate to vehicle body frame (inverse rotation)
        local_frame_points = np.dot(relative_points, rotation_matrix.T)
        
        # Filter by FOV in local frame
        # In vehicle frame: x=forward, y=right, z=down
        
        # Filter by horizontal FOV (angle in xy plane)
        horizontal_angles = np.arctan2(local_frame_points[:, 1], local_frame_points[:, 0])
        h_fov_mask = np.abs(horizontal_angles) <= (self.h_fov / 2.0)
        
        # Filter by vertical FOV (angle in xz plane)
        # vertical_angles = np.arctan2(-local_frame_points[:, 2], local_frame_points[:, 0]) # this is not correct
        # v_fov_mask = np.abs(vertical_angles) <= (self.v_fov / 2.0)
        
        # Only consider points in front of the vehicle (positive x in local frame)
        # forward_mask = local_frame_points[:, 0] > 0
        
        # Combine all FOV masks
        # fov_mask = h_fov_mask & v_fov_mask
        fov_mask = h_fov_mask
        
        # Get filtered points in local frame
        filtered_local_points = local_frame_points[fov_mask]
        
        # Return points in the requested coordinate frame
        if self.pointcloud_output_frame == "world":
            # Transform back to world frame
            # 1. Rotate back to world frame (forward rotation)
            world_relative_points = np.dot(filtered_local_points, rotation_matrix)
            
            # 2. Translate back to world coordinates
            filtered_world_points = world_relative_points + drone_pos
            
            return filtered_world_points
        else:
            # Return points in vehicle local coordinates (default behavior)
            return filtered_local_points
    
    def _quaternion_to_rotation_matrix(self, quaternion):
        """
        Convert quaternion to full 3D rotation matrix
        
        Args:
            quaternion: [x, y, z, w] quaternion
            
        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = quaternion
        
        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return np.eye(3)
        
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Convert to rotation matrix
        rotation_matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        
        return rotation_matrix
    
    def _quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (kept for compatibility)"""
        x, y, z, w = quaternion
        
        # Calculate yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def _create_pointcloud_message(self, points):
        """Create a ROS2 PointCloud2 message from numpy points array"""
        
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        
        # Set frame_id based on output coordinate frame
        if self.pointcloud_output_frame == "world":
            header.frame_id = self.world_frame  # Use world_frame for world coordinates
        else:
            header.frame_id = self.base_frame  # Use base_frame for local coordinates
        
        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Convert points to bytes
        points_float32 = points.astype(np.float32)
        point_data = points_float32.tobytes()
        
        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12  # 3 * 4 bytes (x, y, z as float32)
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
        pointcloud_msg.data = point_data
        pointcloud_msg.is_dense = True
        
        return pointcloud_msg
    
    def _publish_tf_transform(self, position, orientation):
        """Publish TF transform between world and base_link frames"""
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Main function to run the AirSim perception node"""
    
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        node = AirSimPerceptionNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error running AirSim perception node: {str(e)}')
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
