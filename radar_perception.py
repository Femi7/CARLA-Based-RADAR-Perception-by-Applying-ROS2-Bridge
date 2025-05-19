import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import carla
from std_msgs.msg import Header
import time

class RadarPerceptionNode(Node):
    def __init__(self):
        super().__init__('radar_perception')

        # Create a ROS2 publisher
        self.radar_publisher = self.create_publisher(PointCloud2, '/carla/radar', 10)

        try:
            # Connect to CARLA
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.get_logger().info("Connected to CARLA server")

            # Spawn a vehicle
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.*')[0]
            spawn_point = self.world.get_map().get_spawn_points()[0]
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

            if self.vehicle is None:
                self.get_logger().error("Failed to spawn vehicle. Check CARLA availability.")
                return
            
            self.get_logger().info(f"Vehicle spawned at {spawn_point}")

            # Attach a RADAR sensor
            radar_bp = blueprint_library.find('sensor.other.radar')
            radar_bp.set_attribute('horizontal_fov', '30.0')
            radar_bp.set_attribute('vertical_fov', '10.0')
            radar_bp.set_attribute('range', '50.0')

            radar_spawn_point = carla.Transform(carla.Location(x=2.5, z=1.0))
            self.radar = self.world.try_spawn_actor(radar_bp, radar_spawn_point, attach_to=self.vehicle)

            if self.radar is None:
                self.get_logger().error("Failed to attach RADAR sensor to vehicle.")
                self.cleanup()
                return

            self.radar.listen(self.radar_callback)
            self.get_logger().info("RADAR sensor successfully attached")
        
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CARLA connection or sensors: {e}")
            self.cleanup()
            raise

    def radar_callback(self, radar_data):
        """
        Callback function to process RADAR data and publish it to ROS2.
        """
        try:
            points = []
            if not radar_data:
                self.get_logger().warn("Received empty radar data.")
                return

            self.get_logger().info(f"Received radar data with {len(radar_data)} detections")

            for detection in radar_data:
                x = detection.depth * np.cos(detection.azimuth)
                y = detection.depth * np.sin(detection.azimuth)
                z = detection.altitude * detection.depth
                velocity = detection.velocity
                points.append((x, y, z, velocity))

            if points:
                self.get_logger().info(f"Publishing {len(points)} RADAR points")
                point_cloud_msg = self.create_point_cloud_msg("radar", points)
                self.radar_publisher.publish(point_cloud_msg)
            else:
                self.get_logger().warn("No valid radar data to publish.")

        except Exception as e:
            self.get_logger().error(f"Error processing RADAR data: {e}")

    def create_point_cloud_msg(self, frame_id, points):
        """
        Convert RADAR points into a ROS2 PointCloud2 message.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # Use ROS2 clock for timestamp
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        point_cloud_data = np.array(points, dtype=np.float32).flatten().tobytes()

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(points)
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 16  # 4x float32 values per point
        point_cloud_msg.row_step = 16 * len(points)  # Total row size
        point_cloud_msg.data = point_cloud_data
        point_cloud_msg.is_dense = True

        return point_cloud_msg

    def cleanup(self):
        """
        Safely destroy spawned actors.
        """
        try:
            if hasattr(self, 'radar') and self.radar:
                self.radar.destroy()
                self.get_logger().info("RADAR sensor destroyed")
            if hasattr(self, 'vehicle') and self.vehicle:
                self.vehicle.destroy()
                self.get_logger().info("Vehicle destroyed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

    def destroy(self):
        self.cleanup()

def main():
    rclpy.init()
    radar_node = None

    try:
        radar_node = RadarPerceptionNode()
        rclpy.spin(radar_node)
    except Exception as e:
        if radar_node:
            radar_node.get_logger().error(f"Node encountered an error: {e}")
    finally:
        if radar_node:
            radar_node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

