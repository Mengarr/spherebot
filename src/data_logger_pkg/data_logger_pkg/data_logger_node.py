#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
import os
import csv
from collections import defaultdict
import time

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # Record the start time for elapsed time calculation
        self.start_time = time.time()

        # Absolute path to the 'data' directory as a private variable
        self.data_dir = "/home/rohan/spherebot/src/data_logger_pkg/data"
        self.get_logger().info(f"Data directory set to: {self.data_dir}")

        # Define the topics to subscribe to and their message types
        # Categorize topics based on desired logging rate
        self.topics_5hz = {
            '/environment_data/temp_degrees': Float32,
            '/environment_data/pressure_pa': Float32,
            '/gps/point_data': Point
        }

        self.topics_10hz = {
            '/compensated_heading': Float32,
            '/heading_ref': Float32,
            '/motor/joint_vars_state': JointTrajectoryControllerState,
            '/motor/joint_vars': JointTrajectory 
        }

        # Initialize buffers for each topic
        self.data_buffers_5hz = defaultdict(list)
        self.data_buffers_10hz = defaultdict(list)

        # Initialize CSV writers for each topic
        self.csv_files = {}
        self.csv_writers = {}

        # Initialize CSV files and subscribers for 5 Hz topics
        for topic, msg_type in self.topics_5hz.items():
            self.setup_csv_and_subscriber(topic, msg_type, timer_rate='5hz')

        # Initialize CSV files and subscribers for 10 Hz topics
        for topic, msg_type in self.topics_10hz.items():
            self.setup_csv_and_subscriber(topic, msg_type, timer_rate='10hz')

        # Create two timers: one at 5 Hz and another at 10 Hz
        self.timer_5hz = self.create_timer(0.2, self.timer_callback_5hz)   # 5 Hz
        self.timer_10hz = self.create_timer(0.1, self.timer_callback_10hz)  # 10 Hz

    def setup_csv_and_subscriber(self, topic, msg_type, timer_rate):
        """
        Sets up CSV file and subscriber for a given topic.
        """
        filename = f"{topic.strip('/').replace('/', '_')}.csv"
        file_path = os.path.join(self.data_dir, filename)

        # Open CSV file in write mode to clear existing content
        csv_file = open(file_path, 'w', newline='')
        self.csv_files[topic] = csv_file
        csv_writer = csv.writer(csv_file)

        # Define headers based on the topic
        if topic == '/compensated_heading' or topic == '/heading_ref':
            headers = ['elapsed_time', 'data']
        elif topic == '/environment_data/temp_degrees':
            headers = ['elapsed_time', 'temp_degrees']
        elif topic == '/environment_data/pressure_pa':
            headers = ['elapsed_time', 'pressure_pa']
        elif topic == '/gps/point_data':
            headers = ['elapsed_time', 'x', 'y', 'z']
        elif topic == '/motor/joint_vars_state':
            headers = ['elapsed_time', 'u_meas (mm)', 'alphadot_meas (rad/s)', 'phi_meas (rad)']
        elif topic == '/motor/joint_vars':
            headers = ['elapsed_time', 'u_ref (mm)', 'alphadot_ref (rad/s)', 'phi_ref (rad)']
        else:
            headers = ['elapsed_time', 'data']  # Fallback for other topics

        csv_writer.writerow(headers)
        self.csv_writers[topic] = csv_writer

        # Create a subscriber for each topic
        self.create_subscription(
            msg_type,
            topic,
            lambda msg, t=topic: self.listener_callback(msg, t),
            10
        )
        self.get_logger().info(f"Subscribed to {topic} for {timer_rate} logging")

    def listener_callback(self, msg, topic):
        """
        Callback function for subscribed topics. It stores the incoming message
        along with the elapsed time in the buffer corresponding to the topic.
        """
        elapsed_time = time.time() - self.start_time
        elapsed_time_formatted = f"{elapsed_time:.4f}"
        data = self.serialize_message(msg, topic)
        
        # Determine which buffer to use based on topic
        if topic in self.topics_5hz:
            self.data_buffers_5hz[topic].append([elapsed_time_formatted] + data)
        elif topic in self.topics_10hz:
            self.data_buffers_10hz[topic].append([elapsed_time_formatted] + data)

    def serialize_message(self, msg, topic):
        """
        Serializes a ROS message into a list of values for CSV logging.
        Modify this method based on the structure of your messages.
        """
        if isinstance(msg, Imu):
            # Extract orientation (quaternion), angular velocity, and linear acceleration
            orientation = msg.orientation
            angular_velocity = msg.angular_velocity
            linear_acceleration = msg.linear_acceleration

            # Format the data into a structured list
            return [
                orientation.x, orientation.y, orientation.z, orientation.w,
                angular_velocity.x, angular_velocity.y, angular_velocity.z,
                linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
            ]
        elif isinstance(msg, Float32):
            return [msg.data]
        elif isinstance(msg, Point):
            return [msg.x, msg.y, msg.z]
        elif isinstance(msg, JointTrajectoryControllerState):
            u_meas = msg.feedback.positions[0] if len(msg.feedback.positions) > 0 else 0.0
            alphadot_meas = msg.feedback.velocities[1] if len(msg.feedback.velocities) > 1 else 0.0
            phi_meas = msg.feedback.positions[2] if len(msg.feedback.positions) > 2 else 0.0
            return [u_meas, alphadot_meas, phi_meas]
        elif isinstance(msg, JointTrajectory):
            if len(msg.points) > 1:
                u_ref = msg.points[0].positions[0] if len(msg.points[0].positions) > 0 else 0.0
                alphadot_ref = msg.points[0].velocities[1] if len(msg.points[0].velocities) > 1 else 0.0
                phi_ref = msg.points[1].positions[0] if len(msg.points[1].positions) > 0 else 0.0
            else:
                u_ref = 0.0
                alphadot_ref = 0.0
                phi_ref = 0.0
            return [u_ref, alphadot_ref, phi_ref]
        else:
            return [str(msg)]  # Fallback for other message types

    def timer_callback_5hz(self):
        """
        Timer callback running at 5 Hz. Writes buffered data for 5 Hz topics to CSV files.
        """
        self.write_to_csv(self.data_buffers_5hz, '5Hz')

    def timer_callback_10hz(self):
        """
        Timer callback running at 10 Hz. Writes buffered data for 10 Hz topics to CSV files.
        """
        self.write_to_csv(self.data_buffers_10hz, '10Hz')

    def write_to_csv(self, buffer_dict, timer_label):
        """
        Writes the buffered data to their respective CSV files and clears the buffers.
        """
        for topic, buffer in buffer_dict.items():
            if buffer:
                self.csv_writers[topic].writerows(buffer)
                self.csv_files[topic].flush()
                buffer.clear()

    def destroy_node(self):
        """
        Ensure all files are properly closed when the node is destroyed.
        """
        for csv_file in self.csv_files.values():
            csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DataLoggerNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
