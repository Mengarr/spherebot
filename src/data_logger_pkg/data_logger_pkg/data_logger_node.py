#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Example message types
from sensor_msgs.msg import Imu  # Import the Imu message type
import os
import csv
from collections import defaultdict
import time  # For high-precision elapsed time

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
            '/imu/data': Imu
        }

        self.topics_20hz = {
            '/compensated_heading': Float32
        }

        # Initialize buffers for each topic
        self.data_buffers_5hz = defaultdict(list)
        self.data_buffers_20hz = defaultdict(list)

        # Initialize CSV writers for each topic
        self.csv_files = {}
        self.csv_writers = {}

        # Initialize CSV files and subscribers for 5 Hz topics
        for topic, msg_type in self.topics_5hz.items():
            self.setup_csv_and_subscriber(topic, msg_type, timer_rate='5hz')

        # Initialize CSV files and subscribers for 20 Hz topics
        for topic, msg_type in self.topics_20hz.items():
            self.setup_csv_and_subscriber(topic, msg_type, timer_rate='20hz')

        # Create two timers: one at 5 Hz and another at 20 Hz
        self.timer_5hz = self.create_timer(0.2, self.timer_callback_5hz)   # 5 Hz
        self.timer_20hz = self.create_timer(0.05, self.timer_callback_20hz)  # 20 Hz

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

        # Write the header
        headers = ['elapsed_time', 'data']
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
        data = self.serialize_message(msg)
        
        # Determine which buffer to use based on topic
        if topic in self.topics_5hz:
            self.data_buffers_5hz[topic].append([elapsed_time_formatted, data])
            #self.get_logger().debug(f"Received data on {topic} (5Hz): {data}")
        elif topic in self.topics_20hz:
            self.data_buffers_20hz[topic].append([elapsed_time_formatted, data])
            #self.get_logger().debug(f"Received data on {topic} (20Hz): {data}")
        # else:
            # self.get_logger().warning(f"Received data on unknown topic: {topic}")

    def serialize_message(self, msg):
        """
        Serializes a ROS message into a string for CSV logging.
        Modify this method based on the structure of your messages.
        """
        if isinstance(msg, Imu):
            # Extract orientation (quaternion), angular velocity, and linear acceleration
            orientation = msg.orientation
            angular_velocity = msg.angular_velocity
            linear_acceleration = msg.linear_acceleration

            # Format the data into a structured string
            return (
                f"orientation: [{orientation.x:.4f}, {orientation.y:.4f}, "
                f"{orientation.z:.4f}, {orientation.w:.4f}], "
                f"angular_velocity: [{angular_velocity.x:.4f}, {angular_velocity.y:.4f}, "
                f"{angular_velocity.z:.4f}], "
                f"linear_acceleration: [{linear_acceleration.x:.4f}, "
                f"{linear_acceleration.y:.4f}, {linear_acceleration.z:.4f}]"
            )
        elif isinstance(msg, Float32):
            return str(msg.data)
        else:
            return str(msg)  # Fallback for other message types

    def timer_callback_5hz(self):
        """
        Timer callback running at 5 Hz. Writes buffered data for 5 Hz topics to CSV files.
        """
        #self.get_logger().debug("5 Hz timer triggered")
        self.write_to_csv(self.data_buffers_5hz, '5Hz')

    def timer_callback_20hz(self):
        """
        Timer callback running at 20 Hz. Writes buffered data for 20 Hz topics to CSV files.
        """
        #self.get_logger().debug("20 Hz timer triggered")
        self.write_to_csv(self.data_buffers_20hz, '20Hz')

    def write_to_csv(self, buffer_dict, timer_label):
        """
        Writes the buffered data to their respective CSV files and clears the buffers.
        """
        for topic, buffer in buffer_dict.items():
            if buffer:
                self.csv_writers[topic].writerows(buffer)
                self.csv_files[topic].flush()
                #self.get_logger().info(f"Wrote {len(buffer)} entries to {topic} CSV ({timer_label})")
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
