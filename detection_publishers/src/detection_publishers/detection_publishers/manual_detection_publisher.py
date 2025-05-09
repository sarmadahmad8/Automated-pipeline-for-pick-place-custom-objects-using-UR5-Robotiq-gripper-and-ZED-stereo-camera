#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32
import math
import supervision as sv
import cv2
import pyzed.sl as sl
import numpy as np
import subprocess
import re
import time
from collections import deque
from threading import Thread
import onnxruntime as ort

# Add at the top of your script
drawing = False
bbox = None
start_point = None
end_point = None

def draw_rectangle(event, x, y, flags, param):
    global drawing, start_point, end_point, bbox

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_point = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        end_point = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        end_point = (x, y)
        if start_point and end_point:
            x1, y1 = start_point
            x2, y2 = end_point
            bbox = (int((x1 + x2) / 2), int((y1 + y2) / 2), abs(x2 - x1), abs(y2 - y1))

class TF2EchoReader:
    def __init__(self):
        self.process = None
        self.output_queue = deque(maxlen=10)  # Only keep the last 10 lines (or adjust as needed)
        self.translation_pattern = r'Translation:\s*\[([0-9.-]+),\s*([0-9.-]+),\s*([0-9.-]+)\]'
        self.running = False

    def start_command(self):
        # Start the ROS2 tf2_echo command
        self.process = subprocess.Popen(
            ['ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'virtual_camera','100'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        self.running = True

        # Start a background thread to continuously read the process's output
        Thread(target=self._read_output, daemon=True).start()

    def _read_output(self):
        # Read lines continuously from the command's stdout
        while self.running and self.process.poll() is None:
            line = self.process.stdout.readline()
            if line:
                # Append the new line to the deque (only keeps the latest lines)
                self.output_queue.append(line)

    def get_latest_translation(self):
        # Scan the most recent lines for the latest translation data
        combined_output = "".join(self.output_queue)
        matches = re.findall(self.translation_pattern, combined_output)

        if matches:
            # Get the latest match (last in the list)
            latest_translation = list(map(float, matches[-1]))
            print("Latest Translation:", latest_translation)
            return latest_translation
        else:
            print("No translation data found in recent output.")
            return None

    def stop_command(self):
        # Stop the process and the reading thread
        self.running = False
        if self.process:
            self.process.terminate()
            self.process.wait()

def adjust_x_position(y_pixel, image_height, current_y):

    center_y = image_height / 2  # Middle of the image
    max_offset = 0.0125  # Maximum adjustment at the top or bottom

    # Normalize the distance from the center to a range [-1, 1]
    normalized_distance = (y_pixel - center_y) / center_y

    # Scale the adjustment based on the normalized distance
    y_adjustment = -normalized_distance * max_offset

    return current_y + y_adjustment

def adjust_y_position(y_pixel, image_height, current_y):

    center_y = image_height / 2  # Middle of the image
    max_offset = 0.01  # Maximum adjustment at the top or bottom

    # Normalize the distance from the center to a range [-1, 1]
    normalized_distance = (y_pixel - center_y) / center_y

    # Scale the adjustment based on the normalized distance
    y_adjustment = -normalized_distance * max_offset

    return current_y + y_adjustment

class move_group_publisher(Node):
    def __init__(self):
        super().__init__('move_group_publisher')
        
        # ROS2 Publishers
        self.target_position_pub = self.create_publisher(Pose, 'target_position', 10)
        self.width_pub = self.create_publisher(Float32, 'fruit_width', 10)
        self.distance_pub = self.create_publisher(Float32, 'distance_to_fruit', 10)
        
        # Timer for the main loop (10 Hz)
        self.timer = self.create_timer(0.067, self.detect_fruit_callback)

        # Initialize the Roboflow model
        #self.model = get_model(model_id="fruit-detector-n9ajo/2", api_key="DrB6AbeqwlipTFnq48tb")
        #self.model = get_model(model_id="lemon_kumquat_robust/5", api_key="DrB6AbeqwlipTFnq48tb") 
        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 15
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_minimum_distance = 0.2
        init_params.depth_stabilization = 100

        # Open the camera
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Error opening ZED camera.")
            exit(-1)

        # Initialize image container
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        self.tf2_reader = TF2EchoReader()
        self.tf2_reader.start_command()
        time.sleep(2)

    def get_depth_at_point(self, x, y):
        err, point = self.point_cloud.get_value(int(x), int(y))
        if err == sl.ERROR_CODE.SUCCESS and math.isfinite(point[2]):
            return math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)  # Return depth (Z value)
        return float('inf') 

    def detect_fruit_callback(self):
        # Grab a new frame from the ZED camera
        start_time = time.time()
        cv2.namedWindow("ZED Camera - Manual Select")
        cv2.setMouseCallback("ZED Camera - Manual Select", draw_rectangle)

        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_RGBA2RGB)

            if bbox:
                cx, cy, w, h = bbox
                ledge = cx - w // 2 
                redge = cx + w // 2

                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                err, point_cloud_value = self.point_cloud.get_value(int(cx), int(cy))
                err1, point_cloud_value_ledge = self.point_cloud.get_value(ledge, int(cy))
                err2, point_cloud_value_redge = self.point_cloud.get_value(redge, int(cy))

                if err == sl.ERROR_CODE.SUCCESS and math.isfinite(point_cloud_value[2]):
                    self.translation = self.tf2_reader.get_latest_translation()

                    if self.translation:
                        # Use same logic to calculate target position and publish
                        distance = math.sqrt(
                            point_cloud_value[0] ** 2 +
                            point_cloud_value[1] ** 2 +
                            point_cloud_value[2] ** 2
                        )
                        self.get_logger().info(f"Distance to Camera: {distance:.3f} m")
                        self.distance_pub.publish(Float32(data=round(distance,3)))

                        if math.isfinite(point_cloud_value_ledge[2]) and math.isfinite(point_cloud_value_redge[2]):
                            width_fruit = math.sqrt(
                                (point_cloud_value_ledge[0] - point_cloud_value_redge[0]) ** 2 +
                                (point_cloud_value_ledge[1] - point_cloud_value_redge[1]) ** 2 +
                                (point_cloud_value_ledge[2] - point_cloud_value_redge[2]) ** 2
                            )
                            self.get_logger().info(f"Width of fruit: {width_fruit:.3f} m")
                            self.width_pub.publish(Float32(data=round(width_fruit,3)))

                        tool0_position = [
                            round((point_cloud_value[0]+self.translation[0]),3),
                            round((point_cloud_value[2]+self.translation[1]),3),
                            round((-point_cloud_value[1]+self.translation[2]),3)
                        ]

                        tool0_position[2] = adjust_y_position(int(cy), self.image.get_height(), tool0_position[2])
                        tool0_position[0] = adjust_x_position(int(cx), self.image.get_width(), tool0_position[0])

                        target_position = Pose()
                        target_position.position.x = tool0_position[0]
                        target_position.position.y = tool0_position[1]
                        target_position.position.z = tool0_position[2]

                        self.get_logger().info(f"Target Position: x={target_position.position.x}, y={target_position.position.y}, z={target_position.position.z}")
                        self.target_position_pub.publish(target_position)

            # Draw box on the frame if it's being drawn
            if start_point and end_point:
                cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)

            cv2.imshow("ZED Camera - Manual Select", frame)
            end_time = time.time()
            latency = end_time - start_time
            print(f"Loop latency: {latency:.3f} seconds")

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()

    def __del__(self):
        # Clean up resources
        self.zed.close()
        frame = None
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    move_group_publisher_node = move_group_publisher()

    try:
        rclpy.spin(move_group_publisher_node)
    except KeyboardInterrupt:
        pass

    # Destroy node explicitly if it's still active
    move_group_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()