#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
# IMPORT THE NECESSARY MODULES TO GET THE ABSOLUTE PATH
from ament_index_python.packages import get_package_share_directory 
from std_srvs.srv import SetBool # Add this import

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # --- New: State Variable ---
        self.detection_enabled = True # Start enabled by default

        # --- New: Toggle Service ---
        self.toggle_service = self.create_service(
            SetBool,
            '/yolo/toggle_detection', # The service name we will call
            self.toggle_detection_callback
        )
        self.get_logger().info('YOLO Detector toggle service is ready.')
        
        # --- Initialization ---
        self.bridge = CvBridge()
        
        # --- Model Loading (CORRECTED) ---
        
        # 1. Get the absolute path to the package's share directory
        pkg_share_dir = get_package_share_directory('yolo_pkg') # Replace 'yolo_pkg' if necessary
        
        # 2. Construct the absolute path to the model file
        # ASSUMES you have installed best.onnx into 'share/yolo_pkg/models/best.onnx' 
        # (via setup.py, as planned)
        model_path = os.path.join(pkg_share_dir, 'models', 'best.onnx') 
        
        self.model = YOLO(model_path) 
        self.get_logger().info(f'YOLO model loaded successfully from: {model_path}')
        
        # --- ROS Sub/Pub ---
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Image, '/yolo/image_detections', 10)

    def image_callback(self, msg):
        # ----------------------------------------------------
        # NEW: Check if detection is disabled
        # If disabled, publish the raw image and skip all inference logic
        if not self.detection_enabled:
            self.publisher.publish(msg) 
            return
        # ----------------------------------------------------

        # 1. Convert ROS Image to OpenCV Image (BGR format)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # 2. Run YOLO Inference
        results = self.model(cv_image, verbose=False) 
        
        if not results:
            # If results are empty (no detections), just plot the raw frame
            r = None # results[0] would crash if results is empty
            annotated_frame = cv_image # Use raw image if no results and we are enabled
        else:
            r = results[0] 
            # 3. Visualize Results: r.plot() draws bounding boxes onto the frame
            annotated_frame = r.plot() 
        
        # 4. Convert Annotated OpenCV Image back to ROS Image
        try:
            ros_output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_output_msg.header = msg.header # Keep the original timestamp/frame_id
        except Exception as e:
            self.get_logger().error(f'cv_bridge output exception: {e}')
            return
            
        # 5. Publish the Annotated Image
        self.publisher.publish(ros_output_msg)
            
    def toggle_detection_callback(self, request, response):
        self.detection_enabled = request.data

        # Set the response for the service client
        response.success = True
        if self.detection_enabled:
            response.message = "YOLO Detection Enabled."
        else:
            response.message = "YOLO Detection Disabled."

        self.get_logger().info(response.message)
        return response
        
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
