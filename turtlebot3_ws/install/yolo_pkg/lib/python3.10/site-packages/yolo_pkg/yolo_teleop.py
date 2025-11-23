#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pynput import keyboard
import time

# Global variable to track detection state and toggle when 't' is pressed
g_is_detecting = True
g_key_pressed = None

class YoloTeleop(Node):
    def __init__(self):
        super().__init__('yolo_teleop_node')
        
        # 1. Create a service client for the YOLO toggle service
        self.cli = self.create_client(SetBool, '/yolo/toggle_detection')
        
        # Wait until the service server (in yolo_detector.py) is ready
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('YOLO toggle service not available, waiting...')

        self.get_logger().info('YOLO Toggle Teleop Node is ready.')
        self.print_status()

        # 2. Create the non-blocking keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
        # 3. Create a timer to check for key presses periodically
        self.create_timer(0.1, self.timer_callback)

    def print_status(self):
        global g_is_detecting
        status = "ENABLED" if g_is_detecting else "DISABLED"
        print(f"\n======================================")
        print(f"|  YOLO DETECTION STATUS: {status:<15} |")
        print(f"|  Press 'T' to Toggle Detection   |")
        print(f"|  Press Ctrl+C to Exit Node       |")
        print(f"======================================")

    def on_press(self, key):
        global g_key_pressed
        
        # We only care about the key press if the key is 't'
        try:
            if key.char == 't' or key.char == 'T':
                g_key_pressed = key.char
        except AttributeError:
            # Handle special keys if needed (e.g., key.esc)
            pass

    def timer_callback(self):
        global g_is_detecting, g_key_pressed
        
        # Check if the 't' key was pressed since the last check
        if g_key_pressed == 't' or g_key_pressed == 'T':
            # 1. Calculate the new state
            new_state = not g_is_detecting
            
            # 2. Prepare and send the service request
            request = SetBool.Request()
            request.data = new_state
            
            # This is an asynchronous call (it doesn't block the node)
            future = self.cli.call_async(request)
            
            # Reset the key flag immediately to avoid multiple toggles
            g_key_pressed = None 
            
            # 3. Update the internal state and print status once the service request is sent
            g_is_detecting = new_state
            self.print_status()

    def destroy_node(self):
        self.listener.stop() # Stop the keyboard listener thread
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    yolo_teleop = YoloTeleop()
    try:
        rclpy.spin(yolo_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
