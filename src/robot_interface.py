import rclpy
from std_msgs.msg import String
import time

def send_feedback_to_robot(is_correct):
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('robot_interface')
    publisher = node.create_publisher(String, 'face_state', 10)
    
    # Publish 'happy' or 'sad'
    msg = String()
    if is_correct:
        msg.data = 'happy'
    else:
        msg.data = 'sad'
    
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    
    # Delay to allow happy or sad face to show for 3 seconds
    time.sleep(3)

    # Reset to 'neutral' after delay
    msg.data = 'neutral'
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
