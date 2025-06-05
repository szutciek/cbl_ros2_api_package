import random
import websocket

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory
import os

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'request', 10)
        self.get_logger().info('Talker node has been started.')

    def publish_message(self, text: str):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')

def start_websocket_client():
    ws.run_forever(ping_interval=30, ping_timeout=10)

def main():
    global wsapp

    rclpy.init()
    talker = Talker()

    def on_message(wsapp, message):
        print(message)
        talker.publish_message(message)
    
    wsapp = websocket.WebSocketApp("wss://fakens.kanapka.eu/wss", on_message=on_message)

    ws_thread = threading.Thread(target=start_websocket_client, daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()