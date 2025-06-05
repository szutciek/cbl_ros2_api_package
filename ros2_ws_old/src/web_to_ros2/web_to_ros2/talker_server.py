# Filename: web_to_ros2/talker_server.py

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import threading
import random

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


class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/':
            try:
                pkg_path = get_package_share_directory('web_to_ros2')
                file_path = os.path.join(pkg_path, 'index.html')
                with open(file_path, 'rb') as file:
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(file.read())
            except FileNotFoundError:
                self.send_error(404, 'File Not Found: index.html')
        else:
            self.send_error(404, 'Not Found')


    def do_POST(self):
        parsed_url = urlparse(self.path)
        path = parsed_url.path
        query_params = parse_qs(parsed_url.query)

        if path == '/request':
            track = query_params.get('track', [None])[0]
            if track:
                location = random.random()
                self.server.talker.publish_message(f'{track}:{location}')
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"Message published\n")
            else:
                self.send_error(400, 'Bad Request: Missing track parameter')
        else:
            self.send_error(404, 'Not Found')


def run_server(talker, port=8080):
    server_address = ('', port)
    handler = SimpleHTTPRequestHandler
    httpd = HTTPServer(server_address, handler)
    httpd.talker = talker  # Attach the ROS node to the HTTP server
    print(f'Serving HTTP on port {port}...')
    httpd.serve_forever()


def main():
    rclpy.init()
    talker = Talker()

    # Run HTTP server in a separate thread
    server_thread = threading.Thread(target=run_server, args=(talker,), daemon=True)
    server_thread.start()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
