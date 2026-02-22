#!/usr/bin/env python3
# Copyright 2026 Bob Ros
import sys
import os
import time
import threading
from pathlib import Path

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Qt imports (Using PySide6 for best WebEngine support)
try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineCore import QWebEngineSettings
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from rclpy.executors import SingleThreadedExecutor
    from PySide6.QtCore import QUrl, QTimer, QSize
    from PySide6.QtGui import QImage, QPainter
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)

class WebRenderer(Node):
    def __init__(self):
        super().__init__('ui_renderer')
        
        # Parameters
        self.declare_parameter('path', '/tmp/overlay_video')
        self.declare_parameter('width', 854)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/bob/llm_stream')
        
        self.fifo_path = self.get_parameter('path').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Ensure FIFO exists
        if not os.path.exists(self.fifo_path):
            os.mkfifo(self.fifo_path)
        
        # Open FIFO for writing (this will block until sdlviz opens it for reading)
        self.get_logger().info(f"Opening FIFO: {self.fifo_path} (Waiting for reader...)")
        self.fifo_fd = os.open(self.fifo_path, os.O_WRONLY)
        self.get_logger().info("FIFO opened for writing.")

        # Shared state
        self.current_content = ""
        self.lock = threading.Lock()
        
        # ROS Subscription
        self.subscription = self.create_subscription(
            String,
            self.get_parameter('topic').value,
            self.listener_callback,
            10
        )

        # Initialize Qt Application
        self.qt_app = QApplication.instance() or QApplication(sys.argv)
        
        # Create WebEngineView (needed for sizing and rendering)
        self.view = QWebEngineView()
        self.view.resize(self.width, self.height)
        self.page = self.view.page()
        
        # Load local HTML
        html_path = Path(__file__).parent / "overlay.html"
        self.page.load(QUrl.fromLocalFile(str(html_path.absolute())))
        
        # Timer for frame capture
        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_frame)
        self.timer.start(int(1000 / self.fps))

    def listener_callback(self, msg):
        with self.lock:
            self.current_content += msg.data
            # Inject into JS
            js_code = f"if(window.updateContent) window.updateContent({repr(self.current_content)});"
            self.page.runJavaScript(js_code)

    def capture_frame(self):
        # Create a QImage to render into
        image = QImage(self.width, self.height, QImage.Format_ARGB32)
        image.fill(0) # Transparent background
        
        painter = QPainter(image)
        self.view.render(painter)
        painter.end()
        
        # Convert to raw bytes. Qt's ARGB32 is exactly what SDL's BGRA expects
        # (on little-endian systems it's BGRA in memory)
        bits = image.constBits()
        os.write(self.fifo_fd, bits.tobytes())

    def run(self):
        # This keeps the Qt event loop and ROS spinning
        timer = QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0))
        timer.start(10) # 100Hz for ROS processing
        
        sys.exit(self.qt_app.exec())

def main(args=None):
    # Headless mode for Qt
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    
    rclpy.init(args=args)
    renderer = WebRenderer()
    renderer.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
