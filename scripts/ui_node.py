#!/usr/bin/env python3
# Copyright 2026 Bob Ros
import os
import sys
import time
import threading
import signal
from pathlib import Path

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Qt imports
try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineCore import QWebEngineSettings, QWebEnginePage
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtCore import QUrl, QTimer, QSize, QPoint
    from PySide6.QtGui import QImage, QPainter
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)

class CustomPage(QWebEnginePage):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def javaScriptConsoleMessage(self, level, message, lineID, sourceID):
        self.node.get_logger().info(f"[JS] {message} (line {lineID})")

class WebRenderer(Node):
    def __init__(self):
        super().__init__('ui_renderer')
        
        # Log ROS Domain for transparency
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
        self.get_logger().info(f"UI Renderer starting on ROS_DOMAIN_ID: {domain_id}")
        
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
        
        # Open FIFO for writing
        self.get_logger().info(f"Opening FIFO: {self.fifo_path} (Waiting for reader...)")
        self.fifo_fd = os.open(self.fifo_path, os.O_WRONLY)
        self.get_logger().info("FIFO opened for writing. Streaming started.")

        # Shared state
        self.current_content = ""
        self.lock = threading.Lock()
        
        # ROS Subscription
        self.subscription = self.create_subscription(
            String,
            self.get_parameter('topic').value,
            self.listener_callback,
            20 # Larger queue for fast streams
        )

        # Initialize Qt Application
        self.qt_app = QApplication.instance() or QApplication(sys.argv)
        
        # Create WebEngineView
        self.view = QWebEngineView()
        self.view.resize(self.width, self.height)
        self.view.show() # Necessary for layout even in offscreen mode
        
        # Custom page for console logging
        self.page = CustomPage(self)
        self.view.setPage(self.page)
        
        # Load local HTML
        html_path = Path(__file__).parent / "overlay.html"
        self.get_logger().info(f"Loading UI from: {html_path}")
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
        image.fill(0) # Transparent
        
        painter = QPainter(image)
        self.view.render(painter, QPoint(0, 0))
        painter.end()
        
        data = image.constBits().tobytes()
        
        # Robust write to FIFO
        try:
            total_sent = 0
            while total_sent < len(data):
                sent = os.write(self.fifo_fd, data[total_sent:])
                if sent == 0:
                    break
                total_sent += sent
        except OSError as e:
            if e.errno == 32: # Broken pipe
                self.get_logger().warn("Reader disconnected (Broken pipe). Stopping node.")
                QApplication.quit()
            else:
                self.get_logger().error(f"FIFO write failed: {e}")
                sys.exit(1)

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        timer = QTimer()
        timer.timeout.connect(self._ros_spin_once)
        timer.start(10) # 100Hz
        return self.qt_app.exec()

    def _ros_spin_once(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0)
            except Exception:
                pass

def main(args=None):
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    
    # Init ROS
    rclpy.init(args=args)
    
    # Chromium flags
    sys.argv.append("--disable-gpu")
    sys.argv.append("--no-sandbox")
    sys.argv.append("--disable-software-rasterizer")
    sys.argv.append("--single-process")
    
    renderer = WebRenderer()
    exit_code = renderer.run()
    
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
