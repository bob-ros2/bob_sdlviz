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

# Qt imports (Using PySide6 for best WebEngine support)
try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineCore import QWebEngineSettings
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtCore import QUrl, QTimer, QSize, QPoint
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
        
        # Page debugging
        self.page.loadFinished.connect(self._on_load_finished)
        self.page.javaScriptConsoleMessage = self._on_console_message
        
        # Load local HTML
        html_path = Path(__file__).parent / "overlay.html"
        if not html_path.exists():
            self.get_logger().error(f"HTML file not found at: {html_path}")
        else:
            self.get_logger().info(f"Loading HTML from: {html_path}")
            self.page.load(QUrl.fromLocalFile(str(html_path.absolute())))
        
        # Timer for frame capture
        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_frame)
        self.timer.start(int(1000 / self.fps))

    def _on_load_finished(self, ok):
        if ok:
            self.get_logger().info("HTML Page loaded successfully.")
        else:
            self.get_logger().error("HTML Page failed to load! Check file path and Chromium dependencies.")

    def _on_console_message(self, level, message, line, source):
        self.get_logger().info(f"[JS] {message} (line {line})")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received token chunk (len={len(msg.data)})")
        with self.lock:
            self.current_content += msg.data
            # Inject into JS
            js_code = f"if(window.updateContent) window.updateContent({repr(self.current_content)});"
            self.page.runJavaScript(js_code)

    def capture_frame(self):
        # Create a QImage to render into
        image = QImage(self.width, self.height, QImage.Format_ARGB32)
        image.fill(0) # Transparent background (if HTML is transparent)
        
        # Render the view into the image
        painter = QPainter(image)
        self.view.render(painter, QPoint(0, 0))
        painter.end()
        
        # Convert to raw bytes
        data = image.constBits().tobytes()
        
        # Ensure full write to FIFO (prevents corruption/partial frames)
        try:
            total_sent = 0
            while total_sent < len(data):
                sent = os.write(self.fifo_fd, data[total_sent:])
                if sent == 0:
                    break
                total_sent += sent
        except OSError as e:
            self.get_logger().error(f"Pipe write failed: {e}")
            sys.exit(1)

    def run(self):
        # Allow Ctrl+C to work
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        # This keeps ROS spinning within the Qt loop
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
    # Headless and GPU-fix flags for Chromium
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    
    # Chromium command line arguments
    sys.argv.append("--disable-gpu")
    sys.argv.append("--no-sandbox")
    sys.argv.append("--disable-software-rasterizer")
    sys.argv.append("--single-process") # Often helps in restricted Docker/NAS environments
    
    rclpy.init(args=args)
    renderer = WebRenderer()
    exit_code = renderer.run()
    
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
