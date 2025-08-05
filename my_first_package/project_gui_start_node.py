import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import threading
import subprocess

class GuiStartNode(Node):
    def __init__(self):
        super().__init__('gui_start_node')
        self.publisher_ = self.create_publisher(String, 'gui_start_command', 10)
        self.subscription = self.create_subscription(
            String,
            'gui_start_command',
            self.listener_callback,
            10
        )
        self.has_launched = False  # 중복 실행 방지

    def send_start_command(self):
        msg = String()
        msg.data = "start"
        self.publisher_.publish(msg)
        self.get_logger().info("📤 관제화면 열기 명령 전송됨")

    def listener_callback(self, msg):
        if msg.data == "start" and not self.has_launched:
            self.get_logger().info("📥 관제화면 실행 명령 수신됨 → 노드 실행")
            self.has_launched = True
            threading.Thread(target=self.launch_nodes).start()

    def launch_nodes(self):
        subprocess.Popen(['ros2', 'run', 'my_first_package', 'smoke_detector_node'])
        subprocess.Popen(['ros2', 'run', 'my_first_package', 'reset_service_node'])
        subprocess.Popen(['ros2', 'run', 'my_first_package', 'gui_node'])

def run_gui(node):
    root = tk.Tk()
    root.title("화재 감지 시스템 시작")
    root.geometry("400x300")

    label = tk.Label(root, text="화재 감지 시스템", font=("Helvetica", 18))
    label.pack(pady=40)

    start_button = tk.Button(root, text="관제화면 열기", font=("Helvetica", 14), command=node.send_start_command)
    start_button.pack(pady=20)

    def on_close():
        print("🛑 GUI 창 닫힘 → 노드 종료")
        rclpy.shutdown()
        root.destroy()

    exit_button = tk.Button(root, text="종료", font=("Helvetica", 14), command=on_close)
    exit_button.pack(pady=10)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = GuiStartNode()
    gui_thread = threading.Thread(target=run_gui, args=(node,))
    gui_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
