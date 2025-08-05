# reset_service_node.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ResetAlertService(Node):
    def __init__(self):
        super().__init__('reset_service_node')
        self.srv = self.create_service(Empty, 'reset_alert', self.reset_callback)

    def reset_callback(self, request, response):
        self.get_logger().info('Reset alert requested.')
        # 실제 상태 초기화 로직은 GUI에서 처리
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetAlertService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()