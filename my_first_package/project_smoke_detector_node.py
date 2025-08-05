import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import cv2
import numpy as np
import tensorflow as tf
import time
import os

class SmokeDetectorAI(Node):
    def __init__(self):
        super().__init__('smoke_detector_ai_node')

        # ROS 통신
        self.publisher_ = self.create_publisher(String, 'smoke_alert', 10)
        self.reset_srv = self.create_service(Empty, 'reset_alert', self.reset_callback)

        # 상태 추적
        self.published_flags = [False, False, False, False]
        self.fire_counters = [0, 0, 0, 0]

        # 이미지 소스 설정
        self.image_paths = [
            "/home/jaseung/Downloads/latest-pg-2.jpg",
            "/home/jaseung/Downloads/latest-frame.jpg",
            "/home/jaseung/Downloads/latest-frame.jpg"
            #"/home/jaseung/Downloads/fire.jpg"
        ]

        # 파라미터: threshold (default = 0.8)
        self.declare_parameter('smoke_threshold', 0.8)

        # 모델 및 라벨
        self.model = tf.keras.models.load_model('/home/jaseung/Downloads/converted_model')
        self.labels = ["화재 감지", "산", "화재 x"]

        # 카메라 4번은 실시간 처리
        self.video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # 로그 경로
        self.log_path = "/home/jaseung/ros2/fire_log.txt"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

        # 타이머 콜백
        self.timer = self.create_timer(1.0, self.detect_all)

    def reset_callback(self, request, response):
        self.get_logger().info("🔥 리셋 요청 수신 - 카운터 및 플래그 초기화")
        self.fire_counters = [0, 0, 0, 0]
        self.published_flags = [False, False, False, False]
        return response

    def detect_all(self):
        for i in range(4):
            if i < 3:
                frame = cv2.imread(self.image_paths[i])
                if frame is None:
                    self.get_logger().warn(f"카메라 {i+1} 이미지 불러오기 실패")
                    continue
            else:
                ret, frame = self.video_capture.read()
                if not ret:
                    self.get_logger().warn("카메라 4 인식 실패")
                    continue
                cv2.imwrite("frame.jpg", frame)

            # AI 감지
            flame_detected = self.detect_fire_by_ai(frame, i)

            if flame_detected:
                self.fire_counters[i] += 1
            else:
                self.fire_counters[i] = max(0, self.fire_counters[i] - 1)

            if self.fire_counters[i] >= 3 and not self.published_flags[i]:
                now = time.strftime('%Y-%m-%d %H:%M:%S')
                msg = String()
                msg.data = f'🔥 화재 감지됨!\n번호 스크린: {i+1}\n감지 시간: {now}'
                self.publisher_.publish(msg)
                self.get_logger().info(msg.data)

                try:
                    with open(self.log_path, "a") as log_file:
                        log_file.write(f"[{now}] 🔥 화재 감지 - 카메라 {i+1}\n")
                except Exception as e:
                    self.get_logger().error(f"로그 기록 실패: {e}")

                self.published_flags[i] = True

            if self.fire_counters[i] < 3:
                self.published_flags[i] = False

    def detect_fire_by_ai(self, frame, camera_index):
        img = cv2.resize(frame, (224, 224))
        img = img.astype('float32') / 255.0
        img = np.expand_dims(img, axis=0)

        prediction = self.model.predict(img)[0]
        label_index = np.argmax(prediction)
        label = self.labels[label_index]

        threshold = self.get_parameter('smoke_threshold').get_parameter_value().double_value
        confidence = prediction[label_index]

        self.get_logger().info(f"[AI 감지] 결과: {label} ({confidence:.2f})")

        return label == "화재 감지" and confidence >= threshold

def main(args=None):
    rclpy.init(args=args)
    node = SmokeDetectorAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.video_capture.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
