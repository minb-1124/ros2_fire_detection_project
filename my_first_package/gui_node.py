import tkinter as tk
from PIL import Image, ImageTk
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import timef
import re
import pygame  # 🔊 사운드 제어용
import sys

class GuiSubscriber(Node):
    def __init__(self, alert_label, frame_labels):
        super().__init__('gui_subscriber')
        self.alert_label = alert_label
        self.frame_labels = frame_labels
        self.subscription = self.create_subscription(String, 'smoke_alert', self.alert_callback, 10)
        self.blinking_index = None
        self.blink_state = False

        # 🔊 pygame 초기화 및 사운드 로드
        pygame.mixer.init()
        self.alert_sound = pygame.mixer.Sound("/home/jaseung/Downloads/화재경보.mp3")

        self.reset_client = self.create_client(Empty, 'reset_alert')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset_alert service...')

    def alert_callback(self, msg):
        self.get_logger().info(f'Alert received: {msg.data}')
        self.alert_label.config(text=msg.data, fg='red')

        # 🔊 경고음 반복 재생
        if "불꽃" in msg.data or "🔥" in msg.data:
            try:
                self.alert_sound.play(loops=-1)
            except Exception as e:
                self.get_logger().warn(f"Sound playback failed: {e}")

        for label in self.frame_labels:
            label.config(highlightbackground="black", highlightcolor="black", highlightthickness=4)

        match = re.search(r"스크린 번호\s*:\s*(\d+)", msg.data)
        if match:
            screen_index = int(match.group(1)) - 1
            self.blinking_index = screen_index
            self.blink_state = False
            self.blink_frame()

    def blink_frame(self):
        if self.blinking_index is None:
            return
        label = self.frame_labels[self.blinking_index]
        color = "red" if self.blink_state else "black"
        label.config(highlightbackground=color, highlightcolor=color, highlightthickness=4)
        self.blink_state = not self.blink_state
        label.after(500, self.blink_frame)

    def reset_alert(self):
        request = Empty.Request()
        future = self.reset_client.call_async(request)

        def done_callback(fut):
            self.alert_label.config(text="No alert", fg='green')
            for label in self.frame_labels:
                label.config(highlightbackground="black", highlightcolor="black", highlightthickness=4)
            self.blinking_index = None

            # 🔇 소리 끄기
            pygame.mixer.stop()

        future.add_done_callback(done_callback)

def update_image(label, filename):
    if os.path.exists(filename):
        try:
            img = Image.open(filename)
            img = img.resize((320, 240))
            photo = ImageTk.PhotoImage(img)
            label.config(image=photo)
            label.image = photo
        except Exception:
            pass
    label.after(1000, lambda: update_image(label, filename))

def main():
    rclpy.init()
    root = tk.Tk()
    root.title("Control Tower GUI")
    root.geometry("1400x800+100+100")

    # 🔳 회색 배경 설정
    root.configure(bg="#111111")
    canvas = tk.Canvas(root, width=1400, height=800, bg="#111111", highlightthickness=0)
    canvas.pack(fill="both", expand=True)

    # 📛 제목 (상단 중앙)
    title_label = tk.Label(
        root,
        text="화재 감지 관제 시스템",
        fg="#DDDDDD",
        bg="#222222",
        font=("맑은 고딕", 36, "bold")
    )
    canvas.create_window(700, 40, window=title_label)

    # ⏱ 현재 시간 표시 (우측 상단)
    time_label = tk.Label(
        root,
        text="--:--:--",
        fg="#DDDDDD",
        bg="#111111",
        font=("맑은 고딕", 16)
    )
    canvas.create_window(1280, 40, window=time_label)

    def update_time():
        now = time.strftime('%Y-%m-%d %H:%M:%S')
        time_label.config(text=now)
        time_label.after(1000, update_time)

    update_time()

    # 📷 영상 프레임 위치
    frames = []
    filenames = [
        "/home/jaseung/Downloads/latest-pg-2.jpg",
        "/home/jaseung/Downloads/latest-frame.jpg",
        "/home/jaseung/Downloads/latest-frame.jpg",
        #"/home/jaseung/Downloads/fire.jpg",
        "frame.jpg"
    ]
    titles = ["카메라 1", "카메라 2", "카메라 3", "카메라 4"]
    positions = [(50, 120), (390, 120), (50, 430), (390, 430)]

    for i in range(4):
        # 제목 라벨
        title = tk.Label(
            root,
            text=titles[i],
            fg="#DDDDDD",
            bg="#111111",
            font=("맑은 고딕", 14, "bold")
        )
        canvas.create_window(positions[i][0] + 160, positions[i][1] - 20, anchor="center", window=title)

        label = tk.Label(
            root,
            highlightthickness=4,
            highlightbackground="black",
            highlightcolor="black",
            bd=0,
            relief='solid',
            fg="#DDDDDD"
        )
        canvas.create_window(positions[i][0], positions[i][1], anchor="nw", window=label)
        update_image(label, filenames[i])
        frames.append(label)

    #  알림 라벨
    alert_label = tk.Label(
        root,
        text='No alert',
        bg="#111111",
        fg='green',
        font=("맑은 고딕", 26, "bold")
    )
    canvas.create_window(1100, 150, window=alert_label)

    #  fire_log.txt 텍스트 박스 추가
    log_textbox = tk.Text(
        root,
        width=60,
        height=20,
        bg='black',
        fg='white',
        font="Courier 10",
        state='disabled'  # 처음엔 읽기 전용
    )
    log_textbox.place(x=850, y=300)

    def update_log_textbox():
        try:
            with open("/home/jaseung/ros2/fire_log.txt", "r") as f:
                content = f.read()
        except FileNotFoundError:
            content = "[로그 없음]"

        log_textbox.configure(state='normal')
        log_textbox.delete(1.0, tk.END)
        log_textbox.insert(tk.END, content)
        log_textbox.configure(state='disabled')

        root.after(2000, update_log_textbox)
    update_log_textbox()

    # 🚨 ROS2 노드 시작
    node = GuiSubscriber(alert_label, frames)

    # ✅ 종료 함수 정의 (여기!)
    def exit_program():
        print("[종료] 시스템 종료 중...")
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        os._exit(0)


    # 🔄 리셋 버튼
    reset_button = tk.Button(
        root,
        text="Reset",
        font=("맑은 고딕", 18, "bold"),
        fg='gray20',
        bg='white',
        command=node.reset_alert
    )
    canvas.create_window(1000, 650, window=reset_button)

    # ⛔ 종료 버튼
    exit_button = tk.Button(
        root,
        text="종료",
        font=("맑은 고딕", 18, "bold"),
        fg='gray20',
        bg='white',
        command=exit_program  # psutil 없이 바로 종료
    )
    canvas.create_window(1200, 650, window=exit_button)

    # ROS2 실행
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root.mainloop()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
