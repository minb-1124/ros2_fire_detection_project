
# 📡 ROS2 기반 연기 감지 관제 시스템  
**ROS2-Based Smoke Detection and Control System**

## 📘 개요 | Overview

이 프로젝트는 ROS2를 기반으로 CCTV 카메라를 통해 연기를 감지하고, 감지 결과를 실시간으로 GUI에 시각화하며, 감지 로그를 기록하는 관제 시스템입니다.  
This project uses ROS2 to detect smoke from CCTV cameras, display alerts on a real-time GUI, and log detection events in a structured format.

## 🗂️ 프로젝트 구조 | Project Structure

.
├── gui_node.py                      # GUI (관제탑)
├── project_gui_start_node.py       # GUI 시작 명령 수신 노드
├── project_reset_service_node.py   # Reset 서비스 노드
├── project_smoke_detector_node.py  # 연기 감지 노드
├── project_smoke_detection_system.launch.py # 전체 런치 구성
├── setup.py                        # ROS2 패키지 설정
├── fire_log.txt                    # 감지 기록 로그

## ⚙️ 주요 기능 | Main Features

| 구성 요소 | 설명 | Component | Description |
|-----------|------|-----------|-------------|
| 연기 감지 노드 | OpenCV + TensorFlow로 연기 감지, 로그 기록 및 알림 발행 | Smoke Detector Node | Detects smoke using OpenCV + TensorFlow, publishes alert and logs |
| GUI 노드 | 카메라 화면 표시 및 알림창 표시, Reset 버튼 포함 | GUI Node | Displays camera feeds and alert status with reset functionality |
| Reset 서비스 | 사용자 요청 시 경보 초기화 수행 | Reset Service | Resets alert state upon user request |
| GUI 실행 노드 | `/gui_start` 명령 수신 시 GUI 관련 노드 실행 | GUI Start Node | Launches GUI nodes when `/gui_start` is received |
| 로그 파일 | 감지 시간과 카메라 번호를 기록 | Log File | Records time and camera number of smoke detection |

## 🧪 사용 방법 | How to Use

### ✅ 빌드 | Build

cd ~/ros2/robot_ws
ros2humble
colcon build
source install/setup.bash

### ▶️ 런치 실행 | Launch System

ros2 launch my_first_package project_smoke_detection_system.launch.py

### 🖥️ GUI 실행 | Start GUI

ros2 topic pub /gui_start std_msgs/msg/Empty "{}"

### 🔁 알림 리셋 | Reset Alert

GUI에서 **Reset 버튼 클릭** → `/reset_alert` 서비스 호출
Click **Reset button** in the GUI → Calls `/reset_alert` service

## 🧾 감지 로그 예시 | Example Log Output (`fire_log.txt`)

[2025-06-07 20:48:53] 🔥 화재 감지 - 카메라 3
[2025-06-07 20:58:24] 🔥 화재 감지 - 카메라 4
...

## 📡 주요 통신 | ROS2 Topics & Services

| 이름 | 타입 | 방향 | 설명 | Name | Type | Direction | Description |
|------|------|------|------|------|------|-----------|-------------|
| `/alert_status` | `std_msgs/String` | Pub | 감지된 카메라 번호 전송 | `/alert_status` | `std_msgs/String` | Publisher | Sends camera alert info |
| `/reset_alert` | `Trigger` | Service | 알림 초기화 요청 처리 | `/reset_alert` | `Trigger` | Service | Resets GUI alert |
| `/gui_start` | `std_msgs/Empty` | Sub | 관제화면 실행 트리거 | `/gui_start` | `std_msgs/Empty` | Subscriber | Triggers GUI window |

## ⚠️ 참고사항 | Notes

- TensorFlow 모델 파일(`smoke_detection_model.h5`)은 지정된 경로에 위치해야 합니다.
  The smoke detection model (`smoke_detection_model.h5`) must be placed in the correct path.
- 비디오 경로는 `project_smoke_detector_node.py` 내부에서 지정되어 있어야 합니다.
  The video file path must be configured correctly inside `project_smoke_detector_node.py`.
- GUI는 `tkinter` 기반이므로 GUI 환경이 필요한 점에 유의하세요.
  GUI requires a display environment as it uses `tkinter`.

## 💡 개발자 참고 | Developer Notes

- **지원 카메라 수**: 기본 4개, `gui_node.py`에서 조절 가능
  Supports 4 camera feeds; adjustable in `gui_node.py`
- **임계값 조절**: 연기 판단 기준은 `project_smoke_detector_node.py` 내에서 조정
  Threshold for detection is configurable in `project_smoke_detector_node.py`
- **확장성**: 이메일, 슬랙 알림, 이미지 저장 기능 추가 가능
  Easily extensible with features like email alerts, Slack integration, or image capture
