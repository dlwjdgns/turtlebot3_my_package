# ROS 2 Autonomous Patrol & Precision Docking System

본 문서는 `turtlebot3_msgs.action.PrecisionDock` 액션 인터페이스와 `geometry_msgs/msg/TwistStamped` 메시지 타입을 기반으로 설계된 **ROS 2 표준 계층 분리형 자율 순찰(Patrol) 및 정밀 도킹(Precision Docking) 시스템** 통합 기술 명세서입니다.

---

## 1. 시스템 아키텍처 및 제어 흐름

### 1.1 계층 구조

* **Mission Dispatcher (`PatrolManager`)**: 최상위 임무 관리 및 상태 머신(FSM) 전담. Nav2와 도킹 액션 클라이언트를 비동기 호출.
* **Navigation Stack (Nav2)**: 웨이포인트 간 글로벌/로컬 경로 계획 및 일반 주행 담당.
* **Precision Controller (`PrecisionDockingServer`)**: 도킹 요청 시에만 활성화되어 TF 기반 고정밀 PID 후진 제어 수행.
* **Actuation Layer**: `TwistStamped` 메시지를 통해 `/cmd_vel`로 직접 속도 명령 전달.


                  +--------------------------------+
                  |    사용자 / 터미널 제어 (CLI)   |
                  +--------------------------------+
                                  | (Service: /pause, /resume, /dock_and_exit)
                                  | (Topic: /patrol_cmd)
                                  v
                  +--------------------------------+
                  |         PatrolManager          |  <-- [Mission Layer]
                  +--------------------------------+
                       |                      |
      (Nav2 Action)    |                      | (Docking Action)
      /navigate_to_pose|                      | precision_dock
                       v                      v
        +-----------------------+   +-----------------------+
        |      Nav2 Stack       |   | PrecisionDockingServer|  <-- [Control Layer]
        +-----------------------+   +-----------------------+
                   |                            |
                   +-------------+--------------+
                                 | /cmd_vel (TwistStamped)
                                 v
                       +--------------------+
                       | Robot Base Driver  |  <-- [Actuation Layer]
                       +--------------------+

my_patrol_pkg/
├── CMakeLists.txt (또는 setup.py)
├── package.xml
├── launch/
│   └── patrol_system.launch.py
└── my_patrol_pkg/
    ├── __init__.py
    ├── precision_docking_server.py
    └── patrol_manager.py

## topic

### 1. 주행 일시정지
ros2 topic pub --once /patrol_cmd std_msgs/msg/String "{data: 'pause'}"

### 2. 주행 재개
ros2 topic pub --once /patrol_cmd std_msgs/msg/String "{data: 'resume'}"

### 3. 대기점 경유 후 정밀 도킹 및 노드 종료
ros2 topic pub --once /patrol_cmd std_msgs/msg/String "{data: 'dock'}"

## service 

### 1. 주행 일시정지
ros2 service call /patrol_manager/pause std_srvs/srv/Trigger {}

### 2. 주행 재개
ros2 service call /patrol_manager/resume std_srvs/srv/Trigger {}

### 3. 대기점 경유 후 정밀 도킹 및 노드 종료
ros2 service call /patrol_manager/dock_and_exit std_srvs/srv/Trigger {}