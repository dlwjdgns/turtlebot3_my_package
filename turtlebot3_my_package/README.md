### 이 코드는 Nav2 액션 클라이언트를 이용해 지정된 웨이포인트를 순회한 뒤, 목표 위치 도달 후 자체 구현한 PID 제어기와 TF(좌표 변환)를 활용해 오차 범위 수 cm 단위의 정밀 주차/도킹을 수행하는 ROS 2 노드입니다.

## 보조 클래스 및 헬퍼 함수
### PID.__init__(self, P, I, D): 
비례(P), 적분(I), 미분(D) 게인 및 내부 상태(오차 누적값, 직전 오차)를 초기화합니다.PID.update(self, error): 오차(error)를 기반으로 적분항과 미분항을 갱신하고 $P \cdot e + I \cdot \int e + D \cdot \frac{de}{dt}$ 계산 결과를 반환합니다.

### PID.reset(self): 
이전 오차와 누적 적분값을 0으로 리셋하여 제어 전환 시 튐 현상(Windup)을 방지합니다.

### normalize_angle(angle): 
math.atan2(sin, cos)를 이용해 각도를 $[-\pi, \pi]$ 범위로 정규화하여 180도 경계에서의 제어 오류를 방지합니다.
SequentialNavigator 클래스 함수
1. 초기화 및 자세 변환__init__(self):
Nav2 NavigateToPose 액션 클라이언트, TF 버퍼/리스너, /cmd_vel 퍼블리셔, 20Hz 주기 타이머를 생성합니다.
시뮬레이션/실제 환경 플래그(is_sim)에 맞춰 웨이포인트 목록(waypoints_data)과 최종 주차 좌표(parking_goal_data)를 초기화합니다.yaw_degree_to_quaternion(self, degree): 도(degree) 단위의 Yaw 각도를 쿼터니언(geometry_msgs/Quaternion) 메시지로 변환합니다.

### create_pose(self, x, y, degree=0.0): 
map 프레임 기준의 PoseStamped 메시지를 생성합니다.

## Nav2 웨이포인트 순회 (액션 클라이언트)
### start_navigation(self): 
Nav2 액션 서버의 연결 상태를 최대 10초간 대기하고, 연결되면 send_next_goal()을 호출합니다.
send_next_goal(self):모든 웨이포인트를 순회했으면 set_parking_goal()을 호출해 정밀 주차 모드로 전환합니다.
남은 웨이포인트가 있으면 NavigateToPose 목표를 비동기(send_goal_async)로 전송합니다.

### goal_response_callback(self, future): 
액션 서버의 목표 수락 여부를 확인하고, 결과 대기 콜백(get_result_callback)을 등록합니다.

### feedback_callback(self, feedback_msg): 
주행 중 피드백 수신 콜백입니다(현재는 pass 처리).

### get_result_callback(self, future):
주행 성공 시 인덱스를 증가시키고, 1초 딜레이 타이머(_delayed_send_next_goal)를 동작시켜 리소스를 안정화합니다. 실패 시 노드를 종료합니다.

### delayed_send_next_goal(self): 
딜레이 타이머를 해제하고 다음 웨이포인트 전송 함수(send_next_goal)를 호출합니다.

정밀 주차 및 상태 머신 제어

### set_parking_goal(self, x, y, degree=0.0): 
주차 목표 좌표를 등록하고 상태를 rotate_to_goal로 전환하며 PID를 리셋합니다.

### control_loop(self): 
20Hz로 동작하는 메인 루프입니다. TF 버퍼에서 map -> base_link 변환을 조회해 로봇의 현재 위치($x, y$)와 Yaw를 얻은 후, 현재 상태에 맞는 핸들러 함수를 실행하고 /cmd_vel을 퍼블리시합니다.

### handle_rotate_to_goal(self, current_x, current_y, current_yaw, msg):
[1단계] 목표 주차 좌표를 바라보도록 제자리 회전합니다. 각도 오차가 angle_tolerance 이내로 들어오면 move_to_goal 상태로 전환합니다.

### handle_move_to_goal(self, current_x, current_y, current_yaw, msg):
[2단계] 목표 지점까지 전진하면서 동시에 헤딩 각도를 미세 보정합니다. 거리 오차가 distance_tolerance 이내로 들어오면 rotate_to_final 상태로 전환합니다.

### handle_rotate_to_final(self, current_yaw, msg):
[3단계] 목표 지점에서 최종 지정된 Yaw 각도로 제자리 정렬합니다. 정렬이 완료되면 로봇 정지 명령을 퍼블리시하고 타이머를 취소한 뒤 노드를 안전하게 종료(rp.shutdown)합니다.

## 메인 실행부
### main(args=None): 
ROS 2 초기화(rp.init), 노드 인스턴스 생성, 웨이포인트 주행 시작 후 rp.spin()을 통해 콜백 이벤트를 대기하고 예외 발생 및 종료 시 자원을 안전하게 회수합니다.