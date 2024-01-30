#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# 조이스틱 커맨드
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# 조이스틱 상태를 관리하는 클래스
class JoystickState(EventState):

    def __init__(self, set_angular_vel = 0.5, set_linear_vel = 0.5, joystick_topic="/joy_cmd", cmd_vel_topic="/cmd_vel", set_vel_topic="/set_vel"):
        # 기본 설명 참고
        super().__init__(outcomes=['done'])

        # 토픽 설정
        self.gui_topic = joystick_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.set_vel_topic = set_vel_topic

        # 변수 초기화
        self.joystick = None
        self.set_angular_velocity = set_angular_vel
        self.set_linear_velocity = set_linear_vel
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # 조이스틱 커맨드 구독자 및 발행자 설정
        self.sub_gui_command = ProxySubscriberCached({self.gui_topic: String})
        self.sub_gui_command.set_callback(self.gui_topic, self.joystick_callback)
        self.pub_set_vel = ProxyPublisher({self.set_vel_topic: Twist})
        self.pub_cmd_vel = ProxyPublisher({self.cmd_vel_topic: Twist})

        # 조이스틱 플래그
        self.joy_flag = False

        # 조이스틱 커맨드 사전
        self.control_dict = {
            'go': self.go_button,
            'back': self.back_button,
            'stop': self.stop_button,
            'left': self.left_button,
            'right': self.right_button,
            'angular_up': self.angular_up_button,
            'angular_down': self.angular_down_button,
            'linear_up': self.linear_up_button,
            'linear_down': self.linear_down_button,
            'emergency' : self.emergency_button,
        }

        # 액션 클라이언트 실패 가능성
        self._sucess = False
        self._failed = False

    # 조이스틱 콜백 함수
    def joystick_callback(self, msg):
        # 유효한 키 입력 확인 후 해당 함수 실행
        if msg is None:
            return
        
        if msg.data in self.control_dict:
            self.control_dict[msg.data]()
            self.info('조이스틱 커맨드: ' + msg.data)
            # 마지막 메시지 제거
        else:
            self.warn('잘못된 조이스틱 커맨드: ' + msg.data)

    # cmd_vel 메시지 전송 함수
    def send_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub_cmd_vel.publish(self.cmd_vel_topic, msg)

    # set_vel 메시지 전송 함수
    def send_set_vel(self):
        msg = Twist()
        msg.linear.x = self.set_linear_velocity
        msg.angular.z = self.set_angular_velocity
        self.pub_set_vel.publish(self.set_vel_topic, msg)

    # 조이스틱 버튼에 따른 기능 구현
    def go_button(self):
        # 앞으로 이동
        self.linear_velocity = self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def back_button(self):
        # 뒤로 이동
        self.linear_velocity = -self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def stop_button(self):
        # 정지
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def left_button(self):
        # 왼쪽으로 회전
        self.linear_velocity = 0.0
        self.angular_velocity = self.set_angular_velocity
        self.send_cmd_vel()

    def right_button(self):
        # 오른쪽으로 회전
        self.linear_velocity = 0.0
        self.angular_velocity = -self.set_angular_velocity
        self.send_cmd_vel()

    # 속도 조절 버튼 기능
    def angular_up_button(self):
        # 각속도 증가
        self.set_angular_velocity += 0.1
        self.stop_button()
        if self.set_angular_velocity >= 1.0:
            self.set_angular_velocity = 1.0
        self.send_set_vel()

    def angular_down_button(self):
        # 각속도 감소
        self.set_angular_velocity -= 0.1
        self.stop_button()
        if self.set_angular_velocity <= 0.1:
            self.set_angular_velocity = 0.1
        self.send_set_vel()

    def linear_up_button(self):
        # 선속도 증가
        self.set_linear_velocity += 0.1
        self.stop_button()
        if self.set_linear_velocity >= 1.0:
            self.set_linear_velocity = 1.0
        self.send_set_vel()

    def linear_down_button(self):
        # 선속도 감소
        self.set_linear_velocity -= 0.1
        self.stop_button()
        if self.set_linear_velocity <= 0.1:
            self.set_linear_velocity = 0.1
        self.send_set_vel()

    def emergency_button(self):
        # 긴급 정지
        self.stop_button()
        self._sucess = True

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        # 조이스틱 커맨드 구독 시작
        self.info("조이스틱 시작")

    def on_exit(self, userdata):
        # 상태 종료 시 조이스틱 정지
        self.stop_button()
        self.info("조이스틱 정지")

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
