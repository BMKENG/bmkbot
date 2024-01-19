#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# navigate to pose 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# 초기 위치 설정 상태를 정의하는 클래스
class InitPoseState(EventState):

    # 초기화 함수
    def __init__(self, init_pose_topic="/initialpose", init_pose_cmd_topic="/init_pose_cmd", waypoint=[0.0, 0.0, 0.0, 1.0]):

        super().__init__(outcomes=['done'])  # EventState 초기화
        self.init_pose_topic = init_pose_topic  # 초기 위치 설정 토픽
        self.init_pose_cmd_topic = init_pose_cmd_topic  # 초기 위치 명령 토픽

        # 초기 위치 데이터를 발행할 프록시 퍼블리셔 생성
        self.init_pose_pub = ProxyPublisher({self.init_pose_topic: PoseWithCovarianceStamped})
        # 초기 위치 데이터 설정
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.pose.pose.position.x = waypoint[0]
        self.init_pose.pose.pose.position.y = waypoint[1]
        self.init_pose.pose.pose.position.z = waypoint[2]
        self.init_pose.pose.pose.orientation.w = waypoint[3]

        # 초기 위치 명령을 수신할 프록시 서브스크라이버 생성
        self.init_pose_cmd_sub = ProxySubscriberCached({self.init_pose_cmd_topic: String})
        
        self.msg = None  # 수신 메시지 초기화

        self._sucess = False  # 성공 여부 초기화

    # 시작 시 실행되는 함수
    def start(self):
        self.msg = self.init_pose_cmd_sub.get_last_msg(self.init_pose_cmd_topic)
        if self.msg is None:
            return False
        
        if self.msg.data == "start_nav":
            self.init_pose_pub.publish(self.init_pose_topic, self.init_pose)
            self.init_pose_cmd_sub.remove_last_msg(self.init_pose_cmd_topic)
            self.info("init_pose update")
            return True
        else:
            return False

    # 상태 실행 시 호출되는 함수
    def execute(self, userdata):
        self._sucess = self.start()
        if self._sucess:
            # 성공 시 'done' 반환
            return 'done'

    # 상태 진입 시 호출되는 함수
    def on_enter(self, userdata):
        self.info("navigation initial pose enter")
        # 테이블 번호에 따라 이동

    # 상태 종료 시 호출되는 함수
    def on_exit(self, userdata):
        self.info("navigation initial pose exit")

    # 로그 정보 출력 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
