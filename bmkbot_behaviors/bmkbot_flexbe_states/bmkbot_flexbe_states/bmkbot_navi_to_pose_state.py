#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# Navigate to pose 관련 라이브러리
from nav2_msgs.action import NavigateToPose
from bmkbot_interfaces.msg import BMKBOTNaviProgress
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

class BMKBOTNavigateToPoseState(EventState):
    # 초기화 함수
    def __init__(self, timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd", battery_topic="/battery",
                  amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0, table_or_kitchen=0):
        super().__init__(outcomes=['failed', 'done'],
                         input_keys=['waypoint', 'table_num', 'order_table_num', 'table_or_kitchen', 'battery'], output_keys=['battery']) 

        # 타이머 설정
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._start_time = None
        
        # 토픽 설정
        self.navi_to_pos_topic = navi_to_pos_topic
        self.amcl_pose_topic = amcl_pose_topic
        self.navi_progress_topic = navi_progress_topic
        self.emergency_topic = emergency_topic
        self.arrived_table_topic = arrived_table_topic
        self.battery_topic = battery_topic

        self.table_num = table_num
        self.table_or_kitchen = table_or_kitchen
        # navigate to pose 액션 클라이언트 초기화
        ProxyActionClient.initialize(BMKBOTNavigateToPoseState._node)
        self._client = ProxyActionClient({self.navi_to_pos_topic: NavigateToPose})

        # 현재 위치 구독자 및 콜백 함수 설정
        self.model_pose_sub = ProxySubscriberCached({self.amcl_pose_topic : PoseWithCovarianceStamped})
        self.current_pose = None

        # 비상 상황 구독자 및 콜백 함수 설정
        self.sub_navigator = ProxySubscriberCached({self.emergency_topic: String})
        self.sub_navigator.set_callback(self.emergency_topic, self.emergency_callback)

        # 진행률 발행자 설정
        self.remaining_waypoint_pub = ProxyPublisher({self.navi_progress_topic: BMKBOTNaviProgress})
        self.naviprog = BMKBOTNaviProgress()
        self.naviprog.remaining_waypoint = [100, 100, 100, 100, 100, 100, 100, 100]

        # 도착 테이블 발행자 설정
        self.pub_arrived_table = ProxyPublisher({self.arrived_table_topic: Int32})
        self.arrived_table = Int32()

        # 충전량 발행자 설정
        self.pub_battery = ProxyPublisher({self.battery_topic: Int32})
        self.battery = Int32()        

        # 초기 위치 저장 변수 및 피드백 관련 변수 초기화
        self.initial_pose = None
        self.feedback_flag = False
        self.remaining_distance = 0
        self.initial_distance = 0
        self.feedback = None

        # 목표 좌표 저장 변수
        self.waypoint = None
        
        # 액션 클라이언트 실패 가능성
        self._sucess = False
        self._failed = False

        # 비상 상황 컨트롤 딕셔너리
        self.control_dict = {
            "emergency" : lambda : self.emergency()
        }

    # 비상 상황 콜백 함수
    def emergency_callback(self, msg):
        if msg.data in self.control_dict.keys():
            self.control_dict[msg.data]()
            self.info(f"비상 상황: {msg.data}")
        else:
            self.info("비상 상황: 잘못된 명령어")

    # 비상 상황 처리 함수
    def emergency(self):
        self.navi_cancel()
        self.info("비상 상황!!!")
        self._failed = True

    # 목표 위치로 이동하는 함수
    def goToPose(self, waypoint_num):
        self.table_num = waypoint_num
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.waypoint[waypoint_num][0]
        goal_msg.pose.pose.position.y = self.waypoint[waypoint_num][1]
        goal_msg.pose.pose.orientation.z = self.waypoint[waypoint_num][2]
        goal_msg.pose.pose.orientation.w = self.waypoint[waypoint_num][3]

        self.goal_handle = self._client.send_goal(self.navi_to_pos_topic, goal_msg)
        self.info("목표 위치로 이동 명령 발송")

        # waypoint 0일 경우 주방으로 이동
        if waypoint_num == 7:
            self.info("주방으로 이동합니다.")

    # 피드백 처리 함수
    def getFeedback(self):
        # 현재 위치 가져오기
        self.current_pose = self.model_pose_sub.get_last_msg(self.amcl_pose_topic).pose.pose
        # 액션 클라이언트로부터 피드백 받기
        if self._client.has_feedback(self.navi_to_pos_topic) and self.current_pose is not None:
            feedback_msg = self._client.get_feedback(self.navi_to_pos_topic)
            if feedback_msg and feedback_msg.feedback and hasattr(feedback_msg.feedback, 'distance_remaining'):
                # 남은 거리 계산
                distance_remaining = feedback_msg.feedback.distance_remaining
                if self.feedback_flag == False:
                    self.initial_distance = ((self.current_pose.position.x - self.waypoint[self.table_num][0]) ** 2 + 
                                            (self.current_pose.position.y - self.waypoint[self.table_num][1]) ** 2) ** 0.5
                    self.feedback_flag = True

                self.remaining_distance = (distance_remaining / self.initial_distance * 100)
                
                if self.remaining_distance < 0:
                    self.remaining_distance = 0
                elif self.remaining_distance > 100:
                    self.remaining_distance = 100 

                self.remaining_distance = 100 - self.remaining_distance
                self.naviprog.remaining_waypoint[self.table_num] = self.remaining_distance
                self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            else:
                self.warn("피드백에 'distance_remaining'가 없습니다.")
        else:
            self.warn("액션 서버로부터 피드백을 받지 못했습니다.")

    # 네비게이션 취소 함수
    def navi_cancel(self):
        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("네비게이션 취소")
            self.naviprog.remaining_waypoint[self.table_num] = 0
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
        else:
            self.naviprog.remaining_waypoint[self.table_num] = 100
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            self.info("네비게이션 완료")

    # 상태 실행 함수
    def execute(self, userdata):
        self.getFeedback()
        if self._client.has_result(self.navi_to_pos_topic):
            self.info("네비게이션 완료")
            self._sucess = True

        if self._sucess:
            return 'done'
        if self._failed:
            return 'failed'
        
    # 상태 진입 함수
    def on_enter(self, userdata):
        self.battery.data = userdata.battery - 20
        
        if userdata.battery < 20:
            self.battery.data = 20
        self.pub_battery.publish(self.battery_topic, self.battery)
        
        self.table_or_kitchen = userdata.table_or_kitchen
        
        if self.table_or_kitchen == 0 or userdata.table_num == 7 or userdata.table_num == 8:
            # 테이블 서빙 
            self.table_num = userdata.table_num - 1
            self.info("테이블 서빙 " + str(userdata.table_num))
        else:
            # 주방 서빙 
            self.table_num = userdata.order_table_num + 2
            self.info("주방 서빙 " + str(userdata.order_table_num))

        self.waypoint = userdata.waypoint

        self._start_time = self._node.get_clock().now()
        self._sucess = False
        self._failed = False
        self.feedback_flag = False
        self.remaining_distance = 0
        self.initial_distance = 0

        self.info("네비게이션 시작")
        self.goToPose(self.table_num)

    # 상태 종료 함수
    def on_exit(self, userdata):
        userdata.battery = self.battery.data
        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("네비게이션 취소")
            self.naviprog.remaining_waypoint[self.table_num] = 0
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
        else:
            self.naviprog.remaining_waypoint[self.table_num] = 100
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            self.info("네비게이션 완료")

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
