#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
# 메시지
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

# 주문입력시 테이블 번호를 저장하고 대기하는 state
class BMKBOTStartOrder_waitState(EventState):
    def __init__(self, order_table_topic="/order_table", start_navi_topic="/start_nav", arrived_table_topic="/arrived_table"):
        # 기본 설명 참고
        super().__init__(input_keys=['table_num', 'order_table_num'], outcomes=['done'])
        self.info("StartOrder_waitState 초기화")

        self.table_num = None
        self.order_table_topic = order_table_topic
        self.start_navi_topic = start_navi_topic
        self.arrived_table_topic = arrived_table_topic

        # 도착한 테이블을 알리기 위한 발행자
        self.pub_arrived_table = ProxyPublisher({self.arrived_table_topic: Int32})
        self.arrived_table = Int32()

        # 테이블 주문을 위한 발행자
        self.pub_order_table = ProxyPublisher({self.order_table_topic: Int32})
        self.order_table = Int32()

        # 출발을 확인하기 위한 구독자
        self.sub_start_navi = ProxySubscriberCached({self.start_navi_topic : Int32})
        self.start_navi_msg = None

        self.start_navi_flag = False

        self._sucess = False

    def start_navi(self):
        # 출발 메시지 확인
        self.start_navi_msg = self.sub_start_navi.get_last_msg(self.start_navi_topic)
        if self.start_navi_msg is not None:
            self.sub_start_navi.remove_last_msg(self.start_navi_topic)
            self.start_navi_msg = None
            return True
        else:
            return False

    def execute(self, userdata):
        # 출발 확인
        self._sucess = self.start_navi()
        if self._sucess:
            self.info("StartOrder_waitState 성공")
            return 'done'
        else:
            pass
         
    def on_enter(self, userdata):
        # 상태 진입 시 실행
        self.info("StartOrder_waitState 진입")
        self.table_num = userdata.table_num
        self._sucess = False
        if self.table_num == None:
            pass
        if self.table_num == 0:
            # 도착 메시지 발행
            self.arrived_table.data = userdata.order_table_num * 3 + self.table_num
            self.pub_arrived_table.publish(self.arrived_table_topic, self.arrived_table)
        else:
            # 주문 메시지 발행
            self.order_table.data = userdata.order_table_num * 3 + self.table_num
            self.pub_order_table.publish(self.order_table_topic, self.order_table)
        pass
        
    def on_exit(self, userdata):
        self.info("StartOrder_waitState 종료")
        pass

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
