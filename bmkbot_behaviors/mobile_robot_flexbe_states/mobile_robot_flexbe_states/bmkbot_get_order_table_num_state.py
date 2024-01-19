#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

# 메시지
from std_msgs.msg import Int32

# 주문입력시 테이블 번호를 저장하고 대기하는 state
class BMKBOTGetOrderTableNnumState(EventState):
    def __init__(self, order_table_topic="/order_table", battery_topic="/battery"):
        # 기본 설명 참고
        super().__init__(outcomes=['serving', 'charging'], output_keys=['table_num', 'order_table_num', 'battery'], input_keys=['battery']) 
        self.info("GetOrderTableNnumState 초기화")

        self.order_table_topic = order_table_topic
        self.battery_topic = battery_topic
        self.pub_battery = ProxyPublisher({self.battery_topic: Int32})
        # 주문 테이블 구독
        # 주문 입력 받기 - 몇 번 테이블인지
        self.sub_order_table = ProxySubscriberCached({self.order_table_topic : Int32}) 
        self.sub_order_table.set_callback(self.order_table_topic, self.order_table_callback)
        self.order_table_msg = None
        self._sucess = False
        self._serving = False
        self._battery = False

        self.battery = None

    def order_table_callback(self, msg):
        self.order_table_msg = msg

    def battery_check(self):
        if self.order_table_msg is not None:
            return True
        else:
            return False

    def execute(self, userdata):
        self._sucess = self.battery_check()
        if int(userdata.battery) <= 20:
                return 'charging'
        if self._sucess == True:
            
            userdata.table_num = self.order_table_msg.data % 3
            if userdata.table_num == 0:
                userdata.table_num = 3
            
            userdata.order_table_num = self.order_table_msg.data // 3 + 1 
            if userdata.table_num == 3:
                userdata.order_table_num -= 1
            
            self.info("테이블 번호 : {}".format(userdata.table_num) +  "주문 식당 번호 : {}".format(userdata.order_table_num))
            return 'serving'
        else:
            pass
  
    def on_enter(self, userdata):
        self.battery = userdata.battery - 10
        userdata.battery = self.battery 
        if userdata.battery <= 20:
            self.battery = 20
            self._battery = True
        userdata.battery = self.battery
        int32 = Int32()
        int32.data = userdata.battery
        self.pub_battery.publish(self.battery_topic, int32)
        self._sucess = False
        self._serving = False
        self._goback = False
        
        self.order_table_msg= None
        self.info("GetOrderTableNnumState 진입")
        self.serving_flag = False
        self.return_flag = False
        pass
        
    def on_exit(self, userdata):
        self.info("GetOrderTableNnumState 종료")
        pass

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
