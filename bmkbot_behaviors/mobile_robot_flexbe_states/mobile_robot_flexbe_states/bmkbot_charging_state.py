#!/usr/bin/env python

from rclpy.duration import Duration
from flexbe_core import EventState, logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Int32
# 배터리를 충전하는 state
class BMKBOTChargingState(EventState):

    def __init__(self, battery_topic="/battery"):

        super().__init__(outcomes=['done'], input_keys=['battery'], output_keys=['battery'])
        # 대기할 시간을 설정합니다.
    

        # 상태 변수 초기화
        self._start_time = None
        self.battery_topic = battery_topic
        self.pub_battery = ProxyPublisher({self.battery_topic: Int32})
        self.battery = Int32()
        self.one_sec = 0.0
    def on_enter(self, userdata):

        self._start_time = self._node.get_clock().now()


    def execute(self, userdata):
        """
        상태가 활성화되어 있는 동안 주기적으로 호출됩니다.
        지정된 시간이 경과했는지 확인하고, 경과했다면 'done'을 반환합니다.
        """
        
        # 지정된 시간이 경과했는지 확인합니다.
        self.one_sec += 0.5
        if self.one_sec >= 1.0:
            userdata.battery += 1
            self.one_sec = 0.0
            self.battery.data = userdata.battery
            if self.battery.data >= 100:
                self.battery.data = 100
                userdata.battery = self.battery.data
                self.pub_battery.publish(self.battery_topic, self.battery)
                return 'done'
            self.pub_battery.publish(self.battery_topic, self.battery)
            self._start_time = self._node.get_clock().now()
            


    def on_exit(self, userdata):
        pass
    
    def on_stop(self):
        pass