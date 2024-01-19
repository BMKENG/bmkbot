
import rclpy

from std_msgs.msg import Int32

from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread

class ROSNode(QThread):

    def __init__(self):
        super().__init__()
        self._is_running = True
        self.table_num = None
        self.remaining_waypoint = None
        self.order_table_num = None

    def run(self):
        self.node = rclpy.create_node('bmkbot_qt_to_ros_node')

        # 주문을 위한 테이블 번호
        self.pub_order_table = self.node.create_publisher(Int32, '/order_table', 10)
        
        # 로봇이 테이블에 도착했을 때 이벤트 ui를 위한 테이블 번호
        self.sub_arrived_table = self.node.create_subscription(Int32, 'arrived_table', self.arrived_table_callback, 10)
        self.arrived_table = None
        # 로봇 출발을 위한 명령
        self.pub_start_nav = self.node.create_publisher(Int32, 'start_nav', 10)
        # 주문 받은 table number


        while rclpy.ok() and self._is_running:
            rclpy.spin_once(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    def terminate(self):
        self._is_running = False

    def arrived_table_callback(self, msg):
        self.arrived_table = msg.data
        print("arrived_table_callback: ", self.arrived_table)

