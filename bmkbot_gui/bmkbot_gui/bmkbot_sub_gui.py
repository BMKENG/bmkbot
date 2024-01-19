
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QTimer
from std_msgs.msg import Int32



# UI 파일 경로 확장
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

# UI 파일 경로 확장
ui_file_path = os.path.expanduser("~/ros2_ws/src/bmkbot/bmkbot_gui/ui/bmkbot_sub.ui")

# UI 파일 연결
form_class = uic.loadUiType(ui_file_path)[0]


# sub window
class SubWindow(QMainWindow, form_class):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setupUi(self)  # Setup the UI from the .ui file
        self.setWindowTitle("Order Table")
        self.table_num = getattr(self.ros_node, 'table_num', None)  # Safely get table_num or default to None
        # 해당 클레스에 order_table이라는 변수가 있으면 가져오고 없으면 None을 가져온다.
        self.order_table_num = getattr(self.ros_node, 'order_table_num', None)

        self.init_ui()

    def init_ui(self):

        # surving button
        self.serving.clicked.connect(self.button_clicked)
        self.timer = QTimer(self)
        self.timer.start(100)
        self.timer.timeout.connect(self.order_update)
        


    # 주문을 받았다는 것을 알려주는 창을 닫는다.
    # 해당 table로 이동 
    def button_clicked(self):
        self.ros_node.table_num = None
        msg = Int32()
        msg.data = int(self.table_num) + int(self.order_table_num) * 3
        self.ros_node.pub_start_nav.publish(msg)
        self.close()

    def order_update(self):
        print(self.table_num)
        self.table_num = str(self.table_num)
        self.order_table_num = str(self.order_table_num)
        # 0이면 복귀 
        if self.table_num != "0":
            self.statusBar().showMessage(self.table_num + "번 테이블에서 " +  self.order_table_num +  "번 가게의 주문이 들어왔습니다.")
        else:   
            self.statusBar().showMessage('주문하신 음식이 나왔습니다.')
        