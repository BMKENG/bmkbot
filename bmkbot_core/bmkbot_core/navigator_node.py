#! /usr/bin/env python3

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from bmkbot_interfaces.msg import NaviProgress
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class NavigatorNode(Node):
    def __init__(self):
        super().__init__(node_name='gui_basic_navigator')
        self.info('Navigator node started.')
        # initial pose
        self.initial_pose = Pose()
        self.initial_pose.orientation.w = 1.0

        # current amcl_pose
        self.current_pose = None
        # nav2 action client
        self.goal_handle = None
        self.result_future = None

        self.waypoint = [
            [-1.0, -1.0, 0.0, 0.0],
            [1.0, 2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0, 0.0]
        ]

        # feedback을 이용한 진행률 계산 및 출력
        self.feedback_flag = False
        self.feedback = None
        # 초기 거리
        self.initial_distance = 0.0
        
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self._amclPoseCallback,
                                                       amcl_pose_qos)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        # qt gui에서 navigator를 제어하기 위한 subscriber
        self.sub_navigator = self.create_subscription(String, 'navigator', self.navigatorCallback, 10)

        # nav 진행율 표시를 위한 publisher
        self.pub_nav_progress = self.create_publisher(NaviProgress, 'remaining_waypoint', 10)

        self.control_dict = {
            'set_waypoint_1': lambda: self.set_waypoint(0),
            'set_waypoint_2': lambda: self.set_waypoint(1),
            'set_waypoint_3': lambda: self.set_waypoint(2),
            'go_to_pose_1': lambda: self.goToPose(0),
            'go_to_pose_2': lambda: self.goToPose(1),
            'go_to_pose_3': lambda: self.goToPose(2),
            'start_nav': self.setInitialPose,
        }

    def navigatorCallback(self, msg):
        if msg.data in self.control_dict:
            self.control_dict[msg.data]()
            self.info('Navigation command: ' + msg.data)
        else:
            self.warn('Invalid navigation command: ' + msg.data)

    def setInitialPose(self):
        self.info('setInitialPose')
        self._setInitialPose()
        return

    def set_waypoint(self, waypoint_num):
        if self.current_pose is not None:
            self.waypoint[waypoint_num][0] = self.current_pose.position.x
            self.waypoint[waypoint_num][1] = self.current_pose.position.y
            self.waypoint[waypoint_num][2] = self.current_pose.orientation.w
            self.waypoint[waypoint_num][3] = self.current_pose.orientation.z
            
            self.info('waypoint_{}: {}'.format(waypoint_num + 1, self.waypoint[waypoint_num]))
        else:
            self.warn('아직 유효한 waypoint가 저장되지 않았습니다.')
            pass

    def goToPose(self, waypoint_num):
        # Sends a `NavToPose` action request and waits for completion
        self.current_waypoint_num = waypoint_num
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.waypoint[waypoint_num][0]
        goal_msg.pose.pose.position.y = self.waypoint[waypoint_num][1]
        goal_msg.pose.pose.orientation.w = self.waypoint[waypoint_num][2]
        goal_msg.pose.pose.orientation.z = self.waypoint[waypoint_num][3]

        self.info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                      str(goal_msg.pose.pose.position.y) + '...')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        self.feedback_flag  = False
        return True

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return
    
    # feedback을 이용한 진행률 계산 및 출력
    def getFeedback(self, num):
        if self.feedback_flag == False:
            # 현재 좌표와 목표 좌표의 거리를 구함
            distance = ((self.current_pose.position.x - self.waypoint[num][0]) ** 2 + (self.current_pose.position.y - self.waypoint[num][1]) ** 2) ** 0.5
            self.initial_distance = distance
            self.feedback_flag = True

        self.remaining_distance = self.feedback.distance_remaining / self.initial_distance * 100

        if self.remaining_distance < 0.0:
            self.remaining_distance = 0.0
        elif self.remaining_distance > 100.0:
            self.remaining_distance = 100.0

        # nav 진행율 표시를 위한 publisher
        self.info('init: ' +  str(self.initial_distance) + 'remain: ' + str(100 - self.remaining_distance) + 'feedback: ' + str(self.feedback.distance_remaining))
        msg = NaviProgress()
        msg.remaining_waypoint[0] = 100
        msg.remaining_waypoint[1] = 100
        msg.remaining_waypoint[2] = 100
        
        msg.remaining_waypoint[num] = int(100 - self.remaining_distance)
        self.pub_nav_progress.publish(msg)
        return self.remaining_distance


    def _amclPoseCallback(self, msg):
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True
        return
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.getFeedback(self.current_waypoint_num)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def main(args=None):
    rclpy.init(args=args)

    navigator = NavigatorNode()
    while True:
        rclpy.spin_once(navigator)
 

if __name__ == '__main__':
    main()