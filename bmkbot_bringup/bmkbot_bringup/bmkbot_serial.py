
import math 
import serial
import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState 

from time import sleep

# 멀티 스레드
from rclpy.executors import MultiThreadedExecutor



class RPMOdom(Node):
    def __init__(self):
        super().__init__('bmkbot_serial')
        # 오돔 값을 발행하기 위한 publisher
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.odometry_tf_publisher = TransformBroadcaster(self)

        self.serial_port = '/dev/ttyACM0'  # 시리얼 포트 경로
        self.baud_rate = 115200  # 시리얼 통신 속도

        self.wheel_radius = 0.0315  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.2173  # 바퀴 베이스 (단위: m)/

        # 초기 위치와 방향
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0

        # 타이머 설정
        self.timer_period = 0.01 # 20Hz
        self.timer = self.create_timer(self.timer_period, self.read_rpm)
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        # 바퀴 초기 각도
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

            
    def read_rpm(self):
        try:
            # 시리얼 포트로부터 rpm 값을 읽어옴

            received_data = self.ser.readline().decode().strip()
            # data example: (100, 100)
            # 데이터 길이 검증 및 정상 데이터인 경우
            if received_data.startswith('(') and received_data.endswith(')') and received_data.count('(') == 1 and received_data.count(')') == 1:
                # print(received_data)
                split_data = received_data.split(',')
                data1 = split_data[0][1:]
                data2 = split_data[1][:-1]

                print(data1, data2)
                # rp.loginfo("data1: %s, data2: %s", data1, data2)
                # 문자열을 부동 소수점으로 변환
                left_rpm = float(data2) / 6000.0 
                right_rpm = float(data1) / 6000.0
                # print(left_rpm, right_rpm)
                # 좌측, 우측 바퀴의 선속도 계산
                left_wheel_vel = (2 * math.pi * self.wheel_radius * left_rpm) 
                right_wheel_vel = (2 * math.pi * self.wheel_radius * right_rpm)
                # print(left_wheel_vel, right_wheel_vel)
                # 이동 거리 및   방향 갱신
                distance = (left_wheel_vel + right_wheel_vel) / 2.0
                self.orientation_z += (right_wheel_vel - left_wheel_vel) / self.wheel_base
                # print(distance, self.orientation_z)
                # 위치 업데이트
                self.position_x += distance * math.cos(self.orientation_z)
                self.position_y += distance * math.sin(self.orientation_z)

                # 오도메트리 메시지 생성 및 발행
                odometry = Odometry()
                odometry.header = Header()
                odometry.header.stamp = self.get_clock().now().to_msg()
                odometry.header.frame_id = 'odom'
                odometry.child_frame_id = 'base_footprint'
                odometry.pose.pose.position.x = self.position_x
                odometry.pose.pose.position.y = self.position_y
                odometry.pose.pose.orientation.z = math.sin(self.orientation_z / 2.0)
                odometry.pose.pose.orientation.w = math.cos(self.orientation_z / 2.0)
                self.odometry_publisher.publish(odometry)

                # TF 메시지 생성 및 발행
                odom_tf = TransformStamped()
                odom_tf.header.stamp = self.get_clock().now().to_msg()
                odom_tf.header.frame_id = odometry.header.frame_id
                odom_tf.child_frame_id = odometry.child_frame_id
                odom_tf.transform.translation.x = odometry.pose.pose.position.x
                odom_tf.transform.translation.y = odometry.pose.pose.position.y
                odom_tf.transform.translation.z = odometry.pose.pose.position.z 
                odom_tf.transform.rotation = odometry.pose.pose.orientation
                self.odometry_tf_publisher.sendTransform(odom_tf)


                #  joint_state 메시지 생성 및 발행
                self.left_wheel_angle += left_wheel_vel * 0.6 / (self.wheel_radius)
                self.right_wheel_angle += right_wheel_vel * 0.6 / (self.wheel_radius) 
                joint_state = JointState()
                joint_state.header.frame_id = 'base_footprint'
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['left_wheel_joint', 'right_wheel_joint']  # 관절 이름 설정
                joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]  # 관절의 위치(각도) 설정
                joint_state.velocity = [left_wheel_vel, right_wheel_vel]  # 관절의 속도 설정
                joint_state.effort = []  # 관절의 힘 또는 토크 (선택적)

                self.joint_state_publisher.publish(joint_state)
    
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            self.ser.close()
            sleep(0.001)  # 재연결 전에 잠시 대기
            self.ser.open()



class cmd_vel_2_rpm(Node):
    def __init__(self):
        super().__init__('cmd_vel_2_rpm')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.serial_port = '/dev/ttyACM0'  # 시리얼 포트 경로
        self.baud_rate = 115200  # 시리얼 통신 속도
        self.wheel_radius = 0.0315  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.1673  # 바퀴 베이스 (단위: m)/
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        self.ser.write(b"(0, 0)\n")  # 초기 속도 0으로 설정

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x  # 선속도(m/s)
        angular_vel = msg.angular.z  # 각속도(rad/s)

        # 좌측, 우측 바퀴의 선속도 계산
        right_wheel_vel = (linear_vel - angular_vel * self.wheel_base / 2) / self.wheel_radius
        left_wheel_vel = (linear_vel + angular_vel * self.wheel_base / 2) / self.wheel_radius

        # 좌측 바퀴 RPM 값을 발행
        left_wheel_rpm = (left_wheel_vel * 60 / (2 * 3.14159))
        # 우측 바퀴 RPM 값을 발행
        right_wheel_rpm = (right_wheel_vel * 60 / (2 * 3.14159))
        # 시리얼 포트로 RPM 값을 전송
        # data example: (100, 100)
        send_data = f"({left_wheel_rpm}, {right_wheel_rpm})\n" 
        self.ser.write(send_data.encode())



def main(args=None):
    rp.init(args=args)
    odom_pub = RPMOdom()
    cmd_vel_sub= cmd_vel_2_rpm()

    executor = MultiThreadedExecutor()
    executor.add_node(odom_pub)
    executor.add_node(cmd_vel_sub)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odom_pub.destroy_node()
        cmd_vel_sub.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()