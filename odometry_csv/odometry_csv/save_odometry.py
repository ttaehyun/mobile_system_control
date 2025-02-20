import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os

class Float64ArrayCsvSaver(Node):
    def __init__(self):
        super().__init__('odometry_csv')
        
        # 구독할 토픽 이름 설정
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/mobile_system_control/ego_vehicle',  # 대상 토픽 이름
            self.listener_callback,
            10)
        
        # CSV 파일 생성
        self.csv_file_path = os.path.expanduser("~/odometry_path.csv")  # 저장 경로
        self.file = open(self.csv_file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # CSV 헤더 작성
        #self.writer.writerow(['timestamp', 'x', 'y'])  # 'x', 'y'로 구분
        self.get_logger().info(f"CSV 파일 저장 중: {self.csv_file_path}")

    def listener_callback(self, msg):
        # 메시지 데이터 추출
        #timestamp = self.get_clock().now().to_msg().sec  # 현재 시간
        data = msg.data

        # x, y 쌍으로 데이터 저장
       
        x = data[0]
        y = data[1]
        self.writer.writerow([x, y])
        self.get_logger().info(f"저장된 데이터: x={x}, y={y}")

    def destroy_node(self):
        # 노드 종료 시 파일 닫기
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Float64ArrayCsvSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
