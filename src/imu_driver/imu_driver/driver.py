import rclpy
from datetime import datetime
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from rclpy.node import Node 
import serial
from std_msgs.msg import Header
from imu_msg.msg import *
from imu_msg.msg import IMUmsg

class IMU_Driver(Node):
    def __init__(self):
        super().__init__('imuDriver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('sampling_rate', 10)

        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value
        serial_sampling_rate = self.get_parameter('sampling_rate').value
        
        self.port = serial.Serial(serial_port, serial_baud, timeout=3.0)
        self.port.write(b"$VNYMR,07,40*xx")
        
        self.timer = self.create_timer(0.01, self.read_imu_data)
        
        self.data_pub = self.create_publisher(IMUmsg, 'imu', 10)
    
    def read_imu_data(self):
        try:
            rx_data = self.port.readline().decode().strip()

            if "$VNYMR" in rx_data:
                component = rx_data.split(",")

                if len(component) < 13:
                    raise ValueError(f"Data received is incomplete or corrupted: {rx_data}")

                msg = IMUmsg()
                yawD = float(component[1])
                pitchD= float(component[2])
                rollD = float(component[3])
                yawR = np.deg2rad(float(component[1]))
                pitchR= np.deg2rad(float(component[2]))
                rollR = np.deg2rad(float(component[3]))
                magx = float(component[4]) * 1e-4
                magy = float(component[5]) * 1e-4
                magz = float(component[6]) * 1e-4
                accx = float(component[7])
                accy = float(component[8])
                accz = float(component[9])
                gyrox = float(component[10])
                gyroy = float(component[11])
                gyroz = float(component[12][0:9])  

                qx = np.sin(rollR / 2) * np.cos(pitchR / 2) * np.cos(yawR / 2) - np.cos(rollR / 2) * np.sin(pitchR / 2) * np.sin(yawR / 2)
                qy = np.cos(rollR / 2) * np.sin(pitchR / 2) * np.cos(yawR / 2) + np.sin(rollR / 2) * np.cos(pitchR / 2) * np.sin(yawR / 2)
                qz = np.cos(rollR / 2) * np.cos(pitchR / 2) * np.sin(yawR / 2) - np.sin(rollR / 2) * np.sin(pitchR / 2) * np.cos(yawR / 2)
                qw = np.cos(rollR / 2) * np.cos(pitchR / 2) * np.cos(yawR / 2) + np.sin(rollR / 2) * np.sin(pitchR / 2) * np.sin(yawR / 2)

                now = datetime.now()
                hours = now.hour
                minutes = now.minute
                seconds = now.second
                microseconds = now.microsecond

                secs = hours * 3600 + minutes * 60 + seconds
                nsecs = microseconds * 1000

                euler_angles_str = f"Roll: {rollD:.6f}, Pitch: {pitchD:.6f}, Yaw: {yawD:.6f}"
        
                msg.header.stamp.sec = secs  
                msg.header.stamp.nanosec = nsecs  
                msg.header.frame_id = 'IMU1_Frame'
                msg.imu.orientation.x = qx
                msg.imu.orientation.y = qy
                msg.imu.orientation.z = qz
                msg.imu.orientation.w = qw
                msg.imu.linear_acceleration.x = accx
                msg.imu.linear_acceleration.y = accy
                msg.imu.linear_acceleration.z = accz
                msg.imu.angular_velocity.x = gyrox
                msg.imu.angular_velocity.y = gyroy
                msg.imu.angular_velocity.z = gyroz
                msg.mag_field.magnetic_field.x = magx
                msg.mag_field.magnetic_field.y = magy
                msg.mag_field.magnetic_field.z = magz

                msg.euler_angles = euler_angles_str

                self.data_pub.publish(msg)
                self.get_logger().info(f"IMU Data Published: {msg}")
                # self.get_logger().info(f"IMU Data Published: {now}")

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Invalid or incomplete IMU data received: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_driver = IMU_Driver()

    try:
        rclpy.spin(imu_driver)
    except KeyboardInterrupt:
        imu_driver.get_logger().info('IMU Driver shutting down')
    finally:
        imu_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
