import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv
import math

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def getAllElements(cursor, table_name, print_out=False):
    """ Returns a dictionary with all elements of the table database. """
    cursor.execute(f'SELECT * from({table_name})')
    return cursor.fetchall()

def isTopic(cursor, topic_name):
    """ Returns topic_name header if it exists. """
    records = getAllElements(cursor, 'topics')
    for row in records:
        if row[1] == topic_name:
            return row
    return None

def getAllMessagesInTopic(cursor, topic_name):
    """ Returns all timestamps and messages in the topic. """
    topic = isTopic(cursor, topic_name)
    if not topic:
        print(f'Topic {topic_name} not found.')
        return [], []

    records = getAllElements(cursor, 'messages')
    timestamps = [row[2] for row in records if row[1] == topic[0]]
    messages = [row[3] for row in records if row[1] == topic[0]]
    return timestamps, messages

def getAllTopicsNames(cursor):
    """ Returns all topics names. """
    return [row[1] for row in getAllElements(cursor, 'topics')]

def getAllMsgsTypes(cursor):
    """ Returns all message types. """
    return [row[2] for row in getAllElements(cursor, 'topics')]

def get_message_type(cursor, topic_name):
    """ Returns the message type of that specific topic. """
    topic_names = getAllTopicsNames(cursor)
    msg_types = getAllMsgsTypes(cursor)
    return msg_types[topic_names.index(topic_name)] if topic_name in topic_names else None

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    Roll (x-axis rotation), Pitch (y-axis rotation), Yaw (z-axis rotation)
    The result is returned in degrees.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    roll_x = math.degrees(roll_x)
    pitch_y = math.degrees(pitch_y)
    yaw_z = math.degrees(yaw_z)

    return roll_x, pitch_y, yaw_z

if __name__ == "__main__":

    
    csv_file_path = '/home/atharva/Downloads/15min.csv'

    
    bag_file = '/home/atharva/Downloads/15min.db3'

    topic_name = '/imu'

    conn, c = connect(bag_file)

    timestamps, messages = getAllMessagesInTopic(c, topic_name)

    msg_type_str = get_message_type(c, topic_name)
    msg_type = get_message(msg_type_str)

    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        csv_writer.writerow([
            'Secs', 'Nsecs',
            'Quaternion X', 'Quaternion Y', 'Quaternion Z', 'Quaternion W',
            'Acceleration X', 'Acceleration Y', 'Acceleration Z',
            'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
            'Magnetic Field X', 'Magnetic Field Y', 'Magnetic Field Z',
            'Roll', 'Pitch', 'Yaw'  
        ])

        for message in messages:
            deserialized_msg = deserialize_message(message, msg_type)

            secs = deserialized_msg.header.stamp.sec
            nsecs = deserialized_msg.header.stamp.nanosec
            orientation_x = deserialized_msg.imu.orientation.x
            orientation_y = deserialized_msg.imu.orientation.y
            orientation_z = deserialized_msg.imu.orientation.z
            orientation_w = deserialized_msg.imu.orientation.w
            acc_x = deserialized_msg.imu.linear_acceleration.x
            acc_y = deserialized_msg.imu.linear_acceleration.y
            acc_z = deserialized_msg.imu.linear_acceleration.z
            gyro_x = deserialized_msg.imu.angular_velocity.x
            gyro_y = deserialized_msg.imu.angular_velocity.y
            gyro_z = deserialized_msg.imu.angular_velocity.z
            mag_x = deserialized_msg.mag_field.magnetic_field.x
            mag_y = deserialized_msg.mag_field.magnetic_field.y
            mag_z = deserialized_msg.mag_field.magnetic_field.z

            roll, pitch, yaw = quaternion_to_euler(orientation_x, orientation_y, orientation_z, orientation_w)

            csv_writer.writerow([
                secs, nsecs,
                orientation_x, orientation_y, orientation_z, orientation_w,
                acc_x, acc_y, acc_z,
                gyro_x, gyro_y, gyro_z,
                mag_x, mag_y, mag_z,
                roll, pitch, yaw  
            ])

    close(conn)
