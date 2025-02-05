import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class IMUPlotter:
    def __init__(self, csv_file):
        self.data = pd.read_csv(csv_file)

        self.time_values = (self.data['Secs'] + self.data['Nsecs'] * 1e-9).to_numpy()
        # self.time_secs = self.data['Secs'].to_numpy()
        # self.time_nsecs = self.data['Nsecs'].to_numpy()
        self.accel_x = self.data['Acceleration X'].to_numpy()
        self.accel_y = self.data['Acceleration Y'].to_numpy()
        self.accel_z = self.data['Acceleration Z'].to_numpy()
        self.gyro_x = self.data['Angular Velocity X'].to_numpy()
        self.gyro_y = self.data['Angular Velocity Y'].to_numpy()
        self.gyro_z = self.data['Angular Velocity Z'].to_numpy()
        self.mag_x = self.data['Magnetic Field X'].to_numpy()
        self.mag_y = self.data['Magnetic Field Y'].to_numpy()
        self.mag_z = self.data['Magnetic Field Z'].to_numpy()
        self.roll_degrees = self.data['Roll'].to_numpy()
        self.pitch_degrees = self.data['Pitch'].to_numpy()
        self.yaw_degrees = self.data['Yaw'].to_numpy()

       
        self.setup_plots()

    def setup_plots(self):
        
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.title("Magnetic Field X")
        plt.plot(self.time_values, self.mag_x, label="Magnetic Field X (µT)", color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("µT")

        plt.subplot(3, 1, 2)
        plt.title("Magnetic Field Y")
        plt.plot(self.time_values, self.mag_y, label="Magnetic Field Y (µT)", color='g')
        plt.xlabel("Time (s)")
        plt.ylabel("µT")

        plt.subplot(3, 1, 3)
        plt.title("Magnetic Field Z")
        plt.plot(self.time_values, self.mag_z, label="Magnetic Field Z (µT)", color='b')
        plt.xlabel("Time (s)")
        plt.ylabel("µT")

        plt.tight_layout()
        plt.show()

        
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.title("Acceleration X")
        plt.plot(self.time_values, self.accel_x, label="Acceleration X (m/s²)", color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("m/s²")

        plt.subplot(3, 1, 2)
        plt.title("Acceleration Y")
        plt.plot(self.time_values, self.accel_y, label="Acceleration Y (m/s²)", color='g')
        plt.xlabel("Time (s)")
        plt.ylabel("m/s²")

        plt.subplot(3, 1, 3)
        plt.title("Acceleration Z")
        plt.plot(self.time_values, self.accel_z, label="Acceleration Z (m/s²)", color='b')
        plt.xlabel("Time (s)")
        plt.ylabel("m/s²")

        plt.tight_layout()
        plt.show()

        
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.title("Gyroscope X")
        plt.plot(self.time_values, self.gyro_x, label="Angular Velocity X (rad/s)", color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("rad/s")

        plt.subplot(3, 1, 2)
        plt.title("Gyroscope Y")
        plt.plot(self.time_values, self.gyro_y, label="Angular Velocity Y (rad/s)", color='g')
        plt.xlabel("Time (s)")
        plt.ylabel("rad/s")

        plt.subplot(3, 1, 3)
        plt.title("Gyroscope Z")
        plt.plot(self.time_values, self.gyro_z, label="Angular Velocity Z (rad/s)", color='b')
        plt.xlabel("Time (s)")
        plt.ylabel("rad/s")

        plt.tight_layout()
        plt.show()

        
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.title("Roll (degrees)")
        plt.plot(self.time_values, self.roll_degrees, label="Roll", color='c')
        plt.xlabel("Time (s)")
        plt.ylabel("Roll (degrees)")

        plt.subplot(3, 1, 2)
        plt.title("Pitch (degrees)")
        plt.plot(self.time_values, self.pitch_degrees, label="Pitch", color='m')
        plt.xlabel("Time (s)")
        plt.ylabel("Pitch (degrees)")

        plt.subplot(3, 1, 3)
        plt.title("Yaw (degrees)")
        plt.plot(self.time_values, self.yaw_degrees, label="Yaw", color='y')
        plt.xlabel("Time (s)")
        plt.ylabel("Yaw (degrees)")

        plt.tight_layout()
        plt.show()

def main():
    
    csv_file = '/home/atharva/Downloads/15_mins_data_new_0.csv'
    
    imu_plotter = IMUPlotter(csv_file)

if __name__ == '__main__':
    main()
