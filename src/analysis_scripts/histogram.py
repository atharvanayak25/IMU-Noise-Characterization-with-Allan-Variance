import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class IMUPlotter:
    def __init__(self, csv_file):
        # Load CSV data
        self.data = pd.read_csv(csv_file)

        # Extract sensor data from columns and convert to NumPy arrays
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

        # Initialize histogram plots
        self.plot_histograms()

    def plot_histograms(self):
        # Function to calculate appropriate x-axis range and exclude outliers
        def get_range(data, outlier_threshold=0.05):
            lower_percentile = np.percentile(data, 100 * outlier_threshold)
            upper_percentile = np.percentile(data, 100 * (1 - outlier_threshold))
            padding = (upper_percentile - lower_percentile) * 0.05  # Reduced padding for better range
            return (lower_percentile - padding, upper_percentile + padding)

        # Plot histograms for Acceleration X, Y, Z in the same window
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.accel_x, bins=50, color='r', alpha=0.7)
        plt.title("Histogram of Acceleration X (m/s²)")
        plt.xlabel("Acceleration X (m/s²)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.accel_y, bins=50, color='g', alpha=0.7)
        plt.title("Histogram of Acceleration Y (m/s²)")
        plt.xlabel("Acceleration Y (m/s²)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.accel_z, bins=50, color='b', alpha=0.7)
        plt.title("Histogram of Acceleration Z (m/s²)")
        plt.xlabel("Acceleration Z (m/s²)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()

        #Plot without eliminating the outliers
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.gyro_x, bins=100, color='r', alpha=0.7)
        plt.title("Histogram of Angular Velocity X (rad/s)")
        plt.xlabel("Angular Velocity X (rad/s)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.gyro_y, bins=100, color='g', alpha=0.7)
        plt.title("Histogram of Angular Velocity Y (rad/s)")
        plt.xlabel("Angular Velocity Y (rad/s)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.gyro_z, bins=100, color='b', alpha=0.7)
        plt.title("Histogram of Angular Velocity Z (rad/s)")
        plt.xlabel("Angular Velocity Z (rad/s)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()


        # Plot histograms for Gyroscope X, Y, Z in the same window, with outliers removed
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.gyro_x, bins=100, color='r', alpha=0.7, range=get_range(self.gyro_x), density=True)
        plt.title("Histogram of Angular Velocity X (rad/s)")
        plt.xlabel("Angular Velocity X (rad/s)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.gyro_y, bins=100, color='g', alpha=0.7, range=get_range(self.gyro_y), density=True)
        plt.title("Histogram of Angular Velocity Y (rad/s)")
        plt.xlabel("Angular Velocity Y (rad/s)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.gyro_z, bins=100, color='b', alpha=0.7, range=get_range(self.gyro_z), density=True)
        plt.title("Histogram of Angular Velocity Z (rad/s)")
        plt.xlabel("Angular Velocity Z (rad/s)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()


        #plot without eliminating outliers
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.mag_x, bins=100, color='r', alpha=0.7)
        plt.title("Histogram of Magnetic Field X (µT)")
        plt.xlabel("Magnetic Field X (µT)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.mag_y, bins=100, color='g', alpha=0.7)
        plt.title("Histogram of Magnetic Field Y (µT)")
        plt.xlabel("Magnetic Field Y (µT)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.mag_z, bins=100, color='b', alpha=0.7)
        plt.title("Histogram of Magnetic Field Z (µT)")
        plt.xlabel("Magnetic Field Z (µT)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()
        
        # Plot histograms for Magnetic Field X, Y, Z in the same window
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.mag_x, bins=100, color='r', alpha=0.7, range=get_range(self.mag_x), density=True)
        plt.title("Histogram of Magnetic Field X (µT)")
        plt.xlabel("Magnetic Field X (µT)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.mag_y, bins=100, color='g', alpha=0.7, range=get_range(self.mag_y), density=True)
        plt.title("Histogram of Magnetic Field Y (µT)")
        plt.xlabel("Magnetic Field Y (µT)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.mag_z, bins=100, color='b', alpha=0.7, range=get_range(self.mag_z), density=True)
        plt.title("Histogram of Magnetic Field Z (µT)")
        plt.xlabel("Magnetic Field Z (µT)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()

        # Plot histograms for Roll, Pitch, Yaw in the same window
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.hist(self.roll_degrees, bins=50, color='c', alpha=0.7)
        plt.title("Histogram of Roll (degrees)")
        plt.xlabel("Roll (degrees)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 2)
        plt.hist(self.pitch_degrees, bins=50, color='m', alpha=0.7)
        plt.title("Histogram of Pitch (degrees)")
        plt.xlabel("Pitch (degrees)")
        plt.ylabel("Frequency")

        plt.subplot(3, 1, 3)
        plt.hist(self.yaw_degrees, bins=50, color='y', alpha=0.7)
        plt.title("Histogram of Yaw (degrees)")
        plt.xlabel("Yaw (degrees)")
        plt.ylabel("Frequency")

        plt.tight_layout()
        plt.show()

def main():
    # CSV file input (replace 'imu_data.csv' with the path to your CSV file)
    csv_file = '/home/atharva/Downloads/15_mins_data_new_0.csv'
    
    imu_plotter = IMUPlotter(csv_file)

if __name__ == '__main__':
    main()
