import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt


# Function to convert radians/s to degrees/s
def radians_to_degrees(radians):
    return radians * 180.0 / np.pi

def analyze_rosbag(bag_file):
    # Initialize lists to store time and sensor data
    time_stamps = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    accel_x = []
    accel_y = []
    accel_z = []
    yaw = []
    pitch = []
    roll = []

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate through the bag and extract the required messages
        for topic, msg, t in bag.read_messages(topics=['/imu']):
            # Extract time stamps
            time_stamps.append(t.to_sec())

            if topic == '/imu':
                # Extract Gyroscope (angular velocity) data
                gyro_x.append(msg.imu.angular_velocity.x)
                gyro_y.append(msg.imu.angular_velocity.x)
                gyro_z.append(msg.imu.angular_velocity.x)

                # Extract Accelerometer data (linear acceleration)
                accel_x.append(msg.imu.linear_acceleration.x)
                accel_y.append(msg.imu.linear_acceleration.x)
                accel_z.append(msg.imu.linear_acceleration.x)

                # Extract Orientation (VN estimation) data
                quat = msg.imu.orientation
                # Convert quaternion to Euler angles (yaw, pitch, roll)
                euler = quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
                yaw.append(euler[0])
                pitch.append(euler[1])
                roll.append(euler[2])

    # Convert to numpy arrays for easier manipulation
    time_stamps = np.array(time_stamps)
    gyro_x = np.array(gyro_x)
    gyro_y = np.array(gyro_y)
    gyro_z = np.array(gyro_z)
    accel_x = np.array(accel_x)
    accel_y = np.array(accel_y)
    accel_z = np.array(accel_z)
    yaw = np.array(yaw)
    pitch = np.array(pitch)
    roll = np.array(roll)

    # Convert gyro rates from radians/s to degrees/s
    gyro_x_deg = radians_to_degrees(gyro_x)
    gyro_y_deg = radians_to_degrees(gyro_y)
    gyro_z_deg = radians_to_degrees(gyro_z)

    # Plot the data
    # gyro_plot = plot_data(time_stamps, gyro_x_deg, gyro_y_deg, gyro_z_deg)
    # acceleration_plot = plot_data(time_stamps, accel_x, accel_y, accel_z)
    # vn_ext_plot = plot_data(time_stamps, yaw, pitch, roll)
    plot_data(time_stamps, gyro_x_deg, gyro_y_deg, gyro_z_deg, accel_x, accel_y, accel_z, yaw, pitch, roll)

    # # Plot the histograms for rotation in X, Y, Z (yaw, pitch, roll)
    # plot_histograms(yaw, pitch, roll)

# Function to convert quaternion to Euler angles (yaw, pitch, roll)
def quaternion_to_euler(x, y, z, w):
    # Calculate Euler angles from quaternion (assuming the Tait-Bryan convention)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z, pitch_y, roll_x

def plot_data(time_stamps, gyro_x_deg, gyro_y_deg, gyro_z_deg, accel_x, accel_y, accel_z, yaw, pitch, roll):
    # Plot Gyroscope data (in degrees/s)
    plt.figure()
    # plt.subplot(3, 1, 1)
    plt.plot(time_stamps - time_stamps[0], gyro_x_deg, label='Gyro X (deg/s)', marker='o')
    plt.plot(time_stamps - time_stamps[0], gyro_y_deg, label='Gyro Y (deg/s)', marker='x')
    plt.plot(time_stamps - time_stamps[0], gyro_z_deg, label='Gyro Z (deg/s)', marker='^')
    plt.xlabel('Time (s)')
    plt.ylabel('Gyro Rate (degrees/s)')
    plt.title('Gyroscope Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Plot Accelerometer data (in m/s^2)
    plt.figure()
    # plt.subplot(3, 1, 2)
    plt.plot(time_stamps - time_stamps[0], accel_x, label='Accel X (m/s^2)', marker='o')
    plt.plot(time_stamps - time_stamps[0], accel_y, label='Accel Y (m/s^2)', marker='x')
    plt.plot(time_stamps - time_stamps[0], accel_z, label='Accel Z (m/s^2)', marker='^')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('Accelerometer Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    

    # Plot Rotation from VN Estimation (in degrees)
    plt.figure()
    # plt.subplot(3, 1, 3)
    plt.plot(time_stamps - time_stamps[0], yaw, label='Yaw (deg)', marker='o')
    plt.plot(time_stamps - time_stamps[0], pitch, label='Pitch (deg)', marker='x')
    plt.plot(time_stamps - time_stamps[0], roll, label='Roll (deg)', marker='^')
    plt.xlabel('Time (s)')
    plt.ylabel('Rotation (degrees)')
    plt.title('Rotation from VN Estimation')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # plt.tight_layout()
    # plt.show()
    # Create a new figure for the histograms
    plt.figure()

    # Histogram for Yaw (Rotation around Z-axis)
    plt.subplot(3, 1, 1)
    plt.hist(yaw, bins=50, color='blue', edgecolor='black', alpha=0.7)
    plt.xlabel('Yaw (degrees)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Yaw (Rotation around Z-axis)')
    plt.grid(True)
    plt.tight_layout()

    # Histogram for Pitch (Rotation around Y-axis)
    # plt.figure()
    plt.subplot(3, 1, 2)
    plt.hist(pitch, bins=50, color='green', edgecolor='black', alpha=0.7)
    plt.xlabel('Pitch (degrees)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Pitch (Rotation around Y-axis)')
    plt.grid(True)
    plt.tight_layout()
    
    # Histogram for Roll (Rotation around X-axis)
    # plt.figure()
    plt.subplot(3, 1, 3)
    plt.hist(roll, bins=50, color='red', edgecolor='black', alpha=0.7)
    plt.xlabel('Roll (degrees)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Roll (Rotation around X-axis)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Path to your rosbag file
    current_path = os.path.join(os.getcwd(), "../data/")
    imu_data_bag_path = os.path.join(current_path, 'stationary_data.bag')
    analyze_rosbag(imu_data_bag_path)
