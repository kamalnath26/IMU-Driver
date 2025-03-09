import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import allantools as alan


# Read the CSV file
current_path = os.path.join(os.getcwd(), "../data/")
csv_file_path = os.path.join(current_path, 'imu_data.csv')  # Change this to your actual CSV file path
df = pd.read_csv(csv_file_path)

# Extract gyro and accelerometer data
gyro_x = df['gyro_x'].values
gyro_y = df['gyro_y'].values
gyro_z = df['gyro_z'].values
accel_x = df['acc_x'].values
accel_y = df['acc_y'].values
accel_z = df['acc_z'].values
d_time = df['time_stamp'].values

# Calculate the time step (dt) based on the time difference
dt = np.mean(np.diff(d_time))

# Convert raw data into numpy arrays for efficient calculations
gyro_x_np, gyro_y_np, gyro_z_np = np.array(gyro_x), np.array(gyro_y), np.array(gyro_z)
accel_x_np, accel_y_np, accel_z_np = np.array(accel_x), np.array(accel_y), np.array(accel_z)

# Integrate the gyroscope data to estimate angles in radians
angles_x = np.cumsum(gyro_x_np) * dt
angles_y = np.cumsum(gyro_y_np) * dt
angles_z = np.cumsum(gyro_z_np) * dt

# Perform Allan deviation analysis for gyroscope angles (rad/s)
taus_gx, allan_dev_gx, _, _ = alan.oadev(angles_x, rate=1/dt, data_type="phase", taus="all")
taus_gy, allan_dev_gy, _, _ = alan.oadev(angles_y, rate=1/dt, data_type="phase", taus="all")
taus_gz, allan_dev_gz, _, _ = alan.oadev(angles_z, rate=1/dt, data_type="phase", taus="all")

# Perform Allan deviation analysis for accelerometer data (m/s²)
taus_ax, allan_dev_ax, _, _ = alan.oadev(accel_x_np, rate=1/dt, data_type="freq", taus="all")
taus_ay, allan_dev_ay, _, _ = alan.oadev(accel_y_np, rate=1/dt, data_type="freq", taus="all")
taus_az, allan_dev_az, _, _ = alan.oadev(accel_z_np, rate=1/dt, data_type="freq", taus="all")

# Function to extract noise characteristics from Allan deviation results
def get_noise_parameters(taus, allan_dev):
    # Bias Instability: Local minimum of the Allan deviation
    min_idx = np.argmin(allan_dev)
    bias_inst = allan_dev[min_idx]
    min_tau = taus[min_idx]
    
    # Angle Random Walk: Value at tau = 1 second
    idx_1s = np.argmin(np.abs(taus - 1))
    angle_rw = allan_dev[idx_1s]
    
    # Rate Random Walk: Slope of Allan deviation in the flat region after the local minimum
    log_taus, log_allan_dev = np.log10(taus), np.log10(allan_dev)
    slope = np.diff(log_allan_dev) / np.diff(log_taus)
    rate_rw_idx = np.where((slope >= 0.4) & (slope <= 0.6))[0]
    rate_rw = allan_dev[rate_rw_idx[0]] * np.sqrt(taus[rate_rw_idx[0]]) if rate_rw_idx.size > 0 else None

    return angle_rw, bias_inst, rate_rw, min_tau

# Extract noise parameters for each gyro axis
gyro_noise_params = [
    get_noise_parameters(taus_gx, allan_dev_gx),
    get_noise_parameters(taus_gy, allan_dev_gy),
    get_noise_parameters(taus_gz, allan_dev_gz)
]

# Extract noise parameters for each accelerometer axis
accel_noise_params = [
    get_noise_parameters(taus_ax, allan_dev_ax),
    get_noise_parameters(taus_ay, allan_dev_ay),
    get_noise_parameters(taus_az, allan_dev_az)
]

# Plot Allan deviation for gyroscope angles (X, Y, Z)
fig, axes = plt.subplots(3, 1, figsize=(10, 12))
for i, (taus, allan_dev, params, color, axis_label) in enumerate(zip([taus_gx, taus_gy, taus_gz],
                                                                      [allan_dev_gx, allan_dev_gy, allan_dev_gz],
                                                                      gyro_noise_params,
                                                                      ['red', 'green', 'blue'],
                                                                      ['X', 'Y', 'Z'])):
    angle_rw, bias_inst, rate_rw, min_tau = params
    axes[i].loglog(taus, allan_dev, color=color, label=f'Gyro {axis_label} Allan deviation\nAngle RW={angle_rw:.2e}, Bias Inst={bias_inst:.2e}, Rate RW={rate_rw}')
    axes[i].plot(min_tau, bias_inst, 'o', label="Bias Instability")
    axes[i].set_xlabel('Time (s)')
    axes[i].set_ylabel(f'Allan Deviation [rad/s]')
    axes[i].legend()
    axes[i].grid(True)
    print(f"Gyro : Bias Instability = {bias_inst}, Angle Random Walk = {angle_rw}, Rate Random Walk = {rate_rw}")
fig.suptitle('Gyroscope Allan Deviation Analysis')
plt.tight_layout()

# Plot Allan deviation for accelerometer data (X, Y, Z)
fig, axes = plt.subplots(3, 1, figsize=(10, 12))
for i, (taus, allan_dev, params, color, axis_label) in enumerate(zip([taus_ax, taus_ay, taus_az],
                                                                      [allan_dev_ax, allan_dev_ay, allan_dev_az],
                                                                      accel_noise_params,
                                                                      ['purple', 'orange', 'brown'],
                                                                      ['X', 'Y', 'Z'])):
    angle_rw, bias_inst, rate_rw, min_tau = params
    axes[i].loglog(taus, allan_dev, color=color, label=f'Accel {axis_label} Allan deviation\nAngle RW={angle_rw:.2e}, Bias Inst={bias_inst:.2e}, Rate RW={rate_rw}')
    axes[i].plot(min_tau, bias_inst, 'o', label="Bias Instability")
    axes[i].set_xlabel('Time (s)')
    axes[i].set_ylabel(f'Allan Deviation [m/s²]')
    axes[i].legend()
    axes[i].grid(True)
    print(f"Accel : Bias Instability = {bias_inst}, Angle Random Walk = {angle_rw}, Rate Random Walk = {rate_rw}")
fig.suptitle('Accelerometer Allan Deviation Analysis')
plt.tight_layout()
plt.show()