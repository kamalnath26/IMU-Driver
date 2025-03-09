import rosbag
import csv
import os

# Function to parse the $VNYMR string and extract values
def parse_vn_read(vn_read_string):
    # Strip any leading or trailing whitespace from the string
    vn_read_string = vn_read_string.strip()

    # Split the string based on commas (,) and handle the '*' character in the last value
    split_data = vn_read_string.split(',')

    # The last item in the list may contain '*' at the end, so we will remove it
    if '*' in split_data[-1]:
        split_data[-1] = split_data[-1].split('*')[0]

    if len(split_data) == 13:  # We should have 12 values now
        try:
            # Convert values to floats (except for the first part which is the header)
            yaw = float(split_data[1])
            pitch = float(split_data[2])
            roll = float(split_data[3])
            mag_x = float(split_data[4])
            mag_y = float(split_data[5])
            mag_z = float(split_data[6])
            acc_x = float(split_data[7])
            acc_y = float(split_data[8])
            acc_z = float(split_data[9])
            gyro_x = float(split_data[10])
            gyro_y = float(split_data[11])
            gyro_z = float(split_data[12])

            # Return the parsed values as a tuple
            return yaw, pitch, roll, mag_x, mag_y, mag_z, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
        except ValueError as e:
            # If there was an issue converting any value, return None
            print(f"Error parsing values: {e}")
            return None
    else:
        # If the data doesn't match the expected format, return None
        print(f"Unexpected data format: {vn_read_string}")
        return None

# Function to read the rosbag and extract the data
def extract_data_from_rosbag(bag_file, output_csv_file):
    try:
        # Open the rosbag file
        with rosbag.Bag(bag_file, 'r') as bag:
            # Open the CSV file for writing
            with open(output_csv_file, mode='w', newline='') as csv_file:
                # CSV writer object
                fieldnames = ['seq', 'time_stamp', 'yaw', 'pitch', 'roll', 'mag_x', 'mag_y', 'mag_z', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                
                # Write the header
                writer.writeheader()

                # Iterate through the messages in the bag
                for topic, msg, t in bag.read_messages(topics=['/vectornav']):
                    if topic == '/vectornav' and hasattr(msg, 'data'):
                        # Extract the $VNYMR message
                        vn_read_string = msg.data.strip()  # Ensure any leading/trailing whitespaces are removed
                        
                        # Parse the $VNYMR string to extract data
                        data = parse_vn_read(vn_read_string)
                        if data:
                            # Prepare the data for writing to CSV
                            row = {
                                'seq': msg.header.seq,
                                'time_stamp': t.to_sec(),
                                'yaw': data[0],
                                'pitch': data[1],
                                'roll': data[2],
                                'mag_x': data[3],
                                'mag_y': data[4],
                                'mag_z': data[5],
                                'acc_x': data[6],
                                'acc_y': data[7],
                                'acc_z': data[8],
                                'gyro_x': data[9],
                                'gyro_y': data[10],
                                'gyro_z': data[11]
                            }
                            # Write the row to the CSV file
                            writer.writerow(row)
                            print(f"Data from seq {msg.header.seq} written to CSV")

        print(f"Data extraction complete. CSV saved to {output_csv_file}")
    except Exception as error:
        print(f"Error during data extraction: {error}")

# Main entry point
if __name__ == "__main__":
    # Define the path to the rosbag file and output CSV file
    current_path = os.path.join(os.getcwd(), "../data/")
    imu_data_bag_path = os.path.join(current_path, 'LocationA.bag')  # Replace with the path to your rosbag file
    output_csv_file = os.path.join(current_path, 'imu_data.csv')  # Replace with your desired output CSV file name
    
    # Call the function to extract data from rosbag and save to CSV
    extract_data_from_rosbag(imu_data_bag_path, output_csv_file)
