import socket
import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import threading
import queue
from collections import deque
import numpy as np

# UDP receiving settings
UDP_IP = "192.168.4.2"
UDP_PORT = 12345

# ESP8266 control settings
ESP8266_IP = "192.168.4.1"
ESP8266_PORT = 12345

# Data storage for visualization
max_points = 500  # Maximum number of points to keep in memory
x_coords = deque(maxlen=max_points)
y_coords = deque(maxlen=max_points)

# Thread-safe queue for communication between UDP thread and main thread
data_queue = queue.Queue(maxsize=1000)

# Duplicate filtering variables
last_distance = None
last_angle = None

# Statistics
packet_count = 0
valid_packets = 0
last_time = time.time()
running = True

# Control socket for sending commands to ESP8266
control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def parse_lidar_data(data_string):
    """
    Parse the "D:xxx A:xxx" format to extract distance and angle
    Returns: (distance, angle) tuple or None if parsing fails
    """
    try:
        # Use regex to find D: and A: patterns
        distance_match = re.search(r'D:(\d+(?:\.\d+)?)', data_string)
        angle_match = re.search(r'A:(\d+(?:\.\d+)?)', data_string)
        
        if distance_match and angle_match:
            distance = float(distance_match.group(1))
            angle = float(angle_match.group(1))
            return distance, angle
        else:
            return None
    except (ValueError, AttributeError):
        return None

def polar_to_cartesian(distance, angle_degrees):
    """
    Convert polar coordinates to Cartesian coordinates
    Args:
        distance: distance in mm (or whatever unit your sensor uses)
        angle_degrees: angle in degrees
    Returns:
        (x, y) tuple in Cartesian coordinates
    """
    angle_radians = math.radians(angle_degrees)
    x = distance * math.cos(angle_radians)
    y = distance * math.sin(angle_radians)
    return x, y

def send_esp8266_command(command):
    """
    Send START or STOP command to ESP8266
    """
    try:
        control_socket.sendto(command.encode(), (ESP8266_IP, ESP8266_PORT))
        print(f"Sent command: {command}")
    except Exception as e:
        print(f"Error sending command: {e}")

def on_key_press(event):
    """
    Handle keyboard events for controlling ESP8266
    """
    if event.key == 'l':
        send_esp8266_command("START")
    elif event.key == 'k':
        send_esp8266_command("STOP")
    elif event.key == 'c':
        # Clear the plot properly
        x_coords.clear()
        y_coords.clear()
        # Immediately update the scatter plot to show empty data
        scat.set_offsets(np.empty((0, 2)))
        plt.draw()
        print("Plot cleared")

def udp_receiver():
    """
    UDP receiver thread - runs independently to collect data at full speed
    """
    global packet_count, valid_packets, last_time, running
    global last_distance, last_angle
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)  # Short timeout to allow clean shutdown
    
    print(f"UDP receiver listening on {UDP_IP}:{UDP_PORT}...")
    
    try:
        while running:
            try:
                data, addr = sock.recvfrom(1024)
                decoded = data.decode().strip()
                packet_count += 1
                
                # Parse the lidar data
                parsed = parse_lidar_data(decoded)
                if parsed:
                    distance, angle = parsed
                    valid_packets += 1
                    
                    # Duplicate filtering
                    if last_distance is None or last_angle is None or distance != last_distance or angle != last_angle:
                        # Convert to Cartesian coordinates
                        x, y = polar_to_cartesian(distance, angle)
                        
                        # Put in queue for visualization thread
                        try:
                            data_queue.put((x, y), block=False)
                        except queue.Full:
                            # Drop oldest data if queue is full
                            try:
                                data_queue.get_nowait()
                                data_queue.put((x, y), block=False)
                            except queue.Empty:
                                pass
                        
                        last_distance = distance
                        last_angle = angle
                
                # Print statistics every second
                now = time.time()
                if now - last_time >= 1.0:
                    print(f"Packets/sec: {packet_count}, Valid: {valid_packets}, Queue size: {data_queue.qsize()}")
                    packet_count = 0
                    valid_packets = 0
                    last_time = now
                    
            except socket.timeout:
                continue  # Check running flag
            except Exception as e:
                if running:
                    print(f"Error in UDP receiver: {e}")
                
    finally:
        sock.close()
        print("UDP receiver stopped")

def update_plot(frame):
    """
    Animation function to update the plot - processes queued data
    """
    
    # Process all available data from queue
    points_added = 0
    max_points_per_frame = 50  # Limit points processed per frame for smooth animation
    
    while not data_queue.empty() and points_added < max_points_per_frame:
        try:
            x, y = data_queue.get_nowait()
            x_coords.append(x)
            y_coords.append(y)
            points_added += 1
        except queue.Empty:
            break
    
    # Update the scatter plot efficiently
    if len(x_coords) > 0:
        # Convert to numpy arrays for efficient plotting
        x_array = np.array(x_coords)
        y_array = np.array(y_coords)
        scat.set_offsets(np.column_stack((x_array, y_array)))
    
    # Update plot title with RPM
    ax.set_title(f'Real-time LiDAR Data Visualization')
    
    return scat,

# Set up the plot
plt.style.use('fast')  # Use fast rendering style
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-500, 500)  # Adjust based on your expected range
ax.set_ylim(-500, 500)  # Adjust based on your expected range
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('Real-time LiDAR Data Visualization\nControls: [L] Start | [K] Stop | [C] Clear')
ax.grid(True)
ax.set_aspect('equal')

# Create scatter plot
scat = ax.scatter([], [], c='red', s=1, alpha=0.6)

# Start UDP receiver thread
udp_thread = threading.Thread(target=udp_receiver, daemon=True)
udp_thread.start()

# Set up animation with faster update rate
ani = animation.FuncAnimation(fig, update_plot, interval=20, blit=True, cache_frame_data=False)

# Connect key press event to the on_key_press function
fig.canvas.mpl_connect('key_press_event', on_key_press)

print("\n=== LiDAR Control System ===")
print("Controls:")
print("  [L] - Send START command to ESP8266 (start motor & data)")
print("  [K] - Send STOP command to ESP8266 (stop motor & data)")
print("  [C] - Clear the plot")
print("Note: [K] only stops the ESP8266, program keeps running!")
print("Make sure the plot window is in focus to use keyboard controls!")
print("To exit program: Close the plot window or press Ctrl+C")
print("===============================\n")

try:
    plt.show()
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    running = False
    control_socket.close()
    udp_thread.join(timeout=1.0)
    print("Program terminated")
