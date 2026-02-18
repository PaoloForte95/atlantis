import tkinter as tk
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np


from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]

def find_package_dir(uri: str) -> str:
    prefix = "package://"
    if not uri.startswith(prefix):
        return uri  # not a package URI, return unchanged

    rest = uri[len(prefix):]
    parts = rest.split("/", 1)

    package = parts[0]
    package_path = get_package_share_directory(package)

    if len(parts) == 1:
        return package_path  # no subpath, just package dir
    else:
        return f"{package_path}/{parts[1]}"


class RobotPositionSender(Node):
    def __init__(self):
        super().__init__('atlantis_simple_commander')
        self.declare_parameter('robots', [''])
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('actions', ['MoveTo'])

        self.robots = self.get_parameter('robots').get_parameter_value().string_array_value
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.actions = self.get_parameter('actions').get_parameter_value().string_array_value

        # ---- Action clients ----
        self._move_action_clients = {
            robot: ActionClient(self, NavigateToPose, f'{robot}/navigate_to_pose')
            for robot in self.robots
        }
        # ---- Load waypoints ----
        if self.waypoints_file != "":
            self.waypoints_path = find_package_dir(self.waypoints_file)

            self.get_logger().info(f'waypoints_file: {self.waypoints_path}')
            self.waypoints = self.load_waypoints()

            self.create_gui()

    def load_waypoints(self):
        """Load waypoints from the specified file."""
        waypoints = {}
        
        if not self.waypoints_path:
            self.warn("No waypoints file specified")
            return waypoints
            
        if not os.path.exists(self.waypoints_path):
            self.error(f"Waypoints file does not exist: {self.waypoints_path}")
            return waypoints
            
        try:
            with open(self.waypoints_path, 'r') as file:
                content = file.read()
                
            self.debug(f"File content:\n{content}")
            
            # Parse the custom format
            lines = content.strip().split('\n')
            current_waypoint = None
            current_data = {}
            
            for i, line in enumerate(lines):
                original_line = line
                line = line.strip()
                
                self.debug(f"Line {i}: '{original_line}' -> stripped: '{line}'")
                
                if not line:
                    continue
                    
                # Check if this is a waypoint name line (ends with colon, not indented)
                if line.endswith(':'):
                    # Save previous waypoint if exists
                    if current_waypoint and current_data:
                        self.debug(f"Saving waypoint '{current_waypoint}' with data: {current_data}")
                        waypoints[current_waypoint] = current_data
                    
                    # Start new waypoint
                    current_waypoint = line[:-1]  # Remove the colon
                    current_data = {}
                    self.debug(f"Started new waypoint: '{current_waypoint}'")
                    
                elif ':' in line:
                    # Parse coordinate line (should be indented but we'll be flexible)
                    try:
                        key, value = line.split(':', 1)
                        key = key.strip()
                        value = value.strip()
                        current_data[key] = float(value)
                        self.debug(f"Added {key}: {value} to current waypoint")
                    except ValueError as e:
                        self.warn(f"Could not parse line {i}: '{line}' - {e}")
                        continue
            
            # Don't forget the last waypoint
            if current_waypoint and current_data:
                self.debug(f"Saving final waypoint '{current_waypoint}' with data: {current_data}")
                waypoints[current_waypoint] = current_data
                
            self.info(f"Loaded {len(waypoints)} waypoints: {list(waypoints.keys())}")
            
            # Validate that each waypoint has required fields
            valid_waypoints = {}
            for name, data in waypoints.items():
                if 'x' in data and 'y' in data and 'yaw' in data:
                    valid_waypoints[name] = data
                else:
                    self.warn(f"Waypoint '{name}' missing required fields (x, y, yaw). Has: {list(data.keys())}")
            
            self.info(f"Valid waypoints: {len(valid_waypoints)} out of {len(waypoints)}")
            return valid_waypoints
            
        except Exception as e:
            self.error(f"Failed to load waypoints from {self.waypoints_path}: {str(e)}")
            import traceback
            self.error(f"Traceback: {traceback.format_exc()}")
            
        return waypoints

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Robot Action Sender")

        # Create dropdown menu for selecting robot
        self.robot_var = tk.StringVar(self.root)
        self.robot_var.set(self.robots[0])  # default value
        robot_label = tk.Label(self.root, text="Select Robot:")
        robot_label.pack(pady=5)
        self.robot_dropdown = tk.OptionMenu(self.root, self.robot_var, *self.robots)
        self.robot_dropdown.pack(pady=5)

        # Create dropdown menu for selecting action
        action_label = tk.Label(self.root, text="Select Action:")
        action_label.pack(pady=5)
        self.action_var = tk.StringVar(self.root)
        self.action_var.set(self.actions[0])  # default value
        self.action_dropdown = tk.OptionMenu(self.root, self.action_var, *self.actions, command=self.update_fields)
        self.action_dropdown.pack(pady=5)

        # Create entry for specifying pose (manual input)
        self.pose_label = tk.Label(self.root, text="Pose: x,y,theta")
        self.pose_entry = tk.Entry(self.root)

        # Create waypoint dropdown (for predefined waypoints)
        self.waypoint_label = tk.Label(self.root, text="Select Waypoint:")
        self.waypoint_var = tk.StringVar(self.root)
        waypoint_options = list(self.waypoints.keys()) if self.waypoints else ["No waypoints loaded"]
        if waypoint_options and waypoint_options[0] != "No waypoints loaded":
            self.waypoint_var.set(waypoint_options[0])
        self.waypoint_dropdown = tk.OptionMenu(self.root, self.waypoint_var, *waypoint_options)

        # Create entries for material_id and location (initially hidden)
        self.material_label = tk.Label(self.root, text="Material")
        self.material_entry = tk.Entry(self.root)
        self.location_label = tk.Label(self.root, text="Location")
        self.location_entry = tk.Entry(self.root)

        # Show appropriate fields initially
        self.update_fields(self.actions[0])

        # Create send button
        self.send_button = tk.Button(self.root, text="Send Action", command=self.send_action)
        self.send_button.pack(pady=10)

        self.root.mainloop()

    def update_fields(self, selected_action):
        print("selected_action: " + selected_action)
        # Clear all fields
        self.pose_label.pack_forget()
        self.pose_entry.pack_forget()
        self.waypoint_label.pack_forget()
        self.waypoint_dropdown.pack_forget()
        self.material_label.pack_forget()
        self.material_entry.pack_forget()
        self.location_label.pack_forget()
        self.location_entry.pack_forget()

        # Show relevant fields based on selected action
        if selected_action == "MoveTo":
            if self.waypoints:
                # Show waypoint dropdown if waypoints are available
                self.waypoint_label.pack(pady=5)
                self.waypoint_dropdown.pack(pady=5)
            else:
                # Show manual pose entry if no waypoints
                self.pose_label.pack(pady=5)
                self.pose_entry.pack(pady=5)
        else:
            self.material_label.pack(pady=5)
            self.material_entry.pack(pady=5)
            self.location_label.pack(pady=5)
            self.location_entry.pack(pady=5)

    def send_action(self):
        selected_robot = self.robot_var.get()
        action = self.action_var.get()

        if action == "MoveTo":
            if self.waypoints and self.waypoint_var.get() in self.waypoints:
                # Use selected waypoint
                waypoint_name = self.waypoint_var.get()
                waypoint_data = self.waypoints[waypoint_name]
                
                pose_position = [waypoint_data['x'], waypoint_data['y']]
                pose_orientation_theta = waypoint_data['yaw']
                
                self.info(f"Using waypoint '{waypoint_name}': x={waypoint_data['x']}, y={waypoint_data['y']}, yaw={waypoint_data['yaw']}")
            else:
                # Use manual pose entry
                pose_str = self.pose_entry.get()
                if not pose_str:
                    self.error("No pose specified and no waypoints available")
                    return
                    
                # Parse pose string into position and orientation
                pose_list = pose_str.split(',')
                pose_position = [float(x) for x in pose_list[:2]]
                pose_orientation_theta = float(pose_list[2])

            # Convert theta to quaternion
            quaternion = euler_to_quaternion(0, 0, pose_orientation_theta)

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x, pose_msg.pose.position.y = pose_position
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quaternion

            self.goToPose(selected_robot, pose_msg)

        self.root.destroy()
        self.create_gui()

    def goToPose(self, ID, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for robot" + str(ID) + " 'NavigateToPose' action server")
        navigate_to_pose_client = self._move_action_clients[ID]
        if not navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.info("'NavigateToPose' action server not available. The goal was not send...")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Robot' + str(ID) + ' navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                str(pose.pose.position.y) + '...')
        navigate_to_pose_client.send_goal_async(goal_msg, self.feedback_callback)
        return True

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback received: {0}'.format(feedback_msg.feedback.status))


def main(args=None):
    rclpy.init(args=args)
    robot_position_sender = RobotPositionSender()
    rclpy.spin(robot_position_sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()