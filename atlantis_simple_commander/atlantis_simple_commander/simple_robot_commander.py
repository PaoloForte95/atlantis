import tkinter as tk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np


from nav2_msgs.action import NavigateToPose
from material_handler_msgs.action import LoadMaterial
from material_handler_msgs.action import DumpMaterial
from geometry_msgs.msg import PoseStamped

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]


class RobotPositionSender(Node):
    def __init__(self):
        super().__init__('atlantis_simple_commander')
        self.declare_parameter('robot_ids', [1])

        self.robots = self.get_parameter('robot_ids').get_parameter_value().integer_array_value
        self.get_logger().info(f'robots: {list(self.robots)}')
        self._move_action_clients = {robot: ActionClient(self, NavigateToPose, 'robot'+str(robot)+'/navigate_to_pose')
                            for robot in self.robots}
        self._load_action_clients = {robot: ActionClient(self, LoadMaterial, 'robot'+str(robot)+'/send_load')
                            for robot in self.robots}
        self._dump_action_clients = {robot: ActionClient(self, DumpMaterial, 'robot'+str(robot)+'/send_dump')
                            for robot in self.robots}
        self.declare_parameter('actions', ['MoveTo'])
        self.actions = self.get_parameter('actions').get_parameter_value().string_array_value
        self.create_gui()

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
        self.robot_dropdown = tk.OptionMenu(self.root, self.robot_var, *self.robots)
        self.robot_dropdown.pack(pady=10)

        # Create dropdown menu for selecting action
        self.action_var = tk.StringVar(self.root)
        self.action_var.set(self.actions[0])  # default value
        self.action_dropdown = tk.OptionMenu(self.root, self.action_var, *self.actions, command=self.update_fields)
        self.action_dropdown.pack(pady=10)

        # Create entry for specifying pose
        self.pose_label = tk.Label(self.root, text="Pose: x,y,theta")
        self.pose_entry = tk.Entry(self.root)

        # Create entries for material_id and location (initially hidden)
        self.material_label = tk.Label(self.root, text="Material")
        self.material_entry = tk.Entry(self.root)
        self.location_label = tk.Label(self.root, text="Location")
        self.location_entry = tk.Entry(self.root)

        # Show pose fields initially
        self.pose_label.pack()
        self.pose_entry.pack(pady=5)

        # Create send button
        self.send_button = tk.Button(self.root, text="Send Action", command=self.send_action)
        self.send_button.pack(pady=10)

        self.root.mainloop()

    def update_fields(self, selected_action):
        print("selected_action" + selected_action)
        # Clear all fields
        self.pose_label.pack_forget()
        self.pose_entry.pack_forget()
        self.material_label.pack_forget()
        self.material_entry.pack_forget()
        self.location_label.pack_forget()
        self.location_entry.pack_forget()

        # Show relevant fields based on selected action
        if selected_action == "MoveTo":
            self.pose_label.pack()
            self.pose_entry.pack(pady=5)
        else:
            self.material_label.pack(pady=5)
            self.material_entry.pack(pady=5)
            self.location_label.pack(pady=5)
            self.location_entry.pack(pady=5)



    def send_action(self):
        selected_robot = int(self.robot_var.get())
        action = self.action_var.get()

        if action == "MoveTo":
            pose_str = self.pose_entry.get()
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

            self.goToPose(selected_robot,pose_msg)
        elif action == "Load":
            self.send_load_action(selected_robot)
        elif action == "Dump":
            self.send_dump_action(selected_robot)


        self.root.destroy()
        self.create_gui()


    def goToPose(self,ID, pose, behavior_tree=''):
            """Send a `NavToPose` action request."""
            self.debug("Waiting for robot" + str(ID) +" 'NavigateToPose' action server")
            navigate_to_pose_client = self._move_action_clients[ID]
            if not navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.info("'NavigateToPose' action server not available. The goal was not send...")
                return False

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            goal_msg.behavior_tree = behavior_tree

            self.info('Robot' + str(ID) + ' navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                    str(pose.pose.position.y) + '...')
            navigate_to_pose_client.send_goal_async(goal_msg,self.feedback_callback)
            return True

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback received: {0}'.format(feedback_msg.feedback.status))


    def send_load_action(self, robot):
        material = self.material_entry.get()
        location = self.location_entry.get()
        self.get_logger().info(f'Sending Load action to {robot} for material {material} at location {location}')
        # Create and send Load action
        send_load_to_pose_client = self._load_action_clients[robot]
        if not send_load_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.info("'SendLoad' action server not available. The goal was not send...")
            return False
        # Create Load goal message
        goal_msg = LoadMaterial.Goal()
        goal_msg.location = location
        goal_msg.name = material
        send_load_to_pose_client.send_goal_async(goal_msg)

    def send_dump_action(self, robot):
        material = self.material_entry.get()
        location = self.location_entry.get()
        self.get_logger().info(f'Sending Dump action to {robot} for material {material} at location {location}')
        send_dump_to_pose_client = self._dump_action_clients[robot]
        # Create and send Load action
        if not send_dump_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.info("'SendDump' action server not available. The goal was not send...")
            return False
        # Create Load goal message
        goal_msg = DumpMaterial.Goal()
        goal_msg.location = location
        goal_msg.name = material
        send_dump_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_position_sender = RobotPositionSender()
    rclpy.spin(robot_position_sender)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
