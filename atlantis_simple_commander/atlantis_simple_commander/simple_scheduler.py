
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from material_handler_msgs.action import LoadMaterial
from material_handler_msgs.action import DumpMaterial
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf



def create_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = tf.quaternion_from_euler(0,0,yaw)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose

class NavigateToPoseClient(Node):
    def __init__(self, robot:str):
        super().__init__('navigate_to_pose_client')
        self.action_client = ActionClient(self, NavigateToPose, '/' + robot + '/navigate_to_pose')
        self.current_goal_index = 0

    def send_goal(self, pose):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        

        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)


class LoadActionClient(Node):
    def __init__(self, robot):
        super().__init__('load_action_client')
        self.action_client = ActionClient(self, LoadMaterial, '/' + robot + '/send_load')

    def send_goal(self):
        goal_msg = LoadMaterial.Goal()
        goal_msg.location = "load1"
        goal_msg.name = "mat1"
        goal_msg.amount = 10.0
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)


class DumpActionClient(Node):
    def __init__(self, robot):
        super().__init__('dump_action_client')
        self.action_client = ActionClient(self, DumpMaterial, '/' + robot + '/send_dump')

    def send_goal(self):
        goal_msg = DumpMaterial.Goal()
        goal_msg.location = "dump1"
        goal_msg.name = "mat1"
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    pose1 = create_pose(40.0, 116.7,2.6)
    pose2 = create_pose(93.8, 8.6,-0.84)
    navigator = NavigateToPoseClient("robot1")
    load_client = LoadActionClient("robot1")
    dump_client = DumpActionClient("robot1")


    # Send Task 1
    future = navigator.send_goal(pose1)
    rclpy.spin_until_future_complete(navigator, future)
    goal_handle = future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(navigator, get_result_future)
    result = get_result_future.result().result

    # Send Task 2
    future = load_client.send_goal()
    rclpy.spin_until_future_complete(load_client, future)
    goal_handle = future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(load_client, get_result_future)
    result = get_result_future.result().result

    # Send Task 3
    future = navigator.send_goal(pose2)
    rclpy.spin_until_future_complete(navigator, future)
    goal_handle = future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(navigator, get_result_future)
    result = get_result_future.result().result

    # Send Task 4
    future = dump_client.send_goal()
    rclpy.spin_until_future_complete(dump_client, future)
    goal_handle = future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(dump_client, get_result_future)
    result = get_result_future.result().result
    

if __name__ == '__main__':
    main()