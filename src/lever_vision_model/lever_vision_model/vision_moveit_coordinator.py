#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
import numpy as np

class VisionMoveItCoordinator(Node):
    def __init__(self):
        super().__init__('vision_moveit_coordinator')
        
        self.group_name = "arm_group" 
        self.base_frame = "Arm_base_link" 
        self.ee_link = "end_effector_link"

        self.last_pose = None
        self.stable_counter = 0
        
        self.lever_length = 0.17 
        self.safety_margin = 0.03 
        self.slide_step = 0.03    
        
        self.wait_time_1 = 5.0 
        self.wait_time_2 = 5.0 

        self.mission_state = "IDLE" 
        self.target_base_pose = None
        self.engage_direction = 1.0 
        self.state_start_time = 0.0
        
        self.dist_threshold = 0.02 
        self.required_stable_frames = 10 
        
        self.subscription = self.create_subscription(
            PoseStamped, '/lever_pose_arm', self.lever_callback, 10)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.timer = self.create_timer(0.1, self.state_machine_loop)

        self.get_logger().info('Coordinator Ready (NO Angle Constraints + Fixed Y-Axis)')

    def lever_callback(self, msg):
        if self.mission_state != "IDLE": return

        # التصحيح الرياضي: لو الأوكرة شمال (Y موجب) -> نجمع. لو يمين (Y سالب) -> نطرح.
        if msg.pose.orientation.x >= 0:
            self.engage_direction = -1.0 # يمين
            dir_str = "RIGHT"
        else:
            self.engage_direction = 1.0  # شمال
            dir_str = "LEFT"

        if self.last_pose is None:
            self.last_pose = msg.pose
            return

        diff = np.sqrt((msg.pose.position.x - self.last_pose.position.x)**2 +
                       (msg.pose.position.y - self.last_pose.position.y)**2 +
                       (msg.pose.position.z - self.last_pose.position.z)**2)

        if diff < self.dist_threshold:
            self.stable_counter += 1
        else:
            self.stable_counter = 0
            self.last_pose = msg.pose

        if self.stable_counter >= self.required_stable_frames:
            self.target_base_pose = msg.pose
            self.get_logger().info(f"Target Locked! Direction: {dir_str}")
            self.mission_state = "APPROACHING"
            self.send_approach_goal()
            self.stable_counter = 0 

    def send_approach_goal(self):
        approach_pose = Pose()
        approach_pose.position.x = self.target_base_pose.position.x 
        approach_pose.position.z = self.target_base_pose.position.z 
        
        # هنا هتنطبع لك القيمة بالموجب (+0.20) لو الأوكرة شمال
        offset = (self.lever_length + self.safety_margin) * self.engage_direction
        approach_pose.position.y = self.target_base_pose.position.y + offset
        
        approach_pose.orientation.w = 1.0 
        self.current_y = approach_pose.position.y 
        
        self.get_logger().info("=========================================")
        self.get_logger().info(f"STEP: APPROACHING (Offset: {offset:.2f}m)")
        self.send_goal_to_moveit(approach_pose)

    def send_engage_step_1(self):
        step_pose = Pose()
        step_pose.position.x = self.target_base_pose.position.x
        step_pose.position.z = self.target_base_pose.position.z
        
        self.current_y -= (self.slide_step * self.engage_direction)
        step_pose.position.y = self.current_y
        step_pose.orientation.w = 1.0 
        
        self.get_logger().info("=========================================")
        self.get_logger().info("STEP: ENGAGING 1 (Sliding 3cm)")
        self.send_goal_to_moveit(step_pose)

    def send_engage_step_2(self):
        step_pose = Pose()
        step_pose.position.x = self.target_base_pose.position.x
        step_pose.position.z = self.target_base_pose.position.z
        
        self.current_y -= (self.slide_step * self.engage_direction)
        step_pose.position.y = self.current_y
        step_pose.orientation.w = 1.0 
        
        self.get_logger().info("=========================================")
        self.get_logger().info("STEP: ENGAGING 2 (Sliding final 3cm)")
        self.send_goal_to_moveit(step_pose)

    def send_goal_to_moveit(self, target_pose):
        self.get_logger().info(f"--- Sending Coordinates to MoveIt ---")
        self.get_logger().info(f"X: {target_pose.position.x:.3f}, Y: {target_pose.position.y:.3f}, Z: {target_pose.position.z:.3f}")

        if not self._action_client.wait_for_server(timeout_sec=2.0): 
            self.get_logger().error("MoveIt Action Server not available!")
            return
            
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 15
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.start_state.is_diff = True 
        
        # عشان الروبوت يتحرك فعلاً في المحاكاة ويأكد الخطوات
        goal_msg.planning_options.plan_only = False 
        
        # قيد المكان فقط (مفيش قيد زاوية بيخنق الروبوت)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame 
        pos_constraint.link_name = self.ee_link 
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] 
        pos_constraint.constraint_region.primitives.append(s)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Plan REJECTED by MoveIt! (Goal is outside workspace boundaries)')
            self.mission_state = "IDLE" 
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        error_code = future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('SUCCESS: Path Executed.')
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
            if self.mission_state == "APPROACHING": self.mission_state = "WAITING_1"
            elif self.mission_state == "ENGAGING_1": self.mission_state = "WAITING_2"
            elif self.mission_state == "ENGAGING_2": self.mission_state = "LOCKED"
        else: 
            self.get_logger().error(f'FAILED: MoveIt Error Code: {error_code}')
            self.mission_state = "IDLE"

    def state_machine_loop(self):
        if self.mission_state in ["WAITING_1", "WAITING_2"]:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.mission_state == "WAITING_1" and (current_time - self.state_start_time) >= self.wait_time_1:
                self.mission_state = "ENGAGING_1"
                self.send_engage_step_1()
            elif self.mission_state == "WAITING_2" and (current_time - self.state_start_time) >= self.wait_time_2:
                self.mission_state = "ENGAGING_2"
                self.send_engage_step_2()

def main(args=None):
    rclpy.init(args=args)
    node = VisionMoveItCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()