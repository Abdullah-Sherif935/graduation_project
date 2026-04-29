#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import numpy as np
import math

class VisionMoveItCoordinator(Node):
    def __init__(self):
        super().__init__('vision_moveit_coordinator')
        
        self.group_name = "arm_group" 
        self.base_frame = "Arm_base_link" 
        self.ee_link = "end_effector_link"

        self.last_pose = None
        self.stable_counter = 0
        
        self.hook_offset_y = 0.10  
        self.hook_offset_x = 0.00  
        self.hook_offset_z = 0.00  
        
        # ---------------------------------------------------------
        # السر هنا: بما إن الـ Roll فشل ميكانيكياً، هنجرب الـ Pitch
        # لو الـ Pitch لف الهوك بالعرض، يبقى ده المحور الصح للـ URDF بتاعك
        self.hook_roll = 0.0
        self.hook_pitch = 1.5708   # 90 درجة 
        self.hook_yaw = 0.0       
        # ---------------------------------------------------------

        self.mission_state = "IDLE" 
        self.final_target_pose = None
        
        self.dist_threshold = 0.02 
        self.required_stable_frames = 10 

        self.last_success_time = 0.0 
        self.cooldown_period = 180.0  
        
        self.subscription = self.create_subscription(
            PoseStamped, '/lever_pose_arm', self.lever_callback, 10)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info('Coordinator Ready (Pitch Rotation & Flexible Tolerances ON)')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def lever_callback(self, msg):
        if self.mission_state == "COOLDOWN":
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_success_time) > self.cooldown_period:
                self.mission_state = "IDLE"
                self.get_logger().info("Cooldown finished. Ready for a new door!")
            return

        if self.mission_state != "IDLE":
            return

        z_height = msg.pose.position.z
        if not (0.10 < z_height < 0.50):
            return

        if self.last_pose is None:
            self.last_pose = msg.pose
            return

        diff = np.sqrt(
            (msg.pose.position.x - self.last_pose.position.x)**2 +
            (msg.pose.position.y - self.last_pose.position.y)**2 +
            (msg.pose.position.z - self.last_pose.position.z)**2
        )

        if diff < self.dist_threshold:
            self.stable_counter += 1
        else:
            self.stable_counter = 0
            self.last_pose = msg.pose

        if self.stable_counter >= self.required_stable_frames:
            self.final_target_pose = msg.pose
            self.get_logger().info("Target Locked! Moving to Approach position...")
            self.mission_state = "APPROACHING"
            self.send_approach_goal()
            self.stable_counter = 0 

    def send_approach_goal(self):
        approach_pose = Pose()
        approach_pose.position.x = self.final_target_pose.position.x + self.hook_offset_x
        approach_pose.position.y = self.final_target_pose.position.y + self.hook_offset_y
        approach_pose.position.z = self.final_target_pose.position.z + self.hook_offset_z
        
        q = self.euler_to_quaternion(self.hook_roll, self.hook_pitch, self.hook_yaw)
        approach_pose.orientation.x = q[0]
        approach_pose.orientation.y = q[1]
        approach_pose.orientation.z = q[2]
        approach_pose.orientation.w = q[3]
        
        self.send_goal_to_moveit(approach_pose)

    def send_engage_goal(self):
        engage_pose = Pose()
        engage_pose.position.x = self.final_target_pose.position.x
        engage_pose.position.y = self.final_target_pose.position.y
        engage_pose.position.z = self.final_target_pose.position.z
        
        q = self.euler_to_quaternion(self.hook_roll, self.hook_pitch, self.hook_yaw)
        engage_pose.orientation.x = q[0]
        engage_pose.orientation.y = q[1]
        engage_pose.orientation.z = q[2]
        engage_pose.orientation.w = q[3]
        
        self.send_goal_to_moveit(engage_pose)

    def send_goal_to_moveit(self, target_pose):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        
        # زودنا عدد المحاولات لـ 20 عشان ميفقدش الأمل بسرعة
        goal_msg.request.num_planning_attempts = 20 
        goal_msg.request.allowed_planning_time = 5.0
        
        goal_msg.request.start_state.is_diff = True 
        goal_msg.planning_options.plan_only = True 
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame 
        pos_constraint.link_name = self.ee_link 
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] 
        
        pos_constraint.constraint_region.primitives.append(s)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = target_pose.orientation
        
        # وسعنا السماحية لـ 0.5 (براح كافي للرياضيات إنها تلاقي حل من غير ما تضرب 99999)
        ori_constraint.absolute_x_axis_tolerance = 0.5
        ori_constraint.absolute_y_axis_tolerance = 0.5
        ori_constraint.absolute_z_axis_tolerance = 0.5
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Plan Rejected by MoveIt! (IK Solver Failed)')
            self.mission_state = "IDLE" 
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == 1:
            if self.mission_state == "APPROACHING":
                self.get_logger().info('Approach completed! Engaging hook...')
                self.mission_state = "ENGAGING"
                self.send_engage_goal()
                
            elif self.mission_state == "ENGAGING":
                self.get_logger().info('SUCCESS: Lever is hooked!')
                self.mission_state = "COOLDOWN"
                self.last_success_time = self.get_clock().now().nanoseconds / 1e9
        else:
            self.get_logger().error(f'FAILED: MoveIt Error Code: {error_code}')
            self.mission_state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = VisionMoveItCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()