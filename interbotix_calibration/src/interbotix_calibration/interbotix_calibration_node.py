#!/usr/bin/env python3
import abc
import rospy
from rospy import Time, Duration
import sys
import os
from datetime import datetime
import moveit_commander
import numpy as np
from collections import deque
import yaml
import actionlib
from roslaunch.substitution_args import resolve_args
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from interbotix_calibration.srv import SetFloat32, SetFloat32Response
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Header
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation


# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta("ABC", (object,), {"__slots__": ()})


class ActionServer(ABC):
    def __init__(self, action_server, action_type):
        self.client = actionlib.SimpleActionClient(action_server, action_type)
        rospy.logdebug("Waiting for server {} ...".format(action_server))
        self.client.wait_for_server()
        rospy.logdebug("[{}] Connected to server".format(action_server))

    @abc.abstractmethod
    def act(self, action):
        self.client.send_goal(action)

    def reset(self):
        self.client.cancel_all_goals()


class FollowJointTrajectoryActionServer(ActionServer):
    def __init__(self, joint_names, server_name, duration=0.5):
        self.joint_names = joint_names
        self.duration = duration
        super(FollowJointTrajectoryActionServer, self).__init__(server_name, FollowJointTrajectoryAction)

    def act(self, action_raw):
        action = FollowJointTrajectoryGoal()
        action.trajectory = JointTrajectory()
        action.trajectory.header = Header()
        action.trajectory.joint_names = self.joint_names
        action.trajectory.points = [JointTrajectoryPoint()]
        action.trajectory.points[0].positions = action_raw
        action.trajectory.points[0].time_from_start = rospy.Duration(self.duration)
        self.client.send_goal(action)


def substitute_xml_args(param):
    # substitute string
    if isinstance(param, str):
        param = resolve_args(param)
        return param

    # For every key in the dictionary (not performing deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_xml_args(param[key])
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key])


def TransformStamped_to_lists(transform_stamped):
    translation = [
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z,
    ]
    rotation = [
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w,
    ]
    return translation, rotation


def get_transformation_matrix(translation, rotation):
    t = np.array(translation).reshape((3, 1))
    R = Rotation.from_quat(rotation).as_matrix()
    T = np.vstack((np.hstack((R, t)), np.array([0, 0, 0, 1])))
    return T


class EagerxCalibration(object):
    def __init__(self):
        self.event = None

        rospy.init_node("interbotix_calibration", log_level=rospy.INFO)

        self.rate = rospy.Rate(5)

        robot_sim = rospy.get_param("~robot_sim")

        # MoveIt parameters
        self.current_position = None
        self.base_frame = rospy.get_param("~robot_base_frame")
        self.robot_effector_frame = rospy.get_param("~robot_effector_frame")
        self.joint_names = yaml.safe_load(rospy.get_param("~joint_names"))
        self.manipulator_group_name = rospy.get_param("~manipulator_group_name", "interbotix_arm")
        self.end_effector_group_name = rospy.get_param("~end_effector_group_name", "interbotix_gripper")

        self.ee_step = rospy.get_param("~ee_step", 0.01)
        self.jump_threshold = rospy.get_param("~jump_threshold", 0.0)
        self.collision_height = rospy.get_param("~collision_height", 0.1)
        self.base_length = rospy.get_param("~base_length", 0.4)
        self.workspace_length = rospy.get_param("~workspace_length", 2.4)
        self.velocity_scaling_factor = rospy.get_param("~velocity_scaling_factor", 0.3)
        self.acceleration_scaling_factor = rospy.get_param("~acceleration_scaling_factor", 0.3)

        # Calibration params
        self.tracking_marker_frame = rospy.get_param("~tracking_marker_frame")
        self.camera_frame = rospy.get_param("~camera_frame")
        self.transform_filename = rospy.get_param("~transform_filename")
        marker_transform_topic = rospy.get_param("~marker_transform_topic")

        # Calibration poses
        self.diff = np.array([np.pi / 9, np.pi / 18, np.pi / 18, np.pi / 12, np.pi / 12, np.pi / 12])
        self.calibration_pose = np.array([0, np.pi / 18, 0, 0, np.pi / 2, 0])
        if len(self.joint_names) == 5:
            self.diff = np.delete(self.diff, 3)
            self.calibration_pose = np.delete(self.calibration_pose, 3)

        # Transforms deque (for filtering the marker pose during calibration)
        self.marker_translation_window = deque(maxlen=30)
        self.marker_rotation_window = deque(maxlen=30)

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # Initialize robot state
        self.robot_state = RobotState()
        self.robot_state.joint_state.name = self.joint_names

        self.manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        self.end_effector_group = moveit_commander.MoveGroupCommander(self.end_effector_group_name)

        if not robot_sim:
            self.manipulator_group.set_max_velocity_scaling_factor(self.velocity_scaling_factor)
            self.manipulator_group.set_max_acceleration_scaling_factor(self.acceleration_scaling_factor)

        # Five collision objects are added to the scene, such that the base is not in collision, while the rest of the
        # surface is a collision object.

        object_length = (self.workspace_length - self.base_length) / 2.0

        # Add a collision object to the scenes
        p0 = PoseStamped()
        p0.header.frame_id = self.base_frame
        p0.pose.position.z = -0.051
        p0.pose.orientation.w = 1

        # Add a collision object to the scenes
        p1 = PoseStamped()
        p1.header.frame_id = self.base_frame
        p1.pose.position.x = (object_length + self.base_length) / 2.0
        p1.pose.position.z = self.collision_height / 2.0
        p1.pose.orientation.w = 1

        # Add a collision object to the scenes
        p2 = PoseStamped()
        p2.header.frame_id = self.base_frame
        p2.pose.position.x = -(object_length + self.base_length) / 2.0
        p2.pose.position.z = self.collision_height / 2.0
        p2.pose.orientation.w = 1

        # Add a collision object to the scenes
        p3 = PoseStamped()
        p3.header.frame_id = self.base_frame
        p3.pose.position.y = (object_length + self.base_length) / 2.0
        p3.pose.position.z = self.collision_height / 2.0
        p3.pose.orientation.w = 1

        # Add a collision object to the scenes
        p4 = PoseStamped()
        p4.header.frame_id = self.base_frame
        p4.pose.position.y = -(object_length + self.base_length) / 2.0
        p4.pose.position.z = self.collision_height / 2.0
        p4.pose.orientation.w = 1

        scene.add_box("base0", p0, size=(self.workspace_length, self.workspace_length, 0.1))
        scene.add_box("base1", p1, size=(object_length, self.base_length, self.collision_height))
        scene.add_box("base2", p2, size=(object_length, self.base_length, self.collision_height))
        scene.add_box("base3", p3, size=(self.workspace_length, object_length, self.collision_height))
        scene.add_box("base4", p4, size=(self.workspace_length, object_length, self.collision_height))

        self.action_server = FollowJointTrajectoryActionServer(
            self.joint_names, "arm_controller/follow_joint_trajectory", duration=2.0
        )

        # Subscribers
        rospy.Subscriber("~event_in", String, self._event_in_callback)
        rospy.Subscriber(marker_transform_topic, TransformStamped, self._transforc_mallback)

        # TF Buffer, Listener and Broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()

        # Services
        self._state_validity_service = rospy.ServiceProxy("check_state_validity", GetStateValidity)

        rospy.Service("goal", SetFloat32, self._goal_callback)
        rospy.Service("check", SetFloat32, self._check_callback)

    def _event_in_callback(self, msg):
        if self.event is None:
            self.event = msg.data

    def _transforc_mallback(self, msg):
        """Filter marker transform to be robust against outliers"""

        # Get translation and rotation
        translation = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
        rotation = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

        # Add them to window
        self.marker_translation_window.append(translation)
        self.marker_rotation_window.append(rotation)

        # Create np arrays of window
        marker_translation_array = np.array(self.marker_translation_window)
        marker_rotation_array = np.array(self.marker_rotation_window)

        # Apply median filter
        filtered_translation = np.median(marker_translation_array, axis=0)
        median_rotation = np.median(marker_rotation_array, axis=0)
        normalized_median_rotation = median_rotation / np.linalg.norm(median_rotation)
        filtered_rotation = marker_rotation_array[
            np.argmin(np.linalg.norm(marker_rotation_array - normalized_median_rotation, axis=1))
        ]

        # Create transform message
        filtered_transform = TransformStamped()
        filtered_transform.header.stamp = rospy.Time.now()
        filtered_transform.header.frame_id = msg.header.frame_id
        filtered_transform.child_frame_id = self.tracking_marker_frame
        filtered_transform.transform.translation.x = filtered_translation[0]
        filtered_transform.transform.translation.y = filtered_translation[1]
        filtered_transform.transform.translation.z = filtered_translation[2]
        filtered_transform.transform.rotation.x = filtered_rotation[0]
        filtered_transform.transform.rotation.y = filtered_rotation[1]
        filtered_transform.transform.rotation.z = filtered_rotation[2]
        filtered_transform.transform.rotation.w = filtered_rotation[3]

        self.tf_broadcaster.sendTransform(filtered_transform)

    def _goal_callback(self, req):
        response = SetFloat32Response()
        self.action_server.reset()
        try:
            self._move_to_joint_goal(self.manipulator_group, list(req.data))
        except moveit_commander.exception.MoveItCommanderException:
            response.success = False
            return response
        response.success = True
        self.action_server.reset()
        return response

    def _check_callback(self, req):
        self.robot_state.joint_state.position = req.data
        response = SetFloat32Response()
        gsvr = GetStateValidityRequest()
        gsvr.group_name = self.manipulator_group_name
        gsvr.robot_state = self.robot_state
        response.success = self._state_validity_service.call(gsvr).valid
        return response

    def _move_to_joint_goal(self, move_group, joint_goal):
        move_group.get_current_state()
        # Now, we call the planner to compute the plan and execute it.
        succes = move_group.go(joint_goal, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        return succes

    def _move_to_pose_goal(self, move_group, pose_goal):
        move_group.get_current_state()
        # Now, we call the planner to compute the plan and execute it.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        move_group.clear_pose_targets()
        return success

    def _move_along_cartesian_path(self, move_group, waypoints):
        plan, _ = move_group.compute_cartesian_path(
            waypoints,
            self.ee_step,
            self.jump_threshold,
            avoid_collisions=False,
        )
        retimed_plan = self.manipulator_group.retime_trajectory(
            self.manipulator_group.get_current_state(),
            plan,
            velocity_scaling_factor=0.1,
            acceleration_scaling_factor=0.1,
        )
        move_group.execute(retimed_plan, wait=True)

    def _command_home(self):
        target = self.manipulator_group.get_named_target_values("Home")
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_upright(self):
        target = self.manipulator_group.get_named_target_values("Upright")
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_sleep(self):
        self._command_home()
        current_state = self.manipulator_group.get_current_state()
        current_position = current_state.joint_state.position
        target = self.manipulator_group.get_named_target_values("Sleep")
        goal_pos = []
        for idx, joint_name in enumerate(self.joint_names):
            if abs(current_position[idx]) > 0.05:
                rospy.logwarn("[{}] Cannot go to sleep since robot is not in home position!".format(rospy.get_name()))
                return
            goal_pos.append(target[joint_name])
        self.action_server.act(goal_pos)
        rospy.sleep(3)
        self.action_server.reset()

    def _command_open(self):
        target = self.end_effector_group.get_named_target_values("Open")
        self._move_to_joint_goal(self.end_effector_group, target)

    def _command_close(self):
        target = self.end_effector_group.get_named_target_values("Closed")
        self._move_to_joint_goal(self.end_effector_group, target)

    def _command_up(self):
        current_state = self.manipulator_group.get_current_state()
        current_position = current_state.joint_state.position
        target = self.manipulator_group.get_named_target_values("Sleep")
        goal_pos = []
        for idx, joint_name in enumerate(self.joint_names):
            if abs(target[joint_name] - current_position[idx]) > 0.25:
                rospy.logwarn("[{}] Cannot go up since robot is not in sleep position!".format(rospy.get_name()))
                return
            goal_pos.append(target[joint_name] / 2)
        self.action_server.act(goal_pos)
        rospy.sleep(3)
        self.action_server.reset()

    def _command_calibrate(self):
        pose = self.calibration_pose + np.random.uniform(-self.diff, +self.diff, (len(self.joint_names)))
        self._move_to_joint_goal(self.manipulator_group, pose)

    def _command_save(self):
        tstamped_m_ee = self.tf_buffer.lookup_transform(
            self.robot_effector_frame, self.tracking_marker_frame, Time(0), Duration(1)
        )
        t_m_ee, r_m_ee = TransformStamped_to_lists(tstamped_m_ee)

        tstamped_c_rb = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time(0), Duration(1))
        t_c_rb, r_c_rb = TransformStamped_to_lists(tstamped_c_rb)

        # Create save directory
        strftime = datetime.today().strftime("%Y-%m-%d-%H%M")
        save_dir = substitute_xml_args("$(find interbotix_calibration)/config")
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # Save calibration
        calibration = {}
        calibration["camera_to_robot"] = {}
        calibration["camera_to_robot"]["translation"] = t_c_rb
        calibration["camera_to_robot"]["rotation"] = r_c_rb
        calibration["marker_to_ee"] = {}
        calibration["marker_to_ee"]["translation"] = t_m_ee
        calibration["marker_to_ee"]["rotation"] = r_m_ee
        save_path = f"{save_dir}/eye_hand_calibration_{strftime}.yaml"
        with open(save_path, "w") as file:
            yaml.dump(calibration, file)

    def _command_calculate(self):
        calibration_file = substitute_xml_args(f"$(find interbotix_calibration)/config/{self.transform_filename}")
        with open(calibration_file) as f:
            data = yaml.load(f, Loader=yaml.loader.SafeLoader)

        # Get transformation of marker to ee frame
        t_m_ee = data["marker_to_ee"]["translation"]
        r_m_ee = data["marker_to_ee"]["rotation"]
        T_m_ee = get_transformation_matrix(t_m_ee, r_m_ee)

        # Get transformation of ee to robot base frame
        tstamped_ee_rb = self.tf_buffer.lookup_transform(self.base_frame, self.robot_effector_frame, Time(0), Duration(1))
        t_ee_rb, r_ee_rb = TransformStamped_to_lists(tstamped_ee_rb)
        T_ee_rb = get_transformation_matrix(t_ee_rb, r_ee_rb)

        # Get transformation of camera to marker
        # (You get the inverse from aruco detection)
        tstamped_c_m = self.tf_buffer.lookup_transform(self.tracking_marker_frame, self.camera_frame, Time(0), Duration(1))
        t_c_m, r_c_m = TransformStamped_to_lists(tstamped_c_m)
        T_c_m = get_transformation_matrix(t_c_m, r_c_m)

        # Calculate transform of camera_frame to base
        T_c_rb = T_ee_rb @ T_m_ee @ T_c_m

        t_c_rb = T_c_rb[:3, 3].tolist()
        r_c_rb = Rotation.from_matrix(T_c_rb[:3, :3]).as_quat().tolist()
        rospy.logwarn(T_c_rb)

        rospy.loginfo(f"Transformation from camera frame to robot frame is:\n Translation: {t_c_rb}\n Rotation: {r_c_rb}")

    def command(self):
        while not rospy.is_shutdown():
            if self.event is not None:
                if self.event == "calibrate":
                    self._command_calibrate()
                elif self.event == "save":
                    self._command_save()
                elif self.event == "calculate":
                    self._command_calculate()
                elif self.event == "home":
                    self._command_home()
                elif self.event == "upright":
                    self._command_upright()
                elif self.event == "sleep":
                    self._command_sleep()
                elif self.event == "open":
                    self._command_open()
                elif self.event == "close":
                    self._command_close()
                elif self.event == "up":
                    self._command_up()
                self.event = None
            self.rate.sleep()


def main():
    try:
        robot = EagerxCalibration()
        robot.command()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
