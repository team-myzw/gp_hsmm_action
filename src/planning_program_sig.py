#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math
import sys
import warnings

from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import TransformStamped

import numpy as np
import rospy

from sensor_msgs.msg import JointState

import tf.transformations as T
import tf2_ros
from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes
from tmc_manipulation_msgs.msg import BaseMovementType
from tmc_planning_msgs.msg import JointPosition

from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_planning_msgs.srv import PlanWithHandGoals
from tmc_planning_msgs.srv import PlanWithHandGoalsRequest
from tmc_planning_msgs.srv import PlanWithHandLine
from tmc_planning_msgs.srv import PlanWithHandLineRequest
from tmc_planning_msgs.srv import PlanWithJointGoals
from tmc_planning_msgs.srv import PlanWithJointGoalsRequest
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from hsrb_interface import collision_world
from hsrb_interface import exceptions
from hsrb_interface import geometry
from hsrb_interface import robot
from hsrb_interface import robot_model
from hsrb_interface import settings
from hsrb_interface import trajectory
from hsrb_interface import utils

_DEBUG = False

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 10.0

# Max number of iteration of moition planning
_PLANNING_MAX_ITERATION = 10000

# Goal generation probability in moition planning
_PLANNING_GOAL_GENERATION = 0.3

# Goal deviation in motion planning
_PLANNING_GOAL_DEVIATION = 0.3

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 1.0

# Base frame of a mobile base in moition planning
_BASE_TRAJECTORY_ORIGIN = "odom"


def _normalize_np(vec):
    """Normalize 1D numpy.ndarray

    Args:
        vec (numpy.ndarray): A vector to be normalized
    Returns:
        numpy.ndarray: The reuslt of computation
    """
    length = np.linalg.norm(vec)
    if length < sys.float_info.epsilon:
        return vec
    else:
        vec = vec/length
        return vec


def _pose_from_x_axis(axis):
    """Compute a transformation that fits X-axis of its frame to given vector.

    Args:
        axis (geometry.Vector3): A target vector

    Returns:
        geometry.Pose: The result transformation that stored in Pose type.
    """
    axis = np.array(axis, dtype='float64', copy=True)
    axis = _normalize_np(axis)
    unit_x = np.array([1, 0, 0])
    outerp = np.cross(unit_x, axis)
    theta = math.acos(np.dot(unit_x, axis))
    if np.linalg.norm(outerp) < sys.float_info.epsilon:
        outerp = np.array([0, 1, 0])
    outerp = _normalize_np(outerp)
    q = T.quaternion_about_axis(theta, outerp)
    return geometry.Pose(geometry.Vector3(0, 0, 0), geometry.Quaternion(*q))

def _movement_axis_and_distance(pose1, pose2):
    """Compute a vector from the origin of pose1 to pose2 and distance.

    Args:
        pose1 (geometry.Pose): A pose that its origin is used as start.
        pose2 (geometry.Pose): A pose that its origin is used as goal.
    Returns:
        Tuple[geometry.Vector3, float]: The result
    """
    p1 = pose1[0]
    p2 = pose2[0]
    """Normalize Vector3"""
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    z = p2[2] - p1[2]
    length = math.sqrt(x * x + y * y + z * z)
    if length < sys.float_info.epsilon:
        return geometry.Vector3(0.0, 0.0, 0.0), 0.0
    else:
        x /= length
        y /= length
        z /= length
        return geometry.Vector3(x, y, z), length


def _invert_pose(pose):
    """Invert a given pose as if it is a transformation.

    Args:
        pose (geometry.Pose): A pose to be inverted.q
    Returns:
        geometry.Pose: The result of computation
    """
    m = T.compose_matrix(translate=pose[0],
                         angles=T.euler_from_quaternion(pose[1]))
    (_, _, euler, trans, _) = T.decompose_matrix(T.inverse_matrix(m))
    q = T.quaternion_from_euler(euler[0], euler[1], euler[2])
    return geometry.Pose(trans, q)



class JointGroup(robot.Item):

    def __init__(self, name="whole_body"):
        """See class docstring."""
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        arm_config = self._setting['arm_controller_prefix']
        self._arm_client = trajectory.TrajectoryController(arm_config)
        head_config = self._setting['head_controller_prefix']
        self._head_client = trajectory.TrajectoryController(head_config)
#        hand_config = self._setting['hand_controller_prefix']
        self._hand_client = trajectory.TrajectoryController("/hsrb/gripper_effort")
#        self._hand_client = trajectory.TrajectoryController(hand_config)
        base_config = self._setting["omni_base_controller_prefix"]
        self._base_client = trajectory.TrajectoryController(
            base_config,
            "/base_coordinates")
        imp_config = settings.get_entry("trajectory", "impedance_control")
        self._impedance_client = trajectory.ImpedanceController(imp_config)
        joint_state_topic = self._setting["joint_states_topic"]
        self._joint_state_sub = utils.CachingSubscriber(
            joint_state_topic,
            JointState,
            default=JointState())
        timeout = self._setting.get('timeout', None)
        self._joint_state_sub.wait_for_message(timeout)
        self._tf2_buffer = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self._tf2_buffer)
        self._end_effector_frames = self._setting['end_effector_frames']
        self._end_effector_frame = self._end_effector_frames[0]
        self._robot_urdf = robot_model.RobotModel.from_parameter_server()
        self._joint_names = ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint',
                             'base_l_drive_wheel_joint', 'base_r_drive_wheel_joint',
                             'base_roll_joint', 'hand_l_spring_proximal_joint',
                             'hand_motor_joint', 'hand_r_spring_proximal_joint',
                             'head_pan_joint', 'head_tilt_joint', 'wrist_flex_joint',
                             'wrist_roll_joint']
        self._collision_world = None
        self._linear_weight = 50.0
        self._angular_weight = 1.0
        self._planning_timeout = _PLANNING_ARM_TIMEOUT
        self._use_base_timeopt = True

    def _get_joint_state(self):
        """Get a current joint state.

        Returns:
            sensor_msgs.JointState: Current joint state
        """
        joint = JointState()
        joint_sig = self._joint_state_sub.data
        joint.header = joint_sig.header
        for j in self._joint_names:
            l = np.where(np.array(joint_sig.name)==j)[0]
            if len(l)==0:
                joint.name.append(j)
                joint.position.append(0.)
                joint.velocity.append(0.)
                joint.effort.append(0.)
            else:
                joint.name.append(j)
                joint.position.append(joint_sig.position[l[0]])
                joint.velocity.append(joint_sig.velocity[l[0]])
                joint.effort.append(joint_sig.effort[l[0]])
        return joint

    @property
    def joint_names(self):
        return self._get_joint_state().name

    @property
    def joint_positions(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.velocity))

    @property
    def joint_state(self):
        return self._get_joint_state()

    @property
    def joint_limits(self):
        joint_map = self._robot_urdf.joint_map
        return {joint_name: (joint_map[joint_name].limit.lower,
                             joint_map[joint_name].limit.upper)
                for joint_name in self.joint_names}

    @property
    def collision_world(self):
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
        if value is None:
            self._collision_world = None
        elif isinstance(value, collision_world.CollisionWorld):
            self._collision_world = value
        else:
            raise TypeError("value should be CollisionWorld instance")

    @property
    def linear_weight(self):
        return self._linear_weight

    @linear_weight.setter
    def linear_weight(self, value):
        self._linear_weight = value

    @property
    def angular_weight(self):
        return self._angular_weight

    @angular_weight.setter
    def angular_weight(self, value):
        self._angular_weight = value

    @property
    def planning_timeout(self):
        return self._planning_timeout

    @planning_timeout.setter
    def planning_timeout(self, value):
        self._planning_timeout = value

    @property
    def impedance_config(self):
        return self._impedance_client.config

    @impedance_config.setter
    def impedance_config(self, value):
        self._impedance_client.config = value

    @property
    def impedance_config_names(self):
        return self._impedance_client.config_names

    @property
    def use_base_timeopt(self):
        return self._use_base_timeopt

    @use_base_timeopt.setter
    def use_base_timeopt(self, value):
        self._use_base_timeopt = value

    @property
    def end_effector_frame(self):
        """Get or set the target end effector frame of motion planning.

        This attribute affects behaviors of following methods:
        * get_end_effector_pose
        * move_end_effector_pose
        * move_end_effector_by_line
        """
        return self._end_effector_frame

    @end_effector_frame.setter
    def end_effector_frame(self, value):
        if value in set(self._end_effector_frames):
            self._end_effector_frame = value
        else:
            msg = "`ref_frame_id` must be one of end-effector frames({0})"
            raise ValueError(msg.format(self._end_effector_frames))

    @property
    def end_effector_frames(self):
        return tuple(self._end_effector_frames)   

    def get_end_effector_pose(self, ref_frame_id=None):
        """Get a pose of end effector based on robot frame.

        Returns:
            Tuple[Vector3, Quaternion]
        """
        # Default reference frame is a robot frame
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        transform = self._tf2_buffer.lookup_transform(
            ref_frame_id,
            self._end_effector_frame,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
        result = geometry.transform_to_tuples(transform.transform)
        return result

        
    def _lookup_odom_to_ref(self, ref_frame_id):
        """Resolve current reference frame transformation from ``odom``.

        Returns:
            geometry_msgs.msg.Pose:
                A transform from robot ``odom`` to ``ref_frame_id``.
        """
        odom_to_ref_ros = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            ref_frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        ).transform
        odom_to_ref_tuples = geometry.transform_to_tuples(odom_to_ref_ros)
        return geometry.tuples_to_pose(odom_to_ref_tuples)
        
    def move_cartesian_path(self, waypoints, handpoints,ref_frame_id=None):
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        base_frame = settings.get_frame('base')
        origin_to_ref_ros_pose = self._lookup_odom_to_ref(ref_frame_id)
        ##原点のodomからのpose
        origin_to_ref = geometry.pose_to_tuples(origin_to_ref_ros_pose)
        ##原点のodomからのtuple
        origin_to_pose1 = self.get_end_effector_pose('odom')
        ##現在の手先位置
        odom_to_robot_pose = self._lookup_odom_to_ref(base_frame)
        ##足元のodomからのpose
        initial_joint_state = self._get_joint_state()
        ##現在姿勢
        if self._collision_world is not None:
            collision_env = self._collision_world.snapshot('odom')
        else:
            collision_env = None
        ##障害物設定

        arm_traj = None
        base_traj = None
        p = []
        for i in range(len(waypoints)):
            ##各動作点の計算
            origin_to_pose2 = geometry.multiply_tuples(origin_to_ref,
                                                       waypoints[i])
            plan = self._plan_cartesian_path(origin_to_pose1,
                                             origin_to_pose2,
                                             odom_to_robot_pose,
                                             initial_joint_state,
                                             collision_env)
            p.append(plan)
            if arm_traj is None:
                arm_traj = plan.solution
                arm_traj.points = [plan.solution.points[0],plan.solution.points[-1]] 
            elif len(plan.solution.points) > 0:
                arm_traj.points.append(plan.solution.points[-1])
            if base_traj is None:
                base_traj = plan.base_solution
                base_traj.points = [plan.base_solution.points[0],plan.base_solution.points[-1]] 
            elif len(plan.base_solution.points) > 0:
                base_traj.points.append(plan.base_solution.points[-1])

            origin_to_pose1 = origin_to_pose2
            odom_to_robot_pose = RosPose()
            final_transform = plan.base_solution.points[-1].transforms[0]
            odom_to_robot_pose.position.x = final_transform.translation.x
            odom_to_robot_pose.position.y = final_transform.translation.y
            odom_to_robot_pose.position.z = final_transform.translation.z
            odom_to_robot_pose.orientation.x = final_transform.rotation.x
            odom_to_robot_pose.orientation.y = final_transform.rotation.y
            odom_to_robot_pose.orientation.z = final_transform.rotation.z
            odom_to_robot_pose.orientation.w = final_transform.rotation.w
            initial_joint_state = plan.joint_state_after_planning
            collision_env = plan.environment_after_planning

        base_traj.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(arm_traj, base_traj)
        constrained_traj.joint_names.append("hand_motor_joint")
        if type(constrained_traj.points[0].positions) == tuple:
            constrained_traj.points[0].positions = list(constrained_traj.points[0].positions)
            constrained_traj.points[0].velocities = list(constrained_traj.points[0].velocities)
            constrained_traj.points[0].accelerations = list(constrained_traj.points[0].accelerations)
        constrained_traj.points[0].positions.append(handpoints[0])
        constrained_traj.points[0].velocities.append(0.0)
        if handpoints[0] >= 1.0:
            constrained_traj.points[0].accelerations.append(0.1)
        elif handpoints[0] <= 0.1:
            constrained_traj.points[0].accelerations.append(-0.1)
        else:
            constrained_traj.points[0].accelerations.append(0.0)            
        for i in range(len(handpoints)):
            constrained_traj.points[i+1].positions = list(constrained_traj.points[i+1].positions)
            constrained_traj.points[i+1].velocities = list(constrained_traj.points[i+1].velocities)
            constrained_traj.points[i+1].accelerations = list(constrained_traj.points[i+1].accelerations)
            constrained_traj.points[i+1].positions.append(handpoints[i])
            constrained_traj.points[i+1].velocities.append(0.0)
        if handpoints[i] >= 1.0:
            constrained_traj.points[0].accelerations.append(0.1)
        elif handpoints[i] <= 0.1:
            constrained_traj.points[0].accelerations.append(-0.1)
        else:
            constrained_traj.points[0].accelerations.append(0.0)            
        ##この結果の抽出え
        self._execute_trajectory(constrained_traj)        
        return constrained_traj

    def _transform_base_trajectory(self, base_traj):
        """Transform a base trajectory to an ``odom`` frame based trajectory.

        Args:
            base_traj (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A base trajectory based on ``odom`` frame.
        """
        odom_to_frame_transform = self._tf2_buffer.lookup_transform(
            _BASE_TRAJECTORY_ORIGIN,
            base_traj.header.frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT))
        odom_to_frame = geometry.transform_to_tuples(
            odom_to_frame_transform.transform)

        num_points = len(base_traj.points)
        odom_base_traj = JointTrajectory()
        odom_base_traj.points = list(utils.iterate(JointTrajectoryPoint,
                                                   num_points))
        odom_base_traj.header = base_traj.header
        odom_base_traj.joint_names = self._base_client.joint_names

        # Transform each point into odom frame
        previous_theta = 0.0
        for i in range(num_points):
            t = base_traj.points[i].transforms[0]
            frame_to_base = geometry.transform_to_tuples(t)

            # odom_to_base = odom_to_frame * frame_to_base
            (odom_to_base_trans, odom_to_base_rot) = geometry.multiply_tuples(
                odom_to_frame,
                frame_to_base
            )

            odom_base_traj.points[i].positions = [odom_to_base_trans[0],
                                                  odom_to_base_trans[1],
                                                  0]
            roll, pitch, yaw = T.euler_from_quaternion(
                odom_to_base_rot)
            dtheta = geometry.shortest_angular_distance(previous_theta, yaw)
            theta = previous_theta + dtheta

            odom_base_traj.points[i].positions[2] = theta
            previous_theta = theta
        return odom_base_traj

    def _constrain_trajectories(self, joint_trajectory, base_trajectory):
        """Apply constraints to given trajectories.

        Parameters:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                A upper body trajectory
            base_trajectory (trajectory_msgs.msg.JointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A constrained trajectory
        Raises:
            TrajectoryFilterError:
                Failed to execute trajectory-filtering
        """
        odom_base_trajectory = self._transform_base_trajectory(base_trajectory)
        merged_traj = trajectory.merge(joint_trajectory, odom_base_trajectory)

        filtered_merged_traj = trajectory.constraint_filter(merged_traj)
        if not self._use_base_timeopt:
            return filtered_merged_traj

        # Transform arm and base trajectories to time-series trajectories
        filtered_joint_traj = trajectory.constraint_filter(joint_trajectory)
        filtered_base_traj = trajectory.timeopt_filter(odom_base_trajectory)
        last_joint_point = filtered_joint_traj.points[-1]
        last_base_point = filtered_base_traj.points[-1]
        arm_joint_time = last_joint_point.time_from_start.to_sec()
        base_timeopt_time = last_base_point.time_from_start.to_sec()

        # If end of a base trajectory is later than arm one,
        # an arm trajectory is made slower.
        # (1) arm_joint_time < base_timeopt_time: use timeopt trajectory
        if arm_joint_time < base_timeopt_time:
            # Adjusting arm trajectory points to base ones as much as possible
            trajectory.adjust_time(filtered_base_traj, filtered_joint_traj)
            return trajectory.merge(filtered_joint_traj, filtered_base_traj)
        else:
            return filtered_merged_traj        

    def _plan_cartesian_path(self, origin_to_pose1, origin_to_pose2,
                             odom_to_robot_pose,
                             initial_joint_state, collision_env):
        use_joints = (
            b'wrist_flex_joint',
            b'wrist_roll_joint',
            b'arm_roll_joint',
            b'arm_flex_joint',
            b'arm_lift_joint'
        )

        req = PlanWithTsrConstraintsRequest()
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = initial_joint_state
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.use_joints = use_joints
        req.weighted_joints = [b'_linear_base', b'_rotational_base']
        req.weight = [self._linear_weight, self._angular_weight]
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.attached_objects = []
        if collision_env is not None:
            req.environment_before_planning = collision_env
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.extra_constraints = []
        req.extra_goal_constraints = []

        move_axis, distance = _movement_axis_and_distance(origin_to_pose1,
                                                          origin_to_pose2)
        origin_to_axis = _pose_from_x_axis(move_axis)
        pose1_to_axis = geometry.multiply_tuples(_invert_pose(origin_to_pose1),
                                                 origin_to_axis)
        pose1_to_axis = geometry.Pose((0, 0, 0), pose1_to_axis[1])
        origin_to_tsr = geometry.multiply_tuples(
            origin_to_pose1, pose1_to_axis)
        tsr_to_pose1 = _invert_pose(pose1_to_axis)



        # Goal constraint
        tsr_g = TaskSpaceRegion()
        tsr_g.end_frame_id = bytes(self.end_effector_frame)
        tsr_g.origin_to_tsr = geometry.tuples_to_pose(origin_to_pose2)
        tsr_g.tsr_to_end = geometry.tuples_to_pose(geometry.pose())
        tsr_g.min_bounds = [0, 0, 0, 0, 0, 0]
        tsr_g.max_bounds = [0, 0, 0, 0, 0, 0]

        # Line constraint
        tsr_c = TaskSpaceRegion()
        tsr_c.end_frame_id = bytes(self.end_effector_frame)
        tsr_c.origin_to_tsr = geometry.tuples_to_pose(origin_to_tsr)
        tsr_c.tsr_to_end = geometry.tuples_to_pose(tsr_to_pose1)
        tsr_c.min_bounds = [0, 0, 0, -math.pi, -math.pi, -math.pi]
        tsr_c.max_bounds = [distance, 0, 0, math.pi, math.pi, math.pi]

        req.goal_tsrs = [tsr_g]
        req.constraint_tsrs = [tsr_c]

        service_name = self._setting['plan_with_constraints_service']
        plan_service = rospy.ServiceProxy(service_name,
                                          PlanWithTsrConstraints)
        res = plan_service.call(req)
        print(res)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan"
            print(msg)
            print(req)
            raise exceptions.MotionPlanningError(msg, res.error_code)
        return res

    def _execute_trajectory(self, joint_traj):
        clients = []
        if self._impedance_client.config is not None:
            clients.append(self._impedance_client)
        else:
            clients.extend([self._arm_client, self._base_client])
        for joint in joint_traj.joint_names:
            if joint in self._head_client.joint_names:
                clients.append(self._head_client)
            if joint in self._hand_client.joint_names:
                clients.append(self._hand_client)

        joint_states = self._get_joint_state()
##ここ
        for client in clients:
            traj = trajectory.extract(joint_traj, client.joint_names,
                                      joint_states)
#            print(traj)
#            traj = self._check_hand_trajectory(traj)
            client.submit(traj)

        trajectory.wait_controllers(clients)
        
    def _check_hand_trajectory(self,traj):
        if "hand_motor_joint" in traj.joint_names:
            p = traj.points
            f = p[0].positions[0]
            if f >= 1.2:
                flag = True
            else:
                flag = False
            for i in range(len(p)):
                param =  p[i].positions[0]
                if flag:
                    if param <= 0.0:
                        break
                else:
                    if param >= 1.2:
                        break
            pp = p[:i+1]
            traj.points = pp
        return traj
            
        