#!/usr/bin/python3
import math
from abc import ABC, abstractmethod


class ManipPose:

    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def from_list(data):
        return ManipPose(*data)

    def to_list(self):
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


class ManipJointState:

    def __init__(self, joint_value_list):
        self.position = joint_value_list

    def from_list(data):
        return ManipJointState(data)

    def to_list(self):
        return self.position


class IKSolver(ABC):

    def __init__(self, names, lengths, limits):
        self.joint_names = names
        self.limits = limits
        self.lengths = lengths

    @abstractmethod
    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        pass

    @abstractmethod
    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        pass


# Class implementing kinematics of SiriusII manipulator
class SiriusII_IKSolver(IKSolver):

    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        try:
            return self._calculate_IK_solution(target)
        except ValueError:
            raise Exception("No IK solution! Possibly out of range")
        except Exception as e:
            raise e

    def _calculate_IK_solution(self, target: ManipPose) -> ManipJointState:

        solution = self._initialize_solution()
        limits = self.limits
        lengths = self.lengths
        x_t = target.x
        y_t = target.y

        # The reasoning behind this code is explained in docs/Inverse_Kinematics_Formula.pdf

        # Last joint value simply equals to its target value
        solution[4] = target.roll
        # First joint angle can be easily obtained as it is
        # the only joint allowing for movement along y-axis
        solution[0] = math.atan2(y_t, x_t)
        if not checkBounds(solution[0], limits[0]):
            raise Exception("IK solution outside of joint limits!")

        # Project target position to manip plane
        r_t = math.sqrt(x_t * x_t + y_t * y_t)
        # Translate, so link 0 length does not need to be considered
        # in formulas below
        z_t = target.z - lengths[0]

        # Find 3rd link's start position given its absolute angle
        # (target.alpha) and end position (target position)
        alpha = target.pitch
        r_3, z_3 = self._get_link3_startposition(lengths, r_t, z_t, alpha)

        # Find 2nd link's start position
        # The geometry constraints lead to equation system with
        # two solutions. Due to physical constraints of the manipulator
        # only one of these solutions will be possible to reach
        r_2, z_2 = self._find_middlepoint_solution1(lengths, r_3, z_3)
        # Given the position, angles of the joints can be calculated
        solution[1], solution[2], solution[3] = self._calculate_angles(r_3, z_3, r_2, z_2, alpha)
        # Check if the calculated angles are within manipulator's limits
        if self._angles_within_constraints(solution):
            return ManipJointState.from_list(solution)

        raise Exception("IK solution outside of joints limits")

    def _get_link3_startposition(self, lengths, r_t, z_t, alpha):
        r_3 = r_t - lengths[3] * math.cos(alpha)
        z_3 = z_t + lengths[3] * math.sin(alpha)
        return r_3, z_3

    def _angles_within_constraints(self, solution):
        return all([checkBounds(solution[i], self.limits[i]) for i in range(1, len(solution))])

    def _calculate_angles(self, r_3, z_3, r_2, z_2, alpha):
        beta = math.atan2(z_2, r_2)
        gamma = math.atan2(z_2 - z_3, r_3 - r_2)
        angle1 = math.pi / 2 - beta
        angle2 = beta + gamma
        angle3 = alpha - gamma
        return angle1, angle2, angle3

    def _find_middlepoint_solution1(self, l, x, y):
        r_2 = (x * x + l[1] * l[1] - l[2] * l[2] + y * y -
               (y * (x * math.sqrt((l[1] * l[2] * 2.0 - x * x + l[1] * l[1] + l[2] * l[2] - y * y) *
                                   (l[1] * l[2] * 2.0 + x * x - l[1] * l[1] - l[2] * l[2] + y * y)) + (x * x) * y +
                     (l[1] * l[1]) * y - (l[2] * l[2]) * y + y * y * y)) / (x * x + y * y)) / (x * 2.0)
        z_2 = (x * math.sqrt((l[1] * l[2] * 2.0 - x * x + l[1] * l[1] + l[2] * l[2] - y * y) *
                             (l[1] * l[2] * 2.0 + x * x - l[1] * l[1] - l[2] * l[2] + y * y)) + (x * x) * y +
               (l[1] * l[1]) * y - (l[2] * l[2]) * y + y * y * y) / ((x * x) * 2.0 + (y * y) * 2.0)

        return r_2, z_2

    def _initialize_solution(self):
        solution = [0] * len(self.limits)
        return solution

    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        angles = jointstate.position

        l = self.lengths
        solution = ManipPose()

        # Calculate solution using SiriusII-specific FK formula
        d = l[1] * math.sin(
            angles[1]) + l[2] * math.sin(angles[1] + angles[2]) + l[3] * math.sin(angles[1] + angles[2] + angles[3])
        z = l[1] * math.cos(
            angles[1]) + l[2] * math.cos(angles[1] + angles[2]) + l[3] * math.cos(angles[1] + angles[2] + angles[3])
        solution.z = z + l[0]
        solution.x = d * math.cos(angles[0])
        solution.y = d * math.sin(angles[0])
        solution.pitch = angles[1] + angles[2] + angles[3] - 0.5 * math.pi
        solution.roll = angles[4]

        return solution


def checkBounds(value, bounds):
    return value >= bounds[0] and value <= bounds[1]
