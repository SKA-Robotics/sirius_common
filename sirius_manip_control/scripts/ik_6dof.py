#!/usr/bin/python3
import math
import numpy as np
from ik import SiriusII_IKSolver, ManipPose, ManipJointState, IKSolver


# Class implementing kinematics of SiriusII manipulator
class SiriusII_6DofIKSolver(IKSolver):

    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        try:
            return self._calculate_IK_solution(target)
        except ValueError:
            raise Exception("No IK solution! Possibly out of range")
        except Exception as e:
            raise e

    def _calculate_IK_solution(self, target: ManipPose) -> ManipJointState:
        if math.cos(target.pitch) < 0.0:
            raise Exception(
                "Invalid pitch angle! Supported range is [-pi/2, pi/2]")

        l = self.lengths
        x4 = target.x - l[4] * math.cos(target.pitch) * math.cos(target.yaw)
        y4 = target.y - l[4] * math.cos(target.pitch) * math.sin(target.yaw)
        z4 = target.z + l[4] * math.sin(target.pitch)

        siplify_rotation_angle = math.atan2(y4, x4)
        vec_5_4 = np.array([x4 - target.x, y4 - target.y, z4 - target.z])
        local_vec_5_4 = np.array([
            vec_5_4[0] * math.cos(siplify_rotation_angle) +
            vec_5_4[1] * math.sin(siplify_rotation_angle),
            vec_5_4[1] * math.cos(siplify_rotation_angle) -
            vec_5_4[0] * math.sin(siplify_rotation_angle),
            vec_5_4[2],
        ])
        vec_5_4 = local_vec_5_4
        main_plane_versor = np.array([0.0, 1.0, 0.0])

        # TODO: analize cases for parallelism of any vector pair being used below
        z_dir = -1.0 if vec_5_4[2] < 0 else 1.0
        r = math.hypot(vec_5_4[0], vec_5_4[1])
        local_z_axis = np.array([
            -vec_5_4[0] * vec_5_4[2] * z_dir,
            -vec_5_4[1] * vec_5_4[2] * z_dir,
            z_dir * (vec_5_4[0]**2 + vec_5_4[1]**2),
        ])
        local_z_axis /= np.linalg.norm(local_z_axis)
        new_local_z_axis = np.cross(main_plane_versor, vec_5_4)
        new_local_z_axis /= np.linalg.norm(new_local_z_axis)
        vector_roll_angle = np.cross(local_z_axis, new_local_z_axis)

        # [TEMP] print values to see parallelism
        # roll_axis = vec_5_4 / np.linalg.norm(vec_5_4)
        # print('Roll axis rev')
        # print_formated(roll_axis)
        # roll_ang_n = vector_roll_angle / np.linalg.norm(vector_roll_angle)
        # print('Roll angle')
        # print_formated(roll_ang_n)

        # TODO: fix this shit below (should be parallel or antiparallel in any cases)
        # sign = np.dot(vector_roll_angle, vec_5_4) / math.hypot(*vec_5_4) / np.linalg.norm(vector_roll_angle)
        # if (abs(sign - 1) > 0.000_001 and abs(sign + 1) > 0.000_001):
        # print(f"No pararell roll axis: {sign}")

        local_roll_angle_sin = np.dot(vector_roll_angle,
                                      vec_5_4) / math.hypot(*vec_5_4)
        local_roll_angle_cos = np.dot(local_z_axis, new_local_z_axis)
        local_roll_angle = math.atan2(local_roll_angle_sin,
                                      local_roll_angle_cos)

        local_y_axis = np.cross(vec_5_4, new_local_z_axis)
        local_y_axis /= np.linalg.norm(local_y_axis)
        vector_yaw_angle = np.cross(main_plane_versor, local_y_axis)
        local_yaw_angle_sin = np.dot(vector_yaw_angle, new_local_z_axis)
        local_yaw_angle_cos = np.dot(main_plane_versor, local_y_axis)
        local_yaw_angle = math.atan2(local_yaw_angle_sin, local_yaw_angle_cos)

        vector_pitch_angle = np.cross((0.0, 0.0, 1.0), new_local_z_axis)
        local_pitch_angle_sin = np.dot(vector_pitch_angle, main_plane_versor)
        local_pitch_angle_cos = np.dot((0.0, 0.0, 1.0), new_local_z_axis)
        local_pitch_angle = math.atan2(local_pitch_angle_sin,
                                       local_pitch_angle_cos)

        if local_yaw_angle < self.limits[4][
                0] or local_yaw_angle > self.limits[4][1]:
            raise Exception("IK solution outside of joint limits!")

        l_5 = [l[0], l[1], l[2], l[3]]
        limits_5 = [
            self.limits[0],
            self.limits[1],
            self.limits[2],
            self.limits[3],
            self.limits[5],
        ]
        solver_5dof = SiriusII_IKSolver([], l_5, limits_5)
        state_5 = solver_5dof.get_IK_solution(
            ManipPose.from_list([
                x4, y4, z4, target.roll + local_roll_angle, local_pitch_angle,
                0.0
            ]))
        state_6 = state_5.to_list()
        state_6.insert(4, local_yaw_angle)
        result = ManipJointState.from_list(state_6)
        feedback = self.get_FK_solution(result)
        result.position[5] += target.roll - feedback.roll

        print("target", target.to_list())
        print("result", result.position)

        phi = result.position[4]
        theta = result.position[5]
        result.position[4] = phi + theta / 2
        result.position[5] = -phi + theta / 2

        return result

    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        angles = jointstate.position
        top_gear_position = angles[4]
        bottom_gear_position = angles[5]
        angles[5] = (top_gear_position + bottom_gear_position)
        angles[4] = (top_gear_position - bottom_gear_position) / 2
        # print(f"turn_angle={angles[4]}, spin_angle={angles[5]}")

        l = self.lengths
        solution = ManipPose()

        r4 = (l[1] * math.sin(angles[1]) +
              l[2] * math.sin(angles[1] + angles[2]) +
              l[3] * math.sin(angles[1] + angles[2] + angles[3]))
        z4 = (l[1] * math.cos(angles[1]) +
              l[2] * math.cos(angles[1] + angles[2]) +
              l[3] * math.cos(angles[1] + angles[2] + angles[3]))

        local_pitch = angles[1] + angles[2] + angles[3] - 0.5 * math.pi
        dx = l[4] * math.cos(angles[4]) * math.cos(local_pitch)
        dy = l[4] * math.sin(angles[4])
        dz = l[4] * math.cos(angles[4]) * math.sin(-local_pitch)

        solution.z = z4 + dz + l[0]
        solution.x = (r4 + dx) * math.cos(angles[0]) - dy * math.sin(angles[0])
        solution.y = (r4 + dx) * math.sin(angles[0]) + dy * math.cos(angles[0])
        # solution.roll = angles[5] + np.sign(angles[4]) * local_pitch # probably wrong
        # local_roll = math.asin(math.sin(angles[4]) * math.sin(local_pitch))

        local_horizontal_y = np.array([
            -math.sin(angles[4]),
            math.cos(local_pitch) * math.cos(angles[4]), 0.0
        ])
        local_horizontal_y /= np.linalg.norm(local_horizontal_y)
        local_z = np.array([math.sin(local_pitch), 0.0, math.cos(local_pitch)])
        local_y = np.array([
            -math.cos(local_pitch) * math.sin(angles[4]),
            math.cos(angles[4]),
            math.sin(local_pitch) * math.sin(angles[4]),
        ])
        solution.roll = (math.atan2(
            -np.dot(local_z, local_horizontal_y),
            np.dot(local_y, local_horizontal_y),
        ) + angles[5])

        # a = math.sin(angles[4]) * math.cos(local_pitch)
        # b = math.cos(angles[4])
        # solution.roll = math.atan2(abs(math.sin(angles[4])) * math.sin(local_pitch),
        #                            b / math.cos(math.atan2(b, a))) + angles[5]

        # solution.roll = math.asin(math.sin(angles[4]) * math.sin(local_pitch)) + angles[5] # should be good for (-pi/2, pi/2)
        solution.pitch = math.asin(math.sin(local_pitch) * math.cos(angles[4]))
        solution.yaw = (
            math.atan2(math.sin(angles[4]),
                       math.cos(angles[4]) * math.cos(local_pitch)) +
            angles[0])
        solution.yaw = math.pi * (((solution.yaw / math.pi % 2) + 3) % 2 - 1)
        # solution.pitch = math.atan2(math.sin(local_pitch) * math.cos(solution.yaw), math.cos(local_pitch))

        return solution


def print_formated(list: list) -> None:
    print(*["{:.3f},  \t".format(num) for num in list])


def rad(deg):
    return deg * math.pi / 180
