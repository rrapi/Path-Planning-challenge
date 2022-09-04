
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math



def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + np.dot(kmat,kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self):
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]

        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])

        return trans

    @staticmethod
    def basic_jacobian(trans_prev, ee_pos):
        pos_prev = np.array(
            [trans_prev[0, 3], trans_prev[1, 3], trans_prev[2, 3]])
        z_axis_prev = np.array(
            [trans_prev[0, 2], trans_prev[1, 2], trans_prev[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis_prev, ee_pos - pos_prev), z_axis_prev))
        return basic_jacobian


class NLinkArm:
    def __init__(self, dh_params_list):
        self.link_list = []
        for i in range(len(dh_params_list)):
            # print(Link(dh_params_list[i]))
            self.link_list.append(Link(dh_params_list[i]))
        # print("{} link arm".format(len(self.link_list)))

    def transformation_matrix(self, last=True):
        trans = np.identity(4)
        if last:
            for i in range(len(self.link_list)):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
        else:
            for i in range(len(self.link_list)-2):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
        return trans

    def forward_kinematics(self, last=True, plot=False):
        trans = self.transformation_matrix(last=last)

        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        alpha, beta, gamma = self.euler_angle()

        if plot:
            self.fig = plt.figure()
            self.ax = Axes3D(self.fig, auto_add_to_figure=False)
            self.fig.add_axes(self.ax)

            x_list = []
            y_list = []
            z_list = []

            trans = np.identity(4)

            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])
            for i in range(len(self.link_list)):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
                x_list.append(trans[0, 3])
                y_list.append(trans[1, 3])
                z_list.append(trans[2, 3])

            self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                         mew=0.5)
            self.ax.plot([0], [0], [0], "o")

            self.ax.set_xlim(-1, 1)
            self.ax.set_ylim(-1, 1)
            self.ax.set_zlim(-1, 1)

            plt.show()

        return [x, y, z, alpha, beta, gamma]

    def basic_jacobian(self):
        ee_pos = self.forward_kinematics()[0:3]
        basic_jacobian_mat = []

        trans = np.identity(4)
        for i in range(len(self.link_list)):
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
            trans = np.dot(trans, self.link_list[i].transformation_matrix())

        return np.array(basic_jacobian_mat).T

    def get_current_pose(self, last=True, plot=False):
        return self.forward_kinematics(last=last, plot=plot)

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        @param: goal       A list of floats [x, y, z, roll, pitch, yaw]
        @param: actual     A list of floats [x, y, z, roll, pitch, yaw]
        @param: tolerance  A float
        @returns: bool
        """
        assert(type(goal) is list and type(actual) is list)
        assert(len(goal) == len(actual))

        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

        return True

    def reached_waypoint(self, waypoint, actual, tolerance):
        """
        Convenience method for testing a ceratain waypoint has been reached by the eef.
        @param: waypoint       A list of floats [x, y, z]
        @param: actual         A list of floats [x, y, z]
        @param: tolerance  A float
        @returns: bool
        """
        assert(type(waypoint) is list and type(actual) is list)
        assert(len(waypoint) == len(actual))

        for index in range(len(waypoint)):
            if abs(actual[index] - waypoint[index]) > tolerance:
                return False

        return True

    def inverse_kinematics(self, ref_ee_pose, plot=False):
        """
        This function calculates the IK given the desired pose and updates the robot joint angles.
        @param: ref_ee_pose       A list of floats [x, y, z, roll, pitch, yaw]
        @returns: None
        """

        for cnt in range(500):
            ee_pose = self.forward_kinematics()
            diff_pose = np.array(ref_ee_pose) - ee_pose

            basic_jacobian_mat = self.basic_jacobian()
            alpha, beta, gamma = self.euler_angle()

            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            self.update_joint_angles(theta_dot / 100.)

        if plot:
            self.fig = plt.figure()
            self.ax = Axes3D(self.fig)

            x_list = []
            y_list = []
            z_list = []

            trans = np.identity(4)

            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])
            for i in range(len(self.link_list)):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
                x_list.append(trans[0, 3])
                y_list.append(trans[1, 3])
                z_list.append(trans[2, 3])

            self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                         mew=0.5)
            self.ax.plot([0], [0], [0], "o")

            self.ax.set_xlim(-1, 1)
            self.ax.set_ylim(-1, 1)
            self.ax.set_zlim(-1, 1)

            self.ax.plot([ref_ee_pose[0]], [ref_ee_pose[1]], [ref_ee_pose[2]],
                         "o")
            plt.show()

    def goto_pose(self, goal):
        """
        Convenience method for reaching a certain pose by the eef.
        @param: goal       A list of floats [x, y, z, roll, pitch, yaw]
        @returns: final eef position
        """
        print("Going to pose...")
        self.inverse_kinematics(goal)
        print("...done")
        end_pose = self.get_current_pose()

        if self.all_close(goal, end_pose, 0.05):
            print("[RESULT] Pose reached within a tolerance.")
        else:
            print("[RESULT] Pose NOT reached within a tolerance.")

        return end_pose

    def plot_trajectory(self, labels, waypoints, ref_ee_pose):
        """
        Plots the path planning trajectory and environment.
        @param: labels              A list of strings ["A", "B", ...]
        @param: waypoints           A list of floats list [[x1,y1,z1], [x2,y2,z2], ...]
        @param: ref_ee_pose         A list of floats [x, y, z, roll, pitch, yaw]
        @returns: None
        """

        # plot environment

        ideal_path_x = []
        ideal_path_y = []
        ideal_path_z = []

        N = len(waypoints)

        for i in range(N):
            l = labels[i]
            point = waypoints[i]

            x, y, z = point[0], point[1], point[2]
            # roll, pitch, yaw = 
            ideal_path_x.append(x)
            ideal_path_y.append(y)
            ideal_path_z.append(z)
            self.ax.scatter(x, y, z, label=l)

        ideal_path_x = np.asarray(ideal_path_x)
        ideal_path_y = np.asarray(ideal_path_y)
        ideal_path_z = np.asarray(ideal_path_z)

        self.ax.legend(loc="best")
        self.ax.plot(ideal_path_x, ideal_path_y, ideal_path_z, color='black')

        # plot robot trajectory

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                        mew=0.5)
        self.ax.plot([0], [0], [0], "o")

        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)

        self.ax.plot([ref_ee_pose[0]], [ref_ee_pose[1]], [ref_ee_pose[2]],
                        "o")
        # plt.show()
        plt.draw()
        plt.pause(0.001)
        self.ax.clear()
        # plt.clf()

    def path_planning_trajectory(self, labels, waypoints, bound_region=0.2, step=0.1, plot=False):
        """
        Solve the path planning problem through N waypoints.
        @param: labels              A list of strings ["A", "B", ...]
        @param: waypoints           A list of floats list [[x1,y1,z1], [x2,y2,z2], ...]
        @param: bound_region        A float number
        @param: step                A float number
        @param: plot                A boolean 
        @returns: None
        """

        # real robot path
        x_true = []
        y_true = []
        z_true = []

        # ideal robot path
        ideal_path_x = []
        ideal_path_y = []
        ideal_path_z = []

        if plot:
            self.fig = plt.figure()
            self.ax = Axes3D(self.fig)

        N = len(waypoints)

        # iteration over each waypoint
        for i in range(N):
            start_pose = self.get_current_pose()
            label = labels[i]
            waypoint = waypoints[i]

            ref_ee_pose = [waypoint[0], waypoint[1], waypoint[2], start_pose[3], start_pose[4], start_pose[5]]

            print("Waypoint {}...".format(label))

            # move robot by @step to reach a certain waypoint
            done = False
            while not done:
                
                # current pose
                P1 = [start_pose[0], start_pose[1], start_pose[2]] 
                x1, y1, z1 = P1[0], P1[1], P1[2]
                # print("P1=",P1)

                # waypoint pose
                P2 = [ref_ee_pose[0], ref_ee_pose[1], ref_ee_pose[2]]
                x2, y2, z2 = P2[0], P2[1], P2[2]
                # print("P2=",P2)

                # distance between P2 and P1
                dist = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

                if dist > step:
                    
                    scale_factor = step / dist

                    # the next pose is calculated interpolating between P1 and P2 by moving of step size
                    P_next = np.asarray(P1) + np.array([x2-x1, y2-y1, z2-z1]) * scale_factor

                    # check if new pose is in bounded region
                    if i > 0:
                        p1 = np.asarray(waypoints[i-1])
                        p2 = np.asarray(waypoints[i])

                        d = np.linalg.norm(np.cross(p2-p1,P_next-p1))/np.linalg.norm(p2-p1)
                        # print("d=",d)

                        # if the next pose exceeds the limited bound region
                        while d > bound_region:
                            # decrease scale factor to find another point
                            scale_factor = scale_factor - 0.001
                            P_next = np.asarray(P1) + np.array([x2-x1, y2-y1, z2-z1]) * scale_factor
                            d = np.linalg.norm(np.cross(p2-p1,P_next-p1))/np.linalg.norm(p2-p1)

                else:
                    P_next = P2


                # useful to extract the x-axis eef vector
                pose_1 = self.get_current_pose(last=False)
                pose_2 = self.get_current_pose(last=True)

                # two vectors used to align x-axis with segment line (eg. AB)
                vec1 = np.array([pose_2[0]-pose_1[0], pose_2[1]-pose_1[1], pose_2[2]-pose_1[2]])
                vec2 = np.array([waypoint[0]-pose_2[0], waypoint[1]-pose_2[1], waypoint[2]-pose_2[2]])

                # rotation matrix to align the two vectors
                rot_v1v2 = rotation_matrix_from_vectors(vec1, vec2)

                # current eef rotation matrix
                eef_rot = self.transformation_matrix()[0:3, 0:3]

                 # final eef rotation matrix with x-axis aligned to segment
                R = np.matmul(eef_rot, rot_v1v2) 

                # Euler Angles extraction from rotation matrix
                alpha = math.atan2(R[1][2], R[0][2])
                if not (-math.pi / 2 <= alpha <= math.pi / 2):
                    alpha = math.atan2(R[1][2], R[0][2]) + math.pi
                if not (-math.pi / 2 <= alpha <= math.pi / 2):
                    alpha = math.atan2(R[1][2], R[0][2]) - math.pi
                beta = math.atan2(
                    R[0][2] * math.cos(alpha) + R[1][2] * math.sin(alpha),
                    R[2][2])
                gamma = math.atan2(
                    -R[0][0] * math.sin(alpha) + R[1][0] * math.cos(alpha),
                    -R[0][1] * math.sin(alpha) + R[1][1] * math.cos(alpha))


                roll_goal = alpha
                pitch_goal = beta
                yaw_goal = gamma

                # definition of the next eef position [x, y, z, roll, pitch, yaw]
                next_pose = [P_next[0], P_next[1], P_next[2],  roll_goal, pitch_goal, yaw_goal]
                print("NEXT POSE: ",next_pose)
                print("roll=", np.degrees(next_pose[3]))
                print("pitch=", np.degrees(next_pose[4]))
                print("yaw=", np.degrees(next_pose[5]))

                
                end_pose = self.goto_pose(next_pose)
                print("FINAL POSE: ",end_pose)
                print("roll=", np.degrees(end_pose[3]))
                print("pitch=", np.degrees(end_pose[4]))
                print("yaw=", np.degrees(end_pose[5]))

                x_true.append(end_pose[0])
                y_true.append(end_pose[1])
                z_true.append(end_pose[2])
                
                start_pose = end_pose

                # print("waypoint=",waypoint)
                # print("end=",end_pose[:3])

                if self.reached_waypoint(waypoint, end_pose[:3], 0.05):
                    print("Waypoint {} reached!".format(label))
                    done = True

                if plot:
                    self.plot_trajectory(labels, waypoints, end_pose)

        
        # self.fig = plt.figure()
        self.ax = Axes3D(self.fig)

        for i in range(N):
            l = labels[i]
            point = waypoints[i]

            x, y, z = point[0], point[1], point[2]
            # roll, pitch, yaw = 
            ideal_path_x.append(x)
            ideal_path_y.append(y)
            ideal_path_z.append(z)
            self.ax.scatter(x, y, z, label=l)

        ideal_path_x = np.asarray(ideal_path_x)
        ideal_path_y = np.asarray(ideal_path_y)
        ideal_path_z = np.asarray(ideal_path_z)

        self.ax.legend(loc="best")
        self.ax.plot(ideal_path_x, ideal_path_y, ideal_path_z, color='black')
        self.ax.plot(x_true, y_true, z_true, "r--")

        # self.ax.fill_between(ideal_path_x, ideal_path_y - error, ideal_path_y + error, alpha=0.2)

        plt.show()


    def euler_angle(self):
        trans = self.transformation_matrix()

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def set_joint_angles(self, joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] = joint_angle_list[i]

    def update_joint_angles(self, diff_joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] += diff_joint_angle_list[i]

    def plot(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                     mew=0.5)
        self.ax.plot([0], [0], [0], "o")

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        plt.show()