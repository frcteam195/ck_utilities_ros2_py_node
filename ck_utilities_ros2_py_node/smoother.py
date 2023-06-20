#!/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import SmoothBivariateSpline, InterpolatedUnivariateSpline
from geometry import Pose, Twist, Transform, Translation, Rotation
from math import radians, pi
from copy import deepcopy

distance_results = []
velocity_results = []
acceleration_results = []

class TrajectoryPoint:
    def __init__(self):
        self.__pose = Pose()
        self.__velocity = None
        self.__track = None
        self.__accel = None

    def get_projected_point(self):
        translation = Translation()
        translation.x = self.velocity

        rotation = Rotation()
        rotation.yaw = radians(self.__track)
        translation = translation.rotate(rotation)

        twist = Twist()
        twist.linear.x = translation.x
        twist.linear.y = translation.y

        new_point = TrajectoryPoint()
        new_point.pose.position.x = self.__pose.position.x + twist.linear.x
        new_point.pose.position.y = self.__pose.position.y + twist.linear.y
        new_point.velocity = self.velocity
        return new_point

    @property
    def pose(self):
        return self.__pose
    @pose.setter
    def pose(self, value):
        self.__pose = value

    @property
    def velocity(self):
        return self.__velocity
    @velocity.setter
    def velocity(self, value):
        self.__velocity = value

    @property
    def track(self):
        return self.__track
    @track.setter
    def track(self, value):
        self.__track = value

    @property
    def accel(self):
        return self.__accel
    @accel.setter
    def accel(self, value):
        self.__accel = value

class Trajectory:
    def __init__(self):
        self.__max_velocity = None
        self.__max_accel = None
        self.__max_angular_velocity = None
        self.__max_angular_accel = None
        self.__traj_points = None
        self.__result_points = None

    def calculate_trajectory(self):
        points_with_tracks = []
        global distance_results
        global velocity_results
        global acceleration_results

        if self.__max_velocity is None:
            raise Exception("Max velocity must be set")

        for point in self.__traj_points:
            new_point = deepcopy(point)
            if new_point.velocity is None:
                new_point.velocity = self.__max_velocity
            points_with_tracks.append(new_point)
            points_with_tracks.append(new_point.get_projected_point())

        x_points = []
        y_points = []

        for point in points_with_tracks:
            x_points.append(point.pose.position.x)
            y_points.append(point.pose.position.y)

        points = np.array([x_points,y_points]).T

        distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
        distance = np.insert(distance, 0, 0)/distance[-1]

        x_spl = InterpolatedUnivariateSpline(distance, x_points, k=5)
        y_spl = InterpolatedUnivariateSpline(distance, y_points, k=5)
        range = np.linspace(0, 1, 1000)

        self.__result_points = []

        result_x_points = []
        result_y_points = []
        for i in range:
            new_point = TrajectoryPoint()
            new_point.pose.position.x = x_spl(i)
            new_point.pose.position.y = y_spl(i)
            result_x_points.append(new_point.pose.position.x)
            result_y_points.append(new_point.pose.position.y)
            self.__result_points.append(new_point)

        path_points = np.array([result_x_points,result_y_points]).T

        path_distance = np.cumsum( np.sqrt(np.sum( np.diff(path_points, axis=0)**2, axis=1 )) )
        path_distance = np.insert(path_distance, 0, 0)/path_distance[-1]

        # path_distance[0] = 0
        # path_distance[1] = 0
        # path_distance[len(path_distance)-1] = path_distance[len(path_distance)-2]

        print(path_distance)

        flat_beginning = xi = np.linspace(0, pi, 5)

        distance_spl = SmoothBivariateSpline(range, path_distance)
        velocity_spl = distance_spl.derivative()
        accel_spl = velocity_spl.derivative()

        for i in range:
            distance_results.append(distance_spl(i))
            velocity_results.append(velocity_spl(i))
            acceleration_results.append(accel_spl(i))

    @property
    def max_velocity(self):
        return self.__max_velocity
    @max_velocity.setter
    def max_velocity(self, value):
        self.__max_velocity = value

    @property
    def input_trajectory(self):
        return self.__traj_points
    @input_trajectory.setter
    def input_trajectory(self, value):
        self.__traj_points = value

    @property
    def output_trajectory(self):
        return self.__result_points

points = []

point_1 = TrajectoryPoint()
point_1.pose.position.x = 0
point_1.pose.position.y = 0
point_1.track = 0
points.append(point_1)

point_2 = TrajectoryPoint()
point_2.pose.position.x = 0
point_2.pose.position.y = -100
point_2.track = 0
points.append(point_2)

point_3 = TrajectoryPoint()
point_3.pose.position.x = 72
point_3.pose.position.y = -20
point_3.track = 0
points.append(point_3)

traj = Trajectory()
traj.input_trajectory = points
traj.max_velocity = 4
traj.calculate_trajectory()

x_vals = []
y_vals = []
speed_vals = []
chart_range = []
current_range = 0

for point in traj.output_trajectory:
    x_vals.append(point.pose.position.x)
    y_vals.append(point.pose.position.y)
    speed_vals.append(point.velocity)
    chart_range.append(current_range)
    current_range += 1

x_anchors = []
y_anchors = []

for point in points:
    x_anchors.append(point.pose.position.x)
    y_anchors.append(point.pose.position.y)

figure, axis = plt.subplots(4)

axis[0].plot(x_anchors, y_anchors, 'ro', ms=5)

axis[0].plot(x_vals, y_vals, 'b', lw=3)

axis[1].plot(chart_range, distance_results, 'b', lw=3)
axis[2].plot(chart_range, velocity_results, 'b', lw=3)
axis[3].plot(chart_range, acceleration_results, 'b', lw=3)

plt.show()

