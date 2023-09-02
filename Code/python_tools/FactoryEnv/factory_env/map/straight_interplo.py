import numpy as np
import matplotlib.pyplot as plt

x = [0.66, 2.53, 2.53, 5.44, 8.18, 8.49, 10.31, 10.31]
y = [5.09, 5.09, 3.56, 3.56, 3.56, 2.43, 2.61, 3.68]
points = np.column_stack((x, y))

total_distance = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))

num_interp_points = 50
interp_distance = total_distance / (num_interp_points - 1)

interp_points = [points[0]]
interp_yaw_angles = []
interp_trajectory_lengths = [0]

current_distance = 0
current_point_index = 0

for _ in range(num_interp_points - 2):
    current_distance += interp_distance
    while current_point_index < len(points) - 1:
        segment_distance = np.sqrt(np.sum((points[current_point_index + 1] - points[current_point_index])**2))
        if current_distance < segment_distance:
            break
        else:
            current_distance -= segment_distance
            current_point_index += 1

    t = current_distance / segment_distance
    new_point = points[current_point_index] + t * (points[current_point_index + 1] - points[current_point_index])
    distance = np.linalg.norm(new_point - interp_points[-1])
    interp_points.append(new_point)

    dx = new_point[0] - interp_points[-2][0]
    dy = new_point[1] - interp_points[-2][1]
    yaw_angle = np.arctan2(dy, dx)
    interp_yaw_angles.append(yaw_angle)
    interp_trajectory_lengths.append(interp_trajectory_lengths[-1] + distance)

interp_points = np.array(interp_points)

print(len(interp_points))
print(len(interp_yaw_angles))
print(len(interp_trajectory_lengths))
plt.figure(1)
plt.plot(points[:, 0], points[:, 1], 'o', label='Original Points')
plt.plot(interp_points[:, 0], interp_points[:, 1], '.', label='Interpolated Points')
plt.legend()
plt.xlabel("x")
plt.ylabel("y")
plt.title("Interpolated Points")

plt.figure(2)
plt.plot(interp_trajectory_lengths[:-1], np.degrees(interp_yaw_angles), '.-')
plt.xlabel("Trajectory Length")
plt.ylabel("Yaw Angle (degrees)")
plt.title("Yaw Angle vs Trajectory Length")

plt.show()