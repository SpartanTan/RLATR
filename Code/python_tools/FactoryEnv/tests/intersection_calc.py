import numpy as np

def calculate_all_intersections(obstacles, sensor_angles):
    x1, y1 = 0, 0  # Assuming sensor is at the origin of the car frame
    intersections = []

    for theta in sensor_angles:
        closest_intersection = None
        min_distance = float('inf')

        for i in range(obstacles.shape[0]):
            a, b, r = obstacles[i, :]  # Extract x, y, radius for each obstacle
            intersection_points = calculate_intersection(a, b, r, x1, y1, theta)

            for point in intersection_points:
                x, y = point
                distance = np.sqrt((x - x1)**2 + (y - y1)**2)  # Calculate the distance from the sensor
                if distance < min_distance and distance < 1.0:
                    min_distance = distance
                    closest_intersection = point

        if closest_intersection is not None:  # If there's an intersection
            intersections.append(closest_intersection)

    return intersections

def calculate_intersection(a, b, r, x1, y1, theta):
    A = np.cos(theta)**2 + np.sin(theta)**2
    B = 2 * (np.cos(theta) * (x1 - a) + np.sin(theta) * (y1 - b))
    C = (x1 - a)**2 + (y1 - b)**2 - r**2

    # Calculate discriminant
    discriminant = B**2 - 4*A*C

    if discriminant < 0:  # No intersection
        return []
    elif discriminant == 0:  # Tangent line
        t = -B / (2*A)
        x = x1 + t * np.cos(theta)
        y = y1 + t * np.sin(theta)
        return [(x, y)]
    else:  # Two intersection points
        t1 = (-B + np.sqrt(discriminant)) / (2*A)
        x2 = x1 + t1 * np.cos(theta)
        y2 = y1 + t1 * np.sin(theta)
        t2 = (-B - np.sqrt(discriminant)) / (2*A)
        x3 = x1 + t2 * np.cos(theta)
        y3 = y1 + t2 * np.sin(theta)
        return [(x2, y2), (x3, y3)]

def calculate_all_intersections_optimized(obstacles, sensor_angles, max_range=1.0):
    x1, y1 = 0, 0  # Assuming sensor is at the origin of the car frame
    intersections = []
    
    # Compute distances of obstacles to lidar origin and sort by distance
    distances = np.sqrt(obstacles[:, 0]**2 + obstacles[:, 1]**2)
    sorted_indices = np.argsort(distances)
    obstacles = obstacles[sorted_indices]

    for theta in sensor_angles:
        A = np.cos(theta)**2 + np.sin(theta)**2  # Compute A once for each theta
        intersection_found = False

        for i in range(obstacles.shape[0]):
            a, b, r = obstacles[i, :]  # Extract x, y, radius for each obstacle
            B = 2 * (np.cos(theta) * (x1 - a) + np.sin(theta) * (y1 - b))
            C = (x1 - a)**2 + (y1 - b)**2 - r**2

            # Calculate discriminant
            discriminant = B**2 - 4*A*C

            if discriminant >= 0:  # Intersection exists
                # Calculate both intersections
                t1 = (-B + np.sqrt(discriminant)) / (2*A)
                x2 = x1 + t1 * np.cos(theta)
                y2 = y1 + t1 * np.sin(theta)
                t2 = (-B - np.sqrt(discriminant)) / (2*A)
                x3 = x1 + t2 * np.cos(theta)
                y3 = y1 + t2 * np.sin(theta)
                intersections_temp = [(x2, y2, t1), (x3, y3, t2)]
                
                # Sort by t value so that we first check the intersection closer to the sensor
                intersections_temp.sort(key=lambda x: x[2])

                for (x, y, t) in intersections_temp:
                    # Check if the intersection is in the direction of the beam and within max range
                    if theta >= -np.pi/2 and theta <= np.pi/2:  # Forward direction is positive x
                        if x >= x1 and t <= max_range:
                            intersections.append((x, y))
                            intersection_found = True
                            break
                    else:  # Forward direction is negative x
                        if x <= x1 and t <= max_range:
                            intersections.append((x, y))
                            intersection_found = True
                            break

            if intersection_found:
                break  # Stop searching for this beam as soon as we find an intersection

    return intersections


def calculate_all_intersections_vectorized_optimized(obstacles, sensor_angles, max_range=1.0):
    x1, y1 = 0, 0  # Assuming sensor is at the origin of the car frame

    # Compute distances of obstacles to lidar origin and sort by distance
    distances = np.sqrt(obstacles[:, 0]**2 + obstacles[:, 1]**2)
    sorted_indices = np.argsort(distances)
    obstacles = obstacles[sorted_indices]

    # Pre-compute A for all angles
    A_values = np.cos(sensor_angles)**2 + np.sin(sensor_angles)**2

    intersections = []
    measurements = []
    for i, theta in enumerate(sensor_angles):
        A = A_values[i]  # Fetch pre-computed A value
        intersection_found = False

        # Calculate B, C, and discriminant for all obstacles at once
        B = 2 * (np.cos(theta) * (x1 - obstacles[:, 0]) + np.sin(theta) * (y1 - obstacles[:, 1]))
        C = (x1 - obstacles[:, 0])**2 + (y1 - obstacles[:, 1])**2 - obstacles[:, 2]**2
        discriminant = B**2 - 4*A*C

        for j in range(obstacles.shape[0]):
            if discriminant[j] >= 0:  # Intersection exists
                # Calculate both intersections
                t1 = (-B[j] + np.sqrt(discriminant[j])) / (2*A)
                x2 = x1 + t1 * np.cos(theta)
                y2 = y1 + t1 * np.sin(theta)
                t2 = (-B[j] - np.sqrt(discriminant[j])) / (2*A)
                x3 = x1 + t2 * np.cos(theta)
                y3 = y1 + t2 * np.sin(theta)
                intersections_temp = [(x2, y2, t1), (x3, y3, t2)]
                
                # Sort by t value so that we first check the intersection closer to the sensor
                intersections_temp.sort(key=lambda x: x[2])

                for (x, y, t) in intersections_temp:
                    # Check if the intersection is in the direction of the beam and within max range
                    if theta >= -np.pi/2 and theta <= np.pi/2:  # Forward direction is positive x
                        if x >= x1 and t <= max_range:
                            intersections.append((x, y))
                            measurements.append((theta, t))
                            intersection_found = True
                            break
                    else:  # Forward direction is negative x
                        if x <= x1 and t <= max_range:
                            intersections.append((x, y))
                            measurements.append((theta, t))
                            intersection_found = True
                            break

            if intersection_found:
                break  # Stop searching for this beam as soon as we find an intersection
        if not intersection_found:
            measurements.append((theta, 1.0))        
    return intersections, measurements                                          
                                                        
def calculate_all_intersections_slope_optimized(obstacles, slopes, max_range=1.0):
    # Initialize an empty list to hold the intersection points
    intersections = []

    for i, slope in enumerate(slopes):
        coef_a = 1 + slope**2
        a = obstacles[:, 0]
        b = obstacles[:, 1]
        r = obstacles[:, 2]

        # Calculate coefficients of the quadratic equation
        coef_b = -2 * a - 2 * b * slope
        coef_c = a**2 + b**2 - r**2

        # Calculate discriminant
        discriminant = coef_b**2 - 4 * coef_a * coef_c

        # Find valid intersection points
        valid_indices = np.where((discriminant >= 0) & (coef_a != 0))[0]

        if valid_indices.size > 0:
            # Calculate x-coordinates of intersection points
            x1 = (-coef_b[valid_indices] + np.sqrt(discriminant[valid_indices])) / (2 * coef_a)
            x2 = (-coef_b[valid_indices] - np.sqrt(discriminant[valid_indices])) / (2 * coef_a)

            # Calculate y-coordinates of intersection points
            y1 = slope * x1
            y2 = slope * x2

            # Calculate distances from origin to intersection points
            dist1 = np.sqrt(x1**2 + y1**2)
            dist2 = np.sqrt(x2**2 + y2**2)

            # Check if intersection points are within the maximum range and on the correct side
            valid_x1 = dist1 <= max_range
            valid_x2 = dist2 <= max_range
            if slope == 0:
                valid_x1 = valid_x1 & (x1 >= 0)
                valid_x2 = valid_x2 & (x2 >= 0)
            if which_side[i]:
                valid_y1 = y1 >= 0
                valid_y2 = y2 >= 0
            else:
                valid_y1 = y1 < 0
                valid_y2 = y2 < 0
                
            valid_x1 = valid_x1 & valid_y1
            valid_x2 = valid_x2 & valid_y2
            
            x1_valid = x1[valid_x1]
            x2_valid = x2[valid_x2]
            y1_valid = y1[valid_x1]
            y2_valid = y2[valid_x2]
            dist1_valid = dist1[valid_x1]
            dist2_valid = dist2[valid_x2]
            
            valid_points_1 = np.stack((x1_valid, y1_valid, dist1_valid), axis=1)
            valid_points_2 = np.stack((x2_valid, y2_valid, dist2_valid), axis=1)
            valid_points = np.concatenate((valid_points_1, valid_points_2), axis=0)
            
            if valid_points.shape[0] > 0:
                sort_indices = np.argsort(valid_points[:, 2])
                final_valid_point = valid_points[sort_indices][0]
                intersections.append((final_valid_point[0], final_valid_point[1]))
            
            # if np.any(valid_x1) and np.any(valid_x2):
            #     # Find the closest intersection point on the correct side
            #     if np.sign(y1[valid_x1][0]) == np.sign(y2[valid_x2][0]):
            #         if np.abs(y1[valid_x1][0]) < np.abs(y2[valid_x2][0]):
            #             intersections.append((x1[valid_x1][0], y1[valid_x1][0]))
            #         else:
            #             intersections.append((x2[valid_x2][0], y2[valid_x2][0]))
            #     elif np.sign(y1[valid_x1][0]) == 1:
            #         intersections.append((x1[valid_x1][0], y1[valid_x1][0]))
            #     else:
            #         intersections.append((x2[valid_x2][0], y2[valid_x2][0]))
            # elif np.any(valid_x1):
            #     # Only intersection point 1 is valid
            #     intersections.append((x1[valid_x1][0], y1[valid_x1][0]))
            # elif np.any(valid_x2):
            #     # Only intersection point 2 is valid
            #     intersections.append((x2[valid_x2][0], y2[valid_x2][0]))

    return intersections

def calculate_all_intersections_slope_optimized_ele_bk(obstacles, slopes, max_range=1.0):
    # Calculate a, b and r outside the loop
    a = obstacles[:, 0]
    b = obstacles[:, 1]
    r = obstacles[:, 2]

    def calculate_valid_points(x, y, slope):
        dist = np.sqrt(x**2 + y**2)
        valid_x = dist <= max_range
        if slope == 0:
            valid_x = valid_x & (x >= 0)
        elif which_side[i]:
            valid_x = valid_x & (y >= 0)
        else:
            valid_x = valid_x & (y < 0)
        return x[valid_x], y[valid_x], dist[valid_x]

    # Initialize an empty list to hold the intersection points
    intersections = []
    for i, slope in enumerate(slopes):
        coef_a = 1 + slope**2
        coef_b = -2 * a - 2 * b * slope
        coef_c = a**2 + b**2 - r**2

        # Calculate discriminant only where coef_a != 0
        if coef_a != 0:
            discriminant = coef_b**2 - 4 * coef_a * coef_c

            # Find valid intersection points
            valid_mask = discriminant >= 0

            if valid_mask.size > 0:
                # Calculate x and y coordinates of intersection points
                all_x1 = (-coef_b[valid_mask] + np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_x2 = (-coef_b[valid_mask] - np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_y1 = slope * all_x1
                all_y2 = slope * all_x2

                x1, y1, _ = calculate_valid_points(all_x1, all_y1, slope)
                x2, y2, _ = calculate_valid_points(all_x2, all_y2, slope)

                # Concatenate valid points from both intersections
                valid_points = np.concatenate((np.stack((x1, y1), axis=1), np.stack((x2, y2), axis=1)), axis=0)

                if valid_points.shape[0] > 0:
                    # Find the closest intersection point
                    closest_point = valid_points[np.argmin(np.sqrt(valid_points[:, 0]**2 + valid_points[:, 1]**2))]
                    intersections.append(tuple(closest_point))
    return intersections


def calculate_all_intersections_slope_optimized_ele(obstacles, slopes, max_range=1.0):
    # Calculate a, b and r outside the loop
    a = obstacles[:, 0]
    b = obstacles[:, 1]
    r = obstacles[:, 2]

    def calculate_valid_points(x, y, slope):
        dist = np.sqrt(x**2 + y**2)
        valid_x = dist <= max_range
        if slope == 0:
            valid_x = valid_x & (x >= 0)
        elif which_side[i]:
            valid_x = valid_x & (y >= 0)
        else:
            valid_x = valid_x & (y < 0)
        return x[valid_x], y[valid_x], dist[valid_x]

    # Initialize an empty list to hold the intersection points
    intersections = []
    for i, slope in enumerate(slopes):
        coef_a = 1 + slope**2
        coef_b = -2 * a - 2 * b * slope
        coef_c = a**2 + b**2 - r**2

        # Calculate discriminant only where coef_a != 0
        if coef_a != 0:
            discriminant = coef_b**2 - 4 * coef_a * coef_c

            # Find valid intersection points
            valid_mask = discriminant >= 0

            if valid_mask.size > 0:
                # Calculate x and y coordinates of intersection points
                all_x1 = (-coef_b[valid_mask] + np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_x2 = (-coef_b[valid_mask] - np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_y1 = slope * all_x1
                all_y2 = slope * all_x2

                x1, y1, _ = calculate_valid_points(all_x1, all_y1, slope)
                x2, y2, _ = calculate_valid_points(all_x2, all_y2, slope)
                t1 = np.sqrt(x1**2 + y1**2)
                t2 = np.sqrt(x2**2 + y2**2)
                # Concatenate valid points from both intersections
                valid_points = np.concatenate((np.stack((x1, y1, t1), axis=1), np.stack((x2, y2, t2), axis=1)), axis=0)
                if valid_points.shape[0] == 1:
                    intersections.append(tuple(valid_points[0, :2]))
                elif valid_points.shape[0] > 1:
                    # Find the closest intersection point
                    closest_point = valid_points[np.argmin(valid_points[:, 2])]
                    intersections.append(tuple(closest_point[:2]))
    return intersections

def new(obstacles, slopes, max_range=1.0):
    # Calculate a, b and r outside the loop
    a = obstacles[:, 0]
    b = obstacles[:, 1]
    r = obstacles[:, 2]

    # Compute distances of obstacles to lidar origin and sort by distance
    distances = np.sqrt(a**2 + b**2)
    sorted_indices = np.argsort(distances)
    a = a[sorted_indices]
    b = b[sorted_indices]
    r = r[sorted_indices]

    def calculate_valid_points(x, y, slope):
        dist = np.sqrt(x**2 + y**2)
        valid_x = dist <= max_range
        if slope == 0:
            valid_x = valid_x & (x >= 0)
        elif which_side[i]:
            valid_x = valid_x & (y >= 0)
        else:
            valid_x = valid_x & (y < 0)
        return x[valid_x], y[valid_x], dist[valid_x]

    # Initialize an empty list to hold the intersection points
    intersections = []
    for i, slope in enumerate(slopes):
        coef_a = 1 + slope**2
        coef_b = -2 * a - 2 * b * slope
        coef_c = a**2 + b**2 - r**2

        # Calculate discriminant only where coef_a != 0
        if coef_a != 0:
            discriminant = coef_b**2 - 4 * coef_a * coef_c

            # Find valid intersection points
            valid_mask = discriminant >= 0

            if np.any(valid_mask):
                # Calculate x and y coordinates of intersection points
                all_x1 = (-coef_b[valid_mask] + np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_x2 = (-coef_b[valid_mask] - np.sqrt(discriminant[valid_mask])) / (2 * coef_a)
                all_y1 = slope * all_x1
                all_y2 = slope * all_x2

                x1, y1, _ = calculate_valid_points(all_x1, all_y1, slope)
                x2, y2, _ = calculate_valid_points(all_x2, all_y2, slope)

                # Concatenate valid points from both intersections
                valid_points = np.concatenate((np.stack((x1, y1), axis=1), np.stack((x2, y2), axis=1)), axis=0)

                if valid_points.shape[0] > 0:
                    # Find the closest intersection point
                    closest_point = valid_points[np.argmin(np.sqrt(valid_points[:, 0]**2 + valid_points[:, 1]**2))]
                    intersections.append(tuple(closest_point))
    return intersections


def calculate_all_intersections_vectorized_optimized1(obstacles, sensor_angles, max_range=1.0):
    """
    The problem of this version is it it cannot handle two circles overlapped
    """
    x1, y1 = env.atr.state[0], env.atr.state[1]  # Assuming sensor is at the origin of the car frame

    # Compute distances of obstacles to lidar origin and sort by distance
    distances = np.sqrt(obstacles[:, 0]**2 + obstacles[:, 1]**2)
    sorted_indices = np.argsort(distances)
    obstacles = obstacles[sorted_indices]

    # Pre-compute A for all angles
    A_values = np.cos(sensor_angles)**2 + np.sin(sensor_angles)**2

    intersections = []
    measurements = []
    for i, theta in enumerate(sensor_angles):
        A = A_values[i]  # Fetch pre-computed A value
        intersection_found = False

        # Calculate B, C, and discriminant for all obstacles at once
        B = 2 * (np.cos(theta) * (x1 - obstacles[:, 0]) + np.sin(theta) * (y1 - obstacles[:, 1]))
        C = (x1 - obstacles[:, 0])**2 + (y1 - obstacles[:, 1])**2 - obstacles[:, 2]**2
        discriminant = B**2 - 4*A*C

        for j in range(obstacles.shape[0]):
            if discriminant[j] >= 0:  # Intersection exists
                # Calculate both intersections
                t1 = (-B[j] + np.sqrt(discriminant[j])) / (2*A)
                x2 = x1 + t1 * np.cos(theta)
                y2 = y1 + t1 * np.sin(theta)
                t2 = (-B[j] - np.sqrt(discriminant[j])) / (2*A)
                x3 = x1 + t2 * np.cos(theta)
                y3 = y1 + t2 * np.sin(theta)
                intersections_temp = [(x2, y2, t1), (x3, y3, t2)]
                
                # Sort by t value so that we first check the intersection closer to the sensor
                intersections_temp.sort(key=lambda x: x[2])

                for (x, y, t) in intersections_temp:
                    # Check if the intersection is in the direction of the beam and within max range
                    if theta >= -np.pi/2 and theta <= np.pi/2:  # Forward direction is positive x
                        if x >= x1 and t <= max_range:
                            intersections.append((x, y))
                            measurements.append((theta, t))
                            intersection_found = True
                            break
                    else:  # Forward direction is negative x
                        if x <= x1 and t <= max_range:
                            intersections.append((x, y))
                            measurements.append((theta, t))
                            intersection_found = True
                            break

            if intersection_found:
                break  # Stop searching for this beam as soon as we find an intersection
        if not intersection_found:
            measurements.append((theta, 1.0))        
    return intersections, measurements    
