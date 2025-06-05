import math
import numpy as np

from tap.create_clothoids.clothoids.asymmetric_clothoid_spline \
    import create_asym_controlled_clothoid_curve
from tap.create_clothoids.clothoids.clothoid_spline_functions \
    import calculate_heading_from_xy, calculate_intersection_between_point_angle_lines


def create_clothoid_spline(points, tau, clothoid_point_step, clothoid_point_search_step,
                           approximate_fresnel, factors):
    if factors is None:
        factors = [0.5 for _ in range(len(points) - 3)]

    points = remove_repeating_points(points)
    route_center = []
    alpha = None
    norm_dist = 0
    if len(points) < 2:
        print("ERROR: number of points too low")
        route_center = points
    elif len(points) == 2:
        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
        heading = calculate_heading_from_xy(dx, dy) % (2 * math.pi)
        route_center = points
        #route_center.append([dx, dy])
        alpha = heading
    elif len(points) == 3:
        points_temp = [points[0], points[1], points[2]]
        [c_points, end_heading, norm_dist] = create_asym_controlled_clothoid_curve(
            points_temp, tau, clothoid_point_step, clothoid_point_search_step, approximate_fresnel)
        for p_tmp in c_points:
            route_center.append([p_tmp[0], p_tmp[1]])
        alpha = end_heading
    else:
        points_new = []
        points_new.append(points[0])
        for i in range(1, len(points) - 2):
            p1 = points[i]
            points_new.append(p1)
            p2 = points[i + 1]
            points_new.append([p1[0] + factors[i-1] * (p2[0] - p1[0]),
                               p1[1] + factors[i-1] * (p2[1] - p1[1])])
        for i in range(len(points) - 2, len(points)):
            points_new.append(points[i])

        route_center, alpha, norm_dist = create_clothoid_spline_from_final_points(
            points_new, tau, clothoid_point_step, clothoid_point_search_step, approximate_fresnel,
            factors[0])
    return [np.array(route_center), alpha, norm_dist]


def create_clothoid_spline_from_mandatory_points_with_angles(points_start, tau, clothoid_point_step,
                                                             clothoid_point_search_step, approximate_fresnel):
    points = remove_repeating_points(points_start)
    norm_dist = 0
    if len(points) < 2:
        route_center = []
    elif len(points) != len(points_start):
        route_center = []
    else:
        points_new = [points[0][:-1]]
        intersections = []
        valid = True
        for i in range(1, len(points)):
            intersec = calculate_intersection_between_point_angle_lines(points[i - 1][0], points[i - 1][1],
                                                            points[i - 1][2], points[i][0], points[i][1], points[i][2])
            if intersec == None:
                valid = False
            else:
                intersecx, intersecy = intersec
                points_new = points_new + [[intersecx, intersecy]]
                points_new = points_new + [points[i][:-1]]
                intersections = intersections + [[intersecx, intersecy]]

        if len(points_new) != len(remove_repeating_points(points_new)):
            valid = False
        if valid:
            route_center, alpha, norm_dist = create_clothoid_spline_from_final_points(
                points_new, tau, clothoid_point_step, clothoid_point_search_step,
                approximate_fresnel)
            route_center = remove_repeating_points(route_center)
        else:
            route_center = []
    return np.array(route_center), norm_dist


def create_clothoid_spline_from_final_points(points, tau, clothoid_point_step, clothoid_point_search_step,
                                             approximate_fresnel, factor=0.5):
    route_center = []
    alpha = None
    norm_dist = 0
    for i in range(0, len(points) - 1, 2):
        points_temp = [points[i], points[i + 1], points[i + 2]]
        [c_points, end_heading, new_norm_dist] = create_asym_controlled_clothoid_curve(
            points_temp, tau, clothoid_point_step, clothoid_point_search_step, approximate_fresnel,
            factor)
        if new_norm_dist > norm_dist:
            norm_dist = new_norm_dist
        for p_tmp in c_points:
            route_center.append((p_tmp[0], p_tmp[1]))
        alpha = end_heading
    return route_center, alpha, norm_dist


def remove_repeating_points(points_original):
    points = points_original.copy()
    no_points = len(points)
    i = 0
    while i < (no_points-1):
        if points[i][0] == points[i+1][0] and points[i][1] == points[i+1][1]:
            del points[i+1]
            i = i - 1
            no_points = no_points - 1
        i = i + 1

    '''i = 1
    while i < no_points:
        j = 0
        while j < i:
            if points[i][0] == points[j][0] and points[i][1] == points[j][1]:
                del points[i]
                i = i - 1
                j = j - 1
                no_points = no_points - 1
            j = j + 1
        i = i + 1
    '''
    return points


