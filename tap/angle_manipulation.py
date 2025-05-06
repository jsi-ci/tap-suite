import numpy as np


def mod2pi(angle):
    """ Returns the angle in the range [0, 2pi]."""
    return np.mod(angle, 2 * np.pi)


def mod1pi(angle):
    """ Returns the angle in the range [0, pi]."""
    return np.mod(angle, np.pi)


def feasible_angles(angle, tolerance, input_in_radians=False):
    """ Returns the feasible angles for the given angle and tolerance.
    :return: tuple of two angles (in radians)
    """
    if not input_in_radians:
        angle = mod2pi(np.deg2rad(angle))
        tolerance = np.deg2rad(tolerance)

    if tolerance > np.pi:
        return 0, 2 * np.pi
    return mod2pi(angle - tolerance), mod2pi(angle + tolerance)


def get_angle_from_range(min_angle, max_angle, k):
    """ Returns the angle for the given k in the range [min_angle, max_angle]. """
    return mod2pi(mod2pi(max_angle - min_angle) * k + min_angle)


def spherical_to_cartesian(r, phi, theta):
    """ Returns the cartesian coordinates for the given spherical coordinates."""
    x = r * np.cos(theta) * np.cos(phi)
    y = r * np.cos(theta) * np.sin(phi)
    z = r * np.sin(theta)
    return x, y, z


def cartesian_to_spherical(x, y, z):
    """ Returns the spherical coordinates for the given cartesian coordinates."""
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = np.arcsin(z / r)
    phi = mod2pi(np.arctan2(y, x))
    return r, phi, theta


def angle_between_points(point1, point2, horizontal=True):
    """ Returns the angle between the two points in the range [0, 2pi]. """
    if horizontal:
        angle = np.arctan2(point2[1] - point1[1], point2[0] - point1[0])
    else:
        angle = np.arctan2(point2[2] - point1[2], point2[3] - point1[3])
    return mod2pi(angle + 2 * np.pi)


def check_angle_feasibility(angle, angle_range):
    """ Returns True if the angle is in the feasible angle range, False otherwise. """
    start, end = angle_range
    if start <= end:
        return start <= angle <= end
    else:
        # Handle the case where the interval wraps around 0 (circular interval)
        return start <= angle or angle <= end


def get_feasible_angle_range(current, previous_angle, previous_point, first_to_last=True,
                             vertical=False, epsilon=1e-5):
    """Returns the feasible angle range for the current point in format (x, y, k, phi_min, phi_max),
    the previous angle and point coordinates and the construction order. The angle range is
    oriented in the direction of "going into the current point". Only one of two possible ranges is
    considered if this is the vertical alignment. If epsilon > 0, the 'allowed' range for the next
    angle is diminished by the absolute epsilon value on each side."""
    # Calculate angle phi_ca (angle between the current and the previous point)
    phi_ca = angle_between_points([current[0], current[1]], [previous_point[0], previous_point[1]])
    if np.isclose(mod1pi(previous_angle - phi_ca), 0):
        # Special case when the previous point points_h directly at the current point
        phi_range = [mod2pi(previous_angle + np.pi), mod2pi(previous_angle + np.pi)]
        return phi_range
    # Calculate psi (angle range that is acceptable wrt the previous point) (%2pi)
    psi_min, psi_max = calculate_acceptable_angle_range(previous_angle, phi_ca, epsilon=epsilon)
    if np.isclose(current[3], 0) and np.isclose(current[4], 2 * np.pi):
        # Special case when the current point has no angle restrictions
        phi_range = [psi_min, psi_max]
    elif first_to_last and vertical:
        # Special case when the vertical path goes from the first to the last point
        phi_min, phi_max = mod2pi(np.pi + current[3]), mod2pi(np.pi + current[4])
        phi_range = intersection(psi_min, psi_max, phi_min, phi_max)
        if phi_range is None:
            phi_range = [psi_min, psi_max]
    elif not first_to_last and vertical:
        # Special case when the vertical path goes from the last to the first point
        phi_min, phi_max = mod2pi(current[3]), mod2pi(current[4])
        phi_range = intersection(psi_min, psi_max, phi_min, phi_max)
        if phi_range is None:
            phi_range = [psi_min, psi_max]
    else:
        # Get the acceptable ranges for the current point (%pi)
        phi_min1, phi_max1 = mod2pi(current[3]), mod2pi(current[4])
        phi_min2, phi_max2 = mod2pi(np.pi + current[3]), mod2pi(np.pi + current[4])
        # Find the intersection for both ranges variants
        phi_range1 = intersection(psi_min, psi_max, phi_min1, phi_max1)
        phi_range2 = intersection(psi_min, psi_max, phi_min2, phi_max2)
        # If the intersection are both empty, only use the angle range from the first point
        if phi_range1 is None and phi_range2 is None:
            phi_range = [psi_min, psi_max]
        elif phi_range1 is None:
            phi_range = phi_range2
        elif phi_range2 is None:
            phi_range = phi_range1
        else:
            # Both intersections are non-empty, take the one that is closest to phi_ca
            if is_closest_to(phi_ca,
                             get_angle_from_range(phi_range1[0], phi_range1[1], 0.5),
                             get_angle_from_range(phi_range2[0], phi_range2[1], 0.5)):
                phi_range = phi_range1
            else:
                phi_range = phi_range2
    return phi_range


def intersection(alpha_min, alpha_max, beta_min, beta_max):
    """Computes the intersection between two angles. It returns a single intersection in case of two
    obtuse angles and None in case the intersection is empty.
    """
    alpha_min = [mod2pi(alpha_min)]
    alpha_max = [mod2pi(alpha_max)]
    beta_min = [mod2pi(beta_min)]
    beta_max = [mod2pi(beta_max)]

    # Split to two angles (covers cases (350°, 10°))
    if alpha_max <= alpha_min:
        alpha_min.insert(1, 0)
        alpha_max.insert(0, 2 * np.pi)
    if beta_max <= beta_min:
        beta_min.insert(1, 0)
        beta_max.insert(0, 2 * np.pi)

    # Get intersections
    result = []
    for a_min, a_max in zip(alpha_min, alpha_max):
        for b_min, b_max in zip(beta_min, beta_max):
            gamma_min = max(a_min, b_min)
            gamma_max = min(a_max, b_max)
            if gamma_min <= gamma_max:
                # The intersection is non-empty
                if len(result) > 0 and np.isclose(mod2pi(result[-1][1]), mod2pi(gamma_min)):
                    # Extend the previous intersection
                    result[-1][1] = mod2pi(gamma_max)
                else:
                    # Make a new intersection
                    result.append([mod2pi(gamma_min), mod2pi(gamma_max)])

    if len(result) == 0:
        return None
    else:
        return result[0]


def calculate_acceptable_angle_range(phi_a, phi_ca, epsilon=0.0):
    """ Calculates the acceptable angle range for the next point given the current angle phi_a """
    assert epsilon >= 0, "Epsilon must be non-negative"
    min_angle = phi_ca if np.sin(phi_a - phi_ca) >= 0 else phi_a
    max_angle = phi_a if np.sin(phi_a - phi_ca) >= 0 else phi_ca
    return mod2pi(min_angle + epsilon + 2 * np.pi), mod2pi(max_angle - epsilon + 2 * np.pi)


def is_closest_to(angle, angle_closest, angle_furthest):
    """ Returns true if the angle is closest to the angle_closest, false otherwise """
    closest_difference = abs(mod2pi(angle - angle_closest + np.pi) - np.pi)
    furthest_difference = abs(mod2pi(angle - angle_furthest + np.pi) - np.pi)
    return closest_difference <= furthest_difference
