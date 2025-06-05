import numpy as np
from tap.create_clothoids.clothoids.clothoid_spline_functions import *


def calculate_a(x0, y0, c0, x_t0, y_t0, s0, x_n0, y_n0, omega0, k0, n0, x1, c1, x_t1, s1, x_n1, omega1):
    # x = A00 + a * A01
    A00 = x0
    A01 = c0 * x_t0 + s0 * x_n0
    # y = B00 + a * B01
    B00 = y0
    B01 = c0 * y_t0 + s0 * y_n0

    # x = A10 + a * A11
    A10 = x1
    if omega0 == 0:
        A11 = math.inf
    else:
        A11 = math.sqrt(omega1 / omega0) * (c1 * x_t1 + s1 * x_n1)

    ## HACK: if x0 == x1, the third (i.e., else) formula has issues so an approximation is done by moving x1 for a small delta
    if abs(A00 - A10) < 0.025:
        A10 = A00 + 0.025
    ## HACK END

    if k0 == math.inf:
        # x = 2 * n0 - x
        # x = 2 * n0 - A00 - a * A01
        a = (2 * n0 - A00 - A10) / (A11 + A01)
    elif k0 == 0:
        # x = x
        # x = A00 + a * A01
        a = (A00 - A10) / (A11 - A01)
    else:
        # x = (2 * k * y - 2 * k * n - k**2 * x + x) / (k**2 + 1)
        # x = (2 * k0 * B00 + 2 * k0 * a * B01 - 2 * k0 * n0 + A00 + a * A01 - A00 * k0**2 - A01 * k0**2 * a) / (k0**2 + 1)
        a = ((2 * k0 * B00 - 2 * k0 * n0 + A00 - A00 * k0 ** 2 - A10 * k0 ** 2 - A10)
             / (-2 * k0 * B01 - A01 + A01 * k0 ** 2 + A11 * k0 ** 2 + A11))
    return a


def get_distance_between_clothoid_points(omega, point0, tangent0, normal0, omega0, p0_p1_k, p0_p1_n, point2, tangent2,
                                         normal2, p2_p1_k, p2_p1_n, p1_p0__p1_p2_cross_product, approximate_fresnel):
    omega2 = math.pi - omega - omega0
    if omega0 < 0 or omega2 < 0:
        return None
    t0 = math.sqrt(2 * omega0 / math.pi)  # math.sqrt(1 - omega / math.pi)
    t2 = math.sqrt(2 * omega2 / math.pi)
    c0, s0 = calculate_fresnel_integrals(t0, approximate_fresnel)
    c2, s2 = calculate_fresnel_integrals(t2, approximate_fresnel)
    if p1_p0__p1_p2_cross_product > 0:
        p1_p0_a = calculate_a(point0[0], point0[1], c0, tangent0[0], tangent0[1], s0, normal0[0], normal0[1], omega0,
                              p0_p1_k, p0_p1_n, point2[0], c2, tangent2[0], s2, normal2[0], omega2)
        if omega0 == 0:
            p1_p2_a = math.inf
        else:
            p1_p2_a = p1_p0_a * math.sqrt(omega2 / omega0)
    else:
        p1_p2_a = calculate_a(point2[0], point2[1], c2, tangent2[0], tangent2[1], s2, normal2[0], normal2[1], omega2,
                              p2_p1_k, p2_p1_n, point0[0], c0, tangent0[0], s0, normal0[0], omega0)
        p1_p0_a = p1_p2_a * math.sqrt(omega0 / omega2)
    p0 = [point0[0] + p1_p0_a * c0 * tangent0[0] + p1_p0_a * s0 * normal0[0],
          point0[1] + p1_p0_a * c0 * tangent0[1] + p1_p0_a * s0 * normal0[1]]
    if p1_p0__p1_p2_cross_product > 0:
        p0 = reflect_point(p0, p0_p1_k, p0_p1_n)

    if p1_p2_a == math.inf:
        distance = math.inf
    else:
        p2 = [point2[0] + p1_p2_a * c2 * tangent2[0] + p1_p2_a * s2 * normal2[0],
              point2[1] + p1_p2_a * c2 * tangent2[1] + p1_p2_a * s2 * normal2[1]]
        if p1_p0__p1_p2_cross_product < 0:
            p2 = reflect_point(p2, p2_p1_k, p2_p1_n)
        distance = vector_length(subtract_vector(p0, p2))
    return [distance, t0, p1_p0_a, t2, p1_p2_a]


# regarding feasibility see: D.J. Walton, D.S. Meek, "A controlled clothoid spline",
# Computers & Graphics 29 (2005) 353-363
def make_points_feasible(original_points, tau, approximate_fresnel):
    points = []
    for op in original_points:
        points.append([op[0], op[1]])
    original_point0 = None
    original_point2 = None

    point1 = points[1]
    p1_p0_vector = subtract_vector(original_points[1], original_points[0])
    p0_p1_vector = subtract_vector(original_points[0], original_points[1])
    p2_p1_vector = subtract_vector(original_points[2], original_points[1])
    p1_p2_vector = subtract_vector(original_points[1], original_points[2])
    p1_p0_length = vector_length(p1_p0_vector)
    p0_p1_length = vector_length(p0_p1_vector)
    p2_p1_length = vector_length(p2_p1_vector)
    p1_p2_length = vector_length(p2_p1_vector)
    tangent0 = [p0_p1_vector[0] / p0_p1_length, p0_p1_vector[1] / p0_p1_length]
    tangent2 = [p2_p1_vector[0] / p2_p1_length, p2_p1_vector[1] / p2_p1_length]

    p1_p0__p1_p2_dot_product = dot_product(p1_p0_vector, p1_p2_vector)
    omega = math.acos(p1_p0__p1_p2_dot_product / (p1_p0_length * p1_p2_length))
    alpha = math.pi - omega
    t_limit = math.sqrt(2 * alpha / math.pi)
    c_limit, s_limit = calculate_fresnel_integrals(t_limit, approximate_fresnel)
    limit = c_limit / s_limit

    if p1_p0_length > p2_p1_length:
        condition = (p0_p1_length / p2_p1_length + math.cos(alpha)) / math.sin(alpha)
        if condition >= limit:
            original_point0 = [original_points[0][0], original_points[0][1]]
            p1_p0_limit = p1_p2_length * (c_limit / s_limit * math.sin(alpha) - math.cos(alpha))
            point0_multiply = (1 - tau) * p1_p2_length + tau * p1_p0_limit
            points[0] = [point1[0] - point0_multiply * tangent0[0], point1[1] - point0_multiply * tangent0[1]]
    elif p2_p1_length > p1_p0_length:
        condition = (p2_p1_length / p0_p1_length + math.cos(alpha)) / math.sin(alpha)
        if condition >= limit:
            original_point2 = [original_points[2][0], original_points[2][1]]
            p1_p2_limit = p1_p0_length * (c_limit / s_limit * math.sin(alpha) - math.cos(alpha))
            point2_multiply = (1 - tau) * p1_p0_length + tau * p1_p2_limit
            points[2] = [point1[0] - point2_multiply * tangent2[0], point1[1] - point2_multiply * tangent2[1]]
    return [points, original_point0, original_point2]


def find_minimal_distance_parameters(omega, point0, tangent0, normal0, p0_p1_k, p0_p1_n, point2, tangent2, normal2,
            p2_p1_k, p2_p1_n, p1_p0__p1_p2_cross_product, start_omega0, end_omega0, step_omega0, approximate_fresnel):

    min_distance, t0, p1_p0_a, t2, p1_p2_a, omega_min_distance = min_dist(omega,
                    point0, tangent0, normal0, p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n,
                    p1_p0__p1_p2_cross_product, start_omega0, end_omega0, step_omega0, approximate_fresnel)

    step_factor = 10
    while min_distance > 0.00001 and step_omega0 > 1e-15:
        new_step0 = step_omega0 / step_factor
        min_distance_left, t0_left, p1_p0_a_left, t2_left, p1_p2_a_left, omega_min_distance_left = min_dist(omega,
                    point0, tangent0, normal0, p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n,
                    p1_p0__p1_p2_cross_product, omega_min_distance, max(omega_min_distance - step_omega0, start_omega0),
                    -new_step0, approximate_fresnel)
        min_distance_right, t0_right, p1_p0_a_right, t2_right, p1_p2_a_right, omega_min_distance_right = min_dist(omega,
                    point0, tangent0, normal0, p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n,
                    p1_p0__p1_p2_cross_product, omega_min_distance, min(omega_min_distance + step_omega0, end_omega0),
                    new_step0, approximate_fresnel)

        if min_distance_left < min_distance or min_distance_right < min_distance:
            if min_distance_left<min_distance_right:
                (min_distance, t0, p1_p0_a, t2, p1_p2_a, omega_min_distance) = (min_distance_left, t0_left,
                                            p1_p0_a_left, t2_left, p1_p2_a_left, omega_min_distance_left)
            else:
                (min_distance, t0, p1_p0_a, t2, p1_p2_a, omega_min_distance) = (min_distance_right, t0_right,
                                            p1_p0_a_right, t2_right, p1_p2_a_right, omega_min_distance_right)
        step_omega0 = step_omega0 / step_factor
    return min_distance, t0, p1_p0_a, t2, p1_p2_a


def min_dist(omega, point0, tangent0, normal0, p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n,
             p1_p0__p1_p2_cross_product, start_omega0, end_omega0, step_omega0, approximate_fresnel):
    min_distance = math.inf
    t0 = -1
    p1_p0_a = -1
    t2 = -1
    p1_p2_a = -1
    omega_min_distance = start_omega0
    for omega0 in np.arange(start_omega0, end_omega0, step_omega0):
        # omega0 = 1/2 * (math.pi - omega) # ORIGINAL SYMETRIC VERSION

        result = get_distance_between_clothoid_points(omega, point0,
                tangent0, normal0, omega0, p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n,
                p1_p0__p1_p2_cross_product, approximate_fresnel)

        if result is not None:
            [distance, t0_temp, p1_p0_a_temp, t2_temp, p1_p2_a_temp] = result
            if min_distance > distance:
                min_distance = distance
                t0 = t0_temp
                p1_p0_a = p1_p0_a_temp
                t2 = t2_temp
                p1_p2_a = p1_p2_a_temp
                omega_min_distance = omega0

    return min_distance, t0, p1_p0_a, t2, p1_p2_a, omega_min_distance
