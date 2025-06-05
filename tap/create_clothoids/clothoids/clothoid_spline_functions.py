import math
from scipy.special import fresnel

# a controlled clothoid spline
# D.J. Walton, D.S. Meek, "A controlled clothoid spline", Computers & Graphics 29 (2005) 353--63
# http://www.lara.prd.fr/_media/users/franciscogarcia/a_controlled_clothoid_spline.pdf
# symmetrical - may need straight lines appended ad beginning + end
# asymmetrical - not need straight lines


# P0: initial point
# T0: initial tangent vector
# N0: initial normal vector
# a: scaling factor
# phi: angle, where 0 < phi < alpha < pi and alpha is angle from T0 to unit target vector at the joint of the pair
def clothoid_point(phi, P0, T0, N0, a, approximate_fresnel):
    c, s = calculate_fresnel_integrals(phi, approximate_fresnel)
    return [P0[0] + a * c * T0[0] + a * s * N0[0], P0[1] + a * c * T0[1] + a * s * N0[1]]


def calculate_fresnel_integrals(phi, approximate_fresnel):
    if approximate_fresnel:
        c = C(phi)
        s = S(phi)
    else:
        s, c = fresnel(phi)
    return c, s


#
# CLOTHOID APPROXIMATION: approximation of C and S Fresnel integrals
# based on   http://www.dgp.toronto.edu/~mccrae/projects/clothoid/sbim2008mccrae.pdf
#
def C(phi):
    return 1 / 2 - R(phi) * math.sin(1 / 2 * math.pi * (A(phi) - phi ** 2))


def S(phi):
    return 1 / 2 - R(phi) * math.cos(1 / 2 * math.pi * (A(phi) - phi ** 2))


def R(phi):
    return (0.506 * phi + 1) / (1.79 * phi ** 2 + 2.054 * phi + math.sqrt(2))


def A(phi):
    return 1 / (0.803 * phi ** 3 + 1.886 * phi ** 2 + 2.524 * phi + 2)


#
# Vector and point functions
#
def vector_length(A):
    sum_a = 0
    for a in A:
        sum_a += a * a
    return math.sqrt(sum_a)


def dot_product(A, B):
    return A[0] * B[0] + A[1] * B[1]


def cross_product(A, B):
    return A[0] * B[1] - A[1] * B[0]


def curvature(phi, a):
    return math.sqrt(2 * math.pi * phi) / a


def middle_point(A, B):
    return [(A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0]


def subtract_vector(A, B):
    return [B[0] - A[0], B[1] - A[1]]


def add_multiplied_vector(A, B, k):
    return [A[0] + B[0] * k, A[1] + B[1] * k]


def reflect_point(point, k, n):
    if k == math.inf:
        x_cross = n
        y_cross = point[1]
    elif k == 0:
        x_cross = point[0]
        y_cross = n
    else:
        k_orthogonal = -1 / k
        n_orthogonal = point[1] - k_orthogonal * point[0]
        x_cross = (n_orthogonal - n) / (k - k_orthogonal)
        y_cross = k * x_cross + n
    dx = x_cross - point[0]
    dy = y_cross - point[1]
    return [x_cross + dx, y_cross + dy]


def rotate_point(point, center_point, angle):
    s = math.sin(angle)
    c = math.cos(angle)
    x_temp = point[0] - center_point[0]
    y_temp = point[1] - center_point[1]
    x_new = x_temp * c - y_temp * s
    y_new = x_temp * s + y_temp * c
    x_new = x_new + center_point[0]
    y_new = y_new + center_point[1]
    return [x_new, y_new]


def calculate_heading_from_xy(dx, dy):
    if dx == 0:
        if dy >= 0:
            return math.pi/2
        else:
            return math.pi*3/2
    elif dx > 0:
        return math.atan(dy/dx)
    else:
        return math.pi + math.atan(dy/dx)


def calculate_intersection_between_point_angle_lines(x0, y0, a0, x1, y1, a1):
    if abs_angle_diff_direction_invariant(a0, a1) < 1e-6:
        # lines are parallel
        angle_of_line_between = math.atan2(y1-y0, x1-x0)
        if abs_angle_diff_direction_invariant(angle_of_line_between, a0) < 1e-6:
            # lines are the same and go through both points -> return the middle point
            return [(x0+x1)/2.0, (y0+y1)/2.0]
        else:
            # lines are not the same and do not intersect
            return None
    if ((a0 % math.pi) + math.pi) % math.pi == math.pi/2.0:
        # vertical line at x = x0
        return [x0, math.tan(a1) * (x0 - x1) + y1]
    elif ((a1 % math.pi) + math.pi) % math.pi == math.pi/2.0:
        # vertical line at x = x0
        return [x1, math.tan(a0) * (x1 - x0) + y0]
    m0 = math.tan(a0)  # Line 0: y = m0 (x - x0) + y0
    m1 = math.tan(a1)  # Line 1: y = m1 (x - x1) + y1
    x = ((m0 * x0 - m1 * x1) - (y0 - y1)) / (m0 - m1)
    return [x, m0 * (x - x0) + y0]


def abs_angle_diff_direction_invariant(a, b):
    return min(abs(angle_diff(a, b)), abs(angle_diff(a-math.pi, b)))


def angle_diff(a, b):
    c = (b - a) % (2*math.pi)
    if c > math.pi:
        c -= 2*math.pi
    return c
