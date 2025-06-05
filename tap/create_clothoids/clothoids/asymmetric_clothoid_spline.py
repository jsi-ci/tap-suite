import numpy as np

from tap.create_clothoids.clothoids.clothoid_spline_functions import *
from tap.create_clothoids.clothoids.asymmetric_clothoid_spline_functions \
    import make_points_feasible, find_minimal_distance_parameters
from tap.create_clothoids.clothoids.clothoid_spline_functions import calculate_heading_from_xy


def create_asym_controlled_clothoid_curve(original_points, tau, clothoid_point_step,
                                          clothoid_point_search_step, approximate_fresnel,
                                          factor=0.5):
    orig_p1_p0_vector = subtract_vector(original_points[1], original_points[0])
    orig_p1_p2_vector = subtract_vector(original_points[1], original_points[2])
    orig_p2_p1_vector = subtract_vector(original_points[2], original_points[1])
    orig_p1_p0_length = vector_length(orig_p1_p0_vector)
    orig_p1_p2_length = vector_length(orig_p1_p2_vector)
    orig_p2_p1_length = vector_length(orig_p2_p1_vector)

    math_errors = False
    try:
        orig_p1_p0__p1_p2_dot_product = dot_product(orig_p1_p0_vector, orig_p1_p2_vector)
        orig_omega = math.acos(orig_p1_p0__p1_p2_dot_product / (orig_p1_p0_length * orig_p1_p2_length))
    except Exception:
        math_errors = True
        orig_omega = -1

    if (math_errors or (math.pi * 0.999999) <= orig_omega <= (math.pi * 1.000001)
            or (math.pi * -0.000001) <= orig_omega <= (math.pi * 0.000001)):
        tangent2 = [orig_p2_p1_vector[0] / orig_p2_p1_length, orig_p2_p1_vector[1] / orig_p2_p1_length]
        final_heading = (calculate_heading_from_xy(tangent2[0], tangent2[1]) + math.pi) % (2 * math.pi)
        #print("Warning: omega near 2 * PI or 0. Linear approximation used instead of curve")
        return [[original_points[0], original_points[2]], final_heading, 0]
    else:
        try:
            [points, original_point0, original_point2] = make_points_feasible(original_points, tau, approximate_fresnel)

            clothoid_line_points = []
            p1_p0_vector = subtract_vector(points[1], points[0])
            p0_p1_vector = subtract_vector(points[0], points[1])
            p2_p1_vector = subtract_vector(points[2], points[1])
            p1_p2_vector = subtract_vector(points[1], points[2])
            p1_p0_length = vector_length(p1_p0_vector)
            p0_p1_length = vector_length(p0_p1_vector)
            p2_p1_length = vector_length(p2_p1_vector)
            p1_p2_length = vector_length(p1_p2_vector)

            p1_p0__p1_p2_dot_product = dot_product(p1_p0_vector, p1_p2_vector)
            omega = math.acos(p1_p0__p1_p2_dot_product / (p1_p0_length * p1_p2_length))
            p1_p0__p1_p2_cross_product = cross_product(p1_p0_vector, p1_p2_vector)

            point0 = points[0]
            point2 = points[2]
            tangent0 = [p0_p1_vector[0] / p0_p1_length, p0_p1_vector[1] / p0_p1_length]
            tangent2 = [p2_p1_vector[0] / p2_p1_length, p2_p1_vector[1] / p2_p1_length]
            normal0 = [-tangent0[1], tangent0[0]]
            normal2 = [-tangent2[1], tangent2[0]]

            if abs(points[1][0] - points[0][0]) < 0.01:
                p0_p1_k = math.inf
                p0_p1_n = points[0][0]
            else:
                p0_p1_k = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
                p0_p1_n = points[0][1] - p0_p1_k * points[0][0]
            if abs(points[1][0] - points[2][0]) < 0.01:
                p2_p1_k = math.inf
                p2_p1_n = points[2][0]
            else:
                p2_p1_k = (points[1][1] - points[2][1]) / (points[1][0] - points[2][0])
                p2_p1_n = points[2][1] - p2_p1_k * points[2][0]

            start_omega0 = 0.0
            end_omega0 = math.pi - omega
            step_omega0 = clothoid_point_search_step

            min_distance, t0, p1_p0_a, t2, p1_p2_a = find_minimal_distance_parameters(omega, point0, tangent0, normal0,
                    p0_p1_k, p0_p1_n, point2, tangent2, normal2, p2_p1_k, p2_p1_n, p1_p0__p1_p2_cross_product, start_omega0,
                    end_omega0, step_omega0, approximate_fresnel)

            if min_distance > 1:
                tangent2 = [orig_p2_p1_vector[0] / orig_p2_p1_length, orig_p2_p1_vector[1] / orig_p2_p1_length]
                final_heading = (calculate_heading_from_xy(tangent2[0], tangent2[1]) + math.pi) % (2 * math.pi)
                return [[original_points[0], original_points[2]], final_heading, 0]

            if original_point0 is not None:
                clothoid_line_points.append(original_point0)

            for phi in np.arange(0.0, t0, clothoid_point_step):
                cp = clothoid_point(phi, point0, tangent0, normal0, p1_p0_a, approximate_fresnel)
                if p1_p0__p1_p2_cross_product > 0:
                    cp = reflect_point(cp, p0_p1_k, p0_p1_n)
                clothoid_line_points.append(cp)
            cp_m = clothoid_point(t0, point0, tangent0, normal0, p1_p0_a, approximate_fresnel)
            if p1_p0__p1_p2_cross_product > 0:
                cp_m = reflect_point(cp_m, p0_p1_k, p0_p1_n)
            #clothoid_line_points.append(cp_m)

            divide_clothoid = len(clothoid_line_points)
            first_cp = True

            for phi in np.arange(t2, 0, -clothoid_point_step):
                cp = clothoid_point(phi, point2, tangent2, normal2, p1_p2_a, approximate_fresnel)
                if p1_p0__p1_p2_cross_product < 0:
                    cp = reflect_point(cp, p2_p1_k, p2_p1_n)
                if not first_cp:
                    clothoid_line_points.append(cp)
                else:
                    cp = middle_point(cp, cp_m)
                    clothoid_line_points.append(cp)
                    first_cp = False
            cp = clothoid_point(0, point2, tangent2, normal2, p1_p2_a, approximate_fresnel)
            if p1_p0__p1_p2_cross_product < 0:
                cp = reflect_point(cp, p2_p1_k, p2_p1_n)
            clothoid_line_points.append(cp)
            if original_point2 is not None:
                clothoid_line_points.append(original_point2)

            def calc_dist(p1, p2):
                return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5

            dist = sum([calc_dist(clothoid_line_points[i], clothoid_line_points[i+1])
                        for i in range(len(clothoid_line_points) - 1)])
            avg_dist = dist / len(clothoid_line_points)
            dist_between_parts = calc_dist(clothoid_line_points[divide_clothoid],
                                           clothoid_line_points[divide_clothoid+1])
            norm_dist = dist_between_parts / avg_dist


            # initial_heading = (calculate_heading_from_xy(tangent0[0], tangent0[1])) % (2*math.pi)
            final_heading = (calculate_heading_from_xy(tangent2[0], tangent2[1]) + math.pi) % (2 * math.pi)
            return [clothoid_line_points, final_heading, norm_dist]
        except Exception:
            try:
                tangent2 = [orig_p2_p1_vector[0] / orig_p2_p1_length, orig_p2_p1_vector[1] / orig_p2_p1_length]
                final_heading = (calculate_heading_from_xy(tangent2[0], tangent2[1]) + math.pi) % (2 * math.pi)
            except Exception:
                final_heading = -1
            # print("Warning: omega near 2 * PI or 0. Linear approximation used instead of curve")
            return [[original_points[0], original_points[2]], final_heading, 0]
