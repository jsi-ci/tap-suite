import math
import numpy as np
from scipy.signal import argrelextrema
from scipy.spatial import KDTree

from tap.create_clothoids.clothoids import clothoids
from tap import angle_manipulation as am


def clothoid_length(clothoid):
    """ Calculates the length of a clothoid. """
    return np.sum(np.sqrt(np.sum(np.power(clothoid[:-1, :] - clothoid[1:, :], 2), axis=1)))


class Solution:
    """ Class representing a solution to the tunnel alignment problem. """
    def __init__(self, genotype, problem):
        self.problem = problem
        # check if the genotype is of correct length
        if len(genotype) != problem.num_variables:
            raise Exception('genotype is not of correct length')
        self.genotype = genotype

        # evaluation_dict is a dictionary containing the evaluation results for
        # objectives, constraints and hard constraints
        self.evaluation_dict = {}

        if self.problem.variant["order"] == "by_order":
            genotype = self.convert_to_order_by_variables(genotype)

        try:
            if self.problem.variant["points"] == "points_and_angles":
                # read the genotype and create two (horizontal and vertical) lists of points that store
                # coordinates, angles, order, etc. of the points used to construct the clothoid
                self.points_h, self.points_v = self.get_points(genotype, points="points_and_angles")

                # when the angles are given as absolute values, we always perform the alignment
                # in the same direction, that is why we add two additional variables for order here
                if self.problem.variant["angles"] == "absolute":
                    genotype += [0, 0]

                # construct the clothoid from the points
                self.clothoid = self.create_clothoid(points="points_and_angles",
                                                     order_h=genotype[-2], order_v=genotype[-1])
            else:
                # read the genotype and create two (horizontal and vertical) lists of points that store
                # coordinates, angles, order, etc. of the points used to construct the clothoid.
                # Additionally, create two lists of factors that store the factors between free points.
                self.points_h, self.points_v, factors_h, factors_v = \
                    self.get_points(genotype, points="control_points")
                self.clothoid = self.create_clothoid(points="control_points",
                                                     factors_h=factors_h, factors_v=factors_v)
        except:
            self.clothoid = None

        if self.clothoid is not None:
            self.complete_evaluation_dict()

    def convert_to_order_by_variables(self, genotype):
        """ Converts the genotype, from a format without order variables to a format with order
        variables. """

        genotype_variables = self.problem.get_genotype_variables()
        new_genotype_variables = self.problem.get_genotype_variables(order="by_variables")
        n_h = min(math.floor(genotype[genotype_variables.index("n_h")]),
                  self.problem.max_horizontal_turns)
        n_v = min(math.floor(genotype[genotype_variables.index("n_v")]),
                  self.problem.max_vertical_turns)

        if self.problem.variant["points"] == "control_points":
            n_h = n_h + 1
            n_v = n_v + 1

        new_genotype = []
        i, j = 0, 0
        n_h_count, n_v_count = 0, 0
        while i < len(genotype_variables) and j < len(new_genotype_variables):
            if genotype_variables[i] == new_genotype_variables[j]:
                new_genotype.append(genotype[i])
                i += 1
                j += 1
            elif new_genotype_variables[j] == "order":
                if n_h_count < self.problem.num_free_points_h:
                    n_h_count += 1
                    if n_h_count <= n_h:
                        order_value = self.problem.free_points_threshold + \
                                      (1 - self.problem.free_points_threshold) * n_h_count / (n_h + 1)
                        new_genotype.append(order_value)
                    else:
                        new_genotype.append(0)
                elif n_v_count < self.problem.num_free_points_v:
                    n_v_count += 1
                    if n_v_count <= n_v:
                        order_value = self.problem.free_points_threshold + \
                                      (1 - self.problem.free_points_threshold) * n_v_count / (n_v + 1)
                        new_genotype.append(order_value)
                    else:
                        new_genotype.append(0)
                else:
                    raise Exception("Something went wrong in mapping of the genotype: "
                                    "Invalid number of free points.")
                j += 1
            else:
                i += 1

        return new_genotype

    def get_points(self, genotype, points="points_and_angles"):
        """ Returns a list of horizontal and a list of vertical points, that are used to construct
        the solution. """
        num_given_points = self.problem.num_given_points
        num_free_h_points = self.problem.num_free_points_h
        num_free_v_points = self.problem.num_free_points_v

        points_h = []
        points_v = []
        if points == "points_and_angles":
            # start point:
            points_h.append(Point(self.problem, "start_point", "horizontal", k_r=genotype[0],
                                  phi=genotype[1], theta=genotype[2],
                                  k_h=genotype[3], k_v=genotype[4]))
            points_v.append(Point(self.problem, "start_point", "vertical", k_r=genotype[0],
                                  phi=genotype[1], theta=genotype[2],
                                  k_h=genotype[3], k_v=genotype[4]))

            # end point:
            points_h.append(Point(self.problem, "end_point", "horizontal", k_r=genotype[5],
                                  phi=genotype[6], theta=genotype[7],
                                  k_h=genotype[8], k_v=genotype[9]))
            points_v.append(Point(self.problem, "end_point", "vertical", k_r=genotype[5],
                                  phi=genotype[6], theta=genotype[7],
                                  k_h=genotype[8], k_v=genotype[9]))

            # given points_h:
            for i in range(num_given_points):
                k = 10 + 3 * i
                points_h.append(Point(self.problem, "given_point", "horizontal", k_r=genotype[k],
                                      phi=genotype[k + 1], k_h=genotype[k + 2], given_point_idx=i))

            # free horizontal points_h:
            for i in range(num_free_h_points):
                k = 10 + 3 * num_given_points + 4 * i
                if genotype[k + 3] > self.problem.free_points_threshold:
                    points_h.append(Point(self.problem, "free_point", "horizontal", x=genotype[k],
                                          y=genotype[k + 1], k_h=genotype[k + 2], order=genotype[k + 3]))

            # free vertical points_h:
            for i in range(num_free_v_points):
                k = 10 + 3 * num_given_points + 4 * num_free_h_points + 4 * i
                if genotype[k + 3] > self.problem.free_points_threshold:
                    points_v.append(Point(self.problem, "free_point", "vertical", u=genotype[k],
                                          v=genotype[k + 1], k_v=genotype[k + 2], order=genotype[k + 3]))

            return points_h, points_v

        elif points == "control_points":
            # start point:
            points_h.append(Point(self.problem, "start_point", "horizontal", k_r=genotype[0],
                                  phi=genotype[1], theta=genotype[2], k_h=None))
            points_v.append(Point(self.problem, "start_point", "vertical", k_r=genotype[0],
                                  phi=genotype[1], theta=genotype[2], k_v=None))

            # end point:
            points_h.append(Point(self.problem, "end_point", "horizontal", k_r=genotype[3],
                                  phi=genotype[4], theta=genotype[5], k_h=None))
            points_v.append(Point(self.problem, "end_point", "vertical", k_r=genotype[3],
                                  phi=genotype[4], theta=genotype[5], k_v=None))

            # free horizontal points_h:
            for i in range(num_free_h_points):
                k = 6 + 3 * i
                if genotype[k + 2] > self.problem.free_points_threshold:
                    points_h.append(Point(self.problem, "free_point", "horizontal", x=genotype[k],
                                          y=genotype[k + 1], order=genotype[k + 2], k_h=None))

            k = 6 + 3 * num_free_h_points
            factors_h = genotype[k:k + num_free_h_points - 1]

            # free vertical points_h:
            for i in range(num_free_v_points):
                k = 6 + 3 * num_free_h_points + 3 * i
                if genotype[k + 2] > self.problem.free_points_threshold:
                    points_v.append(Point(self.problem, "free_point", "vertical", u=genotype[k],
                                          v=genotype[k + 1], order=genotype[k + 2], k_v=None))

            k = 6 + 4 * num_free_h_points - 1 + 3 * num_free_v_points
            factors_v = genotype[k:k + num_free_v_points - 1]

            return points_h, points_v, factors_h, factors_v

        else:
            raise ValueError(f"Unknown points argument: {points}")

    def create_clothoid(self, points, **kwargs):
        """ Creates a clothoid for the given points. """

        # first create the horizontal clothoid
        if points == "points_and_angles":
            clothoid_h = self.create_2d_clothoid("horizontal", order=kwargs["order_h"])
        else:
            clothoid_h = self.create_2d_clothoid_from_control_points(
                "horizontal", factors=kwargs["factors_h"])

        # if the horizontal clothoid could not be created or is not a 2D array, return None
        if clothoid_h is None or clothoid_h.ndim != 2:
            return None

        # otherwise calculate the length of the horizontal clothoid, which is needed for the
        # vertical clothoid and then create the vertical clothoid
        clothoid_h_len = clothoid_length(clothoid_h)
        if points == "points_and_angles":
            clothoid_v = self.create_2d_clothoid("vertical", order=kwargs["order_v"],
                                                 c_length=clothoid_h_len)
        else:
            clothoid_v = self.create_2d_clothoid_from_control_points(
                "vertical", factors=kwargs["factors_v"], c_length=clothoid_h_len)

        # once again, if the vertical clothoid could not be created, return None
        if clothoid_v is None:
            return None

        # fill the evaluation dictionary with the values of the constraints
        # some constraints/objectives have to be evaluated before the clothoid is joined,
        # while others are evaluated after in the complete_evaluation_dict() method
        h_curvature, h_turns = clothoid_curvature(clothoid_h)
        v_curvature, v_turns = clothoid_curvature(clothoid_v)
        self.evaluation_dict["horizontal_curvature"] = h_curvature
        self.evaluation_dict["vertical_curvature"] = v_curvature
        self.evaluation_dict["gradient"] = max(clothoid_gradient(clothoid_v))
        self.evaluation_dict["horizontal_turns"] = h_turns
        self.evaluation_dict["vertical_turns"] = v_turns

        clothoid = join_clothoids(clothoid_h, clothoid_v, self.problem.min_dist)
        return clothoid

    def complete_evaluation_dict(self):
        """ Evaluates the solution and writes the results to the evaluation_dict """

        # calculate the length of the clothoid and the number of points out of bounds
        self.evaluation_dict["clothoid_length"] = self.clothoid[-1][3]
        self.evaluation_dict["out_of_bound_points"] = self.num_points_out_of_bounds()

        # calculate the price as a sum of basic cost multiplied with the distance of the clothoid
        # and the additional price of each obstacle
        self.evaluation_dict["costs"] = \
            self.evaluation_dict["clothoid_length"] * self.problem.basic_material_cost + \
            sum([obs.calculate_price(self.clothoid) for obs in self.problem.obstacles])

        # calculate the price of the hard constraints - the distance of the clothoid going through
        # the obstacles
        self.evaluation_dict["hard_constraint_obstacles"] = \
            sum([obs.calculate_price(self.clothoid) for obs in
                 self.problem.obstacles if obs.is_hard_constraint])

        # calculate the deviation (distance, horizontal angle, vertical angle) of the points for
        # each of the given points and add it to the evaluation_dict
        self.evaluation_dict["point_distance_deviation"] = 0
        self.evaluation_dict["point_horizontal_angle_deviation"] = 0
        self.evaluation_dict["point_vertical_angle_deviation"] = 0

        points = [self.problem.start_point, self.problem.end_point] + self.problem.given_points
        point_types = ["start_point", "end_point"] + list(range(len(self.problem.given_points)))
        # calculate the deviation for each point and add it to the evaluation_dict
        for point, point_type in zip(points, point_types):
            dist_dev, h_angle_dev, v_angle_dev = self.calculate_point_deviation(point, point_type)
            self.evaluation_dict["point_distance_deviation"] += dist_dev
            self.evaluation_dict["point_horizontal_angle_deviation"] += h_angle_dev
            self.evaluation_dict["point_vertical_angle_deviation"] += v_angle_dev

        self.evaluation_dict["self_intersections"] = self.find_self_intersection()

    def find_self_intersection(self):
        """ Calculates the number of self-intersections of the clothoid. """
        threshold_2d = 10
        threshold_cum_dist = 100
        threshold_z = 10

        # Extract relevant columns
        xy = self.clothoid[:, :2]  # (x, y) coordinates
        z = self.clothoid[:, 2]  # z-coordinate
        cum_dist = self.clothoid[:, 3]  # cumulative distance

        # Build KDTree for fast neighbor search
        tree = KDTree(xy)

        # Query each point for nearby neighbors
        for i, (point, z_i, cum_i) in enumerate(zip(xy, z, cum_dist)):
            indices = tree.query_ball_point(point, threshold_2d)  # Find close points

            for j in indices:
                if j <= i:
                    continue

                # Check cumulative distance condition
                if cum_dist[j] - cum_i < threshold_cum_dist:
                    continue

                # Check z-coordinate difference
                if abs(z[j] - z_i) >= threshold_z:
                    continue

                return 1

        return 0

    def calculate_point_deviation(self, point, point_type):
        """ Calculates the deviation (distance, horizontal angle, vertical angle) of a point
        from the clothoid given by the points array. """
        # find the closest point on the clothoid
        if point_type == "start_point":
            closest_point_idx = 0
        elif point_type == "end_point":
            closest_point_idx = len(self.clothoid) - 1
        else:
            closest_point_idx = self.find_closest_point(point)

        # calculate the distance of the point from the clothoid
        distance = np.linalg.norm([point["x"] - self.clothoid[closest_point_idx][0],
                                   point["y"] - self.clothoid[closest_point_idx][1],
                                   point["z"] - self.clothoid[closest_point_idx][2]])
        if distance > point["r_max"]:
            distance = distance - point["r_max"]
        else:
            distance = 0

        # take also the next point (in case of last point take previous), to calculate the
        # horizontal and vertical angle
        if point_type == "end_point":
            closest_point = self.clothoid[-1]
            next_point = self.clothoid[-2]
        else:
            if closest_point_idx == len(self.clothoid) - 1:
                closest_point_idx -= 1
            closest_point = self.clothoid[closest_point_idx]
            next_point = self.clothoid[closest_point_idx + 1]

        def calculate_angle_deviation(angle, desired_angle, tolerance):
            angle = am.mod2pi(angle)
            desired_angle = am.mod2pi(desired_angle)
            deviation = abs(am.mod2pi(angle - desired_angle + math.pi) - math.pi)
            return max(0, deviation - tolerance)

        # calculate the horizontal angle deviation
        h_angle = am.angle_between_points(closest_point, next_point)
        desired_h_angle = am.mod2pi(np.deg2rad(point["h_angle"]))
        allowed_h_angle_dev = np.deg2rad(point["h_angle_tol"])
        h_angle_dev = calculate_angle_deviation(h_angle, desired_h_angle, allowed_h_angle_dev)

        # calculate the vertical angle deviation (only for start and end point)
        if point_type == "start_point" or point_type == "end_point":
            v_angle = am.angle_between_points(closest_point, next_point, horizontal=False)
            desired_v_angle = am.mod2pi(np.deg2rad(point["v_angle"]))
            allowed_v_angle_dev = np.deg2rad(point["v_angle_tol"])
            v_angle_dev = calculate_angle_deviation(v_angle, desired_v_angle, allowed_v_angle_dev)
        else:
            v_angle_dev = 0

        return distance, h_angle_dev, v_angle_dev

    def find_closest_point(self, point):
        """ Finds the closest point on the clothoid to a given point """
        closest_point_idx = 0
        min_dist = math.inf
        for i, clothoid_point in enumerate(self.clothoid):
            dist = math.sqrt((point["x"] - clothoid_point[0]) ** 2 +
                             (point["y"] - clothoid_point[1]) ** 2 +
                             (point["z"] - clothoid_point[2]) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_point_idx = i
        return closest_point_idx

    def preprocess_points(self, alignment, c_length=None):
        """ Preprocesses the points of the clothoid to be used for the creation of the clothoid. """
        if alignment == "horizontal":
            # if alignment is horizontal take the horizontal points
            return self.points_h

        elif alignment == "vertical":
            # if alignment is vertical take the vertical points,
            # but first convert them to (t, z) coordinates
            # compute the difference between the original last and first point
            first_point = sorted(self.points_h)[0]
            last_point = sorted(self.points_h)[-1]
            z_diff = last_point.coordinates[2] - first_point.coordinates[2]
            points = self.points_v
            # convert points from (u, v) coordinates to (t, z) coordinates
            for point in points:
                point.convert_coordinates_to_tz(self.problem, c_length, z_diff,
                                                first_point.coordinates[2])
            return points
        else:
            raise Exception(f"Invalid alignment ({alignment}).")

    def create_2d_clothoid_from_control_points(self, alignment, factors, c_length=None):
        """ creates a 2d clothoid (from control points) for a given alignment """
        points = self.preprocess_points(alignment, c_length)
        points.sort()
        coordinates_list = []

        for point in points:
            coordinates_list.append([point.coordinates[0], point.coordinates[1]])

        # create the clothoid
        c, norm_dist = clothoids.create_clothoid(np.array(coordinates_list), factors=factors)
        self.evaluation_dict["norm_dist"] = norm_dist

        return c

    def create_2d_clothoid(self, alignment, order=None, c_length=None):
        """ creates a 2d clothoid (from points and angles) for a given alignment """
        points = self.preprocess_points(alignment, c_length)

        # order points according to the given order parameter
        if order <= 1:
            points.sort()
        else:
            points.sort(reverse=True)

        coordinates_angle_list = []

        point = points[0]
        if self.problem.variant.get("angles") == "factors":
            angle = am.get_angle_from_range(point.angle_range[0], point.angle_range[1], point.angle)
        else:
            angle = point.angle

        coordinates_angle_list.append([point.coordinates[0], point.coordinates[1], angle])

        for i in range(1, len(points)):
            prev_point = point
            prev_angle = angle
            point = points[i]

            if self.problem.variant.get("angles") == "factors":
                angle_range = am.get_feasible_angle_range(
                    (point.coordinates[0], point.coordinates[1], point.angle, point.angle_range[0],
                     point.angle_range[1]), prev_angle, prev_point.coordinates)
                point.feasible_angles = angle_range  # just for visualization...

                angle = am.mod2pi(am.get_angle_from_range(angle_range[0], angle_range[1],
                                                          point.angle) + np.pi)
            else:
                angle = point.angle + np.pi

            coordinates_angle_list.append([point.coordinates[0], point.coordinates[1], angle])

        c, norm_dist = clothoids.create_clothoid_from_mandatory_points_with_angles(
            np.array(coordinates_angle_list))
        self.evaluation_dict["norm_dist"] = norm_dist

        # always return the clothoid in the same direction - flip back if necessary
        if order > 1:
            c = np.flip(c, axis=0)

        # go through the clothoid and check if any two points are too close to each other
        # if so, remove the second point
        i = 0
        while i < len(c) - 1:
            if np.linalg.norm(c[i] - c[i + 1]) < 0.1:
                c = np.delete(c, i + 1, axis=0)
            else:
                i += 1

        return c

    def evaluate_constraints_objectives(self):
        """ Evaluates the constraints and objectives of the problem_name. If the hard constraints
        are not satisfied, the constraints are added a penalty. """

        feasible = True
        hard_constraints = []

        if any((math.isnan(value) or value is None) for value in self.evaluation_dict.values()):
            objectives = [None] * len(self.problem.objectives)
            constraints = [1] * len(self.problem.constraints)
            hard_constraints = [1] * len(self.problem.hard_constraints)
            penalty_constraint = [self.problem.penalty]
            return constraints + hard_constraints + penalty_constraint, objectives

        for cons in self.problem.hard_constraints:
            value = self.evaluation_dict.get(cons)

            if value is None:
                # when the value is None, it means that the clothoid is not created
                # then we return None for objectives and maximum value (1) for constraints
                # and a penalty to make sure such solution is always the worst
                objectives = [None] * len(self.problem.objectives)
                constraints = [1] * len(self.problem.constraints)
                hard_constraints = [1] * len(self.problem.hard_constraints)
                penalty_constraint = [self.problem.penalty]
                return constraints + hard_constraints + penalty_constraint, objectives

            if cons == "out_of_bound_points":
                hard_constraints.append(value / len(self.clothoid))
                if value > 0:
                    feasible = False
            elif cons == "hard_constraint_obstacles":
                hard_constraints.append(value / self.clothoid[-1, -1])
                # note: clothoid[-1, -1] = length of the clothoid
                if value > 0:
                    feasible = False
            elif cons == "horizontal_turns":
                if value > self.problem.max_horizontal_turns:
                    hard_constraints.append(1)
                    feasible = False
                else:
                    hard_constraints.append(0)
            elif cons == "vertical_turns":
                if value > self.problem.max_vertical_turns:
                    hard_constraints.append(1)
                    feasible = False
                else:
                    hard_constraints.append(0)
            elif cons == "self_intersections":
                hard_constraints.append(value)
                if value > 0:
                    feasible = False

        # try to evaluate objectives if it is possible, otherwise return None
        try:
            objectives = [self.evaluate_tree_recursively(self.problem.evaluation_tree, obj)
                          for obj in self.problem.objectives]
        except:
            objectives = [None] * len(self.problem.objectives)

        # evaluate constraints
        constraints = [self.evaluate_tree_recursively(self.problem.evaluation_tree,
                                                      cons, obj_or_cons="constraint")
                       for cons in self.problem.constraints]

        # add a penalty constraint if the solution is not feasible
        penalty_constraint = [0] if feasible else [self.problem.penalty]

        return constraints + hard_constraints + penalty_constraint, objectives

    def evaluate_tree_recursively(self, eval_tree, name, obj_or_cons="objective",
                                  add_all_child_values=False):
        """ Evaluates the evaluation tree recursively. """

        def normalize_value(value, bound, direction, weight, norm_min, norm_max):
            """ Returns the value multiplied by the weight if it is over/under the bound,
            depending on the direction (min/max). """
            if obj_or_cons == "constraint":
                if direction == "min":
                    if value > bound:
                        if value > norm_max:
                            return 1
                        return (value - bound) / (norm_max - bound)
                    return 0
                else:
                    if value < bound:
                        if value < norm_min:
                            return 1
                        return (bound - value) / (bound - norm_min)
                    return 0
            else:
                if direction == "min":
                    if value > norm_max:
                        return weight * 1
                    return weight * value / (norm_max - norm_min)
                else:
                    if value < norm_min:
                        return weight * 1
                    return weight * value / (norm_max - norm_min)

        if add_all_child_values:
            # if we are at the right node, we can return the value of the node (or in case,
            # that the node has children, the normalized sum of the values of the children)
            if "inputs" in eval_tree:
                value = 0
                for child_name, child_value in eval_tree["inputs"].items():
                    value += self.evaluate_tree_recursively(
                        child_value, child_name, obj_or_cons=obj_or_cons, add_all_child_values=True)

                return normalize_value(value, eval_tree.get("bound"), eval_tree["direction"],
                                       eval_tree["weight"], eval_tree["norm_min"],
                                       eval_tree["norm_max"])

            else:
                if name not in self.evaluation_dict:
                    return 1
                value = self.evaluation_dict[name]
                return normalize_value(value, eval_tree.get("bound"), eval_tree["direction"],
                                       eval_tree["weight"], eval_tree["norm_min"],
                                       eval_tree["norm_max"])
        else:
            # if we are in a wrong node, we continue to go deeper in the tree (if possible),
            # searching for the node with the right name (param: name)
            if "inputs" in eval_tree:
                value = 0
                for child_name, child_value in eval_tree["inputs"].items():
                    if child_name == name:
                        value += self.evaluate_tree_recursively(
                            child_value, name, obj_or_cons=obj_or_cons, add_all_child_values=True)
                    else:
                        value += self.evaluate_tree_recursively(
                            child_value, name, obj_or_cons=obj_or_cons, add_all_child_values=False)
                return value
            else:
                return 0

    def num_points_out_of_bounds(self):
        """ Returns the number of points that are out of the problem_name bounds. """
        num_points = 0
        limits = self.problem.area_limits

        for point in self.clothoid:
            if not (limits["x"][0] <= point[0] <= limits["x"][1] and
                    limits["y"][0] <= point[1] <= limits["y"][1] and
                    limits["z"][0] <= point[2] <= limits["z"][1]):
                num_points += 1
        return num_points


def join_clothoids(clothoid_h, clothoid_v, min_dist):
    """ joins the horizontal and vertical create_clothoids into one 3D clothoid """
    if clothoid_v.ndim != 2 or clothoid_h.ndim != 2:
        return None

    def get_height_at_dist(d, idx=0):
        """ returns the height of a clothoid_v at a given distance d """
        if idx == len(clothoid_v) - 1:
            return idx, clothoid_v[idx][1]
        elif clothoid_v[idx, 0] <= d <= clothoid_v[idx + 1, 0]:
            c0 = clothoid_v[idx, :]
            c1 = clothoid_v[idx + 1, :]
            return idx, c0[1] + (c1[1] - c0[1]) / (c1[0] - c0[0]) * (d - c0[0])
        else:
            return get_height_at_dist(d, idx + 1)

    idx_v = 0
    cum_dist = 0
    clothoid = []
    for i in range(len(clothoid_h) - 1):
        # get the height at the current horizontal point and append it to the 3D clothoid
        idx_v, height = get_height_at_dist(cum_dist, idx_v)
        clothoid.append([clothoid_h[i, 0], clothoid_h[i, 1], height])

        # compute the distance to the next horizontal point
        # if the distance is larger than the minimum distance, add new points so that
        # the distance between two points is always smaller than the minimum distance
        dist_to_next = np.linalg.norm(clothoid_h[i] - clothoid_h[i + 1])
        num_new_points = int(dist_to_next // min_dist)
        if num_new_points > 0:
            # compute the distance between the added points
            distance_between_points = dist_to_next / (num_new_points + 1)
            dx = clothoid_h[i + 1, 0] - clothoid_h[i, 0]
            dy = clothoid_h[i + 1, 1] - clothoid_h[i, 1]

            for j in range(1, num_new_points + 1):
                # compute new point coordinates and append them to the 3D clothoid
                curr_d = cum_dist + j * distance_between_points
                new_x = clothoid_h[i, 0] + dx * (j / (num_new_points + 1))
                new_y = clothoid_h[i, 1] + dy * (j / (num_new_points + 1))
                idx_v, new_height = get_height_at_dist(curr_d, idx_v)
                clothoid.append([new_x, new_y, new_height])

        # update the cumulative distance
        cum_dist += dist_to_next

    # append the last point
    idx_v, height = get_height_at_dist(cum_dist, idx_v)
    clothoid.append([clothoid_h[-1, 0], clothoid_h[-1, 1], height])

    clothoid = np.array(clothoid)

    # calculate the cumulative distance at each point
    distances = np.linalg.norm(clothoid[1:, :] - clothoid[:-1, :], axis=1)

    cum_distances = distances.cumsum()
    cum_distances = np.insert(cum_distances, 0, 0)
    cum_distances = cum_distances.reshape(-1, 1)

    # Add cum distances to the clothoid array
    clothoid = np.concatenate((clothoid, cum_distances), axis=1)

    return clothoid


def clothoid_curvature(clothoid):
    """ calculates the maximal curvature of a clothoid and the number of turns """

    def menger_curvature(points):
        """ calculates Menger curvature  of a clothoid for a given sequence of three points """
        p1 = np.array(points[0])
        p2 = np.array(points[1])
        p3 = np.array(points[2])
        v1 = p2 - p1
        v2 = p3 - p2
        u1 = v1 / np.linalg.norm(v1)
        u2 = v2 / np.linalg.norm(v2)
        theta = np.arccos(np.clip(np.dot(u1, u2), -1, 1))
        d = np.linalg.norm(p3 - p1)
        k = 2 * np.sin(theta) / d
        return k

    # calculate curvature for each point in the clothoid, except the first and last one
    curvatures = []
    for i in range(1, len(clothoid) - 1):
        curvatures.append(menger_curvature(clothoid[i - 1:i + 2]))

    # compute the number of turns as the number of local maximums in the curvature array
    # (curvature of the clothoid always looks like this: /\/\/\ - where each /\ represents one turn)
    num_turns = len(list(argrelextrema(np.array(curvatures), np.greater)[0]))

    if len(curvatures) == 0:
        return 0, 0
    return max(curvatures), num_turns


def clothoid_gradient(clothoid):
    """ Calculates the absolute gradients at each point of a clothoid """
    gradient = lambda x1, y1, x2, y2: abs((y2 - y1) / (x2 - x1))
    gradients = []

    for i in range(1, len(clothoid)):
        gradients.append(gradient(clothoid[i - 1][0], clothoid[i - 1][1],
                                  clothoid[i][0], clothoid[i][1]))

    if len(gradients) == 0:
        return [0]

    return gradients


class Point:
    """ Class representing a starting, ending, given or free point in the solution.
    Each point has the following attributes:
        - point_type: "start_point", "end_point", "given_point" or "free_point"
        - alignment: "horizontal" or "vertical"
        - coordinates: can be either [x, y, z], [x, y] only ur even [u, v] (which are then
        transformed to [t, z]), depending on the alignment and the point type
        - angle: horizontal or vertical angle of the point, represented as k_h or k_v in the docs.
        Can be absolute or factor of the feasible range of the angle, depending on the variant
        - angle_range: feasible range of the angle
        - order: order of the point in the clothoid sequence
     """

    def __init__(self, problem, point_type, alignment, **kwargs):
        # first check if all the arguments are valid
        # for key, value in kwargs.items():
        #     problem_name.check_valid_arguments(key, value)

        # depending on the point type and the alignment, set all the other attributes of the point
        self.point_type = point_type
        self.alignment = alignment

        if point_type == "start_point":
            self.order = -0.1

            if alignment == "horizontal":
                x, y, z = am.spherical_to_cartesian(kwargs["k_r"] * problem.start_point["r_max"],
                                                    kwargs["phi"], kwargs["theta"])
                self.coordinates = [problem.start_point["x"] + x, problem.start_point["y"] + y,
                                    problem.start_point["z"] + z]
                self.angle = kwargs["k_h"]
                self.angle_range = am.feasible_angles(problem.start_point["h_angle"],
                                                      problem.start_point["h_angle_tol"])
            elif alignment == "vertical":
                self.coordinates = [0, 0]
                self.angle = kwargs["k_v"]

                phi_min, phi_max = am.feasible_angles(problem.start_point["v_angle"],
                                                      problem.start_point["v_angle_tol"])
                inc_min, inc_max = am.feasible_angles(0, problem.max_gradient_degrees)
                self.angle_range = am.intersection(phi_min, phi_max, inc_min, inc_max)

            else:
                raise Exception(f"Invalid alignment ({alignment}).")

        elif point_type == "end_point":
            self.order = 1.1

            if alignment == "horizontal":
                x, y, z = am.spherical_to_cartesian(kwargs["k_r"] * problem.end_point["r_max"],
                                                    kwargs["phi"], kwargs["theta"])
                self.coordinates = [problem.end_point["x"] + x, problem.end_point["y"] + y,
                                    problem.end_point["z"] + z]
                self.angle = kwargs["k_h"]
                self.angle_range = am.feasible_angles(problem.end_point["h_angle"],
                                                      problem.end_point["h_angle_tol"])
            elif alignment == "vertical":
                self.coordinates = [1, 1]
                self.angle = kwargs["k_v"]
                phi_min, phi_max = am.feasible_angles(problem.end_point["v_angle"],
                                                      problem.end_point["v_angle_tol"])
                inc_min, inc_max = am.feasible_angles(-180, problem.max_gradient_degrees)
                self.angle_range = am.intersection(phi_min, phi_max, inc_min, inc_max)
            else:
                raise Exception(f"Invalid alignment ({alignment}).")

        elif point_type == "given_point":

            if alignment == "horizontal":
                self.given_point_idx = kwargs["given_point_idx"]
                this_point = problem.given_points[self.given_point_idx]

                self.order = this_point["order"]

                x, y, _ = am.spherical_to_cartesian(kwargs["k_r"] * this_point["r_max"],
                                                    kwargs["phi"], 0)
                self.coordinates = [this_point["x"] + x, this_point["y"] + y]
                self.angle = kwargs["k_h"]
                self.angle_range = am.feasible_angles(this_point["h_angle"],
                                                      this_point["h_angle_tol"])
            else:
                raise Exception(f"Invalid alignment and point type combination "
                                f"({alignment, point_type}).")

        elif point_type == "free_point":
            self.order = kwargs["order"]
            if alignment == "horizontal":
                self.coordinates = [kwargs["x"], kwargs["y"]]
                self.angle = kwargs["k_h"]
                self.angle_range = [0, 2 * np.pi]
            elif alignment == "vertical":
                self.coordinates = [kwargs["u"], kwargs["v"]]
                self.angle = kwargs["k_v"]
                self.angle_range = am.feasible_angles(0, problem.max_gradient_degrees)
            else:
                raise Exception(f"Invalid alignment ({alignment}).")

        else:
            raise Exception(f"Invalid point type ({point_type}).")

        # used for visualization only
        self.feasible_angles = self.angle_range

    def __lt__(self, other):
        """ Used for sorting the points. """
        return self.order < other.order

    def convert_coordinates_to_tz(self, problem, c_length, z_diff, first_point_z):
        """ Converts the coordinates of a point from x-y to t-z coordinates, as described in the
        document. """
        tan = np.tan(np.deg2rad(problem.max_gradient_degrees))

        t = 0.5 * (self.coordinates[0] * (c_length - z_diff / tan) +
                   self.coordinates[1] * (c_length + z_diff / tan))
        z = 0.5 * (self.coordinates[0] * (z_diff - c_length * tan) +
                   self.coordinates[1] * (z_diff + c_length * tan))
        self.coordinates[0] = t
        self.coordinates[1] = z + first_point_z