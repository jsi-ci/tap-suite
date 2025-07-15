import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as patches

from tap.solution import Solution


class TunnelAlignmentProblem:
    """ Class representing the tunnel alignment problem. """
    def __init__(self, problem_json, variant, problem_name=""):
        """ Initializes the problem from the given problem json file and the given
        variant. It can be in short string form: one of "AFV", "AFO", "AAV", "AAO", "CV" or "CO",
        or in dict form: {
            "points": "points_and_angles" or "control_points",
            "angles": "factor" or "absolute",
            "order": "by_variables" or "by_order",
        }. """
        self.max_horizontal_turns = problem_json["max_horizontal_turns"]
        self.max_vertical_turns = problem_json["max_vertical_turns"]
        self.basic_material_cost = problem_json["basic_material_cost"]
        # calculate the maximum gradient in degrees from the constraint in percentage
        self.max_gradient_degrees = math.atan(problem_json["max_gradient"]) * 180 / math.pi

        # how far out of bounds the control points can be
        self.max_out_of_bounds_control_point_factor = 1.5
        # discretization step for the clothoid
        self.min_dist = 5

        # read the nadir and ideal points from the json file for convergence calculation
        self.nadir = problem_json["nadir"]
        self.ideal = problem_json["ideal"]

        # read the data about the objectives, constraints and hard constraints from the json file
        # objective and constraint values are calculated from the evaluation tree
        self.evaluation_tree = problem_json["evaluation_tree"]
        self.objectives = problem_json["objectives"]
        self.constraints = problem_json["constraints"]
        self.hard_constraints = problem_json["hard_constraints"]
        # set the penalty for the problem to be the number of constraints
        self.penalty = len(self.constraints)

        # read the data about the points from the json file
        self.num_given_points = len(problem_json["points"]["given_points"])
        self.start_point = problem_json["points"]["start_point"]
        self.end_point = problem_json["points"]["end_point"]
        self.given_points = problem_json["points"]["given_points"]
        self.free_points_threshold = problem_json["points"]["free_points_threshold"]

        variants = {
            "AFV": {"points": "points_and_angles", "angles": "factors", "order": "by_variables"},
            "AAV": {"points": "points_and_angles", "angles": "absolute", "order": "by_variables"},
            "AFO": {"points": "points_and_angles", "angles": "factors", "order": "by_order"},
            "AAO": {"points": "points_and_angles", "angles": "absolute", "order": "by_order"},
            "CV": {"points": "control_points", "order": "by_variables"},
            "CO": {"points": "control_points", "order": "by_order"}
        }
        if type(variant) is str:
            self.variant = variants[variant]
            self.variant_short = variant
        else:
            self.variant = variant
            if variant["points"] == "points_and_angles":
                self.variant_short = f"A{variant['angles'][0]}{variant['order'][3]}".upper()
            else:
                self.variant_short = f"C{variant['order'][3]}".upper()

        assert self.variant["points"] in ["points_and_angles", "control_points"]
        assert self.variant["order"] in ["by_variables", "by_order"]
        if "angles" in self.variant:
            assert self.variant["angles"] in ["factors", "absolute"]

        if self.variant["points"] == "points_and_angles":
            self.num_free_points_h = self.max_horizontal_turns - self.num_given_points - 1
            self.num_free_points_v = self.max_vertical_turns - 1
        else:
            self.num_free_points_h = self.max_horizontal_turns
            self.num_free_points_v = self.max_vertical_turns

        # read the data about the bounds from the json file
        self.area_limits = problem_json["bounds"]["limits"]
        # construct a list of Obstacle objects
        self.obstacles = [Obstacle(obs_dict, self.basic_material_cost, self)
                          for obs_dict in problem_json["bounds"]["obstacles"]]

        self.id = f'{problem_json["problem_id"]}_{self.variant_short}'
        self.name = problem_name
        self.num_variables =  len(self.get_genotype_variables())
        self.num_objectives = len(self.objectives)
        self.num_constraints = len(self.constraints) + len(self.hard_constraints) + 1

    @property
    def variable_bounds(self):
        """ Returns the lower and upper bound for the variables of the genotype. """
        variables = self.get_genotype_variables()
        variable_bounds = [self.get_variable_bounds(variable) for variable in variables]
        lower_bounds = [bound[0] for bound in variable_bounds]
        upper_bounds = [bound[1] for bound in variable_bounds]
        return lower_bounds, upper_bounds

    def evaluate(self, x):
        """ Evaluates the given genotype x and returns the constraints and objectives."""
        solution = Solution(x, self)
        constraints, objectives = solution.evaluate_constraints_objectives()
        return constraints, objectives

    def get_genotype_variables(self, points=None, angles=None, order=None):
        """ Returns the variables of the genotype for the problem variant.
        If the parameters are not specified, the current variant is used. """
        if points is None:
            points = self.variant.get("points")
        if angles is None:
            angles = self.variant.get("angles")
        if order is None:
            order = self.variant.get("order")

        if points == "points_and_angles" and angles == "factors" and order == "by_variables":
            genotype = ["k_r", "phi", "theta", "k_h", "k_v"] * 2
            genotype += ["k_r", "phi", "k_h"] * self.num_given_points
            genotype += ["x", "y", "k_h", "order"] * self.num_free_points_h
            genotype += ["u", "v", "k_v", "order"] * self.num_free_points_v
            genotype += ["order_h", "order_v"]

        elif points == "points_and_angles" and angles == "absolute" and order == "by_variables":
            genotype = ["k_r", "phi", "theta", "a_h", "a_v"] * 2
            genotype += ["k_r", "phi", "a_h"] * self.num_given_points
            genotype += ["x", "y", "a_h", "order"] * self.num_free_points_h
            genotype += ["u", "v", "a_v", "order"] * self.num_free_points_v

        elif points == "points_and_angles" and angles == "factors" and order == "by_order":
            genotype = ["k_r", "phi", "theta", "k_h", "k_v"] * 2
            genotype += ["k_r", "phi", "k_h"] * self.num_given_points
            genotype += ["x", "y", "k_h"] * self.num_free_points_h
            genotype += ["u", "v", "k_v"] * self.num_free_points_v
            genotype += ["n_h", "n_v"]
            genotype += ["order_h", "order_v"]

        elif points == "points_and_angles" and angles == "absolute" and order == "by_order":
            genotype = ["k_r", "phi", "theta", "a_h", "a_v"] * 2
            genotype += ["k_r", "phi", "a_h"] * self.num_given_points
            genotype += ["x", "y", "a_h"] * self.num_free_points_h
            genotype += ["u", "v", "a_v"] * self.num_free_points_v
            genotype += ["n_h", "n_v"]

        elif points == "control_points" and order == "by_variables":
            genotype = ["k_r", "phi", "theta"] * 2
            genotype += ["x'", "y'", "order"] * self.num_free_points_h
            genotype += ["f_h"] * (self.num_free_points_h - 1)
            genotype += ["u", "v", "order"] * self.num_free_points_v
            genotype += ["f_v"] * (self.num_free_points_v - 1)

        elif points == "control_points" and order == "by_order":
            genotype = ["k_r", "phi", "theta"] * 2
            genotype += ["x'", "y'"] * self.num_free_points_h
            genotype += ["f_h"] * (self.num_free_points_h - 1)
            genotype += ["u", "v"] * self.num_free_points_v
            genotype += ["f_v"] * (self.num_free_points_v - 1)
            genotype += ["n_h", "n_v"]

        else:
            raise ValueError(f"Invalid combination of points, angles and order. "
                             f"points: {points}, angles: {angles}, order: {order}")

        return genotype

    def get_variable_bounds(self, var):
        """ Returns the lower and upper bound for the given variable of the genotype. """
        if var in ["k_r", "k_h", "k_v", "order", "u", "v", "f_h", "f_v"]:
            return 0, 1
        elif var in ["order_h", "order_v"]:
            return 0, 2
        elif var in ["phi", "a_h", "a_v"]:
            return 0, 2 * np.pi
        elif var == "theta":
            return -np.pi / 2, np.pi / 2
        elif var == "x":
            return self.area_limits["x"][0], self.area_limits["x"][1]
        elif var == "y":
            return self.area_limits["y"][0], self.area_limits["y"][1]
        elif var == "x'":
            middle = (self.area_limits["x"][0] + self.area_limits["x"][1]) / 2
            dist = (self.area_limits["x"][1] - self.area_limits["x"][0])
            return middle - dist / 2 * self.max_out_of_bounds_control_point_factor, \
                   middle + dist / 2 * self.max_out_of_bounds_control_point_factor
        elif var == "y'":
            middle = (self.area_limits["y"][0] + self.area_limits["y"][1]) / 2
            dist = (self.area_limits["y"][1] - self.area_limits["y"][0])
            return middle - dist / 2 * self.max_out_of_bounds_control_point_factor, \
                   middle + dist / 2 * self.max_out_of_bounds_control_point_factor
        elif var == "n_h":
            return 0, self.num_free_points_h + 1
        elif var == "n_v":
            return 0, self.num_free_points_v + 1
        else:
            return -math.inf, math.inf

    def check_valid_arguments(self, key, value):
        """ Checks if the given value is in the valid range for the given key."""
        bounds = self.get_variable_bounds(key)
        return bounds[0] <= value <= bounds[1]

    def plot_problem(self, solutions_list=None):
        """ Plots the problem. """
        fig, axs = plt.subplots(2, 1, figsize=(8, 6), layout='constrained')
        plt.suptitle(f"{self.id}".split("_")[0].replace("_", "-"), fontsize=16)

        cmap, norm, sm = self.get_normalized_colormap()

        clb = plt.colorbar(sm, ax=axs, pad=0.03, fraction=0.05)
        clb.ax.set_title('Material\ncosts')

        """ plot clothoid in xy plane """
        self.plot_problem_on_ax(axs[0], yz="y", cmap=cmap, norm=norm)

        """ plot clothoid in xz plane """
        self.plot_problem_on_ax(axs[1], yz="z", cmap=cmap, norm=norm)

        if solutions_list is not None:
            for x in solutions_list:
                solution = Solution(x, self)
                axs[0].plot(solution.clothoid[:, 0], solution.clothoid[:, 1])
                axs[1].plot(solution.clothoid[:, 0], solution.clothoid[:, 2])

        return fig

    def plot_problem_on_ax(self, ax, yz, cmap, norm, show_text=True):
        """ Plots the problem data on the given axis. """
        ax.set_title(f"(x, {yz}) plane")
        ax.set_xlabel("x")
        ax.set_ylabel(yz, rotation=0, labelpad=10)

        limits = [self.area_limits["x"], self.area_limits[yz]]
        ax.set_xlim((limits[0][0], limits[0][1]))
        ax.set_ylim((limits[1][0], limits[1][1]))

        ax.fill([limits[0][0], limits[0][0], limits[0][1], limits[0][1]],
                [limits[1][0], limits[1][1], limits[1][1], limits[1][0]],
                color=cmap(norm(self.basic_material_cost)), zorder=0)

        min_cost, max_cost = self.get_min_max_obstacle_price()
        plot_texts = []

        def plot_given_points(given_points, c, text=None):
            """ Plots the given points as circles in the given axis."""
            offset = (self.area_limits[yz][1] - self.area_limits[yz][0]) / 10
            for p in given_points:
                ax.add_patch(patches.Circle((p["x"], p[yz]), p["r_max"], edgecolor=c,
                                            facecolor="none", zorder=1.5))
                if text is not None:
                    plot_texts.append(ax.text(p["x"], p[yz] - offset, text,
                                              fontsize=10, zorder=2.5))

        def get_color_zorder(o):
            """ Returns the color and zorder of the given obstacle. """
            if o.is_hard_constraint:
                if cmap.is_gray():
                    return (0, 0, 0), 0.96, None
                return (152 / 255, 148 / 255, 199 / 255), 0.96, None
            # return normalised color and zorder (between 0.05 and 0.95)
            return cmap(norm(o.price)), (o.price - min_cost) / (max_cost - min_cost) * 0.9 + 0.05, \
                None

        # plot given points as circles with radius r_max
        plot_given_points(self.given_points, "blue")
        if show_text:
            plot_given_points([self.start_point], "royalblue", text="start")
            plot_given_points([self.end_point], "royalblue", text="end")
        else:
            plot_given_points([self.start_point], "royalblue")
            plot_given_points([self.end_point], "royalblue")

        # plot obstacles
        for o in self.obstacles:
            color, zorder, hatch = get_color_zorder(o)

            if yz == "y":
                if o.obstacle_type == "circle" or o.obstacle_type == "sphere":
                    ax.add_patch(patches.Circle((o.center[0], o.center[1]), o.r, hatch=hatch,
                                                facecolor=color, zorder=zorder))

                elif o.obstacle_type == "polygon":
                    ax.add_patch(patches.Polygon(o.points, facecolor=color, hatch=hatch,
                                                 zorder=zorder))
            else:
                if o.obstacle_type == "circle":
                    width = 2 * o.r
                    height = o.z[1] - o.z[0]
                    ax.add_patch(patches.Rectangle((o.center[0] - o.r, o.z[0]), width, height, hatch=hatch,
                                                   facecolor=color, zorder=zorder))
                elif o.obstacle_type == "sphere":
                    ax.add_patch(patches.Circle((o.center[0], o.center[2]), o.r, hatch=hatch,
                                                facecolor=color, zorder=zorder))
                elif o.obstacle_type == "polygon":
                    height = o.z[1] - o.z[0]
                    min_x = min([p[0] for p in o.points])
                    width = max([p[0] for p in o.points]) - min_x
                    ax.add_patch(patches.Rectangle((min_x, o.z[0]), width, height, hatch=hatch,
                                                   facecolor=color, zorder=zorder))
        return plot_texts

    def get_normalized_colormap(self, cmap_name="YlOrRd"):
        """ Returns a normalized colormap for the obstacles. """
        min_cost, max_cost = self.get_min_max_obstacle_price()
        if cmap_name == "Greys":
            max_cost *= 1.5
        cmap = mpl.colormaps[cmap_name]
        norm = plt.Normalize(min_cost, max_cost)
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        return cmap, norm, sm

    def get_min_max_obstacle_price(self):
        """ Returns the minimum and maximum price of the obstacles. """
        prices = [obs.price for obs in self.obstacles if not obs.is_hard_constraint]
        if len(prices) == 0:
            return self.basic_material_cost, self.basic_material_cost
        else:
            return min(prices + [self.basic_material_cost]), max(prices + [self.basic_material_cost])


class Obstacle:
    """ Class representing an obstacle in the problem_name. If the price of the obstacle is not
    specified, the obstacle is deemed to be a hard constraint. """

    def __init__(self, obstacle_dict, basic_material_cost, problem):
        self.obstacle_type = obstacle_dict["type"]
        self.points = obstacle_dict.get("points")
        self.z = obstacle_dict.get("z", [problem.area_limits["z"][0], problem.area_limits["z"][1]])
        self.center = obstacle_dict.get("center")
        self.r = obstacle_dict.get("r")
        self.price = obstacle_dict.get("price", None)

        if self.points is not None:
            self.segments = [tuple(sorted([bp0, bp1])) for bp0, bp1 in
                             zip(self.points, self.points[1:])] + \
                            [tuple(sorted([self.points[-1], self.points[0]]))]

        if self.price is None:
            self.above_base_price = 1
            self.is_hard_constraint = True
        else:
            self.is_hard_constraint = False
            self.above_base_price = self.price - basic_material_cost

        self.path_inside = None

    def calculate_price(self, points):
        """ Calculates the price for the given path as the length of the path inside the obstacle
        multiplied by the extra price of the obstacle. This is later added to the base price,
        calculated as the length of the path multiplied by the basic material cost. """
        length_of_path_inside = 0

        # check if the first point is inside the obstacle
        if self.is_inside(points[0]):
            start_point_dist = 0
            inside = True
        else:
            start_point_dist = None
            inside = False

        # loop through all the points and add the length of the path inside the obstacle
        # for consecutive points that are inside the obstacle
        for point in points[1:]:
            if inside:
                if not self.is_inside(point):
                    length_of_path_inside += point[3] - start_point_dist
                    # print("outside", point, "new length", length_of_path_inside)
                    start_point_dist = None
                    inside = False
            else:
                if self.is_inside(point):
                    # print("inside", point)
                    start_point_dist = point[3]
                    inside = True

        # check if the last point is inside the obstacle and add the length of the last connected
        # path inside the obstacle if it is inside
        if self.is_inside(points[-1]) and inside:
            length_of_path_inside += points[-1][3] - start_point_dist

        self.path_inside = length_of_path_inside
        return self.above_base_price * length_of_path_inside

    def is_inside(self, point):
        """ Checks if the given point is inside the obstacle. """
        if self.obstacle_type == "circle":
            return ((self.center[0] - point[0]) ** 2 + (self.center[1] - point[1]) ** 2
                    <= self.r ** 2) and (self.z[0] <= point[2] <= self.z[1])
        if self.obstacle_type == "sphere":
            return (self.center[0] - point[0]) ** 2 + (self.center[1] - point[1]) ** 2 + \
                (self.center[2] - point[2]) ** 2 <= self.r ** 2
        if self.obstacle_type == "polygon":
            if self.z[0] <= point[2] <= self.z[1]:
                # count the number of segments that are directly above the point -
                # if the number of segments is odd, the point is inside the polygon
                # and if it is even, the point is outside the polygon
                count = 0
                for ((x1, y1), (x2, y2)) in self.segments:
                    # first check if the point is between the x coordinates of the segment
                    if x1 <= point[0] < x2:
                        # then check if the point is below the segment
                        if not left_turn((x1, y1), (x2, y2), point):
                            count += 1
                return count % 2 == 1
            else:
                return False


def left_turn(p1, p2, p3):
    """ Checks if the given points form a left turn. """
    return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]) >= 0
