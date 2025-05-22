import warnings as _warnings
try:
    from jmetal.core.problem import FloatProblem
except ImportError:
    FloatProblem = object
    _warnings.warn("jmetalpy not installed, jmetalpy problems will not be available.")


class JMetalProblem(FloatProblem):
    """ Wrapper class for jMetalPy optimization algorithms """
    def __init__(self, problem):
        self.problem = problem
        self.id = problem.id
        self.name = problem.name
        self.ideal = problem.ideal
        self.nadir = problem.nadir

        super(JMetalProblem, self).__init__()

        self.obj_directions = [self.MINIMIZE, self.MINIMIZE]
        self.obj_labels = self.problem.objectives

        lower_bounds, upper_bounds = self.problem.variable_bounds
        self.lower_bound = lower_bounds
        self.upper_bound = upper_bounds

    def number_of_objectives(self):
        return self.problem.num_objectives

    def number_of_constraints(self):
        return self.problem.num_constraints

    def evaluate(self, solution):
        constraints, objectives = self.problem.evaluate(solution.variables)

        solution.objectives = objectives
        solution.constraints = constraints

        return solution

    def name(self):
        return self.name
