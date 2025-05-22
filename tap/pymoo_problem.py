import warnings as _warnings
try:
    from pymoo.core.problem import ElementwiseProblem
except ImportError:
    ElementwiseProblem = object
    _warnings.warn("pymoo not installed, pymoo problems will not be available.")


class PymooProblem(ElementwiseProblem):
    """ Wrapper class for pymoo optimization algorithms """
    def __init__(self, problem):
        self.problem = problem
        lower_bounds, upper_bounds = problem.variable_bounds
        self.id = problem.id
        self.name = problem.name
        self.ideal = problem.ideal
        self.nadir = problem.nadir

        super().__init__(n_var=problem.num_variables,
                         n_obj=problem.num_objectives,
                         n_ieq_constr=problem.num_constraints,
                         xl=lower_bounds,
                         xu=upper_bounds)

    def _evaluate(self, x, out, *args, **kwargs):
        """ Evaluates the problem and saves the results in the output dictionary. """
        constraints, objectives = self.problem.evaluate(x)

        out["F"] = objectives
        out["G"] = constraints

