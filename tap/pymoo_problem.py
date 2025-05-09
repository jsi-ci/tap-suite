import warnings as _warnings
try:
    from pymoo.core.problem import ElementwiseProblem
except ImportError:
    _warnings.warn("pymoo not installed, pymoo problems will not be available.")


class PymooProblem(ElementwiseProblem):
    def __init__(self, problem):
        self.problem = problem
        lower_bounds, upper_bounds = problem.variable_bounds
        self.id = problem.id
        self.name = problem.name

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

