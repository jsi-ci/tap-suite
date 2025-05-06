# tap-suite
A Suite and Generator of Tunnel Alignment Problems


```python
from tap.suite import get_tap_suite
```

###  Run suite problems with Pymoo


```python
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.termination import get_termination
```


```python
suite = get_tap_suite("pymoo")
suite
```




    {'problem_01': <tap.pymoo_problem.PymooProblem at 0x1c2e518b670>,
     'problem_02': <tap.pymoo_problem.PymooProblem at 0x1c2e518ba30>,
     'problem_03': <tap.pymoo_problem.PymooProblem at 0x1c2e5260970>,
     'problem_04': <tap.pymoo_problem.PymooProblem at 0x1c2e5261120>,
     'problem_05': <tap.pymoo_problem.PymooProblem at 0x1c2e5260790>,
     'problem_06': <tap.pymoo_problem.PymooProblem at 0x1c2e5260be0>,
     'problem_07': <tap.pymoo_problem.PymooProblem at 0x1c2e5260fd0>,
     'problem_08': <tap.pymoo_problem.PymooProblem at 0x1c2e5262050>,
     'problem_09': <tap.pymoo_problem.PymooProblem at 0x1c2e5261000>,
     'problem_10': <tap.pymoo_problem.PymooProblem at 0x1c2e5260e50>,
     'problem_11': <tap.pymoo_problem.PymooProblem at 0x1c2e5260d30>,
     'problem_12': <tap.pymoo_problem.PymooProblem at 0x1c2e52620b0>}




```python
problem = suite["problem_04"]
algorithm = NSGA2(pop_size=100)
termination = get_termination("n_eval", 1000)
minimize(problem, algorithm, termination, verbose=True)
```

    ==========================================================================================
    n_gen  |  n_eval  | n_nds  |     cv_min    |     cv_avg    |      eps      |   indicator  
    ==========================================================================================
         1 |      100 |      3 |  0.000000E+00 |  3.0970437952 |             - |             -
         2 |      200 |      4 |  0.000000E+00 |  0.0183102910 |  0.0241828984 |         ideal
         3 |      300 |      7 |  0.000000E+00 |  0.000000E+00 |  0.0167494859 |         ideal
         4 |      400 |      8 |  0.000000E+00 |  0.000000E+00 |  0.3226273917 |         ideal
         5 |      500 |      7 |  0.000000E+00 |  0.000000E+00 |  0.0495190665 |         ideal
         6 |      600 |      8 |  0.000000E+00 |  0.000000E+00 |  0.0543457405 |         ideal
         7 |      700 |     12 |  0.000000E+00 |  0.000000E+00 |  0.0376853236 |             f
         8 |      800 |     11 |  0.000000E+00 |  0.000000E+00 |  0.0152298130 |         ideal
         9 |      900 |     15 |  0.000000E+00 |  0.000000E+00 |  0.0405444601 |         ideal
        10 |     1000 |      9 |  0.000000E+00 |  0.000000E+00 |  0.0251221088 |         nadir
    




    <pymoo.core.result.Result at 0x1c2e52603a0>



### Run suite problems with jMetalPy


```python
from jmetal.algorithm.multiobjective.nsgaii import NSGAII
from jmetal.operator.crossover import SBXCrossover
from jmetal.operator.mutation import PolynomialMutation
from jmetal.util.termination_criterion import StoppingByEvaluations
```


```python
suite = get_tap_suite("jmetalpy")
```


```python
problem = suite["problem_04"]
algorithm = NSGAII(
    problem=problem,
    population_size=100,
    offspring_population_size=100,
    mutation=PolynomialMutation(probability=1.0 / problem.number_of_variables(), distribution_index=20),
    crossover=SBXCrossover(probability=1.0, distribution_index=20),
    termination_criterion=StoppingByEvaluations(max_evaluations=1000)
)
algorithm.run()
```
