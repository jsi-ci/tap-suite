import json

from tap.problem import TunnelAlignmentProblem
from tap.pymoo_problem import PymooProblem
from tap.jmetalpy_problem import JMetalProblem


def get_tap_suite(interface=None):
    """ returns the suite of 12 tunnel alignment problems
    """
    suite_problems = [
        ("problem_01", "left-right_hard=2_hor=1_ver=2_soft=1", "CO"),
        ("problem_02", "15-areas_hor=3_ver=1", "AFO"),
        ("problem_03", "lr-ud_hor=4_ver=3", "AFO"),
        ("problem_04", "15-areas_hor=1_ver=2", "CV"),
        ("problem_05", "left-right_hard=2_hor=1_ver=1_soft=2", "AAO"),
        ("problem_06", "next-to-tunnel_num=0_hor=1_ver=1", "AFV"),
        ("problem_07", "15-areas_hor=5_ver=2", "CV"),
        ("problem_08", "next-to-tunnel_num=2_hor=1_ver=1", "CO"),
        ("problem_09", "left-right_hard=2_hor=1_ver=2_soft=1", "AAO"),
        ("problem_10", "next-to-tunnel_num=0_hor=1_ver=1", "CV"),
        ("problem_11", "next-to-tunnel_num=1_hor=1_ver=1", "AAO"),
        ("problem_12", "next-to-tunnel_num=1_hor=1_ver=1", "AFO")
    ]

    suite = {}
    for i, (problem_name, layout, variant) in enumerate(suite_problems):
        problem_data = json.load(open(f"tap/problems/{layout}.json"))
        problem = TunnelAlignmentProblem(problem_data, variant)

        if interface is None:
            suite[problem_name] = problem
        elif interface == "pymoo":
            suite[problem_name] = PymooProblem(problem)
        elif interface == "jmetalpy":
            suite[problem_name] = JMetalProblem(problem)
        else:
            raise ValueError(f"Unknown problem interface: {interface}")

    return suite
