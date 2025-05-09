import json

from tap.problem import TunnelAlignmentProblem
from tap.pymoo_problem import PymooProblem
from tap.jmetalpy_problem import JMetalProblem


def get_tap_suite(interface=None):
    """ returns the suite of 12 tunnel alignment problems, with the specified interface:
    "pymoo", "jmetalpy" or None (default). If None is specified, the problems are returned as
    TunnelAlignmentProblem objects."""
    suite_problems = [
        ("TAP1", "left-right_hard=2_hor=1_ver=2_soft=1", "CO"),
        ("TAP2", "15-areas_hor=3_ver=1", "AFO"),
        ("TAP3", "lr-ud_hor=4_ver=3", "AFO"),
        ("TAP4", "15-areas_hor=1_ver=2", "CV"),
        ("TAP5", "left-right_hard=2_hor=1_ver=1_soft=2", "AAO"),
        ("TAP6", "next-to-tunnel_num=0_hor=1_ver=1", "AFV"),
        ("TAP7", "15-areas_hor=5_ver=2", "CV"),
        ("TAP8", "next-to-tunnel_num=2_hor=1_ver=1", "CO"),
        ("TAP9", "left-right_hard=2_hor=1_ver=2_soft=1", "AAO"),
        ("TAP10", "next-to-tunnel_num=0_hor=1_ver=1", "CV"),
        ("TAP11", "next-to-tunnel_num=1_hor=1_ver=1", "AAO"),
        ("TAP12", "next-to-tunnel_num=1_hor=1_ver=1", "AFO")
    ]

    suite = []
    for i, (problem_name, layout, variant) in enumerate(suite_problems):
        problem_data = json.load(open(f"tap/problems/{layout}.json"))
        problem = TunnelAlignmentProblem(problem_data, variant, problem_name=problem_name)

        if interface is None:
            suite.append(problem)
        elif interface == "pymoo":
            suite.append(PymooProblem(problem))
        elif interface == "jmetalpy":
            suite.append(JMetalProblem(problem))
        else:
            raise ValueError(f"Unknown problem interface: {interface}")

    return suite
