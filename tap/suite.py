import json
import os

from tap import TunnelAlignmentProblem, PymooProblem, JMetalProblem


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
    return _get_tap_problems(suite_problems, interface)


def get_tap_300_problems(interface=None):
    """ returns the list of all 300 tunnel alignment problems, with the specified interface:
    "pymoo", "jmetalpy" or None (default). If None is specified, the problems are returned as
    TunnelAlignmentProblem objects."""

    problem_layouts = os.listdir("tap/problems")
    suite_problems = []
    i = 1
    for layout in problem_layouts:
        for variant in ["CO", "CV", "AFO", "AFV", "AAO", "AAV"]:
            suite_problems.append((f"TAP300-{i}", layout[:-5], variant))
            i += 1

    return _get_tap_problems(suite_problems, interface)


def _get_tap_problems(suite_problems, interface=None):
    suite = []
    for problem_name, layout, variant in suite_problems:
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
