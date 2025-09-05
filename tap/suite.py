import json
from importlib import resources
import os

from tap import TunnelAlignmentProblem, PymooProblem, JMetalProblem


def get_tap_suite(interface=None):
    """ returns the suite of 15 tunnel alignment problems, with the specified interface:
    "pymoo", "jmetalpy" or None (default). If None is specified, the problems are returned as
    TunnelAlignmentProblem objects."""
    suite_problems = [
        ("TAP1", "15-areas_hor=3_ver=2", "AFO"),
        ("TAP2", "next-to-tunnel_num=0_hor=1_ver=1", "AFO"),
        ("TAP3", "lr-ud_hor=2_ver=3", "AFO"),
        ("TAP4", "15-areas_hor=2_ver=1", "AFO"),
        ("TAP5", "left-right_hard=2_hor=1_ver=1_soft=2", "AAV"),
        ("TAP6", "left-right_hard=2_hor=2_ver=1_soft=1", "CO"),
        ("TAP7", "u-turn_hor=2_ver=1", "CO"),
        ("TAP8", "15-areas_hor=3_ver=2", "AAO"),
        ("TAP9", "next-to-tunnel_num=3_hor=2_ver=1", "CO"),
        ("TAP10", "next-to-tunnel_num=0_hor=1_ver=1", "AAV"),
        ("TAP11", "loop_hor=5_ver=2", "AFV"),
        ("TAP12", "left-right_hard=3_hor=1_ver=1_soft=1", "AAO"),
        ("TAP13", "next-to-tunnel_num=1_hor=1_ver=1", "AFO"),
        ("TAP14", "loop_hor=4_ver=2", "AAV"),
        ("TAP15", "next-to-tunnel_num=1_hor=1_ver=1", "AAO")
    ]
    return _get_tap_problems(suite_problems, interface)


def get_tap_300_problems(interface=None):
    """ returns the list of all 300 tunnel alignment problems, with the specified interface:
    "pymoo", "jmetalpy" or None (default). If None is specified, the problems are returned as
    TunnelAlignmentProblem objects."""

    problem_layouts = resources.files("tap.problems").iterdir()
    suite_problems = []
    i = 1
    for layout in problem_layouts:
        for variant in ["CO", "CV", "AFO", "AFV", "AAO", "AAV"]:
            suite_problems.append((f"TAP300-{i}", os.path.basename(layout)[:-5], variant))
            i += 1

    return _get_tap_problems(suite_problems, interface)


def _get_tap_problems(suite_problems, interface=None):
    suite = []
    for problem_name, layout, variant in suite_problems:
        with resources.files("tap.problems").joinpath(f"{layout}.json").open("r") as f:
            problem_data = json.load(f)

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
