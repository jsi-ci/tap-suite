""" This package implements the tunnel alignment problem and includes a suite of 12 benchmark
instances. It also provides functionality for generating custom problem instances.

Authors: Nace Sever, Erik Dovgan, Tea Tušar, 2025
License: BSD 3-Clause, see LICENSE file.
"""

__author__ = "Nace Sever, Erik Dovgan, Tea Tusar"
__license__ = "BSD 3-clause"
__version__ = "0.0.2"

from tap.problem import TunnelAlignmentProblem
from tap.jmetalpy_problem import JMetalProblem
from tap.pymoo_problem import PymooProblem
from tap.suite import get_tap_suite, get_tap_300_problems
