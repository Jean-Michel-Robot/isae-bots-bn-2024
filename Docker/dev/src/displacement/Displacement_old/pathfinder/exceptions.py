# -*- coding: utf-8 -*-

"""
@file: exceptions.py
@status: OK.
"""

class PathNotFoundError(RuntimeError):
    """Exception raised when no path found."""
    pass

class TimeOutError(RuntimeError):
    """Exception raised when timed out in PF."""
    pass