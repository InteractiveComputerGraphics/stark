# -*- coding: utf-8 -*-
import inspect
import numpy as np

# Import all
from .pystark import *
from .test_sim import *

ZERO = np.array([0.0, 0.0, 0.0])
ONES = np.array([1.0, 1.0, 1.0])
UNITX = np.array([1.0, 0.0, 0.0])
UNITY = np.array([0.0, 1.0, 0.0])
UNITZ = np.array([0.0, 0.0, 1.0])
MM = 0.001


# ---------------------------------------------------------------------------
# Reference-cycle / finalization-order fix for add_time_event callbacks
#
# The C++ Simulation stores time-event callbacks as std::function<void(double)>
# objects.  Because C++ std::function is opaque to Python's cyclic garbage
# collector (nanobind types have no tp_traverse), any Python object captured
# inside a lambda registered with add_time_event becomes part of an
# GC-invisible reference cycle.  In particular:
#
#   lambda t: simulation.set_gravity(...)   ← captures the Python simulation
#
# creates the cycle:
#   Python simulation → C++ Simulation → std::function → lambda → Python simulation
#
# Python can never break this cycle automatically, so the Simulation object
# is never properly collected.  Even without a cycle, when the interpreter
# exits it tears down module globals in an arbitrary order, and the C++
# destructor (which calls Py_DECREF on stored callables) may fire while
# Python's internal state is partially dismantled, causing the
# "leaked N instances" warning and the subsequent segfault (exit code 139).
#
# Fix: route every add_time_event callback through a thin _TimeEventWrapper
# that we can "clear" (set _fn = None) once run() returns.  After clearing,
# the wrapper no longer holds any reference to the user's lambda, so all
# cycles are broken and user objects can be collected normally.  The C++ side
# still holds the wrapper alive (via std::function) until the Simulation is
# destroyed, but at that point the wrapper is a trivially-small dead object
# with no Python references inside it — safe to decrement from the destructor.
# ---------------------------------------------------------------------------

_NativeSimulation = Simulation


class _TimeEventWrapper:
    """Single-use indirection that lets us nullify the user callback after
    simulation.run() returns, breaking any reference cycles."""
    __slots__ = ['_fn']

    def __init__(self, fn):
        self._fn = fn

    def __call__(self, t: float):
        if self._fn is not None:
            self._fn(t)

    def clear(self):
        """Release the user callback reference."""
        self._fn = None


class Simulation(_NativeSimulation):
    """Drop-in replacement for the native Simulation that safely manages
    the lifetime of Python callbacks registered via add_time_event."""

    def __init__(self, settings):
        super().__init__(settings)
        # Keeps _TimeEventWrapper objects alive for the C++ side while run() is active
        self._event_wrappers = []

    def add_time_event(self, t0: float, t1: float, callback) -> None:
        wrapper = _TimeEventWrapper(callback)
        self._event_wrappers.append(wrapper)
        super().add_time_event(t0, t1, wrapper)

    def run(self, *args, **kwargs):
        # The C++ run(double, std::function<void()>) callback takes no arguments.
        # If the user passes a callable that expects one argument (time), wrap it
        # to call callback(self.get_time()) so the API feels natural.
        new_args = list(args)
        if len(new_args) >= 2 and callable(new_args[1]):
            cb = new_args[1]
            try:
                n_params = sum(
                    1 for p in inspect.signature(cb).parameters.values()
                    if p.default is inspect.Parameter.empty
                    and p.kind not in (inspect.Parameter.VAR_POSITIONAL,
                                       inspect.Parameter.VAR_KEYWORD)
                )
            except (ValueError, TypeError):
                n_params = 0
            if n_params == 1:
                sim = self
                new_args[1] = lambda fn=cb: fn(sim.get_time())
        result = super().run(*new_args, **kwargs)
        # Break all references from wrappers to user callbacks.
        # The C++ std::function still holds the wrappers alive, but now
        # wrapper._fn is None, so no Python objects are reachable from C++.
        for w in self._event_wrappers:
            w.clear()
        self._event_wrappers.clear()
        return result
