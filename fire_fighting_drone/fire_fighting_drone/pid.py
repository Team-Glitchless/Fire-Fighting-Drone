"""A small vectorised PID controller.

This is a bug-fixed, framework-agnostic port of the original ROS 1
``pid.py``. The original mixed scalar initial state (``self.intError =
0.0``) with vector usage (``update(target, state)`` is always called with
3-element numpy arrays by the trajectory follower), which raised
``TypeError``/``IndexError`` the first time clamping was attempted. It also
never actually updated ``lastError`` (``self.lastError = self.lastError`` is
a no-op), so the derivative term was always computed against a stale value.

Both issues are fixed here. The controller no longer depends on ``rospy``;
callers pass in the current time (seconds, monotonic) so this module works
unchanged under ROS 1, ROS 2, or plain Python.
"""
import time

import numpy as np


class PID:
    def __init__(self, Kp=0.5, Ki=0.03, Kd=0.05, maxI=10, maxOut=0.5, dim=3):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.dim = dim
        self.maxI = maxI
        self.maxOut = maxOut
        self.reset()
        self.lastTime = time.time()

    def update(self, target, state, now=None):
        """Compute the PID output for the given target/state vectors.

        Args:
            target: array-like, desired state (e.g. target position).
            state: array-like, current state (e.g. current position).
            now: optional current time in seconds; defaults to ``time.time()``
                so this class works without a ROS clock.
        """
        target = np.asarray(target, dtype=float)
        state = np.asarray(state, dtype=float)
        current_time = time.time() if now is None else now

        self.target = target
        self.state = state
        self.error = self.target - self.state

        dTime = current_time - self.lastTime
        dError = self.error - self.lastError

        p = self.error
        self.intError = self.intError + self.error * dTime
        if dTime > 0:
            d = dError / dTime
        else:
            d = np.zeros_like(self.error)

        i = np.copy(self.intError)
        if self.maxI is not None:
            i = np.clip(i, -self.maxI, self.maxI)
            self.intError = i

        # Remember last time/error for the next call.
        self.lastTime = current_time
        self.lastError = self.error

        output = self.Kp * p + self.Ki * i + self.Kd * d
        if self.maxOut is not None:
            output = np.clip(output, -self.maxOut, self.maxOut)

        self.output = output
        return output

    def setKp(self, Kp):
        self.Kp = Kp

    def setKi(self, Ki):
        self.Ki = Ki

    def setKd(self, Kd):
        self.Kd = Kd

    def setMaxI(self, maxI):
        self.maxI = maxI

    def reset(self):
        self.target = np.zeros(self.dim)
        self.error = np.zeros(self.dim)
        self.state = np.zeros(self.dim)
        self.intError = np.zeros(self.dim)
        self.lastError = np.zeros(self.dim)
        self.output = np.zeros(self.dim)
