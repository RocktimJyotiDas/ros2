"""Robotiq Hand-E over the Robotiq URCap's socket server (TCP port 63352 on the UR controller).

Built on the lab's client, ur3e_plan_runner.move_ur3e.Robotiq (github.com/alexhjkim/ur3e_experiments):
same connection, activation (ACT 0 / ATR 0 -> STA 0 -> ACT 1 -> STA 3), fault check (FLT) and
waits. Its move() uses fixed SPE 128 / FOR 128; probing needs a chosen grip force and a slow close,
so move() here takes both, and width() reads the finger opening.

Registers: POS (0 open .. 255 closed), SPE (0 slowest), FOR (0 = minimum force), OBJ (0 moving,
1 contact opening, 2 contact closing, 3 at position), PRE (echo of the requested POS).
Force: the Hand-E grips at 20-185 N and FOR maps roughly linearly onto that range, so nothing
below 20 N is possible (the simulation uses the same forces; see configs/real.yaml).
Width: width = w0 + slope * POS, calibrated with gauge blocks (`calibrate_gripper`); compression is
a width difference, so only the slope matters for it.
"""

import time

try:
    from ur3e_plan_runner.move_ur3e import Robotiq
except ImportError:  # the lab package isn't built: only MockHandE is usable (--mock / --mock_gripper)
    class Robotiq:
        def __init__(self, *a, **k):
            raise ImportError("ur3e_plan_runner (github.com/alexhjkim/ur3e_experiments) is required for the real Hand-E")

F_MIN, F_MAX = 20.0, 185.0  # Hand-E grip force range, N
STROKE_M = 0.050


def force_to_reg(force_n):
    if force_n < F_MIN - 1e-6 or force_n > F_MAX + 1e-6:
        raise ValueError(f"Hand-E force must be within {F_MIN}-{F_MAX} N, got {force_n}")
    return int(round((force_n - F_MIN) / (F_MAX - F_MIN) * 255))


class HandE(Robotiq):
    def __init__(self, host, activate=True, width_model=(STROKE_M, -STROKE_M / 255)):
        super().__init__(host, activate)
        self.w0, self.slope = width_model

    def width(self, samples=5):
        """Finger opening in metres, averaged over a few reads, and the raw POS values."""
        pos = [self.get("POS") for _ in range(samples)]
        return self.w0 + self.slope * (sum(pos) / len(pos)), pos

    def move(self, position, force_n=F_MIN, speed=0):
        fault = self.get("FLT")
        if fault:
            raise RuntimeError(f"Robotiq fault {fault}")
        self.set(POS=position, SPE=speed, FOR=force_to_reg(force_n), GTO=1)
        self.wait(lambda: self.get("PRE") == position)
        self.wait(lambda: self.get("OBJ") != 0)
        return self.get("OBJ")

    def open(self, force_n=F_MIN, speed=128):
        return self.move(0, force_n, speed)

    def close_on(self, force_n, speed=0):
        """Close until contact (OBJ 2) or fully closed (OBJ 3) at `force_n`."""
        return self.move(255, force_n, speed)

    def shutdown(self):
        self.socket.close()  # Robotiq.close() closes the socket; close_on() is the gripper motion


class MockHandE:
    """Stands in for the gripper with --mock / --mock_gripper: a rigid 30 mm object."""

    def __init__(self, *a, **k):
        self._w = STROKE_M

    def get(self, name):
        return round((STROKE_M - self._w) / STROKE_M * 255) if name == "POS" else 0

    def width(self, samples=5):
        return self._w, [self.get("POS")] * samples

    def open(self, force_n=F_MIN, speed=128):
        self._w = STROKE_M
        return 3

    def close_on(self, force_n, speed=0):
        self._w = 0.030
        time.sleep(0.01)
        return 2

    def shutdown(self):
        pass
