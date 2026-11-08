"""Bounded Levenberg-Marquardt for small problems (numpy only)."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Fit:
    x: np.ndarray
    rms: float                 # residual RMS at the solution
    std: np.ndarray            # 1-sigma of x from the Jacobian, scaled by the residual
    iterations: int


def least_squares(residual: Callable[[np.ndarray], np.ndarray], x0, lower=None, upper=None,
                  step: float | np.ndarray = 1e-4, max_iter: int = 100, tol: float = 1e-10) -> Fit:
    """Bounded Levenberg-Marquardt with a forward-difference Jacobian; ``step``
    per parameter or one for all."""
    x = np.asarray(x0, dtype=float).copy()
    steps = np.broadcast_to(np.asarray(step, dtype=float), x.shape)
    lo = np.full_like(x, -np.inf) if lower is None else np.asarray(lower, float)
    hi = np.full_like(x, np.inf) if upper is None else np.asarray(upper, float)

    def jac(x, r):
        J = np.empty((r.size, x.size))
        for i in range(x.size):
            d = np.zeros_like(x)
            d[i] = steps[i]
            J[:, i] = (residual(x + d) - r) / steps[i]
        return J

    r = residual(x)
    cost, lam, it = r @ r, 1e-3, 0
    for it in range(1, max_iter + 1):
        J = jac(x, r)
        A, g = J.T @ J, J.T @ r
        while True:
            dx = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            x_new = np.clip(x + dx, lo, hi)
            r_new = residual(x_new)
            if r_new @ r_new < cost:
                break
            lam *= 10
            if lam > 1e12:
                break
        if lam > 1e12:
            break
        improved = cost - r_new @ r_new
        x, r, cost, lam = x_new, r_new, r_new @ r_new, max(lam / 10, 1e-12)
        if improved < tol * max(cost, 1e-30) or np.abs(dx).max() < 1e-12:
            break
    J = jac(x, r)
    dof = max(r.size - x.size, 1)
    cov = np.linalg.pinv(J.T @ J) * (cost / dof)
    return Fit(x, float(np.sqrt(cost / r.size)), np.sqrt(np.diag(cov)), it)
