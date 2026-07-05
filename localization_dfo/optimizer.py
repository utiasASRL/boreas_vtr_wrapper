import numpy as np


class ObjectiveLogger:
    def __init__(self, func):
        self.func = func
        self.xs = []
        self.fs = []

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        f = float(self.func(x))
        self.xs.append(x.copy())
        self.fs.append(f)
        return f

    @property
    def history_x(self):
        return np.asarray(self.xs)

    @property
    def history_f(self):
        return np.asarray(self.fs)


def run_imfil_direct(method, objective, x0, bounds, budget):
    from skquant.opt import minimize

    out = minimize(objective, x0, bounds, budget, method=method)
    if isinstance(out, tuple):
        return out[0], out[1] if len(out) > 1 else None
    return out, None


def extract_best_from_result(result, logger):
    if hasattr(result, "x") and hasattr(result, "fun"):
        return np.asarray(result.x, dtype=float), float(result.fun)

    if isinstance(result, dict) and "x" in result:
        f = result.get("fun", result.get("fval", None))
        if f is not None:
            return np.asarray(result["x"], dtype=float), float(f)

    idx = int(np.argmin(logger.history_f))
    return logger.history_x[idx], float(logger.history_f[idx])
