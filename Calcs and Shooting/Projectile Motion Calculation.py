import math

G = 9.81

class OutOfRange(Exception):
    pass

def miss(beta_deg, x_f, y_f, u, L, x_t, y_t, y_0):
    b = math.radians(beta_deg)
    dx = x_f - (x_t + L * math.cos(b))
    t = dx / (u * math.cos(b))
    y = y_0 + y_t + L * math.sin(b) + u * math.sin(b) * t - 0.5 * G * t ** 2
    return y - y_f

def bisect(lo, hi, f, tol=1e-4):
    while hi - lo > tol:
        mid = (lo + hi) / 2
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2

def launch_angle(x_f, y_f, u, L, x_t, y_t, y_0, prefer="low", beta_min=-10, beta_max=58, step=0.1):
    if u <= 0 or not math.isfinite(x_f) or not math.isfinite(y_f):
        raise ValueError("bad input")
    if x_f <= x_t + L:
        raise OutOfRange("fire is too close to the turret")

    def f(beta):
        return miss(beta, x_f, y_f, u, L, x_t, y_t, y_0)

    roots = []
    for i in range(round((beta_max - beta_min) / step)):
        lo = beta_min + i * step
        hi = lo + step
        if f(lo) * f(hi) <= 0:
            roots.append(bisect(lo, hi, f))

    if not roots:
        raise OutOfRange("no launch angle reaches the fire")
    if prefer == "low":
        return roots[0]
    return roots[-1]