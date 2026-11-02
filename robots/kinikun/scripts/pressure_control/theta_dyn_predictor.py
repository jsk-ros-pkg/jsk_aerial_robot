# auto-generated dynamic model
import numpy as np
coef = np.array([0.07012353258384375, 2.903999395088137, 0.5685600068092452, -0.9077077990312703, -0.6038315749134802, -1.9607455663072952, -1.139246351631749, 1.3318991699281935, 0.03255609251330523, -0.7032055885699282], dtype=float)
deg = 3
tau0 = 0.839882
tau1 = -0.434217

def f_static(ps, pd):
    ps = np.asarray(ps); pd = np.asarray(pd)
    feats=[]
    for i in range(deg+1):
        for j in range(deg+1-i):
            feats.append((ps**i)*(pd**j))
    X = np.column_stack(feats) if (np.ndim(ps)>0 or np.ndim(pd)>0) else np.array(feats, float).reshape(1,-1)
    return (X @ coef)

def step_one(theta_prev, ps_k, pd_k, dt_k):
    tau = max(1e-3, tau0 + tau1*float(ps_k))
    f = float(f_static(ps_k, pd_k))
    return theta_prev + (dt_k/tau)*(f - theta_prev)

def simulate(theta0, ps, pd, dt):
    ps=np.asarray(ps); pd=np.asarray(pd); dt=np.asarray(dt)
    th=np.empty_like(ps, dtype=float); x=float(theta0)
    for k in range(ps.size):
        x = step_one(x, ps[k], pd[k], dt[k])
        th[k]=x
    return th
