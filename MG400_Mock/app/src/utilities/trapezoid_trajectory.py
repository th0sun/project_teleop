import numpy as np
import math

def cal_trapezoid_time(pos_init, pos_target, acc, v_l, v_s=0.):
    dist = abs(pos_target - pos_init)
    if dist < 1e-6:
        return 0, 0, 0
        
    v_s = abs(v_s)
    v_s = min(v_s, v_l)
    
    # Prevent overshoot in simulation by capping initial velocity
    v_s_max_allowed = math.sqrt(2 * acc * dist)
    if v_s > v_s_max_allowed:
        v_s = v_s_max_allowed
        
    d_acc = (v_l**2 - v_s**2) / (2 * acc) if v_l > v_s else 0
    d_dec = (v_l**2) / (2 * acc)
    
    if d_acc + d_dec <= dist:
        t_acc = (v_l - v_s) / acc if v_l > v_s else 0
        t_dec = v_l / acc
        t_const = (dist - d_acc - d_dec) / v_l
        return t_acc, t_const, t_dec
    else:
        v_peak = math.sqrt(acc * dist + (v_s**2) / 2.0)
        t_acc = (v_peak - v_s) / acc if v_peak > v_s else 0
        t_dec = v_peak / acc
        return t_acc, 0, t_dec

def gene_trapezoid_traj(pos_init, pos_target, acc, v_l, timestep, v_s=0.):
    dist = abs(pos_target - pos_init)
    if dist < 1e-6:
        return np.array([pos_target])
        
    t_acc, t_const, t_dec = cal_trapezoid_time(pos_init, pos_target, acc, v_l, v_s)
    sign = np.sign(pos_target - pos_init)
    
    v_s = abs(v_s)
    v_s = min(v_s, v_l)
    v_s_max_allowed = math.sqrt(2 * acc * dist)
    if v_s > v_s_max_allowed:
        v_s = v_s_max_allowed
        
    traj = []
    
    if t_acc > 0:
        n_acc = int(math.ceil(t_acc / timestep))
        for i in range(1, n_acc + 1):
            t = i * timestep
            if t > t_acc: t = t_acc
            traj.append(pos_init + sign * (v_s * t + 0.5 * acc * t**2))
        pos_after_acc = pos_init + sign * (v_s * t_acc + 0.5 * acc * t_acc**2)
        v_peak = v_s + acc * t_acc
    else:
        pos_after_acc = pos_init
        v_peak = v_s

    pos_after_const = pos_after_acc
    if t_const > 0:
        n_const = int(math.ceil(t_const / timestep))
        for i in range(1, n_const + 1):
            t = i * timestep
            if t > t_const: t = t_const
            traj.append(pos_after_acc + sign * (v_peak * t))
        pos_after_const = pos_after_acc + sign * (v_peak * t_const)
        
    if t_dec > 0:
        n_dec = int(math.ceil(t_dec / timestep))
        for i in range(1, n_dec + 1):
            t = i * timestep
            if t > t_dec: t = t_dec
            traj.append(pos_after_const + sign * (v_peak * t - 0.5 * acc * t**2))
            
    if traj:
        traj[-1] = pos_target
    else:
        traj.append(pos_target)
        
    return np.array(traj)
