#!/usr/bin/env python3
"""Separate speed-cap effects and geometric reach using a recorded base trace.

Replay assumes ideal joint tracking and holds the recorded base trajectory fixed;
it is not a new Gazebo run and does not model motor load/torque-speed curves.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_world_core import ArmKinematics, transform


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('result',type=Path)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    out=args.output or args.result/'limit_analysis'; out.mkdir(parents=True,exist_ok=True)
    summary=json.loads((args.result/'summary.json').read_text())
    target=np.array(summary['target_world'])
    rows=[row for row in csv.DictReader((args.result/'samples.csv').open()) if row['trial']=='world_hold']
    time=np.array([float(r['time_s']) for r in rows])
    measured_q=np.array([[float(r['q%d'%i]) for i in range(1,5)] for r in rows])
    bases=np.array([transform([float(r['base_'+c]) for c in 'xyz'],Rotation.from_rotvec([float(r['base_r'+c]) for c in 'xyz']).as_matrix()) for r in rows])
    model=Path(__file__).resolve().parents[1]/'robots/grape_with_arm/gimbalrotor.urdf.xacro'
    xml=subprocess.check_output(['xacro',str(model)],text=True)
    k=ArmKinematics(xml)
    home=np.array(summary['configuration']['home'])
    desired=np.array([np.linalg.inv(b)@target for b in bases])
    values=dict(zip(k.names,np.zeros(4)))
    root_to_fc=k.evaluate(k.base_chain,values)
    anchors=np.array([(np.linalg.inv(root_to_fc)@k.evaluate(k.chain('arm_joint%d_link1'%i),values))[:3,3] for i in range(1,5)])
    # Triangle inequality gives a rigorous (possibly loose) outer reach bound.
    radius=float(np.sum(np.linalg.norm(np.diff(anchors,axis=0),axis=1))+np.linalg.norm(k.fk(np.zeros(4))[:3,3]-anchors[-1]))
    radius_excess=np.maximum(0.,np.linalg.norm(desired[:,:3,3]-anchors[0],axis=1)-radius)
    result=dict(source=str(args.result),assumption='Ideal servo, fixed recorded base trace; no torque/load dynamics',
                velocity_urdf_rad_s=k.urdf_velocity.tolist(), outer_reach_radius_m=radius,
                definitely_outside_outer_reach_fraction=float(np.mean(radius_excess>.001)),
                outer_reach_excess_max_m=float(radius_excess.max()), replay={})
    replay_errors={}
    for name,cap in [('cap_0_6',.6),('cap_2_0',2.),('catalog_12v',None)]:
        k.velocity=k.urdf_velocity.copy() if cap is None else np.minimum(cap,k.urdf_velocity)
        q=measured_q[0].copy(); errors=[]; limited=[]; clipped=[]; singular=[]
        for i,(t,base) in enumerate(zip(time,bases)):
            dt=float(np.clip(t-time[i-1] if i else 1/30,.001,.05))
            q,sv,sat=k.step(q,base,target,dt,home)
            errors.append(np.linalg.norm((base@k.fk(q))[:3,3]-target[:3,3]))
            limited.append(k.last_diagnostics['velocity_limited'])
            clipped.append(k.last_diagnostics['joint_limit_clipped']); singular.append(sv[-1])
        errors=np.array(errors); replay_errors[name]=errors
        result['replay'][name]=dict(position_rms_m=float(np.sqrt(np.mean(errors**2))),position_max_m=float(errors.max()),
                                   velocity_limited_fraction=float(np.mean(limited)),joint_limit_clipped_fraction=float(np.mean(clipped)),
                                   minimum_position_singular_value=float(np.min(singular)))
        print(name,result['replay'][name],flush=True)
    # Position-only inverse kinematics removes time/rate and orientation objectives.
    # Multiple initial guesses reduce local-minimum risk; it remains a numerical
    # best-found residual, distinct from the rigorous outer-radius test above.
    selected=list(range(0,len(time),4)); best_errors=[]; solutions=[]; previous=home.copy()
    rng=np.random.default_rng(7)
    for n,i in enumerate(selected):
        p_desired=desired[i,:3,3]
        def residual(q): return k.fk(q)[:3,3]-p_desired
        seeds=[previous,home,measured_q[i]]+[rng.uniform(k.lower,k.upper) for unused in range(2)]
        best=None
        for seed in seeds:
            fit=least_squares(residual,np.clip(seed,k.lower+1e-8,k.upper-1e-8),bounds=(k.lower,k.upper),
                              max_nfev=80,ftol=1e-9,xtol=1e-9,gtol=1e-9)
            if best is None or np.linalg.norm(fit.fun)<np.linalg.norm(best.fun): best=fit
            if np.linalg.norm(best.fun)<1e-5: break
        previous=best.x; solutions.append(best.x.tolist()); best_errors.append(float(np.linalg.norm(best.fun)))
        if n%20==0: print('Position-only IK',n,'/',len(selected),flush=True)
    best_errors=np.array(best_errors)
    result['position_only_snapshot_ik']=dict(samples=len(selected),rms_residual_m=float(np.sqrt(np.mean(best_errors**2))),
            max_residual_m=float(best_errors.max()),fraction_residual_over_1cm=float(np.mean(best_errors>.01)),
            caveat='Multistart numerical best found, not a proof of global optimality; no velocity/orientation requirement')
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(out/'curves.npz',time=time,outer_excess=radius_excess,snapshot_time=time[selected],
                        snapshot_error=best_errors,snapshot_q=solutions,**replay_errors)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,5))
    actual=np.array([float(row['position_error_m']) for row in rows])
    ax.plot(time,actual*1000,label='Recorded Gazebo TCP',color='black',lw=2)
    for name,e in replay_errors.items(): ax.plot(time,e*1000,label='Ideal servo replay: '+name)
    ax.plot(time[selected],best_errors*1000,'o-',ms=3,label='Position-only snapshot IK (no rate limit)')
    ax.fill_between(time,0,radius_excess*1000,alpha=.2,label='Guaranteed reach-bound excess')
    ax.set(xlabel='time [s]',ylabel='TCP position error [mm]',title='Same recorded base motion: speed versus reach')
    ax.grid(True); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(out/'limits.png'); plt.close(fig)
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    main()
