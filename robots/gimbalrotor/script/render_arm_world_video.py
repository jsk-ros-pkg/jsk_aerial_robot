#!/usr/bin/env python3
"""Render recorded Gazebo evaluation as a side-by-side 3-D schematic MP4.

TCP position/orientation uses the independent Gazebo truth logged as errors;
robot/arm geometry is reconstructed from base observation, measured q and URDF.
This is a data visualization, not a recording of the Gazebo GUI.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from scipy.spatial.transform import Rotation
from arm_world_core import ArmKinematics, transform


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('result',type=Path)
    p.add_argument('--urdf',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--fps',type=int,default=24)
    args=p.parse_args()
    summary=json.loads((args.result/'summary.json').read_text())
    cfg=summary['configuration']
    if args.urdf:
        xml=args.urdf.read_text()
    else:
        model=Path(__file__).resolve().parents[1]/'robots/grape_with_arm/gimbalrotor.urdf.xacro'
        xml=subprocess.check_output(['xacro',str(model)],text=True)
    k=ArmKinematics(xml,cfg['base_link'],cfg['tip_link'],cfg['tcp'])
    rows=list(csv.DictReader((args.result/'samples.csv').open()))
    fields=[n for n in rows[0] if n!='trial']
    data={trial:np.array([[float(row[n]) for n in fields] for row in rows if row['trial']==trial])
          for trial in ['fixed_joints','world_hold']}
    column={n:i for i,n in enumerate(fields)}
    target=np.array(summary['target_world'])
    origin=target[:3,3]
    fixed_base=np.linalg.inv(k.evaluate(k.base_chain,{}))
    link_chains=[k.chain('arm_joint%d_link1'%i) for i in range(1,5)]

    plt.rcParams.update({'font.family':'DejaVu Sans','text.color':'#e6edf5','axes.labelcolor':'#b9c8db',
                         'xtick.color':'#9aacc4','ytick.color':'#9aacc4','axes.edgecolor':'#3d4c62',
                         'font.size':11,'axes.facecolor':'#111c2d','figure.facecolor':'#09111e'})
    fig=plt.figure(figsize=(16,9),dpi=100)
    grid=fig.add_gridspec(3,2,height_ratios=[.12,1,.32],hspace=.18,wspace=.12)
    title=fig.text(.5,.955,'WORLD-FIXED ARM TIP  |  GAZEBO EVALUATION',ha='center',fontsize=22,weight='bold')
    fig.text(.5,.916,'take9 mocap waveform at %g%% scale  /  measured simulation data  /  1x playback'%(cfg['scale']*100),ha='center',color='#9fb3cd',fontsize=12)
    axes=[fig.add_subplot(grid[1,i],projection='3d') for i in range(2)]
    plots=[fig.add_subplot(grid[2,i]) for i in range(2)]
    colors=['#f8b76c','#4ee1bd']
    labels=['FIXED ARM JOINTS','WORLD-TIP COMPENSATION']
    artists=[]
    theta=np.linspace(0,2*np.pi,28)
    for i,(ax,trial) in enumerate(zip(axes,data)):
        ax.set_facecolor('#09111e')
        ax.set(xlim=(-.7,.22),ylim=(-.4,.4),zlim=(-.3,.38))
        ax.set_box_aspect([.92,.8,.68]); ax.view_init(elev=23,azim=-58)
        ax.set_xlabel('world X - target X [m]',labelpad=7); ax.set_ylabel('world Y - target Y [m]',labelpad=7); ax.set_zlabel('world Z - target Z [m]',labelpad=7)
        ax.tick_params(labelsize=8)
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
            axis.set_pane_color((.06,.10,.17,1))
            axis._axinfo['grid']['color']=(.2,.27,.36,.35)
        ax.set_title(labels[i],color=colors[i],fontsize=16,pad=8,weight='bold')
        body,=ax.plot([],[],[],color='#89a9cc',lw=3)
        rings=[ax.plot([],[],[],color='#89a9cc',lw=1.2,alpha=.8)[0] for unused in range(4)]
        arm,=ax.plot([],[],[],color=colors[i],lw=4,marker='o',markersize=5)
        trail,=ax.plot([],[],[],color=colors[i],lw=1.5,alpha=.6)
        actual,=ax.plot([],[],[],marker='o',color=colors[i],markersize=8)
        ax.plot([0],[0],[0],marker='+',color='#ff6888',ms=15,mew=2)
        target_axes=[]; tip_axes=[]
        for j,c in enumerate(['#ff6888','#7fdf8c','#78abff']):
            end=target[:3,j]*.065
            target_axes.append(ax.plot([0,end[0]],[0,end[1]],[0,end[2]],color=c,lw=1.2,ls='--',alpha=.65)[0])
            tip_axes.append(ax.plot([],[],[],color=c,lw=2)[0])
        text=ax.text2D(.02,.89,'',transform=ax.transAxes,fontsize=12,color=colors[i])
        artists.append((body,rings,arm,trail,actual,tip_axes,text))
        a=data[trial]; timecol=a[:,column['time_s']]
        plots[0].plot(timecol,a[:,column['position_error_m']]*1000,color=colors[i],lw=1.8,label=labels[i])
        plots[1].plot(timecol,np.rad2deg(a[:,column['orientation_error_rad']]),color=colors[i],lw=1.8,label=labels[i])
    max_time=min(a[-1,column['time_s']] for a in data.values())
    cursors=[]
    for ax,label in zip(plots,['TCP position error [mm]','TCP orientation error [deg]']):
        ax.set_xlim(0,max_time); ax.set_ylim(bottom=0)
        ax.set_xlabel('trial time [s]'); ax.set_ylabel(label); ax.grid(True,color='#354259',alpha=.5)
        ax.legend(loc='upper left',fontsize=8,facecolor='#111c2d',edgecolor='#354259',labelcolor='#dce7f5')
        cursors.append(ax.axvline(0,color='#e7eef8',lw=1,alpha=.7))
    fig.subplots_adjust(top=.9,bottom=.17,left=.055,right=.96)
    footer=fig.text(.5,.026,'Log reconstruction, not a Gazebo GUI recording.  Pink +: fixed world target.  Colored dot: actual Gazebo TCP.',
                    ha='center',fontsize=10,color='#91a7c1')
    clock=fig.text(.5,.075,'',ha='center',fontsize=13,color='#e6edf5')

    def line3(line,points):
        line.set_data(points[:,0],points[:,1]); line.set_3d_properties(points[:,2])

    def frame(t):
        clock.set_text('t = %.2f s    |    base and arm: measured-state schematic'%t)
        for idx,trial in enumerate(data):
            a=data[trial]
            row=np.array([np.interp(t,a[:,column['time_s']],a[:,j]) for j in range(a.shape[1])])
            q=np.array([row[column['q%d'%j]] for j in range(1,5)])
            base=transform([row[column['base_'+c]] for c in 'xyz'],Rotation.from_rotvec([row[column['base_r'+c]] for c in 'xyz']).as_matrix())
            values=dict(zip(k.names,q))
            root=base@fixed_base
            points=np.array([base[:3,3]]+[(root@k.evaluate(chain,values))[:3,3] for chain in link_chains]+[(base@k.fk(q))[:3,3]])-origin
            body,rings,arm,trail,actual,tip_axes,text=artists[idx]
            corners=np.array([[.17,.17,0],[-.17,.17,0],[-.17,-.17,0],[.17,-.17,0]])
            line3(body,(base[:3,:3]@corners[[0,2,1,3,0]].T).T+base[:3,3]-origin)
            for ring,center in zip(rings,corners):
                local=center+np.column_stack([.065*np.cos(theta),.065*np.sin(theta),np.zeros(len(theta))])
                line3(ring,(base[:3,:3]@local.T).T+base[:3,3]-origin)
            line3(arm,points)
            ep=np.array([row[column['e'+c+'_m']] for c in 'xyz'])
            er=np.array([row[column['er'+c+'_rad']] for c in 'xyz'])
            line3(actual,ep.reshape(1,3))
            recent=a[(a[:,column['time_s']]<=t)&(a[:,column['time_s']]>=t-2)]
            line3(trail,recent[:,[column['e'+c+'_m'] for c in 'xyz']])
            rot=Rotation.from_rotvec(er).as_matrix()@target[:3,:3]
            for j,line in enumerate(tip_axes):
                line3(line,np.array([ep,ep+rot[:,j]*.065]))
            text.set_text('position  %5.1f mm\norientation  %4.2f deg'%(row[column['position_error_m']]*1000,np.rad2deg(row[column['orientation_error_rad']])))
        for cursor in cursors:
            cursor.set_xdata([t,t])

    output=args.output or args.result/'arm_world_comparison.mp4'
    output.parent.mkdir(parents=True,exist_ok=True)
    writer=FFMpegWriter(fps=args.fps,codec='libx264',bitrate=2500,extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
    total=int(max_time*args.fps)
    with writer.saving(fig,str(output),dpi=100):
        frame(0)
        for unused in range(args.fps): writer.grab_frame()
        for n in range(total):
            frame(n/args.fps); writer.grab_frame()
            if n%args.fps==0: print('Rendered %d / %d seconds'%(n//args.fps,int(max_time)),flush=True)
        frame(max_time)
        b=summary['trials']['fixed_joints']; c=summary['trials']['world_hold']
        clock.set_text('POSITION RMS: %.1f -> %.1f mm   |   ORIENTATION RMS: %.2f -> %.2f deg'%(b['position_rms_m']*1000,c['position_rms_m']*1000,b['orientation_rms_deg'],c['orientation_rms_deg']))
        footer.set_text('Position-first control with 4 arm joints: full 6-DoF pose locking is not achieved.')
        for unused in range(args.fps*2): writer.grab_frame()
    fig.savefig(output.with_suffix('.png'),dpi=100)
    print(output,flush=True)


if __name__=='__main__':
    main()
