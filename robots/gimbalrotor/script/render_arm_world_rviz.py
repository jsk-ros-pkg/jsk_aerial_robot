#!/usr/bin/env python3
"""Record native RViz RobotModel rendering of the measured Gazebo trajectories.

Run on a dedicated Xvfb display and a private ROS_MASTER_URI. Meshes/materials
come directly from grape_with_arm's URDF. Gimbal and gripper angles were not in
the evaluation CSV, so these joints are displayed at zero (not fabricated data).
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDockWidget, QToolBar
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QImage
from rviz import bindings as rviz
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_world_core import ArmKinematics, transform


def add_captions(path, summary, chapter_duration):
    """Composite labels after capture; native GL windows may obscure Qt labels."""
    source=path.with_name(path.stem+'_native.mp4')
    path.replace(source)
    font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    bold='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
    filters=['drawbox=x=0:y=0:w=1600:h=104:color=0x101824:t=fill',
             'drawbox=x=0:y=788:w=1600:h=112:color=0x101824:t=fill']
    def label(text,x,y,size=18,color='white',enable=None,strong=False):
        # Generated labels contain no user-controlled filter expressions.
        text=text.replace(':','\\:')
        f="drawtext=fontfile=%s:text='%s':expansion=none:x=%s:y=%s:fontsize=%d:fontcolor=%s"%(bold if strong else font,text,x,y,size,color)
        if enable: f+=":enable='%s'"%enable
        filters.append(f)
    label('ARM TIP WORLD HOLD  |  RViz / actual URDF meshes','(w-text_w)/2',14,25,strong=True)
    label('Measured Gazebo trajectories | take9 sway at %g%% | pink cross = fixed world target'%(summary['configuration']['scale']*100),'(w-text_w)/2',51,16,'0xa5bad2')
    labels=summary.get('video_labels',['FIXED ARM JOINTS','WORLD-TIP COMPENSATION'])
    label(labels[0],'(800-text_w)/2',77,21,'0xffbc73',strong=True)
    label(labels[1],'800+(800-text_w)/2',77,21,'0x50dfbd',strong=True)
    for i,(key,color) in enumerate([('fixed_joints','0xffbc73'),('world_hold','0x50dfbd')]):
        trial=summary['trials'][key]
        text='Position RMS: %.1f mm   |   Orientation RMS: %.2f deg'%(trial['position_rms_m']*1000,trial['orientation_rms_deg'])
        label(text,str(i*800)+'+(800-text_w)/2',810,18,color)
    label('FULL ROBOT','(w-text_w)/2',847,18,enable='lt(t,%g)'%chapter_duration)
    label('ARM CLOSE-UP','(w-text_w)/2',847,18,enable='between(t,%g,%g)'%(chapter_duration,chapter_duration*2))
    label('COMPARISON COMPLETE','(w-text_w)/2',847,18,enable='gte(t,%g)'%(chapter_duration*2))
    label('Gazebo log replay in RViz. Unlogged gimbals/gripper shown neutral. Position priority; full pose lock is not achieved.',
          '(w-text_w)/2',882,12,'0x91a6bf')
    subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(source),'-vf',','.join(filters),'-an',
                    '-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--fps',type=int,default=20)
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--caption-only',action='store_true')
    parser.add_argument('--compare-result',type=Path,help='Compare world_hold in two runs, instead of fixed versus compensated')
    args=parser.parse_args()
    output=args.output or args.result/'arm_world_rviz.mp4'
    summary=json.loads((args.result/'summary.json').read_text())
    target=np.array(summary['target_world'])
    rows=list(csv.DictReader((args.result/'samples.csv').open()))
    fields=[f for f in rows[0] if f!='trial']; col={f:i for i,f in enumerate(fields)}
    data={key:np.array([[float(row[f]) for f in fields] for row in rows if row['trial']==key])
          for key in ['fixed_joints','world_hold']}
    targets=[target,target]
    if args.compare_result:
        other=json.loads((args.compare_result/'summary.json').read_text())
        other_rows=list(csv.DictReader((args.compare_result/'samples.csv').open()))
        data={'fixed_joints':data['world_hold'],
              'world_hold':np.array([[float(row[f]) for f in fields] for row in other_rows if row['trial']=='world_hold'])}
        targets=[target,np.array(other['target_world'])]
        summary['trials']={'fixed_joints':summary['trials']['world_hold'],'world_hold':other['trials']['world_hold']}
        summary['video_labels']=['EARLIER 0.6 RAD/S CAP','XL430-W250 / 12 V LIMITS']
        summary['comparison_sources']=[str(args.result),str(args.compare_result)]
        if args.output is None: output=args.compare_result/'arm_world_rviz_speed_comparison.mp4'
    (output.with_suffix('.json')).write_text(json.dumps(dict(sources=summary.get('comparison_sources',[str(args.result)]),
            labels=summary.get('video_labels',['FIXED ARM JOINTS','WORLD-TIP COMPENSATION']),
            trials=summary['trials'],scale=summary['configuration']['scale'],fps=args.fps,
            caveat=('Separate Gazebo runs; base trajectories differ slightly. Per-run fixed targets are translated to display origin.'
                    if args.compare_result else 'Two trials in one Gazebo run share a fixed world target; base trajectories differ slightly.')),
            indent=2)+'\n')
    chapter_duration=int(min(a[-1,col['time_s']] for a in data.values())*args.fps)/args.fps
    if args.caption_only:
        add_captions(output,summary,chapter_duration)
        return
    model=Path(__file__).resolve().parents[1]/'robots/grape_with_arm/gimbalrotor.urdf.xacro'
    xml=subprocess.check_output(['xacro',str(model)],text=True)
    kin=ArmKinematics(xml)
    root_to_fc=kin.evaluate(kin.base_chain,{})
    description=ET.fromstring(xml)
    joints=[]
    for joint in description.findall('joint'):
        origin=joint.find('origin'); axis=joint.find('axis'); mimic=joint.find('mimic')
        xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
        rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
        joints.append((joint.get('name'),joint.get('type'),joint.find('parent').get('link'),joint.find('child').get('link'),
                       transform(xyz,Rotation.from_euler('xyz',rpy).as_matrix()),
                       np.fromstring(axis.get('xyz','1 0 0'),sep=' ') if axis is not None else np.array([1.,0,0]),
                       (mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0'))) if mimic is not None else None))
    app=QApplication([])
    master=None; encoder=None; success=False
    log=(output.parent/'rviz_render.log').open('w')
    try:
        if rosgraph.is_master_online():
            raise RuntimeError('Use a private, unused ROS_MASTER_URI for rendering')
        port=os.environ['ROS_MASTER_URI'].rsplit(':',1)[1].rstrip('/')
        master=subprocess.Popen(['roscore','-p',port],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline: raise RuntimeError('Private ROS master did not start')
            time.sleep(.1)
        rospy.init_node('arm_world_rviz_replay',disable_signals=True)
        rospy.set_param('/rviz_replay/robot_description',xml)
        tf_pub=tf2_ros.TransformBroadcaster()
        marker_pubs=[rospy.Publisher('/rviz_replay/'+prefix+'/markers',MarkerArray,queue_size=1,latch=True) for prefix in ['fixed','hold']]
        window=QWidget(); window.setWindowTitle('Arm world hold | native RViz renderer')
        window.setFixedSize(1600,900)
        window.setStyleSheet('QWidget {background:#101824;color:#e2eaf4;} QLabel {font-family:DejaVu Sans;}')
        layout=QVBoxLayout(window); layout.setContentsMargins(16,12,16,10); layout.setSpacing(6)
        heading=QLabel('ARM TIP WORLD HOLD  |  RViz / actual URDF meshes'); heading.setAlignment(Qt.AlignCenter)
        heading.setStyleSheet('font-size:25px;font-weight:bold;'); layout.addWidget(heading)
        sub=QLabel('Measured Gazebo trajectories  |  take9 sway at %g%%  |  pink cross = fixed world target'%(summary['configuration']['scale']*100))
        sub.setStyleSheet('font-size:16px;color:#a5bad2'); sub.setAlignment(Qt.AlignCenter); layout.addWidget(sub)
        row_layout=QHBoxLayout(); layout.addLayout(row_layout,1)
        managers=[]; panels=[]; infos=[]
        labels=summary.get('video_labels',['FIXED ARM JOINTS','WORLD-TIP COMPENSATION'])
        for prefix,label,color in zip(['fixed','hold'],labels,['#ffbc73','#50dfbd']):
            column=QVBoxLayout(); row_layout.addLayout(column,1)
            name=QLabel(label); name.setAlignment(Qt.AlignCenter); name.setStyleSheet('font-size:21px;font-weight:bold;color:'+color)
            column.addWidget(name)
            panel=rviz.VisualizationFrame(); panel.setSplashPath(''); panel.initialize()
            panel.setMenuBar(None); panel.setStatusBar(None); panel.setHideButtonVisibility(False)
            for dock in panel.findChildren(QDockWidget): dock.hide()
            for toolbar in panel.findChildren(QToolBar): toolbar.hide()
            column.addWidget(panel,1)
            manager=panel.getManager(); manager.setFixedFrame('world')
            initial=manager.getRootDisplayGroup().getDisplayAt(0)
            if initial is not None: initial.setEnabled(False)
            grid=manager.createDisplay('rviz/Grid','World grid',True)
            grid.subProp('Cell Size').setValue(.1); grid.subProp('Plane Cell Count').setValue(14)
            grid.subProp('Offset').subProp('Z').setValue(-.3)
            grid.subProp('Alpha').setValue(.3)
            robot=manager.createDisplay('rviz/RobotModel','Actual grape_with_arm',True)
            robot.subProp('Robot Description').setValue('/rviz_replay/robot_description')
            robot.subProp('TF Prefix').setValue(prefix)
            robot.subProp('Alpha').setValue(1.)
            markers=manager.createDisplay('rviz/MarkerArray','World target and measured TCP',True)
            markers.subProp('Marker Topic').setValue('/rviz_replay/'+prefix+'/markers')
            manager.getViewManager().setCurrentViewControllerType('rviz/Orbit')
            view=manager.getViewManager().getCurrent()
            view.subProp('Distance').setValue(2.15)
            view.subProp('Yaw').setValue(4.3); view.subProp('Pitch').setValue(.4)
            view.subProp('Focal Point').subProp('X').setValue(-.25)
            view.subProp('Focal Point').subProp('Z').setValue(.03)
            info=QLabel('Loading model...'); info.setAlignment(Qt.AlignCenter)
            info.setStyleSheet('font-size:19px;color:'+color); column.addWidget(info)
            managers.append(manager); panels.append(panel); infos.append(info)
        clock=QLabel(''); clock.setAlignment(Qt.AlignCenter); clock.setStyleSheet('font-size:18px;'); layout.addWidget(clock)
        foot=QLabel('Native RViz rendering of simulation logs. Unlogged gimbals/gripper shown neutral. Position priority; full pose locking is not achieved.')
        foot.setAlignment(Qt.AlignCenter); foot.setStyleSheet('font-size:12px;color:#91a6bf;'); layout.addWidget(foot)
        window.show(); window.move(0,0)

        def stamped(parent,child,matrix,stamp):
            msg=TransformStamped(); msg.header.stamp=stamp; msg.header.frame_id=parent; msg.child_frame_id=child
            msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=matrix[:3,3]
            q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=q
            return msg

        def state(t,chapter):
            stamp=rospy.Time.now(); transforms=[]
            for i,(prefix,key) in enumerate(zip(['fixed','hold'],data)):
                this_target=targets[i]
                a=data[key]; values_row=np.array([np.interp(t,a[:,col['time_s']],a[:,j]) for j in range(a.shape[1])])
                values={name:0. for name, *_ in joints}
                values.update({'arm_joint%d_joint1'%j:values_row[col['q%d'%j]] for j in range(1,5)})
                base=transform([values_row[col['base_'+c]] for c in 'xyz'],Rotation.from_rotvec([values_row[col['base_r'+c]] for c in 'xyz']).as_matrix())
                base[:3,3]-=this_target[:3,3]
                transforms.append(stamped('world',prefix+'/root',base@np.linalg.inv(root_to_fc),stamp))
                for name,kind,parent,child,origin,axis,mimic in joints:
                    angle=values[name]
                    if mimic is not None: angle=values[mimic[0]]*mimic[1]+mimic[2]
                    local=origin
                    if kind in ('continuous','revolute'):
                        local=origin@transform(rotation=Rotation.from_rotvec(axis*angle).as_matrix())
                    elif kind=='prismatic': local=origin@transform(axis*angle)
                    transforms.append(stamped(prefix+'/'+parent,prefix+'/'+child,local,stamp))
                ep=np.array([values_row[col['e'+c+'_m']] for c in 'xyz'])
                markers=MarkerArray()
                goal=Marker(); goal.header.frame_id='world'; goal.ns='goal'; goal.id=0; goal.type=Marker.LINE_LIST; goal.action=Marker.ADD
                goal.pose.orientation.w=1.; goal.scale.x=.0025
                goal.color.r=1.; goal.color.g=.2; goal.color.b=.45; goal.color.a=1.
                for j in range(3):
                    for sign in [-1,1]:
                        v=this_target[:3,j]*.035*sign
                        goal.points.append(Point(*v))
                markers.markers.append(goal)
                tcp=Marker(); tcp.header.frame_id='world'; tcp.ns='tcp'; tcp.id=1; tcp.type=Marker.SPHERE; tcp.action=Marker.ADD
                tcp.pose.orientation.w=1.; tcp.pose.position=Point(*ep)
                tcp.scale.x=tcp.scale.y=tcp.scale.z=.009
                tcp.color.r=.2; tcp.color.g=1.; tcp.color.b=.75; tcp.color.a=1.
                markers.markers.append(tcp); marker_pubs[i].publish(markers)
                infos[i].setText('Position: %5.1f mm    |    Orientation: %4.2f deg'%(values_row[col['position_error_m']]*1000,np.rad2deg(values_row[col['orientation_error_rad']])))
            tf_pub.sendTransform(transforms)
            clock.setText(chapter+'   |   t = %.2f s'%t)

        def pump(duration=.075):
            until=time.monotonic()+duration
            while time.monotonic()<until:
                app.processEvents(); time.sleep(.005)

        # TF listeners and mesh loads run asynchronously. Warm up on a fixed state.
        for unused in range(45): state(6.,'Loading native model'); pump(.06)
        preview=output.with_suffix('.png')
        app.primaryScreen().grabWindow(0).save(str(preview))
        print('Preview saved: '+str(preview),flush=True)
        if args.preview:
            success=True
            return
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1600x900',
            '-r',str(args.fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],
            stdin=subprocess.PIPE,stderr=log)
        def grab():
            image=app.primaryScreen().grabWindow(0).toImage().convertToFormat(QImage.Format_RGB888)
            bits=image.bits(); bits.setsize(image.sizeInBytes())
            encoder.stdin.write(bytes(bits))
        end=min(a[-1,col['time_s']] for a in data.values())
        for closeup in [False,True]:
            chapter='ARM CLOSE-UP' if closeup else 'FULL ROBOT'
            for manager in managers:
                view=manager.getViewManager().getCurrent()
                view.subProp('Distance').setValue(.95 if closeup else 2.15)
                view.subProp('Focal Point').subProp('X').setValue(-.12 if closeup else -.25)
            for n in range(int(end*args.fps)):
                state(n/args.fps,chapter); pump(); grab()
                if n%args.fps==0: print(chapter+': rendered '+str(n//args.fps)+' s',flush=True)
        for unused in range(args.fps*2): state(end,'Comparison complete'); pump(); grab()
        encoder.stdin.close(); encoder.wait(timeout=30)
        if encoder.returncode: raise RuntimeError('ffmpeg failed; see '+str(log.name))
        add_captions(output,summary,chapter_duration)
        print('Video saved: '+str(output),flush=True)
        success=True
    finally:
        if encoder is not None and encoder.poll() is None: encoder.terminate(); encoder.wait(timeout=15)
        rospy.signal_shutdown('Render complete')
        if master is not None:
            master.terminate()
            try: master.wait(timeout=15)
            except subprocess.TimeoutExpired: master.kill(); master.wait()
        log.close()
        if success:
            # Embedded RViz owns OGRE/plugin C++ globals. After closing all files
            # and child processes, avoid unsafe Python/C++ destructor ordering.
            sys.stdout.flush(); sys.stderr.flush(); os._exit(0)


if __name__=='__main__':
    main()
