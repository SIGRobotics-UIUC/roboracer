#!/usr/bin/env python3
"""Offline MAP failure reconstruction from finalized bags; never drives the car.

Plots use GT frame index so ROS-clock jumps cannot masquerade as physical time.
Counterfactuals are isolated LUT/guidance queries, not full closed-loop simulations.
"""
import argparse
import base64
import bisect
import html
import json
from pathlib import Path
import sqlite3

import numpy as np

NS = '/autodrive/roboracer_1/'


def wrap(x):
    return np.arctan2(np.sin(x), np.cos(x))


def load(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    wanted = {'/ground_truth/odom', '/odom', '/localized_pose', '/drive',
              NS+'steering', NS+'throttle_command', NS+'collision_count'}
    data = {name: [] for name in wanted}
    if not (path/'metadata.yaml').exists():
        raise ValueError('Need a finalized bag')
    for db in sorted(path.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as con:
            topics = {i:(name,get_message(kind)) for i,name,kind in con.execute('SELECT id,name,type FROM topics') if name in wanted}
            for tid, received, blob in con.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
                if tid not in topics: continue
                name, kind = topics[tid]; m = deserialize_message(blob, kind)
                t = (m.header.stamp.sec+m.header.stamp.nanosec*1e-9) if hasattr(m, 'header') else received*1e-9
                row = dict(t=t)
                if name in ('/ground_truth/odom', '/odom', '/localized_pose'):
                    pose = m.pose if name == '/localized_pose' else m.pose.pose
                    q = pose.orientation
                    row.update(x=pose.position.x,y=pose.position.y,
                               yaw=float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
                    if name != '/localized_pose':
                        row.update(v=m.twist.twist.linear.x, rate=m.twist.twist.angular.z)
                elif name == '/drive': row.update(v=m.drive.speed,steer=m.drive.steering_angle)
                else: row['value'] = m.data
                data[name].append(row)
    for values in data.values(): values.sort(key=lambda r:r['t'])
    return data


def latest(seq, t):
    i = bisect.bisect_right([r['t'] for r in seq],t)-1
    if i < 0: raise ValueError('Missing preceding sample')
    return seq[i]


def map_pose(row, origin):
    dx,dy=row['x']-origin['x_m'],row['y']-origin['y_m']
    c,s=np.cos(origin['yaw_rad']),np.sin(origin['yaw_rad'])
    return np.array([c*dx+s*dy,-s*dx+c*dy,float(wrap(row['yaw']-origin['yaw_rad']))])


def query(rows, lut, x, y, yaw, speed, lookahead):
    from control.map_guidance import Trajectory,l1_acceleration
    trajectory=Trajectory(rows)
    trajectory.closest=int(np.argmin(np.linalg.norm(trajectory.xy-[x,y],axis=1)))
    target,distance,eta,error=trajectory.target(x,y,yaw,lookahead,3.)
    effective=max(speed,float(lut.speeds[0]))
    acceleration=l1_acceleration(effective,eta,distance)
    steering,clamped,column=lut.query(acceleration,effective)
    return dict(target=target, heading_to_target_deg=float(np.degrees(eta)),
                demanded_lateral_acceleration_mps2=acceleration,
                lookup_speed_mps=effective, steering_deg=float(np.degrees(np.clip(steering,-.524,.524))),
                lut_clamped=bool(clamped), column=int(column))


def investigate(path, track, output):
    import yaml
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from control.map_lut import Lookup
    data=load(path); gt=data['/ground_truth/odom']
    origin=yaml.safe_load((track/'map_to_world.yaml').read_text())
    raceline=np.loadtxt(track/'raceline.csv',delimiter=';')
    collisions=data[NS+'collision_count']
    collision=next((b['t'] for a,b in zip(collisions,collisions[1:]) if b['value']>a['value']),None)
    jumps=[i for i in range(1,len(gt)) if gt[i]['t']-gt[i-1]['t']>.5 and gt[i-1]['v']>.5
           and (collision is None or gt[i]['t']<collision)]
    if not jumps: raise ValueError(f'No moving timestamp jump before failure in {path}')
    index=jumps[-1]; event=gt[index]['t']; before=gt[index-1]
    resumed=next(r for r in data['/drive'] if r['t']>=event and r['v']>.1)
    pose=latest(data['/localized_pose'],resumed['t'])
    odom=latest(data['/odom'],resumed['t'])
    actual=latest(gt,resumed['t']); actual_map=map_pose(actual,origin)
    prior_drive=latest(data['/drive'],before['t'])
    # Prefer a command shortly after the last sensor frame, before the clock step.
    candidates=[r for r in data['/drive'] if before['t']<=r['t']<event-.5 and r['v']>.1]
    if candidates: prior_drive=candidates[-1]
    model=json.loads(Path('src/control/config/map_model.json').read_text())
    lut=Lookup('src/control/config/autodrive_map_lut.npz',model)
    nearest=int(np.argmin(np.linalg.norm(raceline[:,1:3]-[pose['x'],pose['y']],axis=1)))
    lookahead=float(np.clip(.4*raceline[nearest,5]*.5,.6,3.))
    counterfactuals={
        'recorded_inputs_query':query(raceline,lut,pose['x'],pose['y'],pose['yaw'],odom['v'],lookahead),
        'correct_heading_only':query(raceline,lut,pose['x'],pose['y'],actual_map[2],odom['v'],lookahead),
        'correct_heading_and_speed':query(raceline,lut,pose['x'],pose['y'],actual_map[2],actual['v'],lookahead),
        'ground_truth_pose_and_speed':query(raceline,lut,*actual_map,actual['v'],lookahead)}
    prior_pose=latest(data['/localized_pose'],before['t']+.045)
    ghost=latest(data['/localized_pose'],event)
    trial=json.loads((path.parent/'result.json').read_text())
    clocks=trial.get('clock_discontinuities',[])
    clock=min(clocks,key=lambda r:abs(r['ros_stamp']-event)) if clocks else None
    if clock and abs(clock['ros_stamp']-event)>.1: clock=None
    result=dict(bag=str(path), jump_ros_stamp=event, ros_stamp_interval_s=event-before['t'],
        clock_event=clock, actual_speed_before_mps=before['v'],actual_speed_after_mps=actual['v'],
        odom_speed_before_mps=latest(data['/odom'],before['t'])['v'],odom_speed_after_mps=odom['v'],
        steering_before_deg=float(np.degrees(prior_drive['steer'])),
        steering_first_resumed_deg=float(np.degrees(resumed['steer'])),
        requested_speed_before_mps=prior_drive['v'],requested_speed_after_mps=resumed['v'],
        localized_heading_error_at_resume_deg=float(np.degrees(wrap(pose['yaw']-actual_map[2]))),
        localized_position_error_at_resume_m=float(np.linalg.norm(np.array([pose['x'],pose['y']])-actual_map[:2])),
        actual_yaw_change_across_jump_deg=float(np.degrees(wrap(actual['yaw']-before['yaw']))),
        localized_yaw_change_to_extrapolated_pose_deg=float(np.degrees(wrap(ghost['yaw']-prior_pose['yaw']))),
        extrapolated_position_jump_m=float(np.hypot(ghost['x']-prior_pose['x'],ghost['y']-prior_pose['y'])),
        collision_delay_ros_s=collision-event if collision else None,
        ground_truth_map_position_after_jump=actual_map[:2].tolist(),
        replay_steering_agreement_error_deg=abs(counterfactuals['recorded_inputs_query']['steering_deg']-float(np.degrees(resumed['steer']))),
        counterfactuals=counterfactuals,
        replay_assumptions='Nearest waypoint initializes closest-index history; default lookahead and scale 0.5. Lookup queries omit rate history. A safety stop clears that history in the current controller. These are not closed-loop predictions.')
    output.mkdir(parents=True,exist_ok=True)
    name=path.parent.name
    (output/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    stamps=np.array([r['t'] for r in gt]); frames=np.arange(len(gt))-index
    coordinate=lambda seq:np.interp([r['t'] for r in seq],stamps,frames)
    window=lambda seq:[r for r in seq if -8.5<=float(np.interp(r['t'],stamps,frames))<=16.5]
    fig,axes=plt.subplots(6,1,figsize=(12,15),sharex=True)
    for topic,label,color in [('/ground_truth/odom','Simulator body speed','black'),('/odom','Wheel-odometry speed','#d55e00'),('/drive','Requested speed','#0072b2')]:
        rr=window(data[topic]);axes[0].plot(coordinate(rr),[r['v'] for r in rr],label=label,color=color)
    axes[0].set_ylabel('Speed (m/s)')
    rr=window(data['/drive']);axes[1].plot(coordinate(rr),np.degrees([r['steer'] for r in rr]),label='Steering command')
    rr=window(data[NS+'steering']);axes[1].plot(coordinate(rr),np.degrees([r['value'] for r in rr]),label='Steering feedback')
    axes[1].set_ylabel('Steering (deg)')
    rr=window(data['/localized_pose']);truth=np.array([map_pose(g,origin) for g in gt])
    tt=np.array([r['t'] for r in rr]);true_yaw=np.interp(tt,stamps,np.unwrap(truth[:,2]))
    axes[2].plot(coordinate(rr),np.degrees(wrap(np.array([r['yaw'] for r in rr])-true_yaw)),label='Localized heading minus ground truth')
    axes[2].set_ylabel('Heading error (deg)')
    tx=np.interp(tt,stamps,truth[:,0]);ty=np.interp(tt,stamps,truth[:,1])
    axes[3].plot(coordinate(rr),np.hypot(np.array([r['x'] for r in rr])-tx,np.array([r['y'] for r in rr])-ty),label='Localized position error')
    axes[3].set_ylabel('Position error (m)')
    rr=window(data[NS+'throttle_command']);axes[4].plot(coordinate(rr),[r['value'] for r in rr],label='Adapter throttle command')
    axes[4].set_ylabel('Normalized throttle')
    rr=window(gt);axes[5].plot(coordinate(rr),[r['rate'] for r in rr],label='Actual yaw rate')
    axes[5].set_ylabel('Yaw rate (rad/s)')
    for ax in axes:
        ax.axvline(0,color='red',linestyle='--',label='ROS timestamp jump')
        if collision and stamps[0]<=collision<=stamps[-1]:
            ax.axvline(float(np.interp(collision,stamps,frames)),color='.4',linestyle=':',label='Collision counter changed')
        ax.grid(alpha=.25);ax.legend(fontsize=8,loc='best')
    axes[-1].set_xlim(-8,16)
    axes[-1].set_xlabel('Ground-truth frame index relative to timestamp jump (not elapsed seconds)')
    fig.suptitle(name+': failure reconstruction — ROS-time jump compressed on frame axis')
    fig.tight_layout(rect=(0,0,1,.97));fig.savefig(output/(name+'.png'),dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',action='append',required=True)
    ap.add_argument('--track-dir',default='tracks/porto_odom_trial')
    ap.add_argument('--output',required=True)
    args=ap.parse_args();output=Path(args.output)
    cases=[investigate(Path(p),Path(args.track_dir),output) for p in args.bag]
    (output/'summary.json').write_text(json.dumps(cases,indent=2)+'\n')
    description=('Recorded evidence: a ROS-time discontinuity is followed by false low wheel speed, '
        'large localization heading/position errors, a freshness stop and opposite-lock steering on resume. '
        'The extrapolated pose jump is consistent with integrating prior velocity/gyro across the clock step. '
        'The isolated heading correction reverses the erroneous steering direction in the primary hairpin case. '
        'No production parameters or controller code were changed; no new driving was performed.')
    body='<h1>MAP hairpin failure investigation</h1><p>'+html.escape(description)+'</p>'
    for case in cases:
        name=Path(case['bag']).parent.name
        body+='<h2>'+html.escape(name)+'</h2><pre>'+html.escape(json.dumps(case,indent=2))+'</pre>'
        body+='<img style="max-width:100%" src="data:image/png;base64,'+base64.b64encode((output/(name+'.png')).read_bytes()).decode()+'">'
    (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Hairpin investigation</title>'
        '<style>body{font-family:system-ui;max-width:1200px;margin:25px auto;padding:15px}pre{white-space:pre-wrap;background:#f3f4f5;padding:15px}</style>'+body)
    print(json.dumps(cases,indent=2))
