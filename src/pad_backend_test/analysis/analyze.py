"""Reproduce the comparison; latest batch is provisionally labeled SITL."""
import os
os.environ['MPLCONFIGDIR']='/tmp/pad-analysis-mpl'
import argparse
import json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', type=Path, default=Path(__file__).resolve().parent / 'recordings')
parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
args = parser.parse_args()
ROOT = args.input
OUT = args.output
OUT.mkdir(parents=True, exist_ok=True)
runs=[]; phases=[]; excluded=[]
for file in sorted(ROOT.glob('*.jsonl')):
    rows=[json.loads(l) for l in file.open()]; meta=rows[0]
    if not rows[-1].get('success') or len(meta['sequence'])!=19:
        excluded.append(file.name); continue
    group='sim-simtime' if meta.get('use_sim_time') else ('SITL (provisional)' if '20261005T000' in file.name and file.name.split('T')[-1] >= '000613' else 'sim')
    names=[s['name'] for s in meta['sequence']]
    commands=[r for r in rows if r['kind']=='command' and r['phase'] in names]
    t0=commands[0]['ros_time_ns']
    clock=lambda r:(r['ros_time_ns']-t0)*1e-9
    infos=[r for r in rows if r['kind']=='info' and r['message']['pose_world_valid']]
    t=np.array([clock(r) for r in infos]); xyz=np.array([[r['message']['pose_world']['position'][a] for a in 'xyz'] for r in infos])
    unique=np.r_[True,np.diff(t)>0]; t=t[unique]; xyz=xyz[unique]
    bounds=[clock(next(c for c in commands if c['phase']==n)) for n in names]
    cleanup=next(r for r in rows if r['kind']=='command' and r['phase']=='return')
    bounds.append(clock(cleanup))
    wall=cleanup['elapsed']-commands[0]['elapsed']
    run=dict(file=file.name,group=group,id=meta['id'],wall_seconds=wall,sequence_seconds=bounds[-1],speed=bounds[-1]/wall,
             commands_per_second=len(commands)/bounds[-1],info_per_second=sum((t>=0)&(t<bounds[-1]))/bounds[-1],
             invalid_info=sum(not r['message']['pose_world_valid'] for r in rows if r['kind']=='info'),
             max_info_gap=float(np.max(np.diff(t[(t>=0)&(t<bounds[-1])]))))
    runs.append(run)
    for i,step in enumerate(meta['sequence']):
        lo,hi=bounds[i:i+2]; desired=np.array(step['velocity'][:3]); duration=hi-lo
        displacement=np.array([np.interp(hi,t,xyz[:,a])-np.interp(lo,t,xyz[:,a]) for a in range(3)])
        sel=(t>=hi-1)&(t<hi)
        steady=np.array([np.polyfit(t[sel]-lo,xyz[sel,a],1)[0] for a in range(3)])
        direction=desired/np.linalg.norm(desired) if np.linalg.norm(desired)>0 else np.zeros(3)
        phases.append(dict(file=file.name,group=group,id=meta['id'],phase=step['name'],duration=duration,
                           displacement_error_m=float(np.linalg.norm(displacement-desired*duration)),
                           along_displacement_m=float(displacement@direction),
                           steady_speed_mps=float(steady@direction) if any(desired) else float(np.linalg.norm(steady)),
                           steady_vector_error_mps=float(np.linalg.norm(steady-desired)),
                           drift_m=float(np.linalg.norm(displacement)),
                           dx=displacement[0],dy=displacement[1],dz=displacement[2]))
    run['_t']=t;run['_xyz']=xyz;run['_bounds']=bounds;run['_meta']=meta
for name,values in [('runs',runs),('phases',phases)]:
    fields=[k for k in values[0] if not k.startswith('_')]
    with (OUT/f'{name}.csv').open('w') as f:
        w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(values)
colors={'sim':'#2463aa','sim-simtime':'#dd8500','SITL (provisional)':'#15855b'}
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
for run in runs:
    t=run['_t'];p=run['_xyz'];b=run['_bounds'];grid=np.arange(0,min(b[-1],68),.1)
    # Symmetric 0.4-second secant smooths differencing noise; plotting only.
    vel=np.array([(np.interp(grid+.2,t,p[:,a])-np.interp(grid-.2,t,p[:,a]))/.4 for a in range(3)])
    for a,ax in enumerate(axes):
        ax.plot(grid,vel[a],color=colors[run['group']],alpha=.65,lw=1,label=run['group'] if run['id']==0 else None)
ref=next(r for r in runs if r['group']=='sim')
for a,ax in enumerate(axes):
    ax.step(ref['_bounds'],[s['velocity'][a] for s in ref['_meta']['sequence']]+[0],where='post',color='black',ls='--',lw=1,label='Command')
    ax.set_ylabel(f'v{"xyz"[a]} (m/s)');ax.grid(alpha=.2);ax.legend(loc='upper right',ncol=4)
axes[-1].set_xlabel('Seconds from first command (ROS time; simulated seconds for sim-simtime)')
fig.suptitle('Velocity response — 3 aircraft per mode; derived from world position\n0.4 s centered difference, individual runs overlaid; latest batch provisionally SITL')
fig.tight_layout();fig.savefig(OUT/'velocity_comparison.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,5))
for run in runs:
    for ax,phase in zip(axes,['down','down_with_settle']):
        i=[s['name'] for s in run['_meta']['sequence']].index(phase);start=run['_bounds'][i]
        g=np.arange(-1,5,.05)+start;t=run['_t'];z=run['_xyz'][:,2]
        v=(np.interp(g+.2,t,z)-np.interp(g-.2,t,z))/.4
        ax.plot(g-start,v,color=colors[run['group']],alpha=.7,label=run['group'] if run['id']==0 else None)
for ax,title in zip(axes,['Up → down immediately','Settled → down']):
    ax.axvline(0,color='gray');ax.axhline(-.15,color='black',ls='--');ax.set(title=title,xlabel='Seconds from down command',ylabel='Estimated vz (m/s)');ax.grid(alpha=.2);ax.legend()
fig.tight_layout();fig.savefig(OUT/'vertical_reversals.png',dpi=160)
summary={}
for group in colors:
    rr=[r for r in runs if r['group']==group];pp=[p for p in phases if p['group']==group]
    summary[group]={key:float(np.mean([r[key] for r in rr])) for key in ['wall_seconds','sequence_seconds','speed','commands_per_second','info_per_second','max_info_gap']}
    summary[group]['phase_means']={name:{k:float(np.mean([p[k] for p in pp if p['phase']==name])) for k in ['steady_speed_mps','displacement_error_m','drift_m','dz']} for name in names}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2));print('Excluded:',excluded)
