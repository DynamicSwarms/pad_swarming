"""Compare explicit old/new wall-time simulation recordings; run from this directory."""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/pad-comparison-mpl')
from pathlib import Path
import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent
FILES={
'Sim':['simulation-cf0-20261007T071331.757326Z.jsonl','simulation-cf1-20261007T071159.423975Z.jsonl','simulation-cf2-20261007T070930.932456Z.jsonl'],
'Simx3':['simulation-cf0-20261007T071518.640594Z.jsonl','simulation-cf1-20261007T071549.662677Z.jsonl','simulation-cf2-20261007T071621.081840Z.jsonl']}

colors={'Sim':'#2875b8','Simx3':'#d65424'}
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
zoom,zaxes=plt.subplots(1,2,figsize=(12,4.5),sharey=True)
metrics=[]; reference=None;manifest=[]
for label,files in FILES.items():
 for filename in files:
  rows=[json.loads(l) for l in (BASE/'recordings'/filename).open()]
  meta=rows[0];seq=meta['sequence'];assert rows[-1]['success'] and meta['use_sim_time'] == (label == 'Simx3')
  if reference is None: reference=seq
  assert seq==reference
  cmds=[r for r in rows if r['kind']=='command']
  starts=[next(r for r in cmds if r['phase']==s['name'])['ros_time_ns'] for s in seq]
  end=next(r for r in cmds if r['phase']=='return')['ros_time_ns'];origin=starts[0]
  bounds=(np.array(starts+[end])-origin)/1e9
  info=[r for r in rows if r['kind']=='info' and r['message']['pose_world_valid']]
  t=np.array([(r['ros_time_ns']-origin)/1e9 for r in info]);p=np.array([[r['message']['pose_world']['position'][a] for a in 'xyz'] for r in info])
  unique=np.r_[True,np.diff(t)>0];t=t[unique];p=p[unique]
  # Match phase boundaries onto nominal sequence time; retain actual time for derivatives.
  nominal=np.r_[0,np.cumsum([s['duration'] for s in seq])]
  grid=np.arange(0,nominal[-1],.05);actual=np.interp(grid,nominal,bounds)
  v=np.array([(np.interp(actual+.2,t,p[:,a])-np.interp(actual-.2,t,p[:,a]))/.4 for a in range(3)])
  for a,ax in enumerate(axes):ax.plot(grid,v[a],color=colors[label],alpha=.65,lw=1,label=label if meta['id']==0 else None)
  for ax,name in zip(zaxes,['down','down_with_settle']):
   idx=[s['name'] for s in seq].index(name);g=np.arange(-1,5,.05);at=bounds[idx]+g
   vz=(np.interp(at+.2,t,p[:,2])-np.interp(at-.2,t,p[:,2]))/.4
   ax.plot(g,vz,color=colors[label],alpha=.65,label=label if meta['id']==0 else None)
  manifest.append({'group':label,'file':filename,'sequence_seconds':float(bounds[-1])})
  for i,s in enumerate(seq):
   lo,hi=bounds[i:i+2];sel=(t>=hi-1)&(t<hi)
   slope=[np.polyfit(t[sel]-lo,p[sel,a],1)[0] for a in range(3)]
   delta=[np.interp(hi,t,p[:,a])-np.interp(lo,t,p[:,a]) for a in range(3)]
   metrics.append(dict(group=label,file=filename,id=meta['id'],phase=s['name'],vx=slope[0],vy=slope[1],vz=slope[2],dx=delta[0],dy=delta[1],dz=delta[2]))
for a,ax in enumerate(axes):
 ax.step(nominal,[s['velocity'][a] for s in seq]+[0],where='post',color='black',ls='--',lw=1,label='Command')
 ax.set_ylabel(f'v{"xyz"[a]} (m/s)');ax.grid(alpha=.2);ax.legend(ncol=3,loc='upper right')
axes[-1].set_xlabel('Sequence time (s; simulated seconds for Simx3), aligned by command phase')
fig.suptitle('New Sim vs Simx3 — only the six latest runs (cf0, cf1, cf2)\nVelocity derived from world position with a centered 0.4 s difference')
fig.tight_layout();fig.savefig(BASE/'sim_vs_simx3_sim.png',dpi=160)
for ax,title,previous in zip(zaxes,['Immediate up → down','Settled → down'],[.15,0]):
 ax.step([-1,0,5],[previous,-.15,-.15],where='post',color='black',ls='--',label='Command')
 ax.set(title=title,xlabel='Seconds from down command',ylabel='vz (m/s)');ax.grid(alpha=.2);ax.legend()
zoom.tight_layout();zoom.savefig(BASE/'sim_vs_simx3_vertical.png',dpi=160)
with (BASE/'sim_vs_simx3_metrics.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(metrics[0]));w.writeheader();w.writerows(metrics)
(BASE/'sim_vs_simx3_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
lines=['# New Sim vs Simx3','', 'Only the six latest runs: three Sim wall-time runs and three Simx3 simulation-time runs, cf0–cf2, with identical 68-second sequences. Earlier incomplete/interrupted October 7 runs are excluded. Exact filenames are in `sim_vs_simx3_manifest.json`.','', '| Phase | Sim vz (m/s) | Simx3 vz (m/s) | Sim dz (m) | Simx3 dz (m) |','|---|---:|---:|---:|---:|']
for phase in ['up','down','up_with_settle','down_with_settle','up_forward']:
 values=[]
 for key in ['vz','dz']:
  for label in FILES:values.append(np.mean([r[key] for r in metrics if r['group']==label and r['phase']==phase]))
 lines.append('| '+phase+' | '+' | '.join(f'{v:.4f}' for v in values)+' |')
lines += ['', 'Velocity metrics use a linear fit over the last second of each phase; displacement is interpolated between actual command boundaries. Values are means over three aircraft. Plots show each run, using actual ROS time for velocity differentiation and matching phase boundaries to nominal sequence time for the full-sequence overlay. The centered 0.4-second difference smooths transitions; it cannot resolve exact latency or high-frequency oscillation. These recordings alone do not establish the exact compiled controller version.','', 'Run `python3 plot_six.py` to reproduce (NumPy and Matplotlib required). No original recordings are modified.']
(BASE/'sim_vs_simx3_report.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
