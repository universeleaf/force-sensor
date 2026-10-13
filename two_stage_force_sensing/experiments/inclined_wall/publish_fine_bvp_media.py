"""Publish all saved fine-step BVP cases; no mechanics rerun or frame filtering."""
from pathlib import Path
import argparse,csv,json,hashlib
import numpy as np,h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import imageio_ffmpeg
ROOT=Path(__file__).resolve().parents[2]
C=['#197f96','#d45b32'];REF='#273347'
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def col(rows,k):return np.array([float(r[k]) for r in rows])
def load_case(sc,level,stamp,out):
 tag=f'fine_bvp_{sc}_{level}mm_{stamp}';pub=ROOT/'results/published'/tag;rr=read(pub/'estimates.csv');cache=out/'shapes.npz'
 if cache.exists():
  with np.load(cache) as f:d={k:f[k] for k in f.files}
 else:
  with h5py.File(ROOT/'results/runs'/f'fine_bvp_{sc}_truth_{stamp}'/'truth.mat') as f:
   t=f['truth'];d={k:t[k][()] for k in ['p','contactForce','tipForce','contactS','sMm','planePoint','planeNormal']}
  shapes=[];contacts=[]
  with h5py.File(ROOT/'results/runs'/tag/'comparison.mat') as f:
   for ref in f['runs'][:,0]:
    sh=f[ref]['mechanics/evaluation/shape'];shapes.append(sh['p'][()]);cp=sh['contactP'];contacts.append(np.full(3,np.nan) if cp.attrs.get('MATLAB_empty',False) else cp[()][0])
  d['estimateP']=np.array(shapes);d['estimateContactP']=np.array(contacts);np.savez_compressed(cache,**d)
 assert np.allclose(np.sqrt(np.mean(np.sum((d['estimateP']-d['p'])**2,axis=2),axis=1)),col(rr,'shapeRmsMm'))
 for prefix in ['tip','contact']:
  d[prefix+'Estimate']=np.column_stack([col(rr,prefix+'F'+a) for a in 'xyz'])
  assert np.allclose(np.linalg.norm(d[prefix+'Estimate']-d[prefix+'Force'],axis=1),col(rr,prefix+'ErrorN'))
 return rr,d

def publish(sc,level,stamp):
 base=ROOT/'results/published'/f'fine_bvp_comparison_{stamp}';out=base/'media'/f'{sc}_{level}mm';out.mkdir(parents=True,exist_ok=True)
 rr,d=load_case(sc,level,stamp,out);T=len(rr);x=np.arange(1,T+1);turn=(T+1)//2;target=level.replace('p','.')
 audit={int(float(r['frame'])):r for r in read(base/'frames.csv') if r['scenario']==sc and float(r['targetMm'])==float(target)}
 bv=np.array([float(audit[k]['bvpSuccess']) if k in audit else np.nan for k in x]);label=f'{sc.capitalize()} | noise target {target} mm | one-pass rod + fixed-load BVP'
 plt.rcParams.update({'font.size':10,'figure.facecolor':'white','axes.spines.top':False,'axes.spines.right':False})
 def timeline(ax):
  ax.axvline(turn,color='gray',ls=':',lw=1);ax.set_xlim(1,T);ax.set_xlabel('Frame (dotted line: reversal)');ax.grid(alpha=.2)
 fig,axs=plt.subplots(2,3,figsize=(14,7),layout='constrained')
 for row,prefix in enumerate(['tip','contact']):
  for j,a in enumerate('XYZ'):
   ax=axs[row,j];ax.plot(x,d[prefix+'Force'][:,j],color=REF,ls='--',label='Reference');ax.plot(x,d[prefix+'Estimate'][:,j],color=C[0],label='Estimate');ax.set_ylabel(f'{prefix.capitalize()} {a} [N]');timeline(ax);ax.legend()
 fig.suptitle(label);fig.savefig(out/'force_components.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(4,2,figsize=(14,12),layout='constrained')
 specs=[(['tipErrorN','contactErrorN'],'Force error norm [N]'),(['shapeRmsMm','shapeTipErrorMm'],'Shape / endpoint error [mm]'),(['seconds','geometrySeconds'],'Solve / geometry time [s]'),(['iterations','selectedIterations'],'Total / selected SQP iterations'),(['fbgRms'],'Normalized FBG RMS'),(['tipMomentNmm'],'Terminal moment infinity norm [N mm]'),(['gapMm','penetrationMm'],'Contact gap / penetration [mm]'),(['workNmm'],'Friction work [N mm]')]
 for ax,(keys,ylabel) in zip(axs.flat,specs):
  for key in keys:ax.plot(x,col(rr,key),label=key,lw=1.2)
  if keys==['fbgRms']:ax.axhline(1.24,color='red',ls='--',label='FBG threshold 1.24')
  if keys==['tipMomentNmm']:ax.axhline(2e-4,color='red',ls='--',label='Moment diagnostic tolerance');ax.set_yscale('symlog',linthresh=1e-6)
  if keys==['workNmm']:ax.axhline(1e-8,color='red',ls='--',label='Positive-work threshold')
  for k in x[bv==0]:ax.axvline(k,color='red',alpha=.22)
  ax.set_ylabel(ylabel);timeline(ax);ax.legend(fontsize=8)
 fig.suptitle(label+'\nRed vertical lines: failed BVP correction; failures remain in all plots');fig.savefig(out/'errors_timing_physics.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(3,1,figsize=(13,8),layout='constrained')
 for ax,keys,ylabel in zip(axs,[['stage1TipErrorMm','stage1ShapeRmsMm'],['normalErrorDeg'],['arcErrorMm']],[ 'Stage 1 reconstruction [mm]','Normal error [degrees]','Contact arc-length error [mm]']):
  for key in keys:ax.plot(x,col(rr,key),label=key)
  ax.set_ylabel(ylabel);timeline(ax);ax.legend()
 fig.suptitle(label);fig.savefig(out/'geometry_stage.png',dpi=150);plt.close(fig)
 fig=plt.figure(figsize=(16,10),dpi=100);fig.text(.035,.965,label,fontsize=18,weight='bold');subtitle=fig.text(.035,.935,'',fontsize=11)
 ax3=fig.add_axes([.04,.54,.40,.35],projection='3d');ax2=fig.add_axes([.57,.55,.36,.33])
 # Fixed axes cover every saved estimate, including failure excursions.
 pts=np.concatenate([d['p'].reshape(-1,3),d['estimateP'].reshape(-1,3)]);lo=pts.min(0)-10;hi=pts.max(0)+10;lo[1]=min(lo[1],-35);hi[1]=max(hi[1],35)
 xx,yy=np.meshgrid(np.linspace(lo[0],hi[0],4),np.linspace(lo[1],hi[1],4));n=d['planeNormal'].ravel();pt=d['planePoint'].ravel();zz=pt[2]-(n[0]*(xx-pt[0])+n[1]*(yy-pt[1]))/n[2]
 ax3.plot_surface(xx,yy,zz,alpha=.12,color='gray');ax3.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='X [mm]',ylabel='Y [mm]',zlabel='Z [mm]');ax3.set_box_aspect(hi-lo);ax3.view_init(20,-65)
 ax2.plot(xx[0],zz[0],color='gray',label='Wall');ax2.set(xlim=(lo[0],hi[0]),ylim=(lo[2],hi[2]),xlabel='X [mm]',ylabel='Z [mm]',title='XZ projection');ax2.set_aspect('equal',adjustable='box');ax2.grid(alpha=.15)
 tr3,=ax3.plot([],[],[],color=REF,lw=2.5,label='Reference');es3,=ax3.plot([],[],[],color=C[0],ls='--',lw=2,label='Estimate');tr2,=ax2.plot([],[],color=REF,lw=2.5,label='Reference');es2,=ax2.plot([],[],color=C[0],ls='--',lw=2,label='Estimate')
 c3,=ax3.plot([],[],[],'x',color=C[1],ms=8);c2,=ax2.plot([],[],'x',color=C[1],ms=8);ax3.legend(loc='upper left',fontsize=8);ax2.legend(fontsize=8)
 status=fig.text(.035,.487,'',fontsize=10,family='DejaVu Sans Mono',va='top');cursors=[]
 for row,prefix in enumerate(['tip','contact']):
  for j,a in enumerate('XYZ'):
   ax=fig.add_axes([.055+.325*j,.255-.17*row,.28,.12]);ax.plot(x,d[prefix+'Force'][:,j],color=REF,ls='--',lw=1.3);ax.plot(x,d[prefix+'Estimate'][:,j],color=C[0],lw=1.1);ax.set_title(f'{prefix.capitalize()} {a} [N]',fontsize=10);ax.set_xlim(1,T);ax.grid(alpha=.2);ax.axvline(turn,color='gray',ls=':');ax.tick_params(labelsize=8);cursors.append(ax.axvline(1,color=C[1]));
 fig.text(.035,.025,'All saved frames shown | dashed dark: reference; teal: estimate | full circular cone, exhaustive modes, SQP cap 100 | illustrative playback',fontsize=9)
 modes={'0':'separation','1':'sticking','2':'sliding','zeros(1,0)':'free'}
 def update(k):
  q=rr[k];act=f"wall distance {float(q['distanceMm']):.1f} mm" if sc=='insertion' else f"base angle {float(q['angleDeg']):.2f} deg"
  subtitle.set_text(f"Frame {k+1}/{T} | {act} | {'forward' if k<turn else 'return'} | selected mode: {modes.get(q['mode'],q['mode'])}")
  a=d['p'][k];b=d['estimateP'][k];tr3.set_data_3d(a.T);es3.set_data_3d(b.T);tr2.set_data(a[:,0],a[:,2]);es2.set_data(b[:,0],b[:,2]);cp=d['estimateContactP'][k];c3.set_data_3d(*[[v] for v in cp]);c2.set_data([cp[0]],[cp[2]])
  state='RESTORED SEED' if int(q['restoredSeed']) else ('BVP OK' if bv[k] else 'BVP FAILED')
  physical='PASS' if float(q['solverSuccess']) else 'FLAGGED'
  status.set_text(f"{state} | nonlinear mode: {physical} | time {float(q['seconds']):.3f} s | SQP it {q['iterations']} | IVPs {q['ivps']}\nTip/contact error {float(q['tipErrorN']):.3f}/{float(q['contactErrorN']):.3f} N | shape RMS {float(q['shapeRmsMm']):.3f} mm | FBG RMS {float(q['fbgRms']):.3f} / 1.24\nGap {float(q['gapMm']):.3g} mm | terminal moment {float(q['tipMomentNmm']):.3g} N mm | friction work {float(q['workNmm']):.3g} N mm")
  status.set_color('#a32432' if state=='BVP FAILED' else REF)
  for c in cursors:c.set_xdata([k+1,k+1])
 matplotlib.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe();writer=FFMpegWriter(fps=10,codec='libx264',extra_args=['-crf','23','-pix_fmt','yuv420p','-movflags','+faststart'])
 snapshots={0,turn-1,T-1,*list(np.flatnonzero(bv==0))}
 with writer.saving(fig,str(out/'trajectory.mp4'),100):
  for k in range(T):
   update(k)
   if k in snapshots:fig.savefig(out/f'frame_{k+1:03d}.png',dpi=100)
   writer.grab_frame();writer.grab_frame()
   if (k+1)%50==0:print(sc,level,'rendered',k+1,'/',T,flush=True)
 plt.close(fig)
 meta=dict(scenario=sc,noiseTargetMm=float(target),sourceFrames=T,encodedFrames=T*2,fps=10,durationSeconds=T*.2,width=1600,height=1000,omittedFrames=0,bvpFailedFrames=x[bv==0].tolist(),shapeSource='Saved post-correction IVP; pre-correction fallback for BVP exception',sourceStamp=stamp)
 (out/'metadata.json').write_text(json.dumps(meta,indent=2));print('DONE',out,flush=True)
 return meta

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--stamp',default='20261012_step5');ap.add_argument('--case',choices=['insertion','rotation']);ap.add_argument('--level',choices=['0','0p5','1','2']);args=ap.parse_args()
 records=[]
 for sc in ([args.case] if args.case else ['insertion','rotation']):
  for lv in ([args.level] if args.level else ['0','0p5','1','2']):records.append(publish(sc,lv,args.stamp))
 out=ROOT/'results/published'/f'fine_bvp_comparison_{args.stamp}'/'media';(out/'manifest.json').write_text(json.dumps(records,indent=2))
 lines=['# Fine-step rod plus BVP videos and plots','','Every saved frame is shown. Playback is illustrative, not physical time. Failed corrections remain visible and are flagged. Force curves show world XYZ components. Static plots use full ranges without clipping outliers.','', '| Case | Video | Forces | Errors, timing and physics | Geometry |','|---|---|---|---|---|']
 for r in records:
  name=r['scenario']+'_'+str(r['noiseTargetMm']).removesuffix('.0').replace('.','p')+'mm'
  lines.append(f'| {name} | [MP4]({name}/trajectory.mp4) | [PNG]({name}/force_components.png) | [PNG]({name}/errors_timing_physics.png) | [PNG]({name}/geometry_stage.png) |')
 lines+=['','shapes.npz is a compact rendering source; published estimates.csv and combined frames.csv provide forces/diagnostics. This allows rendering without the large local MATLAB checkpoints. No solver rerun is performed.']
 (out/'README.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
