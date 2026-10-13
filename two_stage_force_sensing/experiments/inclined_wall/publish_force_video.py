"""Videos of saved shapes and reference/estimated XYZ tip/contact forces."""
from pathlib import Path
import argparse,csv,json,shutil
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.lines import Line2D
import imageio_ffmpeg
ROOT=Path(__file__).resolve().parents[2]
COLORS=['#087f8c','#d26927']
def read(p):
 with p.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def values(r,k):return np.array([float(x[k]) for x in r])
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['insertion','rotation'],default='insertion');args=parser.parse_args();rotation=args.case=='rotation'
 tag='inclined_wall_rotation_20261010' if rotation else 'inclined_wall_modes_conditional_1e6_20261010'
 run=ROOT/'results/runs'/tag;pub=ROOT/'results/published'/tag;out=pub/('video' if rotation else 'video_forces');out.mkdir(parents=True,exist_ok=True)
 truthpath=(run/'truth.mat') if rotation else ROOT/'results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth.mat'
 rows=read(pub/'estimates.csv');groups=[[r for r in rows if int(r['method'])==j+1] for j in range(2)];T=len(groups[0]);assert all(len(g)==T for g in groups)
 with h5py.File(truthpath) as f:
  tr=f['truth'];truth={k:np.array(tr[k]) for k in ['p','contactS','contactForce','tipForce','distanceMm']}
  angles=tr['angleDeg'][()].ravel() if rotation else None
 reference_modes=[r['mode'] for r in read(run/'truth_frames.csv')] if rotation else None
 p=np.zeros((2,T,201,3));cp=[[None]*T for _ in range(2)]
 with h5py.File(run/'comparison.mat') as f:
  for j in range(2):
   for k in range(T):
    sh=f[f['runs'][k,j]]['mechanics/evaluation/shape'];p[j,k]=sh['p'][()]
    if not sh['contactP'].attrs.get('MATLAB_empty',False):cp[j][k]=sh['contactP'][()]
 for j,g in enumerate(groups):
  assert np.allclose(np.sqrt(np.mean(np.sum((p[j]-truth['p'])**2,axis=2),axis=1)),values(g,'shapeRmsMm'))
  for prefix,ref,err in [('tip',truth['tipForce'],'tipErrorN'),('contact',truth['contactForce'],'contactErrorN')]:
   estimated=np.column_stack([values(g,prefix+'F'+axis) for axis in 'xyz'])
   assert np.allclose(np.linalg.norm(estimated-ref,axis=1),values(g,err))
 plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#fafbfc','text.color':'#253347','axes.labelcolor':'#253347'})
 fig=plt.figure(figsize=(16,12),dpi=120)
 title='BASE ROTATION  |  0 to 45 degrees and back' if rotation else 'INCLINED WALL  |  insertion and withdrawal'
 fig.text(.04,.965,title,fontsize=22,weight='bold')
 fig.text(.04,.94,'200 mm rod  |  80 mm straight + 120 mm / 90 degree precurve  |  true tip load [1, 0, 0] N  |  true friction mu = 0.3',fontsize=11)
 subtitle=fig.text(.04,.913,'',fontsize=12,weight='bold')
 lines=[];contact=[];dots=[];status=[]
 for j in range(2):
  ax=fig.add_axes([.06+.5*j,.565,.38,.31],projection='3d' if rotation else None)
  ax.set_title(['Ours: mu = 0.3','Ours: mu = 0'][j],color=COLORS[j],fontsize=15,weight='bold')
  if rotation:
   xx,yy=np.meshgrid(np.linspace(-20,75,3),np.linspace(-70,70,3));zz=xx+80*np.sqrt(2);ax.plot_surface(xx,yy,zz,alpha=.13,color='#667788')
   ax.set(xlim=(-20,145),ylim=(-70,70),zlim=(45,190),xlabel='X [mm]',ylabel='Y [mm]',zlabel='Z [mm]');ax.view_init(elev=20,azim=-65);ax.set_box_aspect((165,140,145))
   l1,=ax.plot([],[],[],color='#273347',lw=3,label='Reference shape');l2,=ax.plot([],[],[],color=COLORS[j],lw=2,ls='--',label='Estimated shape');dot,=ax.plot([],[],[],'o',mfc='white',mec='#273347',ms=7);c,=ax.plot([],[],[],'x',color=COLORS[j],ms=8)
  else:
   xx=np.linspace(-15,145,100);wall=xx+80*np.sqrt(2);ax.fill_between(xx,wall,240,color='#dbe1e8',alpha=.6);ax.plot(xx,wall,color='#667788',lw=1.5)
   ax.set(xlim=(-15,145),ylim=(-5,190),xlabel='X [mm]',ylabel='Z [mm]');ax.set_aspect('equal',adjustable='box')
   l1,=ax.plot([],[],color='#273347',lw=3,label='Reference shape');l2,=ax.plot([],[],color=COLORS[j],ls='--',lw=2,label='Estimated shape');dot,=ax.plot([],[],'o',mfc='white',mec='#273347',ms=7);c,=ax.plot([],[],'x',color=COLORS[j],ms=8)
  ax.legend(loc='upper left',fontsize=8);ax.grid(alpha=.15);lines.append((l1,l2));dots.append(dot);contact.append(c)
  status.append(fig.text(.05+.5*j,.515,'',va='top',fontsize=9,linespacing=1.35,family='DejaVu Sans Mono'))
 legend=[Line2D([0],[0],color='#273347',ls='--',lw=2,label='Reference'),Line2D([0],[0],color=COLORS[0],lw=2,label='Estimated mu = 0.3'),Line2D([0],[0],color=COLORS[1],lw=2,label='Estimated mu = 0')]
 fig.legend(handles=legend,loc='center',bbox_to_anchor=(.5,.432),ncol=3,frameon=False)
 cursors=[]
 for row,prefix in enumerate(['tip','contact']):
  for a,axis in enumerate('xyz'):
   ax=fig.add_axes([.065+.32*a,.28-.18*row,.27,.12]);ref=truth[prefix+'Force'][:,a]
   for j,g in enumerate(groups):ax.plot(range(1,T+1),values(g,prefix+'F'+axis),color=COLORS[j],lw=1.8)
   ax.plot(range(1,T+1),ref,color='#273347',ls='--',lw=1.5)
   ax.set_title(('Tip load ' if prefix=='tip' else 'Contact force ')+axis.upper()+' [N]',fontsize=11,weight='bold');ax.set_xlim(1,T);ax.grid(alpha=.2);ax.set_xlabel('Frame',fontsize=9);ax.tick_params(labelsize=8)
   ax.axvline(13 if rotation else 21,color='gray',ls=':',alpha=.4);cursors.append(ax.axvline(1,color='#b72b38',lw=1.3))
 fig.text(.04,.03,'Conditional mode search | FBG RMS gate 1e-6 /mm | Saved results; all fallbacks and warnings retained | Playback is illustrative, not real time.',fontsize=10)
 sc=truth['contactS'].ravel()
 def setline(line,a):
  if rotation:line.set_data_3d(a[:,0],a[:,1],a[:,2])
  else:line.set_data(a[:,0],a[:,2])
 def update(k):
  desc=(f"{'FORWARD' if k<=12 else 'REVERSE'} ROTATION | angle {angles[k]:.2f} degrees | fixed wall distance 40 mm" if rotation else f"{'INSERTION' if k<=20 else 'WITHDRAWAL'} | wall distance {truth['distanceMm'].ravel()[k]:.0f} mm")
  subtitle.set_text(f'{desc} | frame {k+1}/{T}'+(f' | reference: {reference_modes[k]}' if rotation and k else ''))
  for j,g in enumerate(groups):
   q=g[k];actual=truth['p'][k];setline(lines[j][0],actual);setline(lines[j][1],p[j,k]);c=cp[j][k]
   if np.isfinite(sc[k]):setline(dots[j],np.array([[np.interp(sc[k],np.arange(201),actual[:,a]) for a in range(3)]]))
   else:setline(dots[j],np.zeros((0,3)))
   setline(contact[j],c if c is not None else np.zeros((0,3)))
   mode={'0':'separation','1':'sticking','2':'sliding','3':'frictionless','zeros(1,0)':'free'}.get(q['mode'],q['mode']);accepted=q['modeAccepted']=='1';label='FIT OK' if accepted else 'FIT REJECTED (fallback)'
   if q['reason']=='saved-insertion-state':label='SAVED INSERTION STATE'
   flag=q['flags'] or 'none';flag=flag[:79]+'...' if len(flag)>82 else flag
   status[j].set_text(f"{mode} | {label} | {float(q['seconds']):.2f} s | {q['ivps']} IVPs\nFBG RMS {float(q['curvatureRmsPerMm']):.2e} /mm | shape RMS {float(q['shapeRmsMm']):.4f} mm\nTip error {float(q['tipErrorN']):.4f} N | contact error {float(q['contactErrorN']):.4f} N\nFlags: {flag}")
   status[j].set_color('#253347' if accepted else '#a42936')
  for c in cursors:c.set_xdata([k+1,k+1])
 matplotlib.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe();writer=FFMpegWriter(fps=12,codec='libx264',bitrate=5000,extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
 name='rotation_force_comparison.mp4' if rotation else 'insertion_force_comparison.mp4'
 with writer.saving(fig,str(out/name),120):
  for k in range(T):
   update(k)
   for _ in range(8):writer.grab_frame()
   if k in [0,T//2,T-1]:fig.savefig(out/f'frame_{k+1:02d}.png',dpi=120)
 plt.close(fig)
 if not rotation:
  seedpub=ROOT/'results/published/inclined_wall_rotation_20261010';seedpub.mkdir(parents=True,exist_ok=True);shutil.copy2(out/'frame_21.png',seedpub/'maximum_insertion_frame.png')
 (out/'video_metadata.json').write_text(json.dumps(dict(frames=T,encodedFrames=T*8,fps=12,durationSeconds=T*8/12,size=[1920,1440],forcePlots='World XYZ components: reference, mu=0.3 estimate, mu=0 estimate',sourceRun=tag),indent=2))
 print(out/name)
if __name__=='__main__':main()
