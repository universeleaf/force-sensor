"""Render the unfiltered inclined-wall experiment; no solver or data changes."""
from pathlib import Path
import argparse, csv, json, hashlib, shutil
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[2]
COLORS = ['#087f8c', '#d26927']
NAMES = ['Ours: assumed mu = 0.3', 'Ours: assumed mu = 0']

def csv_rows(path):
    with path.open(newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))

def numeric(rows, key):
    return np.array([float(r[key]) for r in rows])

def truthy(value):
    return str(value).lower() in ('1', 'true')

def read_data(run, allow_incomplete=False):
    with h5py.File(run / 'truth.mat', 'r') as f:
        t = f['truth']
        truth = {k: np.array(t[k]) for k in ['p', 'contactS', 'contactForce', 'basePose', 'distanceMm']}
    rows = csv_rows(run / 'estimates.csv')
    refs = csv_rows(run / 'truth_frames.csv')
    assert (len(rows) == 82 or allow_incomplete) and len(refs) == 41, 'Finish both full trajectories before publishing.'
    groups = [[r for r in rows if int(r['method']) == j+1] for j in range(2)]
    actual_groups = [list(g) for g in groups]
    for j in range(2):
        for k in range(len(groups[j]),41):
            missing={key:'nan' for key in rows[0]}
            missing.update(method=str(j+1),mu=str([.3,0][j]),frame=str(k+1),converged='0',relaxedFeasible='0',reason='exception' if k==len(actual_groups[j]) else 'not-attempted',flags='IVP RHS budget exceeded' if k==len(actual_groups[j]) else 'not attempted after exception')
            groups[j].append(missing)
    p = np.full((2, 41, 201, 3),np.nan); cp = [[None]*41 for _ in range(2)]
    with h5py.File(run / 'comparison.mat', 'r') as f:
        for j in range(2):
            for k in range(len(actual_groups[j])):
                r = f[f['runs'][k, j]]
                sh = r['mechanics/evaluation/shape']
                p[j,k] = sh['p'][:]
                if not sh['contactP'].attrs.get('MATLAB_empty', False):
                    cp[j][k] = sh['contactP'][:]
    for j in range(2):
        assert [int(r['frame']) for r in groups[j]] == list(range(1,42))
        assert all(int(r['objectiveTerms']) == 48 for r in actual_groups[j])
        count=len(actual_groups[j])
        assert np.allclose(np.sqrt(np.mean(np.sum((p[j,:count]-truth['p'][:count])**2,axis=2),axis=1)), numeric(actual_groups[j],'shapeRmsMm'))
    for key in ['estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm']:
        count=min(map(len,actual_groups))
        assert np.allclose(numeric(groups[0][:count],key), numeric(groups[1][:count],key),equal_nan=True), key
    return truth, groups, refs, p, cp

def summarize(groups, refs):
    active = numeric(refs, 'normalN') > 1e-8
    summary = []
    for j, rows in enumerate(groups):
        for scope, mask in [('all', np.ones(41,dtype=bool)), ('true_contact', active)]:
            sel = [r for r,b in zip(rows,mask) if b and r['reason'].startswith('fmincon')]
            if not sel: continue
            result = dict(method=NAMES[j], scope=scope, frames=len(sel), scheduledFrames=int(mask.sum()), unavailableFrames=int(mask.sum())-len(sel))
            for key in ['seconds','geometrySeconds','mechanicsSeconds','iterations','ivps','tipErrorN','contactErrorN','shapeRmsMm']:
                x=numeric(sel,key)
                result[key+'_mean']=float(np.mean(x)); result[key+'_median']=float(np.median(x)); result[key+'_max']=float(np.max(x))
            for key in ['tipErrorN','contactErrorN','shapeRmsMm']:
                result[key+'_rms']=float(np.sqrt(np.mean(numeric(sel,key)**2)))
            result['nativePositive']=sum(truthy(r['converged']) for r in sel)
            result['exit1']=sum(r['reason']=='fmincon-exit-1' for r in sel)
            result['exit2']=sum(r['reason']=='fmincon-exit-2' for r in sel)
            result['relaxedFeasible']=sum(truthy(r['relaxedFeasible']) for r in sel)
            result['diagnosticFlagFrames']=sum(bool(r['flags']) for r in sel)
            result['exactLawFlagFrames']=sum(bool(r['exactLawFlags']) for r in sel)
            summary.append(result)
    return summary

def render(truth, groups, refs, p, cp, published, conditional=False):
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':11, 'axes.spines.top':False,
        'axes.spines.right':False,'axes.labelcolor':'#253347','text.color':'#253347',
        'axes.titleweight':'bold','figure.facecolor':'#fafbfC'})
    fig=plt.figure(figsize=(16,9), dpi=120)
    fig.text(.045,.958,'CONDITIONAL MODES  |  inclined-wall insertion and withdrawal' if conditional else 'INCLINED WALL  |  insertion and withdrawal',fontsize=23,weight='bold')
    fig.text(.045,.922,'200 mm rod: 80 mm straight + 120 mm / 90 degree precurve   |   tip load +X, 1 N   |   noiseless Bishop measurements',fontsize=11)
    subtitle=fig.text(.045,.886,'',fontsize=14,weight='bold')
    axs=[fig.add_axes([.065,.42,.39,.42]),fig.add_axes([.565,.42,.39,.42])]
    lines=[]; dots=[]; bases=[]; contact=[]; status=[]
    xx=np.linspace(-15,145,100); wall=xx+80*np.sqrt(2)
    for j,ax in enumerate(axs):
        ax.fill_between(xx,wall,240,color='#dbe1e8',alpha=.65)
        ax.plot(xx,wall,color='#667788',lw=2,label='Fixed 45 degree wall')
        ax.set(xlim=(-15,145),ylim=(-5,190),xlabel='World X [mm]',ylabel='World Z [mm]')
        ax.set_aspect('equal', adjustable='box');ax.grid(alpha=.12)
        ax.set_title(NAMES[j],color=COLORS[j],fontsize=16)
        line1,=ax.plot([],[],color='#273347',lw=3,alpha=.8,label='True shape')
        line2,=ax.plot([],[],color=COLORS[j],lw=2,ls='--',label='Estimated shape')
        base,=ax.plot([],[],'s',color='#273347',ms=7)
        dot,=ax.plot([],[],'o',mfc='white',mec='#273347',ms=9,mew=2)
        c,=ax.plot([],[],'x',color=COLORS[j],ms=9,mew=2)
        lines.append((line1,line2));dots.append(dot);bases.append(base);contact.append(c)
        ax.legend(loc='upper left',fontsize=8,framealpha=.95)
        status.append(fig.text(.065+.5*j,.365 if conditional else .34,'',va='top',fontsize=10,linespacing=1.5,family='DejaVu Sans Mono'))
    small=[fig.add_axes([.06,.10,.26,.13]),fig.add_axes([.385,.10,.26,.13]),fig.add_axes([.71,.10,.25,.13])]
    for ax,title in zip(small,['Tip-force error [N]','Contact-force error [N]','True contact loads [N]']):
        ax.set_title(title,fontsize=12);ax.set_xlim(1,41);ax.set_xlabel('Frame (insertion 1-21, withdrawal 22-41)',fontsize=9);ax.grid(alpha=.2)
        ax.axvline(21,color='#667788',lw=1,ls=':')
    for a,key in zip(small[:2],['tipErrorN','contactErrorN']):
        for j in range(2):a.semilogy(range(1,42),np.maximum(numeric(groups[j],key),1e-5),color=COLORS[j],lw=1.8,label='mu = '+str([.3,0][j]))
        a.set_ylim(1e-5, max(100,max(np.nanmax(numeric(g,key)) for g in groups)*2));a.legend(fontsize=8,loc='upper left',ncol=2)
    small[2].plot(range(1,42),numeric(refs,'normalN'),color='#273347',lw=2,label='Normal')
    small[2].plot(range(1,42),numeric(refs,'tangentN'),color='#78549a',lw=2,label='Signed tangent')
    small[2].legend(fontsize=8,ncol=2,loc='upper right')
    cursors=[a.axvline(1,color='#c03d45',lw=1.5) for a in small]
    fig.text(.045,.025,'Same frictional truth (mu = 0.3) for both estimators. XZ projection; Y error reported. All failures retained. Playback is illustrative, not real-time.',fontsize=10)
    sc=truth['contactS'].ravel()
    def update(k):
        r=refs[k]; direction='INSERTION' if k<=20 else 'WITHDRAWAL'
        arc=f"{sc[k]:.1f} mm" if np.isfinite(sc[k]) else 'no contact'
        subtitle.set_text(f"{direction}   |   frame {k+1:02d}/41   |   perpendicular wall distance {float(r['distanceMm']):.0f} mm   |   true contact s = {arc}")
        for j in range(2):
            q=groups[j][k];a=truth['p'][k];b=p[j,k]
            lines[j][0].set_data(a[:,0],a[:,2]);lines[j][1].set_data(b[:,0],b[:,2]);bases[j].set_data([a[0,0]],[a[0,2]])
            if np.isfinite(sc[k]):
                xy=np.array([np.interp(sc[k],np.arange(201),a[:,v]) for v in [0,2]])
                dots[j].set_data([xy[0]],[xy[1]])
            else:dots[j].set_data([],[])
            if cp[j][k] is not None: contact[j].set_data(cp[j][k][:,0],cp[j][k][:,2])
            else:contact[j].set_data([],[])
            if q['reason'] in ('exception','not-attempted'):
                status[j].set_text('NO ESTIMATE: unchanged mu = 0 run crashed at frame 4.\nIVP RHS budget exceeded on an optimizer trial.\nSubsequent frames were not attempted.\nNo held or substituted estimate is displayed.')
                status[j].set_color('#9d2834')
                continue
            reason=q['reason'].replace('fmincon-exit-','')
            labels={'1':'optimality exit','2':'small-step exit','0':'budget exhausted','-2':'infeasible exit'}
            state=labels.get(reason,'exit '+reason)
            if not truthy(q['modeFeasible'] if conditional else q['relaxedFeasible']):state+=' / infeasible'
            fitline=''
            if conditional:
                mode={'0':'separation','1':'sticking','2':'sliding','3':'frictionless contact','zeros(1,0)':'free'}.get(q['mode'],q['mode'])
                state=mode+' / '+('accepted' if truthy(q['modeAccepted']) else 'FIT REJECTED: fallback')
                fitline=f"FBG RMS {float(q['curvatureRmsPerMm']):.2e} /mm | gate 1e-6 /mm\n"
            flag=q['flags'] or 'none'
            if len(flag)>57:flag=flag[:54]+'...'
            status[j].set_text(f"{state} | {float(q['seconds']):.2f} s | {q['iterations']} iter | {q['ivps']} IVPs\n" +
                fitline+f"Tip error {float(q['tipErrorN']):.3f} N | contact error {float(q['contactErrorN']):.3f} N\n"
                f"Shape RMS {float(q['shapeRmsMm']):.4f} mm | max |Y| {np.max(np.abs(b[:,1])):.4f} mm\nFlags: {flag}")
            status[j].set_color('#9d2834' if not truthy(q['converged']) or (conditional and not truthy(q['modeAccepted'])) else '#253347')
        for cur in cursors:cur.set_xdata([k+1,k+1])
    matplotlib.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe()
    writer=FFMpegWriter(fps=12,codec='libx264',bitrate=4500,extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
    with writer.saving(fig,str(published/'inclined_wall_comparison.mp4'),120):
        for k in range(41):
            update(k)
            for _ in range(8):writer.grab_frame()
            if k in [3,14,20,21,40]:fig.savefig(published/f'frame_{k+1:02d}.png',dpi=120)
    plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    keys=['tipErrorN','contactErrorN','shapeRmsMm','seconds','iterations','ivps']
    titles=['Tip-force error [N]','Contact-force error [N]','Centerline RMS error [mm]','Solve + diagnosis time [s]','SQP iterations','Augmented IVP evaluations']
    for ax,key,title in zip(axes.ravel(),keys,titles):
        for j in range(2):ax.plot(range(1,42),numeric(groups[j],key),color=COLORS[j],label=NAMES[j])
        ax.axvline(21,color='#777777',ls=':');ax.set_title(title);ax.set_xlabel('Frame');ax.grid(alpha=.2)
        if key in ['tipErrorN','contactErrorN','shapeRmsMm','seconds']:ax.set_yscale('symlog',linthresh=1e-3)
    axes[0,0].legend(fontsize=9);fig.suptitle('Inclined wall: every frame, including solver failures',fontsize=17)
    fig.savefig(published/'comparison_metrics.png',dpi=160);plt.close(fig)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run-id',default='inclined_wall_bidirectional_20261008');parser.add_argument('--allow-incomplete',action='store_true');args=parser.parse_args()
    run=ROOT/'results/runs'/args.run_id/'inclined_wall';pub=ROOT/'results/published'/args.run_id
    if args.allow_incomplete:pub=pub/'unreduced_attempt'
    pub.mkdir(parents=True,exist_ok=True)
    truth,groups,refs,p,cp=read_data(run,args.allow_incomplete);summary=summarize(groups,refs)
    (pub/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    with (pub/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    for name in ['configuration.json','truth_frames.csv','estimates.csv']:shutil.copy2(run/name,pub/name)
    files=list((ROOT/'experiments/inclined_wall').glob('*.m'))+list((ROOT/'matlab/+tsfs').glob('*.m'))+[Path(__file__)]
    (pub/'source_hashes.json').write_text(json.dumps({str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2),encoding='utf-8')
    render(truth,groups,refs,p,cp,pub)
    print(json.dumps(summary,indent=2));print('Published:',pub)

if __name__=='__main__':main()
