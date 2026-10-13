"""Render saved conditional-mode trajectories, preserving fit-failure labels."""
from pathlib import Path
import argparse,json,hashlib
import h5py
import numpy as np
from publish_inclined import ROOT,csv_rows,numeric,render

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--run-id',default='inclined_wall_modes_conditional_1e6_20261010');args=parser.parse_args()
 source=ROOT/'results/runs/inclined_wall_bidirectional_20261008/inclined_wall'
 run=ROOT/'results/runs'/args.run_id;pub=ROOT/'results/published'/args.run_id;out=pub/'video';out.mkdir(exist_ok=True)
 rows=csv_rows(pub/'estimates.csv');refs=csv_rows(source/'truth_frames.csv');assert len(rows)==82 and len(refs)==41
 groups=[[r for r in rows if int(r['method'])==j+1] for j in range(2)]
 with h5py.File(source/'truth.mat','r') as f:
  truth={k:np.array(f['truth'][k]) for k in ['p','contactS','contactForce','basePose','distanceMm']}
 p=np.zeros((2,41,201,3));cp=[[None]*41 for _ in range(2)]
 with h5py.File(run/'comparison.mat','r') as f:
  for j in range(2):
   for k in range(41):
    sh=f[f['runs'][k,j]]['mechanics/evaluation/shape'];p[j,k]=sh['p'][:]
    if not sh['contactP'].attrs.get('MATLAB_empty',False):cp[j][k]=sh['contactP'][:]
 for j in range(2):
  assert [int(r['frame']) for r in groups[j]]==list(range(1,42))
  assert np.allclose(np.sqrt(np.mean(np.sum((p[j]-truth['p'])**2,axis=2),axis=1)),numeric(groups[j],'shapeRmsMm'))
 render(truth,groups,refs,p,cp,out,conditional=True)
 meta=dict(run=args.run_id,frames=41,encodedFrames=328,fps=12,durationSeconds=328/12,projection='XZ; full-3D Y error shown',thresholdRawRmsPerMm=1e-6,sourceCsvSha256=hashlib.sha256((pub/'estimates.csv').read_bytes()).hexdigest(),notes='Saved estimates only. Mu=0 fit failures displayed as fallback in red. Playback is illustrative, not real time.')
 (out/'video_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
 print(out/'inclined_wall_comparison.mp4')
if __name__=='__main__':main()
