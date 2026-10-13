"""Fix an inherited descriptive flag in this benchmark's polygon artifacts.
No numerical result is modified. Retain prior JSON and verify state-array hashes.
"""
from pathlib import Path
import hashlib,json,sys
import h5py,numpy as np
ROOT=Path(__file__).resolve().parents[2];STAMP=sys.argv[1] if len(sys.argv)>1 else '20261012_unbounded'
def numerical_hash(f):
 h=hashlib.sha256()
 for ref in f['runs'][()].ravel():
  if not ref:continue
  o=f[ref];m=o['mechanics'];e=m['evaluation']
  for g,key in [(m,'z'),(e,'r'),(e['shape'],'x'),(e['shape'],'p'),(e['shape'],'prediction')]:h.update(g[key][()].tobytes())
 return h.hexdigest()
notes=[]
for p in sorted((ROOT/'results/runs').glob(f'singlepass_polygon_*_{STAMP}/comparison.mat')):
 pub=ROOT/'results/published'/p.parent.name;jp=pub/'configuration.json';cfg=json.loads(jp.read_text(encoding='utf-8-sig'))
 with h5py.File(p,'r+') as f:
  before=numerical_hash(f);names=[]
  def visit(name,obj):
   if isinstance(obj,h5py.Dataset) and name.split('/')[-1]=='exactCircularFriction' and obj.size==1 and obj.dtype!=h5py.ref_dtype and bool(obj[()].ravel()[0]):names.append(name)
  f.visititems(visit)
  for name in names:f[name][...]=np.zeros(f[name].shape,dtype=f[name].dtype)
  after=numerical_hash(f);assert before==after
 if cfg.get('exactCircularFriction') is not False:
  old=pub/'configuration_before_label_correction.json'
  if not old.exists():old.write_bytes(jp.read_bytes())
  cfg['exactCircularFriction']=False;jp.write_text(json.dumps(cfg,indent=2))
 notes.append(dict(run=p.parent.name,correctedFields=len(names),stateArraysSHA256=after))
out=ROOT/'results/published'/f'singlepass_comparison_{STAMP}';out.mkdir(parents=True,exist_ok=True);(out/'metadata_label_correction.json').write_text(json.dumps(notes,indent=2));print(json.dumps(notes,indent=2))
