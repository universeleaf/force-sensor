"""Decode all eight MP4s, verify frame counts, and build the local media gallery."""
from pathlib import Path
import json,hashlib,sys
import imageio_ffmpeg
ROOT=Path(__file__).resolve().parents[2]
STAMP=sys.argv[1] if len(sys.argv)>1 else '20261012_step5'
OUT=ROOT/'results/published'/f'fine_bvp_comparison_{STAMP}'/'media'
records=json.loads((OUT/'manifest.json').read_text());assert len(records)==8
verified=[];cards=[]
for r in records:
 level=str(r['noiseTargetMm']).removesuffix('.0').replace('.','p');name=f"{r['scenario']}_{level}mm";folder=OUT/name
 n,sec=imageio_ffmpeg.count_frames_and_secs(str(folder/'trajectory.mp4'))
 assert n==r['encodedFrames'] and abs(sec-r['durationSeconds'])<.11,(name,n,sec)
 assert r['omittedFrames']==0
 entry=dict(case=name,decodedFrames=n,seconds=sec,sha256=hashlib.sha256((folder/'trajectory.mp4').read_bytes()).hexdigest());verified.append(entry)
 links=' '.join(f'<a href="{name}/{fn}.png">{label}</a>' for fn,label in [('force_components','Force components'),('errors_timing_physics','Errors / timing / physics'),('geometry_stage','Stage 1 geometry')])
 cards.append(f'<section><h2>{r["scenario"].capitalize()} · {r["noiseTargetMm"]:g} mm noise</h2><video controls preload="metadata" poster="{name}/frame_001.png" src="{name}/trajectory.mp4"></video><p>{links}</p><p>{r["sourceFrames"]} source frames · {sec:g} seconds playback · failed BVP frames: {r["bvpFailedFrames"] or "none"}</p></section>')
 print('VERIFIED',name,n,sec,flush=True)
(OUT/'validation.json').write_text(json.dumps(verified,indent=2))
(OUT/'index.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Fine-step force sensing</title><style>body{font:16px system-ui;color:#263445;background:#f5f7fa;margin:30px auto;max-width:1400px;padding:0 20px}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(450px,1fr));gap:24px}section{background:white;padding:20px;border-radius:12px}video{width:100%}a{color:#087c93;margin-right:16px}p{line-height:1.6}h1{margin-bottom:8px}</style><h1>Fine-step force sensing</h1><p>Insertion: 0.4 mm steps. Rotation: 0.75° steps. One rod linearization, exhaustive full circular contact modes, one fixed-load BVP correction. Every saved frame is shown, including failures. Playback is illustrative, not physical time.</p><p><a href="../comparison.png">Coarse versus fine comparison</a><a href="README.md">Media index</a><a href="../summary.csv">Summary data</a></p><main>'+''.join(cards)+'</main></html>',encoding='utf-8')
print('All eight videos decoded successfully; gallery and checksums saved.')
