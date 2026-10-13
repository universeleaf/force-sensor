# Fine-step rod plus BVP videos and plots

Every saved frame is shown. Playback is illustrative, not physical time. Failed corrections remain visible and are flagged. Force curves show world XYZ components. Static plots use full ranges without clipping outliers.

| Case | Video | Forces | Errors, timing and physics | Geometry |
|---|---|---|---|---|
| insertion_0mm | [MP4](insertion_0mm/trajectory.mp4) | [PNG](insertion_0mm/force_components.png) | [PNG](insertion_0mm/errors_timing_physics.png) | [PNG](insertion_0mm/geometry_stage.png) |
| insertion_0p5mm | [MP4](insertion_0p5mm/trajectory.mp4) | [PNG](insertion_0p5mm/force_components.png) | [PNG](insertion_0p5mm/errors_timing_physics.png) | [PNG](insertion_0p5mm/geometry_stage.png) |
| insertion_1mm | [MP4](insertion_1mm/trajectory.mp4) | [PNG](insertion_1mm/force_components.png) | [PNG](insertion_1mm/errors_timing_physics.png) | [PNG](insertion_1mm/geometry_stage.png) |
| insertion_2mm | [MP4](insertion_2mm/trajectory.mp4) | [PNG](insertion_2mm/force_components.png) | [PNG](insertion_2mm/errors_timing_physics.png) | [PNG](insertion_2mm/geometry_stage.png) |
| rotation_0mm | [MP4](rotation_0mm/trajectory.mp4) | [PNG](rotation_0mm/force_components.png) | [PNG](rotation_0mm/errors_timing_physics.png) | [PNG](rotation_0mm/geometry_stage.png) |
| rotation_0p5mm | [MP4](rotation_0p5mm/trajectory.mp4) | [PNG](rotation_0p5mm/force_components.png) | [PNG](rotation_0p5mm/errors_timing_physics.png) | [PNG](rotation_0p5mm/geometry_stage.png) |
| rotation_1mm | [MP4](rotation_1mm/trajectory.mp4) | [PNG](rotation_1mm/force_components.png) | [PNG](rotation_1mm/errors_timing_physics.png) | [PNG](rotation_1mm/geometry_stage.png) |
| rotation_2mm | [MP4](rotation_2mm/trajectory.mp4) | [PNG](rotation_2mm/force_components.png) | [PNG](rotation_2mm/errors_timing_physics.png) | [PNG](rotation_2mm/geometry_stage.png) |

shapes.npz is a compact rendering source; published estimates.csv and combined frames.csv provide forces/diagnostics. This allows rendering without the large local MATLAB checkpoints. No solver rerun is performed.
