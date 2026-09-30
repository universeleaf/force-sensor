"""Summarize paired multi-contact MATLAB trials without hiding failed estimates.

Usage: python scripts/summarize_multi_contact_benchmark.py
"""

from __future__ import annotations

import json
import math
import random
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "out/benchmarks/multi_contact/comparison.json"
OUTPUT = ROOT / "out/benchmarks/multi_contact/summary.md"
METHOD_NAMES = {
    "EnFiRCE_environment": "EnFiRCE (environment)",
    "shape_only_point_loads": "Shape-only point loads",
    "ablation_no_gap": "No contact-gap penalty",
    "ablation_no_tangency": "No tangency penalty",
    "ablation_no_geometry_penalties": "No gap/tangency penalties",
    "ablation_plane_offset_1mm": "One plane offset by 1 mm",
    "fixed_shifted_plane": "Shifted plane, treated as exact",
    "latent_plane_std_0p1mm": "Latent plane offset, prior SD 0.1 mm",
    "latent_plane_std_1mm": "Latent plane offset, prior SD 1 mm",
}


def rmse(cases: list[dict], field: str) -> float:
    values = [case[field] for case in cases if case.get("completed") and case.get(field) is not None]
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def mean(cases: list[dict], field: str) -> float:
    values = [case[field] for case in cases if case.get("completed") and case.get(field) is not None]
    return sum(values) / len(values) if values else math.nan


def seed_bootstrap(cases: list[dict], field: str, repeats: int = 2000) -> tuple[float, float]:
    seeds = sorted({case["seed"] for case in cases})
    if len(seeds) < 2:
        return math.nan, math.nan
    groups = {seed: [case for case in cases if case["seed"] == seed] for seed in seeds}
    generator = random.Random(1809)
    values = []
    for _ in range(repeats):
        sampled = [case for _seed in generator.choices(seeds, k=len(seeds)) for case in groups[_seed]]
        values.append(rmse(sampled, field))
    values.sort()
    return values[int(0.025 * repeats)], values[int(0.975 * repeats) - 1]


def row(cases: list[dict], method: str, *, interval: bool = False) -> str:
    selected = [case for case in cases if case["method"] == method]
    passed = [case for case in selected if case.get("completed")]
    contact = rmse(passed, "contactForceRmseN")
    ci = seed_bootstrap(passed, "contactForceRmseN") if interval else None
    value = f"{contact:.2e}" if abs(contact) < 0.001 else f"{contact:.3f}"
    if ci and all(math.isfinite(item) for item in ci):
        bounds = [f"{number:.2e}" if abs(number) < 0.001 else f"{number:.3f}" for number in ci]
        value += f" [{bounds[0]}, {bounds[1]}]"
    reviews = sum(bool(case.get("requiresReview")) for case in passed)
    return (
        f"| {METHOD_NAMES[method]} | {len(passed)}/{len(selected)} | "
        f"{value} | {rmse(passed, 'contactArcRmseMm'):.3f} | "
        f"{rmse(passed, 'totalForceErrorN'):.3f} | "
        f"{mean(passed, 'seconds'):.3f} | {reviews}/{len(passed)} |"
    )


def table(cases: list[dict], methods: list[str], *, interval: bool = False) -> list[str]:
    result = [
        "| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    result.extend(row(cases, method, interval=interval) for method in methods)
    return result


def main() -> None:
    report = json.loads(SOURCE.read_text(encoding="utf-8"))
    cases = report["cases"]
    standard = ["EnFiRCE_environment", "shape_only_point_loads"]
    nominal = [case for case in cases if case["sensorCount"] == 24 and case["noiseStdPerMm"] == 2.5e-5]
    lines = [
        "# Multi-contact software benchmark",
        "",
        "Generated from `comparison.json` by `scripts/summarize_multi_contact_benchmark.py`.",
        f"State: `{report['state']}`; {len(cases)} estimates; {sum(not c.get('completed') for c in cases)} failed estimates.",
        "Forward truth is independently solved planar, frictionless rod equilibrium in fixed two- or three-contact channels. "
        "Every comparison pair uses the same sparse curvature packet, stiffness and base pose. Both methods are told contact count/order; "
        "only EnFiRCE receives plane points/normals. These are internally generated scenes, not original-paper benchmark datasets.",
        "",
        "Contact force RMSE is the root mean square Euclidean vector error over contacts and frames; "
        "total force RMSE uses the resultant including the independent tip load. "
        "Brackets are a descriptive 95% bootstrap interval resampling the three random seeds as clusters; "
        "two frames per seed are correlated and three seeds do not establish population confidence or superiority.",
        "Review flags come from each method's own numerical/physical checks and are not an equivalent accuracy classifier across methods.",
        "",
        "## Nominal noise: 24 FBG observations, 2.5e-5 /mm",
        "",
    ]
    lines.extend(table(nominal, standard, interval=True))
    for scene_id in report["sceneIds"]:
        lines.extend(["", f"### {scene_id}", ""])
        lines.extend(table([case for case in nominal if case["sceneId"] == scene_id], standard, interval=True))
    lines.extend(["", "## Sensor density and noise", ""])
    for noise in report["noiseStdPerMm"]:
        for count in report["sensorCounts"]:
            subset = [case for case in cases if case["noiseStdPerMm"] == noise and case["sensorCount"] == count]
            lines.extend([f"### Noise {noise:g} /mm; {count} observations", ""])
            lines.extend(table(subset, standard, interval=True))
            lines.append("")
    lines.extend(["## Geometry ablations at nominal noise", ""])
    lines.extend(table(nominal, list(report["methods"]), interval=True))
    matched = defaultdict(dict)
    for case in nominal:
        matched[(case["sceneId"], case["seed"], case["frameIndex"])][case["method"]] = case
    paired = [pair for pair in matched.values() if all(pair.get(method, {}).get("completed") for method in standard)]
    wins = sum(pair["EnFiRCE_environment"]["contactForceRmseN"] < pair["shape_only_point_loads"]["contactForceRmseN"] for pair in paired)
    lines.extend([
        "",
        f"At nominal noise, EnFiRCE has lower contact-vector error on {wins}/{len(paired)} paired frames. "
        "This is an internal comparison, not evidence of SOTA against published systems.",
        "",
        "## Interpretation boundary",
        "",
        "These tests do not reproduce published multi-contact systems or compare them under identical sensing, "
        "hardware, contact-mode uncertainty and ground-truth conditions. The estimator assumes known contact count/order, "
        "planarity and frictionless point contacts. A one-millimetre plane-offset run probes calibration sensitivity; "
        "it does not establish a general robustness threshold. Solver times exclude forward-truth generation, MATLAB startup, "
        "video rendering and file I/O. Full run inputs and numerical outputs are saved in each scene's `truth.mat` and `trials.mat`.",
        "",
    ])
    geometry_path = SOURCE.with_name("plane_uncertainty.json")
    if geometry_path.exists():
        geometry = json.loads(geometry_path.read_text(encoding="utf-8"))
        if geometry.get("state") == "complete":
            lines.extend([
                "## Plane-calibration sensitivity replay",
                "",
                "The first plane point is displaced by +1 mm along its normal in the inverse input; the forward truth and curvature packets are unchanged. "
                "For the latent variants, the plane point offset is an estimated state with a zero-mean Gaussian prior, "
                "not the ground-truth displacement. These prior scales were fixed before scoring.",
                "",
            ])
            lines.extend(table(geometry["cases"], geometry["methods"], interval=True))
            lines.append("")
            for method in geometry["methods"][1:]:
                offsets = [case["planePointOffsetMm"][0] for case in geometry["cases"] if case["method"] == method and case.get("completed")]
                lines.append(f"{METHOD_NAMES[method]}: mean inferred first-plane offset {sum(offsets) / len(offsets):.3f} mm (true correction −1 mm).")
            lines.append("")
    OUTPUT.write_text("\n".join(lines), encoding="utf-8")
    print(OUTPUT)


if __name__ == "__main__":
    main()
