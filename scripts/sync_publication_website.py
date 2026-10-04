"""Copy verified completed evidence into the separate project website checkout.

No Git mutation or publishing occurs here. Local drafts and failed runs cannot
be synchronized as completed evidence. Copied MP4 bytes retain their identity.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

from render_formulation_factors import ROOT, sha, nonpositive_exit
from verify_publication_release import verify
from compile_publication_manuscript import verify_build


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sync(site: Path) -> None:
    site = site.resolve()
    if not site.is_relative_to(ROOT) or not (site / ".git").is_dir():
        raise ValueError("Use a separate website Git checkout inside this workspace.")
    verify()
    verify_build()
    publication = ROOT / "out/benchmarks/publication"
    completion = read(publication / "completion.json")
    if completion["state"] != "complete" or completion["runRecord"]["state"] != "complete":
        raise ValueError("The publication workflow is still incomplete.")
    for record in completion["runRecord"]["source"] + completion["runRecord"]["dependency"]["files"]:
        source = (ROOT / record["path"]).resolve()
        if not source.is_relative_to(ROOT) or sha(source) != record["sha256"]:
            raise ValueError(f"Execution source changed: {record['path']}")
    steps = {v["name"]: v for v in completion["completedSteps"]}
    if set(steps) != {"factors", "literature", "derivatives", "engineering"}:
        raise ValueError("Required software experiment steps are missing.")
    reports = {}
    for name, step in steps.items():
        source = (ROOT / step["path"]).resolve()
        if not source.is_relative_to(ROOT) or sha(source) != step["sha256"]:
            raise ValueError(f"Completed step checksum mismatch: {name}")
        reports[name] = read(source)
    checks = reports["engineering"]
    if not checks["allPassed"] or not all(c["passed"] for c in checks["checks"]):
        raise ValueError("Engineering checks did not all pass.")
    summary = read(publication / "paired_summary.json")
    figure_provenance = read(publication / "figure_provenance.json")
    if summary["state"] != "complete":
        raise ValueError("Comparison plots have not been generated.")
    for record in figure_provenance["sourceLedgers"] + figure_provenance["outputs"]:
        source = (ROOT / record["path"]).resolve()
        if not source.is_relative_to(ROOT) or sha(source) != record["sha256"]:
            raise ValueError(f"Figure provenance mismatch: {record['path']}")
    destination = site / "static/results"
    destination.mkdir(parents=True, exist_ok=True)
    copied = []

    def copy(source: Path, name: str | None = None, scope: str = "") -> None:
        source = source.resolve()
        if not source.is_relative_to(ROOT) or not source.is_file():
            raise ValueError(f"Missing/outside publication artifact: {source}")
        target = destination / (name or source.name)
        shutil.copyfile(source, target)
        if sha(source) != sha(target):
            raise ValueError(f"Copied bytes differ: {source}")
        copied.append({"source": source.relative_to(ROOT).as_posix(),
                       "websitePath": target.relative_to(site).as_posix(),
                       "sha256": sha(target), "bytes": target.stat().st_size, "scope": scope})

    for name in ("matched_baselines", "contact_magnitudes", "method_overview", "local_uncertainty"):
        for extension in ("svg", "png", "pdf"):
            copy(publication / "figures" / f"{name}.{extension}")
    for name in ("factor_accuracy", "coverage_runtime"):
        for extension in ("svg", "png", "pdf"):
            copy(ROOT / "out/benchmarks/formulation_factors/figures" / f"{name}.{extension}")
    for extension in ("svg", "png", "pdf"):
        copy(ROOT / "out/benchmarks/literature/figures" / f"solved_contact_geometry.{extension}",
             scope="Historical completed full-window solved geometry; not a newly generated window.")
    for name in ("paired_source_data.csv", "contact_magnitude_source_data.csv", "paired_summary.json", "figure_provenance.json", "completion.json", "runtime_host.json"):
        copy(publication / name)
    copy(ROOT / "out/benchmarks/formulation_factors/source_data.csv", "factor_source_data.csv")
    copy(ROOT / "out/benchmarks/formulation_factors/plot_provenance.json", "factor_plot_provenance.json")
    copy(ROOT / "out/benchmarks/formulation_derivatives/comparison.json", "derivative_comparison.json")
    copy(publication / "EnFiRCE_draft.pdf", scope="Compiled research draft; author information unset.")
    copy(publication / "manuscript_build.json")
    single_path = ROOT / "out/demos/latest/comparison.json"
    multiple_path = ROOT / "out/demos/multi_contact/comparison.json"
    single, multiple = read(single_path), read(multiple_path)
    for ledger in (single, multiple):
        if ledger["state"] != "complete" or not all(c["completed"] for c in ledger["cases"]):
            raise ValueError("Continuous demo ledger is incomplete.")
    copy(single_path, "single_contact_video_ledger.json")
    copy(multiple_path, "multi_contact_video_ledger.json")
    ceiling = next(c for c in single["cases"] if c["id"] == "ceiling_hook")
    source = ROOT / "out/demos" / ceiling["artifactFolder"] / "forces.mp4"
    original = ROOT / "out/demos" / ceiling["sourceArtifactFolder"] / "forces.mp4"
    if sha(source) != sha(original) or ceiling["videoSourceStateCount"] != 12:
        raise ValueError("Canonical ceiling video differs from its original solved run.")
    copy(source, "ceiling_hook.mp4", "Historical single-contact 3-D estimator; 12 independently solved states, 60 playback frames.")
    for name in ("s_channel_two_contact", "tapered_channel_two_contact", "serpentine_three_contact"):
        case = next(c for c in multiple["cases"] if c["id"] == name)
        source = ROOT / "out/demos" / case["artifactFolder"] / case["video"]["videoPath"]
        if sha(source) != case["video"]["sha256"] or case["stateCount"] != 12:
            raise ValueError(f"Multicontact video does not match its ledger: {name}")
        copy(source, name + ".mp4", case["scope"])
    additional = None
    extra_folder = ROOT / "out/benchmarks/generalization/v1"
    if (extra_folder / "summary.json").is_file():
        from render_generalization_comparison import verified_summary
        additional = verified_summary(extra_folder)
        for name in ("generalization_accuracy", "generalization_geometry", "generalization_magnitudes", "generalization_load_errors"):
            for extension in ("svg", "png", "pdf"):
                copy(extra_folder / "figures" / f"{name}.{extension}")
        for name in ("source_data.csv", "force_data.csv", "summary.json", "plot_provenance.json", "fixture_diagnostics.json"):
            copy(extra_folder / name, "generalization_" + name)
        for record in additional["sourceLedgers"]:
            path = ROOT / record["path"]
            copy(path, "generalization_" + path.parent.name + "_comparison.json")
    nominal = [v for v in summary["groups"] if abs(v["noiseStdPerMm"] - 2.5e-5) < 1e-12]
    wins, comparisons = 0, 0
    for scene in {v["scene"] for v in nominal}:
        ours = next(v for v in nominal if v["scene"] == scene and v["method"] == "full")
        for other in [v for v in nominal if v["scene"] == scene and v["method"] != "full"]:
            comparisons += 1
            wins += ours["contactRmseN"]["mean"] < other["contactRmseN"]["mean"]
    conclusion = (f"At nominal noise, EnFiRCE has lower seed-mean contact-vector error in {wins} of {comparisons} scene/baseline comparisons. "
                  "Tip and total-force errors are reported separately; this controlled result is not a universal performance ranking.")
    factor_cases = reports["factors"]["cases"]
    web_summary = {"state": "complete", "releaseDate": datetime.now().strftime("%Y-%m-%d"),
                   "factorRuns": len(factor_cases), "baselineRuns": len(reports["literature"]["cases"]),
                   "pairedPackets": summary["pairedPackets"], "engineeringChecks": len(checks["checks"]),
                   "factorExceptions": reports["factors"]["failureCount"],
                   "factorNonpositiveExits": sum(nonpositive_exit(c["solver"].get("exitflag")) for c in factor_cases),
                   "baselineExceptions": reports["literature"]["failureCount"],
                   "baselineNonpositiveExits": sum(nonpositive_exit(c["solver"].get("exitflag")) for c in reports["literature"]["cases"]),
                   "nominalGroups": nominal, "comparisonConclusion": conclusion,
                   "publicationRunId": completion["runRecord"]["runId"], "sotaEstablished": False}
    if additional:
        groups = {(r["scene"], r["method"]): r for r in additional["summary"]}
        wins = sum(groups[(scene, "full")]["contactRmseN"] < groups[(scene, method)]["contactRmseN"]
                   for scene in additional["sceneDefinitions"] for method in ("point", "gaussian"))
        web_summary["additionalComparison"] = {
            "caseCount": additional["caseCount"], "failureCount": additional["failureCount"],
            "groups": additional["summary"], "sotaEstablished": False,
            "conclusion": f"The additional {additional['caseCount']}-call matrix has lower EnFiRCE seed-mean contact-vector error in {wins}/12 configuration/adaptation comparisons. "
                          "Shape and plane observations are sampled. These are fixed configurations and disclosed adaptations, not an official-method ranking."}
    summary_path = destination / "website_summary.json"
    summary_path.write_text(json.dumps(web_summary, indent=2) + "\n", encoding="utf-8")
    manifest = {"schemaVersion": 1, "releaseDate": web_summary["releaseDate"],
                "publicationRunId": completion["runRecord"]["runId"], "artifacts": copied,
                "summarySha256": sha(summary_path), "manuscriptSourceSha256": sha(ROOT / "EnFiRCE_overleaf/main.tex"),
                "bibliographySha256": sha(ROOT / "EnFiRCE_overleaf/refs.bib"),
                "releaseScriptSha256": sha(Path(__file__)),
                "scope": "Byte copies of completed scientific results and archived continuous simulation videos; no inferred/interpolated quantitative results."}
    (destination / "release_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Synchronized {len(copied)} verified files; {summary['pairedPackets']} paired packets.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", type=Path, default=ROOT / "EnFiRCE_website")
    sync(parser.parse_args().site)
