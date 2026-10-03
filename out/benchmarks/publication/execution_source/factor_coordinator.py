"""Compute independent scene/seed shards without changing MATLAB inference.

Each process owns a result directory. The coordinator only merges complete,
checksum-verified cases with the same source/dependency/configuration identity.
The two derivative timing runs execute sequentially after all shards exit.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from render_formulation_factors import ROOT, sha

FACTORS = ROOT / "out/benchmarks/formulation_factors"
PUBLICATION = ROOT / "out/benchmarks/publication"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".pending")
    temporary.write_text(json.dumps(value, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    os.replace(temporary, path)


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def safe(path: Path) -> Path:
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f"Artifact leaves the workspace: {path}")
    return path


def copy_checked(source: Path, destination: Path, digest: str) -> None:
    safe(source)
    safe(destination)
    if sha(source) != digest:
        raise ValueError(f"Source artifact changed: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha(destination) != digest:
            raise ValueError(f"Conflicting destination artifact: {destination}")
    else:
        shutil.copyfile(source, destination)


def copy_case(case: dict, source_root: Path, destination_root: Path) -> None:
    source = safe(source_root / case["artifactFolder"])
    destination = safe(destination_root / case["artifactFolder"])
    for source_file, dest_file, key in (
        (source.parent / "input.mat", destination.parent / "input.mat", "inputSha256"),
        (source.parent / "truth.mat", destination.parent / "truth.mat", "truthSha256"),
        (source / "estimate.mat", destination / "estimate.mat", "estimateSha256"),
        (source / "forces.csv", destination / "forces.csv", "csvSha256"),
    ):
        copy_checked(source_file, dest_file, case[key])


def validate_identity(report: dict, reference: dict) -> None:
    for field in ("source", "dependency"):
        if report["runRecord"][field] != reference["runRecord"][field]:
            raise ValueError(f"Worker {field} differs from the frozen experiment.")
    if report["referenceComparisonSha256"] != reference["referenceComparisonSha256"]:
        raise ValueError("Worker reference fixtures changed.")
    for item in report["runRecord"]["source"] + report["runRecord"]["dependency"]["files"]:
        if sha(safe(ROOT / item["path"])) != item["sha256"]:
            raise ValueError(f"Executed source has changed: {item['path']}")


def run(matlab: Path, jobs: int) -> None:
    if jobs < 1 or jobs > 6:
        raise ValueError("Use between one and six independent MATLAB processes.")
    if not (FACTORS / "comparison.json").exists():
        FACTORS.mkdir(parents=True, exist_ok=True)
        # Initialize metadata only; no fabricated inference cases or scores.
        expression = ("addpath('rod');addpath(genpath('LCP-Continuum'));"
                      "o=struct('quickMode',false,'sceneIds',{{'three_contact','spatial_sliding'}},"
                      "'seeds',[11 23 37],'noiseLevelsPerMm',[0 2.5e-5 5e-5],"
                      "'sampleEnvironment',false,'methods',{{'full','no_temporal','cone_only',"
                      "'no_geometry','no_camera','legacy_partitions'}},'resume',true,"
                      "'solverOptions',struct,'outputName','formulation_factors');"
                      "r=struct('state','running','runRecord',new_run_record(),'options',o,"
                      "'cases',{{}},'referenceComparisonSha256',file_sha256('out/formulation_window/comparison.json'),"
                      "'failureCount',0,'sotaEstablished',false,'scope','Fixed two-state equilibria; independent measured curvature noise.');"
                      "atomic_write_artifact('out/benchmarks/formulation_factors/comparison.json','json',r);")
        subprocess.run([str(matlab), "-singleCompThread", "-batch", expression], cwd=ROOT, check=True,
                       creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
    original = read(FACTORS / "comparison.json")
    validate_identity(original, original)
    original_cases = original["cases"] or []
    if isinstance(original_cases, dict):
        original_cases = [original_cases]
    if any(not c["completed"] for c in original_cases):
        raise ValueError("Resolve recorded inference exceptions before parallel continuation.")
    plan_path = PUBLICATION / "parallel_plan.json"
    if plan_path.exists():
        plan = read(plan_path)
        if (plan["source"] != original["runRecord"]["source"] or plan["options"] != original["options"] or
                plan["coordinatorSha256"] != sha(Path(__file__))):
            raise ValueError("Existing parallel plan belongs to another code/configuration.")
    else:
        plan = {"state": "running", "startedAtUtc": stamp(), "jobs": jobs,
                "coordinatorSha256": sha(Path(__file__)),
                "source": original["runRecord"]["source"], "options": original["options"],
                "preexistingCaseIds": [c["id"] for c in original_cases], "shards": []}
        write(PUBLICATION / "pre_parallel_comparison.json", original)
        for scene, seed in itertools.product(original["options"]["sceneIds"], original["options"]["seeds"]):
            output = f"publication_worker_{scene}_seed_{seed}"
            options = copy.deepcopy(original["options"])
            options.update(sceneIds=[scene], seeds=seed, outputName=output)
            folder = ROOT / "out/benchmarks" / output
            ledger_path = folder / "comparison.json"
            if ledger_path.exists():
                raise ValueError(f"Unowned worker directory already exists: {folder}")
            ledger = copy.deepcopy(original)
            ledger.update(state="running", options=options, failureCount=0,
                          cases=[c for c in original_cases if c["sceneId"] == scene and c["seed"] == seed])
            ledger["runRecord"].update(runId=str(uuid.uuid4()), startedAtUtc=stamp(), state="running")
            for case in ledger["cases"]:
                copy_case(case, FACTORS, folder)
            write(ledger_path, ledger)
            plan["shards"].append({"scene": scene, "seed": seed, "outputName": output,
                                   "ledger": ledger_path.relative_to(ROOT).as_posix()})
        write(plan_path, plan)
    pending = list(plan["shards"])
    active: list[tuple[dict, subprocess.Popen, object]] = []
    last_progress = 0.0
    failure = None
    while pending or active:
        while pending and len(active) < jobs and failure is None:
            shard = pending.pop(0)
            ledger = read(ROOT / shard["ledger"])
            if ledger["state"] == "complete" and not ledger["failureCount"]:
                validate_identity(ledger, original)
                continue
            expression = ("addpath('rod');addpath(genpath('LCP-Continuum'));"
                          f"saved=jsondecode(fileread('{shard['ledger']}'));"
                          "r=run_formulation_factor_protocol(saved.options);"
                          "assert(strcmp(r.state,'complete')&&r.failureCount==0);")
            log_path = ROOT / "out/benchmarks" / shard["outputName"] / "worker.log"
            stream = log_path.open("wb")
            process = subprocess.Popen([str(matlab), "-singleCompThread", "-batch", expression],
                                       cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                       creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            active.append((shard, process, stream))
        for item in active[:]:
            shard, process, stream = item
            status = process.poll()
            if status is None:
                continue
            stream.close()
            active.remove(item)
            ledger = read(ROOT / shard["ledger"])
            if status or ledger["state"] != "complete" or ledger["failureCount"]:
                failure = f"Shard failed: {shard['outputName']}, process exit={status}"
            else:
                validate_identity(ledger, original)
        if time.monotonic() - last_progress >= 30:
            progress = []
            for shard in plan["shards"]:
                ledger = read(ROOT / shard["ledger"])
                cases = ledger["cases"] or []
                if isinstance(cases, dict):
                    cases = [cases]
                progress.append({"scene": shard["scene"], "seed": shard["seed"],
                                 "completed": sum(c["completed"] for c in cases), "state": ledger["state"]})
            print(json.dumps({"atUtc": stamp(), "progress": progress}), flush=True)
            last_progress = time.monotonic()
        if failure and not active:
            plan.update(state="failed", failure=failure)
            write(plan_path, plan)
            raise RuntimeError(failure)
        time.sleep(2)
    merged, shard_records = [], []
    shard_archive = PUBLICATION / "shards"
    for shard in plan["shards"]:
        path = ROOT / shard["ledger"]
        ledger = read(path)
        validate_identity(ledger, original)
        if ledger["state"] != "complete" or ledger["failureCount"]:
            raise ValueError(f"Cannot merge incomplete shard: {path}")
        expected_options = copy.deepcopy(original["options"])
        expected_options.update(sceneIds=[shard["scene"]], seeds=shard["seed"], outputName=shard["outputName"])
        if ledger["options"] != expected_options:
            raise ValueError("Worker configuration changed.")
        for case in ledger["cases"]:
            if not case["completed"]:
                raise ValueError(f"Incomplete case: {case['id']}")
            copy_case(case, path.parent, FACTORS)
            merged.append(case)
        archive = shard_archive / (shard["outputName"] + ".json")
        copy_checked(path, archive, sha(path))
        shard_records.append({"path": archive.relative_to(ROOT).as_posix(), "sha256": sha(archive),
                              "artifactRoot": FACTORS.relative_to(ROOT).as_posix(), "caseCount": len(ledger["cases"])})
    recorded = {(c["sceneId"], c["method"], c["seed"], c["noiseStdPerMm"]) for c in merged}
    expected = set(itertools.product(original["options"]["sceneIds"], original["options"]["methods"],
                                    original["options"]["seeds"], original["options"]["noiseLevelsPerMm"]))
    if recorded != expected or len(merged) != len(expected):
        raise ValueError("Merged cases do not equal the configured Cartesian product.")
    original.update(state="complete", cases=merged, failureCount=0)
    original["runRecord"]["state"] = "complete"
    original["execution"] = {"mode": "independent_scene_seed_processes", "maximumConcurrentMatlab": jobs,
                             "singleComputationalThreadPerWorker": True,
                             "reusedSequentialCaseIds": plan["preexistingCaseIds"],
                             "timingScope": "Case calls include covariance. Worker timing shares host resources; not isolated latency.",
                             "shards": shard_records}
    write(FACTORS / "comparison.json", original)
    plan.update(state="complete", completedAtUtc=stamp(), mergedCaseCount=len(merged), shardsEvidence=shard_records)
    write(plan_path, plan)
    print("All 108 factor cases merged; starting sequential matched baselines, derivative timing and checks.", flush=True)
    expression = "addpath('rod');addpath(genpath('LCP-Continuum'));r=force('publication');assert(strcmp(r.state,'complete'));disp('PUBLICATION_WORKFLOW_COMPLETE');"
    with (ROOT / "out/publication_workflow.log").open("wb") as stream:
        status = subprocess.run([str(matlab), "-singleCompThread", "-batch", expression], cwd=ROOT,
                                stdout=stream, stderr=subprocess.STDOUT,
                                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0).returncode
    if status:
        raise RuntimeError(f"Sequential publication workflow failed, exit={status}; see its completion ledger.")
    print("PUBLICATION_WORKFLOW_COMPLETE", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--matlab", type=Path, default=Path("D:/bin/matlab.exe"))
    parser.add_argument("--jobs", type=int, default=6)
    args = parser.parse_args()
    PUBLICATION.mkdir(parents=True, exist_ok=True)
    lock = PUBLICATION / "coordinator.lock"
    # Refuse overlapping coordinators rather than racing on the final ledger.
    # A forcefully killed run retains this local PID record for inspection.
    with lock.open("x", encoding="utf-8") as stream:
        json.dump({"pid": os.getpid(), "startedAtUtc": stamp()}, stream)
    try:
        run(args.matlab, args.jobs)
    finally:
        lock.unlink()
