"""Verify published evidence bytes, including Git's staged representation.

Use --staged after selecting public artifacts and before committing. Checking
only the working tree would miss Git newline conversions and unstaged results.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess

from render_formulation_factors import ROOT, sha


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def case_identity(case: dict) -> dict:
    """Normalize only MATLAB's documented singleton/empty JSON encodings.

    A subsequent MATLAB resume can collapse a singleton struct array and turn
    a decoded null into []. Numeric values, array order, flags, and hashes must
    still compare exactly to the archived original worker evidence.
    """
    normalized = deepcopy(case)
    solver = normalized.get("solver", {})
    for key in ("stages", "activeBranchPolish"):
        if isinstance(solver.get(key), dict):
            solver[key] = [solver[key]]
    coverage = normalized.get("coverage", {})
    for key in ("meanIntervalWidthN", "empiricalCoverage"):
        if key in coverage and coverage[key] == []:
            coverage[key] = None
    return normalized


def verify(staged: bool = False) -> dict[str, str]:
    publication = ROOT / "out/benchmarks/publication"
    completion = read(publication / "completion.json")
    if completion["state"] != "complete" or completion["quickMode"]:
        raise ValueError("Only a complete, non-smoke run is publishable.")
    expected: dict[str, str] = {}

    def record(path: str | Path, digest: str) -> None:
        absolute = (ROOT / path).resolve()
        if not absolute.is_relative_to(ROOT):
            raise ValueError(f"Path leaves the workspace: {path}")
        name = absolute.relative_to(ROOT).as_posix()
        if name in expected and expected[name] != digest:
            raise ValueError(f"Conflicting identities: {name}")
        if sha(absolute) != digest:
            raise ValueError(f"Working-tree checksum mismatch: {name}")
        expected[name] = digest

    steps = {s["name"]: s for s in completion["completedSteps"]}
    if set(steps) != {"factors", "literature", "derivatives", "engineering"}:
        raise ValueError("Four completed steps are required.")
    for s in completion["runRecord"]["source"]:
        record(s["path"], s["sha256"])
    # External mechanics are pinned separately and are not staged in this repo.
    for s in completion["runRecord"]["dependency"]["files"]:
        path = (ROOT / s["path"]).resolve()
        if not path.is_relative_to(ROOT) or sha(path) != s["sha256"]:
            raise ValueError(f"Mechanics dependency changed: {s['path']}")
    record(publication / "completion.json", sha(publication / "completion.json"))
    for s in steps.values():
        record(s["path"], s["sha256"])
    for name in ("factors", "literature"):
        ledger_path = ROOT / steps[name]["path"]
        ledger = read(ledger_path)
        if ledger["state"] != "complete" or ledger["failureCount"]:
            raise ValueError(f"Incomplete inference ledger: {name}")
        if ledger["runRecord"]["source"] != completion["runRecord"]["source"]:
            raise ValueError(f"Inference ledger used different source: {name}")
        if ledger["runRecord"]["dependency"] != completion["runRecord"]["dependency"]:
            raise ValueError(f"Inference ledger used different mechanics: {name}")
        for c in ledger["cases"]:
            folder = ledger_path.parent / c["artifactFolder"]
            for path, key in ((folder.parent / "input.mat", "inputSha256"),
                              (folder.parent / "truth.mat", "truthSha256"),
                              (folder / "estimate.mat", "estimateSha256"),
                              (folder / "forces.csv", "csvSha256")):
                record(path, c[key])
        if name == "factors" and "execution" in ledger:
            execution = ledger["execution"]
            plan_path = publication / "parallel_plan.json"
            plan = read(plan_path)
            if (plan["state"] != "complete" or plan["mergedCaseCount"] != len(ledger["cases"]) or
                    plan["source"] != ledger["runRecord"]["source"] or
                    plan["options"] != ledger["options"] or plan["shardsEvidence"] != execution["shards"]):
                raise ValueError("Parallel completion plan differs from the merged experiment.")
            record(plan.get("coordinatorSourceArchive", "scripts/run_publication_parallel.py"),
                   plan["coordinatorSha256"])
            sequential = plan.get("sequentialCoordinator")
            if sequential is not None:
                if sequential["state"] != "complete" or sequential["processExitCode"] != 0:
                    raise ValueError("Sequential publication completion did not finish successfully.")
                record(sequential["path"], sequential["sha256"])
                record("scripts/run_publication_parallel.py", sequential["sha256"])
            record(plan_path, sha(plan_path))
            original_cases = {c["id"]: c for c in ledger["cases"]}
            shard_ids = []
            for evidence in execution["shards"]:
                record(evidence["path"], evidence["sha256"])
                shard = read(ROOT / evidence["path"])
                if (shard["state"] != "complete" or shard["failureCount"] or
                        shard["runRecord"]["source"] != ledger["runRecord"]["source"] or
                        shard["runRecord"]["dependency"] != ledger["runRecord"]["dependency"]):
                    raise ValueError("Parallel shard identity or completion differs.")
                if evidence["artifactRoot"] != ledger_path.parent.relative_to(ROOT).as_posix():
                    raise ValueError("Parallel shard maps to a different artifact tree.")
                for case in shard["cases"]:
                    if case_identity(case) != case_identity(original_cases.get(case["id"], {})):
                        raise ValueError(f"Merged case differs from worker evidence: {case['id']}")
                    shard_ids.append(case["id"])
            if len(shard_ids) != len(original_cases) or set(shard_ids) != set(original_cases):
                raise ValueError("Parallel shards do not cover the complete factor matrix exactly once.")
        plot_provenance = ledger_path.parent / "plot_provenance.json"
        plots = read(plot_provenance)
        if plots["sourceComparisonSha256"] != sha(ledger_path):
            raise ValueError("Plots refer to a different inference ledger.")
        record("scripts/render_formulation_factors.py", plots["rendererSha256"])
        record(plot_provenance, sha(plot_provenance))
        for s in plots["outputs"]:
            record(s["path"], s["sha256"])
    figures = read(publication / "figure_provenance.json")
    required_ledgers = {(steps[name]["path"], steps[name]["sha256"]) for name in ("factors", "literature")}
    if {(s["path"], s["sha256"]) for s in figures["sourceLedgers"]} != required_ledgers:
        raise ValueError("Figure ledgers differ from this completed publication protocol.")
    record("scripts/render_formulation_publication.py", figures["rendererSha256"])
    for s in figures["sourceLedgers"] + figures["outputs"]:
        record(s["path"], s["sha256"])
    record(publication / "figure_provenance.json", sha(publication / "figure_provenance.json"))
    derivative_path = ROOT / steps["derivatives"]["path"]
    derivative = read(derivative_path)
    if (derivative["state"] != "complete" or
            derivative["runRecord"]["source"] != completion["runRecord"]["source"] or
            derivative["runRecord"]["dependency"] != completion["runRecord"]["dependency"] or
            [c["id"] for c in derivative["cases"]] != ["separate", "shared"]):
        raise ValueError("Derivative evidence is incomplete or belongs to different code.")
    record(derivative_path.parent / "input.mat", derivative["inputSha256"])
    for c in derivative["cases"]:
        record(derivative_path.parent / (c["id"] + ".mat"), c["estimateSha256"])
    checks = read(ROOT / steps["engineering"]["path"])
    if (checks["runRecord"]["source"] != completion["runRecord"]["source"] or
            checks["runRecord"]["dependency"] != completion["runRecord"]["dependency"]):
        raise ValueError("Engineering checks belong to different code/dependencies.")
    if not checks["allPassed"] or not all(c["passed"] for c in checks["checks"]):
        raise ValueError("Engineering checks failed.")
    if staged:
        from compile_publication_manuscript import verify_build
        for name, digest in verify_build().items():
            record(name, digest)
        for name, digest in expected.items():
            result = subprocess.run(["git", "show", ":" + name], cwd=ROOT, capture_output=True)
            if result.returncode:
                raise ValueError(f"Required artifact is not in the Git index: {name}")
            actual = hashlib.sha256(result.stdout).hexdigest()
            if actual != digest:
                raise ValueError(f"Staged bytes differ from the executed artifact: {name}")
    print(f"Verified {len(expected)} source/data/figure files in the working tree" +
          (" and Git index." if staged else "."))
    return expected


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--staged", action="store_true")
    verify(parser.parse_args().staged)
