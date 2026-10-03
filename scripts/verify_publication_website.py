"""Verify the separate website's copied evidence, optionally in its Git index."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from render_formulation_factors import ROOT, sha
from verify_publication_release import verify
from compile_publication_manuscript import verify_build


def verify_website(site: Path, staged: bool = False) -> None:
    verify()
    verify_build()
    site = site.resolve()
    if not site.is_relative_to(ROOT) or not (site / ".git").is_dir():
        raise ValueError("Use the separate website checkout inside this workspace.")
    result = site / "static/results"
    manifest = json.loads((result / "release_manifest.json").read_text(encoding="utf-8"))
    completion = json.loads((ROOT / "out/benchmarks/publication/completion.json").read_text(encoding="utf-8"))
    if manifest["publicationRunId"] != completion["runRecord"]["runId"]:
        raise ValueError("Website and completed publication protocol differ.")
    for file, key in (("main.tex", "manuscriptSourceSha256"), ("refs.bib", "bibliographySha256")):
        if sha(ROOT / "EnFiRCE_overleaf" / file) != manifest[key]:
            raise ValueError("Manuscript changed after website synchronization.")
    if sha(ROOT / "scripts/sync_publication_website.py") != manifest["releaseScriptSha256"]:
        raise ValueError("Website synchronization script changed.")
    files = {"static/results/website_summary.json": manifest["summarySha256"],
             "static/results/release_manifest.json": sha(result / "release_manifest.json")}
    for record in manifest["artifacts"]:
        original = (ROOT / record["source"]).resolve()
        copied = (site / record["websitePath"]).resolve()
        if not original.is_relative_to(ROOT) or not copied.is_relative_to(site):
            raise ValueError("An evidence path leaves its checkout.")
        if sha(original) != record["sha256"] or sha(copied) != record["sha256"]:
            raise ValueError(f"Website copy differs from original: {record['websitePath']}")
        if copied.stat().st_size != record["bytes"]:
            raise ValueError(f"Website byte count differs: {record['websitePath']}")
        if record["websitePath"] in files:
            raise ValueError("Duplicate website evidence path.")
        files[record["websitePath"]] = record["sha256"]
    for path, digest in files.items():
        if sha(site / path) != digest:
            raise ValueError(f"Website checksum mismatch: {path}")
        if staged:
            data = subprocess.run(["git", "show", ":" + path], cwd=site, capture_output=True, check=True).stdout
            if hashlib.sha256(data).hexdigest() != digest:
                raise ValueError(f"Staged website bytes differ: {path}")
    print(f"Verified {len(files)} website artifacts against the original completed evidence" +
          (" and website Git index." if staged else "."))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", type=Path, default=ROOT / "EnFiRCE_website")
    parser.add_argument("--staged", action="store_true")
    args = parser.parse_args()
    verify_website(args.site, args.staged)
