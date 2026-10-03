"""Compile the supplied TeX project and bind its PDF to actual source bytes.

Uses an existing latexmk installation; never installs a TeX distribution.
The complete experiment/figure release must already be verified.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import time

from pypdf import PdfReader

from render_formulation_factors import ROOT, sha

PUBLICATION = ROOT / "out/benchmarks/publication"
PAPER = ROOT / "EnFiRCE_overleaf"


def source_snapshot() -> list[dict]:
    files = [p for p in PAPER.rglob("*") if p.is_file() and ".git" not in p.parts and
             (p.suffix in {".tex", ".bib", ".cls", ".bst"} or
              (p.suffix == ".pdf" and "figs" in p.parts))]
    return [{"path": p.relative_to(ROOT).as_posix(), "sha256": sha(p)} for p in sorted(files)]


def verify_build() -> dict[str, str]:
    path = PUBLICATION / "manuscript_build.json"
    record = json.loads(path.read_text(encoding="utf-8"))
    if (record["state"] != "complete" or record["source"] != source_snapshot() or
            record["compilerScriptSha256"] != sha(Path(__file__))):
        raise ValueError("The PDF build is incomplete or manuscript inputs changed; recompile.")
    pdf = ROOT / record["pdfPath"]
    if (pdf.resolve() != (PUBLICATION / "EnFiRCE_draft.pdf").resolve() or
            sha(pdf) != record["pdfSha256"] or sha(PAPER / "main.pdf") != record["pdfSha256"]):
        raise ValueError("The public PDF differs from its completed manuscript build.")
    pages = len(PdfReader(pdf).pages)
    if pages != record["pageCount"] or not 1 <= pages <= 8:
        raise ValueError("Unexpected draft page count.")
    return {Path(__file__).relative_to(ROOT).as_posix(): sha(Path(__file__)),
            path.relative_to(ROOT).as_posix(): sha(path),
            pdf.relative_to(ROOT).as_posix(): record["pdfSha256"]}


def compile_project() -> None:
    from verify_publication_release import verify
    verify()
    executable = shutil.which("latexmk")
    if executable is None:
        detected = Path("D:/texlive/2024/bin/windows/latexmk.exe")
        if detected.is_file():
            executable = str(detected)
    if executable is None:
        raise RuntimeError("An existing latexmk installation is required for this multi-file template.")
    snapshot = source_snapshot()
    record = {"state": "running", "startedAtUtc": datetime.now(timezone.utc).isoformat(),
              "source": snapshot, "compilerScriptSha256": sha(Path(__file__)),
              "command": ["latexmk", "-pdf", "-interaction=nonstopmode",
                                             "-halt-on-error", "main.tex"]}
    path = PUBLICATION / "manuscript_build.json"
    path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    start = time.perf_counter()
    with (ROOT / "out/latex_build.log").open("wb") as stream:
        completed = subprocess.run([executable, *record["command"][1:]], cwd=PAPER,
                                   stdout=stream, stderr=subprocess.STDOUT)
    record["seconds"] = time.perf_counter() - start
    record["exitCode"] = completed.returncode
    if completed.returncode or snapshot != source_snapshot():
        record["state"] = "failed"
        path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        raise RuntimeError("LaTeX failed or inputs changed during compilation; inspect out/latex_build.log.")
    pdf = PAPER / "main.pdf"
    pages = len(PdfReader(pdf).pages)
    if not 1 <= pages <= 8:
        record["state"] = "failed"
        record["pageCount"] = pages
        path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        raise ValueError(f"Draft has {pages} pages; revise its layout before publication.")
    destination = PUBLICATION / "EnFiRCE_draft.pdf"
    shutil.copyfile(pdf, destination)
    record.update(state="complete", pageCount=pages, pdfPath=destination.relative_to(ROOT).as_posix(),
                  pdfSha256=sha(pdf), completedAtUtc=datetime.now(timezone.utc).isoformat())
    path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    verify_build()
    print(f"Compiled and verified manuscript: {pages} pages, {destination.stat().st_size} bytes.")


if __name__ == "__main__":
    compile_project()
