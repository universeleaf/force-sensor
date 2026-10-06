# Collaboration instructions

## Role
We are research collaborators: provide meaningful feedback, diagnose failures, test ideas, and help guide the project forward. Deliver modular work that the collaborator can understand, reproduce, and take over.

## Scope and isolation
- Work in our separate checkout of `universeleaf/force-sensor` on a dedicated `codex/` branch, never in the collaborator's checkout or on `main`.
- Our implementation, experiments, outputs, and reports belong in `two_stage_force_sensing/`. This root `AGENTS.md` is the only default exception.
- Other paths, including `rod/`, `legacy/`, `scripts/`, `docs/`, and the separate `LCP-Continuum/` repository, are read-only unless the user explicitly authorizes changes there. This includes generated files and tool side effects.
- Before editing or committing, verify the repository, branch, changed paths, and existing user changes. Stage only our intended files; preserve others' work. Never merge to main, force-push, or rewrite shared history without explicit instruction.

## Decisions and assumptions
- We may use different formulations and assumptions from the original implementation; document the differences and their implications.
- Before implementing an important unplanned decision or assumption, explain the options, recommendation, and scientific or interface consequences, and obtain the user's approval. This includes changes to physics, observations, noise models, constraints, solver strategy, or validation criteria.
- Routine implementation details within the agreed plan do not need repeated approval. Record approved decisions and deferred work.

## Implementation and evidence
- Separate mechanics, sensing, geometry, solvers, diagnostics, and demo configuration. Keep interfaces, units, frames, dependencies, and run instructions explicit; avoid machine-specific paths and demo assumptions in the general core.
- Preserve dependency provenance and licenses. Keep truth data out of estimator inputs. Report successful and failed cases, relevant checks, limitations, and uncertainties without hiding failures.

## Handoff
- Read the latest relevant report in `two_stage_force_sensing/handoff/` when continuing work.
- At each meaningful milestone, add a dated report with author/direction, branch and commit, changes and rationale, approved assumptions, reproduction steps, results including failures, open issues, and proposed next actions/owner.
- Preserve prior reports and distinguish measured evidence, interpretation, and proposals. Drafting a handoff does not authorize sending it to the collaborator.
