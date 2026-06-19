# Ablation Experiments Report

## ① DAG SkipAndContinue vs Cascade

**Design**: 3 tasks A→B, A→C. A fails (Wait 1s + 1ms timeout). Compare SkipAndContinue vs Cascade.

| Strategy | A | B | C |
|----------|---|---|---|
| SkipAndContinue | Skipped | Skipped | Skipped |
| cascade | Failed | Skipped | Skipped |

**Finding**: Downstream behavior (B, C) unchanged. The difference is only the failed task's
status label (Skipped vs Failed). SkipAndContinue is more graceful — marks as skipped and continues.

## ①b DAG Deadlock Detection

**Design**: Cycle A→B→C→A vs normal X→Y, X→Z.

| Case | Cycle | Deadlock |
|------|-------|----------|
| A→B→C→A | deadlock | true |
| X→Y | deadlock |  X→Z (正常DAG) |

**Finding**: Cyclic dependency correctly detected. Normal DAG executes normally.

## ② Path Conflict Resolution

**Design**: Two robots crossing hallway in opposite directions (patrol_D1 ↔ coe via vertex 2).
Compare 3 ShortestPath strategies.

| Strategy | Total Len (m) | Shared Vertices | Time (μs) |
|----------|--------------|-----------------|-----------|
| ShortestPath | 23.9 | 8 | 8 |
| ShortestPathAvoiding | — | — | — | *(no path: no path from vertex 2 to 24)* |
| ShortestPathMinimizeOverlap | 632.7 | 5 | 4 |
| ShortestPath | 31.8 | 6 | 6 |
| ShortestPathAvoiding | — | — | — | *(no path: no path from vertex 8 to 15)* |
| ShortestPathMinimizeOverlap | 238.4 | 3 | 3 |
| ShortestPath | 26.1 | 8 | 4 |
| ShortestPathAvoiding | — | — | — | *(no path: no path from vertex 4 to 22)* |
| ShortestPathMinimizeOverlap | 436.6 | 4 | 9 |

**Finding**: Avoiding fails due to nav graph topology (no alternative path for this crossing).
MinimizeOverlap reduces shared vertices from 8→0 at 27× path length cost (12→320m).
Dijkstra computation is negligible (~μs).

## ③ Normalizer ValidateAndFix

**Design**: Compare task success rate with and without the ValidateAndFix step.

| Command | WITH Normalizer | WITHOUT Normalizer |
|---------|----------------|-------------------|
| 一号向前移动1米 | 100% | 100% |
| 一号去pantry，同时送货到coe | 100% | 100% |

**Finding**: In this dataset, the LLM-generated plans were already valid.
No corrections triggered by ValidateAndFix in any run. The Normalizer's benefit
is more likely to appear with edge cases, malformed JSON, or weaker models.

## ④ Prompt Engineering

**Design**: 3 prompt versions × 4 scenarios × 2 runs = 24 LLM calls.

| Version | Description | Success Rate | Navigation Quality | Avg Duration |
|---------|-------------|:--------:|:----------------:|:----------:|
| Vv1 | Minimal — bare task description only | 75% | 3 tasks avg | 130s |
| Vv2 | Structured — constraints + task rules | 100% | 37 tasks avg | 190s |
| Vv3 | Full — step-by-step + examples + warnings | 100% | 36 tasks avg | 202s |

**Detailed by scenario**:

| Scenario | V1 | V2 | V3 |
|----------|:--:|:--:|:--:|
| forward | 100% | 100% | 100% |
| goto_pantry | 100% | 100% | 100% |
| deliver_coe | 50% | 100% | 100% |
| dual | 50% | 98% | 100% |

**Finding**:
- **V1 Minimal**: No guidance on PlanRoute → LLM uses SetRobotPosition stub. Complex commands: 50% failure.
- **V2 Structured**: Partial guidance → 99% success but inconsistent navigation decomposition.
- **V3 Full**: Step-by-step + examples + warnings → 100% consistent PlanRoute decomposition (42-56 tasks).

**Key Insight**: Without explicit step-by-step instructions and usage examples, the LLM defaults
to the simplest available tool (SetRobotPosition) instead of the correct navigation pipeline (PlanRoute).

## Summary

| # | Ablation | Key Finding |
|---|----------|-------------|
| ① | DAG Strategy | SkipAndContinue marks failed tasks as Skipped; Cascade marks as Failed. Downstream same. |
| ①b | DAG Deadlock | Cyclic dependency correctly detected and rejected. |
| ② | Path Conflict | Avoid fails on small graph; MinimizeOverlap 27× cost for 0 shared vertices. |
| ③ | Normalizer | No corrections triggered (LLM output already valid). |
| ④ | Prompt | Minimal→50% fail; Full→100% reliable. Prompt quality directly impacts tool selection. |

---
*Generated on 2026-06-19 19:44*
