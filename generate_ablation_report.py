#!/usr/bin/env python3
"""Generate ablation experiment charts and summary report."""

import csv, os, sys
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

EXPERIMENTS_DIR = '/home/mofus/rmf_ws/llm_robot_agent/experiments'
OUTPUT_DIR = EXPERIMENTS_DIR

def load_csv(name):
    path = os.path.join(EXPERIMENTS_DIR, name)
    if not os.path.exists(path): return None
    with open(path) as f: return list(csv.DictReader(f))

# ── ① DAG Strategy ──

def chart_dag_strategy(data):
    if not data: return
    fig, ax = plt.subplots(figsize=(8, 5))
    strategies = sorted(set(r['strategy'] for r in data))
    x = np.arange(len(strategies))
    w = 0.25
    for i, status in enumerate(['status_a', 'status_b', 'status_c']):
        vals = [next(r[status] for r in data if r['strategy'] == s) for s in strategies]
        ax.bar(x + i * w, [{'Completed':3,'Skipped':2,'Failed':1}.get(v,0) for v in vals], w, label=f'Task {status[-1].upper()}')
    ax.set_xticks(x + w)
    ax.set_xticklabels(strategies)
    ax.set_ylabel('Status (1=Failed 2=Skipped 3=Completed)')
    ax.set_title('Ablation ①: DAG Strategy — Task Status')
    ax.legend()
    # add text labels
    for i, s in enumerate(strategies):
        for j, status in enumerate(['status_a', 'status_b', 'status_c']):
            v = next(r[status] for r in data if r['strategy'] == s)
            ax.text(i + j*w, {'Completed':3,'Skipped':2,'Failed':1}.get(v,0)+0.05, v,
                    ha='center', va='bottom', fontsize=8, rotation=90)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'chart_dag_strategy.png'), dpi=150)
    plt.close(fig)
    print('  => chart_dag_strategy.png')

# ── ①b DAG Deadlock ──

def chart_dag_deadlock(data):
    if not data: return
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = [r['cycle_description'][:30] for r in data]
    vals = [1 if r['deadlock_detected'] == 'true' else 0 for r in data]
    colors = ['#d32f2f' if v else '#388e3c' for v in vals]
    ax.barh(labels, vals, color=colors, height=0.5)
    ax.set_xlim(0, 1.5)
    ax.set_xlabel('Deadlock Detected')
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['No', 'Yes'])
    ax.set_title('Ablation ①b: DAG Deadlock Detection')
    for i, (l, v) in enumerate(zip(labels, vals)):
        ax.text(v + 0.05, i, 'Detected' if v else 'Not detected',
                va='center', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'chart_dag_deadlock.png'), dpi=150)
    plt.close(fig)
    print('  => chart_dag_deadlock.png')

# ── ② Path Conflict ──

def chart_path_conflict(data):
    if not data: return
    rows = [r for r in data if r['errors'] == '']
    if not rows: return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    labels = [r['strategy'] for r in rows]
    total_len = [float(r['total_distance_m']) for r in rows]
    shared = [int(r['shared_vertices']) for r in rows]
    name_map = {'ShortestPath':'ShortestPath\n(baseline)',
                'ShortestPathAvoiding':'Avoiding',
                'ShortestPathMinimizeOverlap':'Minimize\nOverlap'}
    short_labels = [name_map.get(l, l) for l in labels]

    colors = ['#2196f3', '#ff9800', '#4caf50']
    ax1.bar(short_labels, total_len, color=colors[:len(labels)])
    ax1.set_ylabel('Total Path Length (m)')
    ax1.set_title('Path Length')
    for i, v in enumerate(total_len):
        ax1.text(i, v + 5, f'{v:.1f}m', ha='center', fontsize=9)

    ax2.bar(short_labels, shared, color=colors[:len(labels)])
    ax2.set_ylabel('Shared Vertices')
    ax2.set_title('Conflict (Shared Vertices)')
    for i, v in enumerate(shared):
        ax2.text(i, v + 0.2, str(v), ha='center', fontsize=9)

    fig.suptitle('Ablation ②: Path Conflict Strategy', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'chart_path_conflict.png'), dpi=150)
    plt.close(fig)
    print('  => chart_path_conflict.png')

# ── ④ Prompt Ablation ──

def chart_prompt_ablation(data):
    if not data: return
    agg = defaultdict(lambda: {'n':0, 'succ':0, 'dur':0, 'tasks':0})
    for r in data:
        k = (r['version'], r['scenario'])
        agg[k]['n'] += 1
        agg[k]['succ'] += float(r['success_rate_pct'])
        agg[k]['dur'] += float(r['duration_s'])
        agg[k]['tasks'] += int(r['total'])

    versions = sorted(set(k[0] for k in agg))
    scenarios = ['forward', 'goto_pantry', 'deliver_coe', 'dual']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    x = np.arange(len(scenarios))
    w = 0.25
    for i, v in enumerate(versions):
        vals = []
        for s in scenarios:
            a = agg.get((v, s))
            vals.append(a['succ']/a['n'] if a else 0)
        ax1.bar(x + i*w, vals, w, label=f'V{v}')
    ax1.set_xticks(x + w)
    ax1.set_xticklabels(scenarios, rotation=15)
    ax1.set_ylabel('Avg Success Rate (%)')
    ax1.set_ylim(0, 110)
    ax1.set_title('Success Rate')
    ax1.axhline(100, color='gray', ls='--', lw=0.5)
    ax1.legend()

    for i, v in enumerate(versions):
        vals = []
        for s in scenarios:
            a = agg.get((v, s))
            vals.append(a['dur']/a['n'] if a else 0)
        ax2.bar(x + i*w, vals, w, label=f'V{v}')
    ax2.set_xticks(x + w)
    ax2.set_xticklabels(scenarios, rotation=15)
    ax2.set_ylabel('Avg Duration (s)')
    ax2.set_title('Duration')
    ax2.legend()

    fig.suptitle('Ablation ④: Prompt Engineering Impact', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'chart_prompt_ablation.png'), dpi=150)
    plt.close(fig)
    print('  => chart_prompt_ablation.png')

    # Task count for goto_pantry (plan quality proxy)
    fig2, ax = plt.subplots(figsize=(7, 4))
    for i, v in enumerate(versions):
        a = agg.get((v, 'goto_pantry'))
        val = a['tasks']/a['n'] if a else 0
        ax.bar(v, val, color=['#2196f3','#ff9800','#4caf50'][i], width=0.4)
        ax.text(i, val + 1, f'{val:.0f}', ha='center', fontsize=10)
    ax.set_ylabel('Avg Task Count')
    ax.set_title('Goto Pantry: Task Count (Navigation Quality Proxy)')
    fig2.tight_layout()
    fig2.savefig(os.path.join(OUTPUT_DIR, 'chart_prompt_taskcount.png'), dpi=150)
    plt.close(fig2)
    print('  => chart_prompt_taskcount.png')

# ── Report ──

def generate_report():
    report_path = os.path.join(EXPERIMENTS_DIR, 'ABLATION_REPORT.md')
    lines = [
        '# Ablation Experiments Report',
        '',
        '## ① DAG SkipAndContinue vs Cascade',
        '',
        '**Design**: 3 tasks A→B, A→C. A fails (Wait 1s + 1ms timeout). Compare SkipAndContinue vs Cascade.',
        '',
        '| Strategy | A | B | C |',
        '|----------|---|---|---|',
    ]
    d = load_csv('ablation_dag_strategy.csv')
    if d:
        for s in sorted(set(r['strategy'] for r in d)):
            r = next(row for row in d if row['strategy'] == s)
            lines.append(f'| {s} | {r["status_a"]} | {r["status_b"]} | {r["status_c"]} |')

    lines += [
        '',
        '**Finding**: Downstream behavior (B, C) unchanged. The difference is only the failed task\'s',
        'status label (Skipped vs Failed). SkipAndContinue is more graceful — marks as skipped and continues.',
        '',
        '## ①b DAG Deadlock Detection',
        '',
        '**Design**: Cycle A→B→C→A vs normal X→Y, X→Z.',
        '',
        '| Case | Cycle | Deadlock |',
        '|------|-------|----------|',
    ]
    d2 = load_csv('ablation_dag_deadlock.csv')
    if d2:
        for r in d2:
            lines.append(f'| {r["cycle_description"]} | deadlock | {r["deadlock_detected"]} |')

    lines += [
        '',
        '**Finding**: Cyclic dependency correctly detected. Normal DAG executes normally.',
        '',
        '## ② Path Conflict Resolution',
        '',
        '**Design**: Two robots crossing hallway in opposite directions (patrol_D1 ↔ coe via vertex 2).',
        'Compare 3 ShortestPath strategies.',
        '',
        '| Strategy | Total Len (m) | Shared Vertices | Time (μs) |',
        '|----------|--------------|-----------------|-----------|',
    ]
    d3 = load_csv('ablation_path_conflict.csv')
    if d3:
        for r in d3:
            if r['errors']:
                lines.append(f'| {r["strategy"]} | — | — | — | *(no path: {r["errors"][:50]})* |')
            else:
                lines.append(f'| {r["strategy"]} | {float(r["total_distance_m"]):.1f} | {r["shared_vertices"]} | {int(r["r1_compute_us"])+int(r["r2_compute_us"])} |')

    lines += [
        '',
        '**Finding**: Avoiding fails due to nav graph topology (no alternative path for this crossing).',
        'MinimizeOverlap reduces shared vertices from 8→0 at 27× path length cost (12→320m).',
        'Dijkstra computation is negligible (~μs).',
        '',
        '## ③ Normalizer ValidateAndFix',
        '',
        '**Design**: Compare task success rate with and without the ValidateAndFix step.',
        '',
        '| Command | WITH Normalizer | WITHOUT Normalizer |',
        '|---------|----------------|-------------------|',
    ]
    d4 = load_csv('ablation_normalizer.csv')
    if d4:
        for cmd in sorted(set(r['command'] for r in d4), key=lambda x: ['一号向前移动1米','一号去pantry，同时送货到coe'].index(x) if x in ['一号向前移动1米','一号去pantry，同时送货到coe'] else 0):
            with_r = [r for r in d4 if r['command'] == cmd and 'WITH' in r['mode']]
            wo_r = [r for r in d4 if r['command'] == cmd and 'WITHOUT' in r['mode']]
            lines.append(f'| {cmd} | {sum(float(r["success_rate_pct"]) for r in with_r)/len(with_r):.0f}% | {sum(float(r["success_rate_pct"]) for r in wo_r)/len(wo_r):.0f}% |')

    lines += [
        '',
        '**Finding**: In this dataset, the LLM-generated plans were already valid.',
        'No corrections triggered by ValidateAndFix in any run. The Normalizer\'s benefit',
        'is more likely to appear with edge cases, malformed JSON, or weaker models.',
        '',
        '## ④ Prompt Engineering',
        '',
        '**Design**: 3 prompt versions × 4 scenarios × 2 runs = 24 LLM calls.',
        '',
        '| Version | Description | Success Rate | Navigation Quality | Avg Duration |',
        '|---------|-------------|:--------:|:----------------:|:----------:|',
    ]
    d5 = load_csv('ablation_prompt.csv')
    if d5:
        agg = defaultdict(lambda: {'n':0, 'succ':0, 'dur':0, 'tasks':0})
        for r in d5:
            agg[r['version']]['n'] += 1
            agg[r['version']]['succ'] += float(r['success_rate_pct'])
            agg[r['version']]['dur'] += float(r['duration_s'])
            agg[r['version']]['tasks'] += int(r['total'])
        labels = {
            'v1': 'Minimal — bare task description only',
            'v2': 'Structured — constraints + task rules',
            'v3': 'Full — step-by-step + examples + warnings'
        }
        for v in ['v1', 'v2', 'v3']:
            a = agg[v]
            lines.append(f'| V{v} | {labels[v]} | {a["succ"]/a["n"]:.0f}% | {a["tasks"]/a["n"]:.0f} tasks avg | {a["dur"]/a["n"]:.0f}s |')

    lines += [
        '',
        '**Detailed by scenario**:',
        '',
        '| Scenario | V1 | V2 | V3 |',
        '|----------|:--:|:--:|:--:|',
    ]
    if d5:
        for s in ['forward', 'goto_pantry', 'deliver_coe', 'dual']:
            vals = []
            for v in ['v1', 'v2', 'v3']:
                runs = [r for r in d5 if r['scenario'] == s and r['version'] == v]
                if runs:
                    rate = sum(float(r['success_rate_pct']) for r in runs) / len(runs)
                    vals.append(f'{rate:.0f}%')
                else:
                    vals.append('—')
            lines.append(f'| {s} | {vals[0]} | {vals[1]} | {vals[2]} |')

    lines += [
        '',
        '**Finding**:',
        '- **V1 Minimal**: No guidance on PlanRoute → LLM uses SetRobotPosition stub. Complex commands: 50% failure.',
        '- **V2 Structured**: Partial guidance → 99% success but inconsistent navigation decomposition.',
        '- **V3 Full**: Step-by-step + examples + warnings → 100% consistent PlanRoute decomposition (42-56 tasks).',
        '',
        '**Key Insight**: Without explicit step-by-step instructions and usage examples, the LLM defaults',
        'to the simplest available tool (SetRobotPosition) instead of the correct navigation pipeline (PlanRoute).',
        '',
        '## Summary',
        '',
        '| # | Ablation | Key Finding |',
        '|---|----------|-------------|',
        '| ① | DAG Strategy | SkipAndContinue marks failed tasks as Skipped; Cascade marks as Failed. Downstream same. |',
        '| ①b | DAG Deadlock | Cyclic dependency correctly detected and rejected. |',
        '| ② | Path Conflict | Avoid fails on small graph; MinimizeOverlap 27× cost for 0 shared vertices. |',
        '| ③ | Normalizer | No corrections triggered (LLM output already valid). |',
        '| ④ | Prompt | Minimal→50% fail; Full→100% reliable. Prompt quality directly impacts tool selection. |',
        '',
        '---',
        f'*Generated on {__import__("datetime").datetime.now().strftime("%Y-%m-%d %H:%M")}*',
    ]

    with open(report_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'  => {os.path.relpath(report_path)}')

def main():
    print('Generating ablation experiment charts and report...\n')
    chart_dag_strategy(load_csv('ablation_dag_strategy.csv'))
    chart_dag_deadlock(load_csv('ablation_dag_deadlock.csv'))
    chart_path_conflict(load_csv('ablation_path_conflict.csv'))
    chart_prompt_ablation(load_csv('ablation_prompt.csv'))
    print()
    generate_report()
    print(f'\nDone. Outputs in {EXPERIMENTS_DIR}/')

if __name__ == '__main__':
    main()
