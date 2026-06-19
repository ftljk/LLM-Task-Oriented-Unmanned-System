#!/usr/bin/env python3
"""Prompt 消融实验：V1/V2/V3 × 4 场景 × 2 次"""

import subprocess as sp, re, time, os, sys

BIN = '/tmp/llm_agent_bin'
DIR = '/home/mofus/rmf_ws/llm_robot_agent'
SCENARIOS = [
    ('一号向前移动1米', 'forward'),
    ('一号去pantry', 'goto_pantry'),
    ('送货到coe', 'deliver_coe'),
    ('一号去pantry，同时送货到coe', 'dual'),
]
PROMPT_VERSIONS = ['v1', 'v2', 'v3']
LABELS = {'v1': 'V1 Minimal', 'v2': 'V2 Structured', 'v3': 'V3 Full'}
ENV = os.environ.copy()
ENV.update(dict(
    GOTOOLCHAIN='local', GOPROXY='https://goproxy.cn,direct',
    GOPATH='/home/mofus/go', GOMODCACHE='/home/mofus/go/pkg/mod',
    ARK_API_KEY='03978d91-d03d-463a-bf1f-488c6307727d',
    ARK_MODEL_ID='deepseek-v3-2-251201',
))

def build():
    print('Building binary...', flush=True)
    r = sp.run(['/home/mofus/go1.23/bin/go', 'build', '-o', BIN, '.'],
               cwd=DIR, env=ENV, capture_output=True, text=True, timeout=60)
    if r.returncode:
        print('Build failed:', r.stderr)
        sys.exit(1)
    print('OK', flush=True)

def run_agent(cmd, version, timeout=600):
    args = [BIN, '--simulator=go', f'--prompt-version={version}']
    input_text = f'{cmd}\n1\n/status\n/quit\n'
    start = time.time()
    r = sp.run(args, cwd=DIR, input=input_text, capture_output=True,
               text=True, timeout=timeout, env=ENV)
    dur = time.time() - start
    return r.stdout, dur

def parse_metrics(text):
    summary = re.search(r'Total:\s*(\d+)\s*\|\s*Success:\s*(\d+)\s*\|\s*Failed:\s*(\d+)\s*\|\s*Skipped:\s*(\d+)', text)
    has_planroute = 'PlanRoute' in text
    has_submit = 'Select option' in text or 'SubmitPlanOptions' in text

    errors = []
    if 'wrong format' in text.lower():
        errors.append('format_error')
    if 'error calling tool' in text.lower() or 'tool call error' in text.lower():
        errors.append('tool_error')
    if 'deadlock' in text.lower():
        errors.append('deadlock')
    if summary is None:
        errors.append('no_execution')

    return {
        'summary': summary,
        'planroute_called': has_planroute,
        'has_submit': has_submit,
        'errors': '; '.join(errors) if errors else '',
    }

def main():
    build()
    results = []

    for scenario_cmd, scenario_label in SCENARIOS:
        for pv in PROMPT_VERSIONS:
            for run_num in range(2):
                label = LABELS[pv]
                print(f'  [{scenario_label}] {label} — Run {run_num+1}/2', flush=True)
                print(f'    Cmd: {scenario_cmd}', flush=True)
                try:
                    output, dur = run_agent(scenario_cmd, pv)
                except sp.TimeoutExpired:
                    print(f'    TIMEOUT', flush=True)
                    results.append(dict(cmd=scenario_cmd, label=scenario_label,
                        version=pv, run=run_num+1, total=0, success=0,
                        success_rate=0, planroute_called=False, has_submit=False,
                        errors='timeout', duration=600))
                    continue

                m = parse_metrics(output)

                if m['summary']:
                    s = m['summary']
                    rate = int(s.group(2))/int(s.group(1))*100 if int(s.group(1))>0 else 0
                    print(f'    ✅ Total={s.group(1)} S={s.group(2)} F={s.group(3)} Sk={s.group(4)} '
                          f'({rate:.0f}%) PR={m["planroute_called"]}', flush=True)
                else:
                    rate = 0
                    print(f'    ⚠️  No execution summary', flush=True)
                    if 'error' in output.lower():
                        for line in output.split('\n'):
                            if 'error' in line.lower() or 'Error' in line:
                                print(f'    → {line[:120]}', flush=True)
                                break

                if m['errors']:
                    print(f'    ❌ {m["errors"]}', flush=True)

                results.append(dict(cmd=scenario_cmd, label=scenario_label,
                    version=pv, run=run_num+1,
                    total=int(m['summary'].group(1)) if m['summary'] else 0,
                    success=int(m['summary'].group(2)) if m['summary'] else 0,
                    success_rate=rate,
                    planroute_called=m['planroute_called'],
                    has_submit=m['has_submit'],
                    errors=m['errors'],
                    duration=round(dur, 1)))
                time.sleep(2)

    # ── Report ──
    print('\n\n' + '='*65)
    print('  Prompt 消融实验结果')
    print('='*65)
    print(f'  {"Version":<18} {"Scenario":<15} {"N":<3} {"Succ%":<8} {"PR":<4} {"AvgDur":<8}')
    print('  ' + '-'*68)

    agg = {}
    for r in results:
        k = (r['version'], r['label'])
        agg.setdefault(k, {'n':0, 'succ':0, 'dur':0, 'pr':0})
        agg[k]['n'] += 1
        agg[k]['succ'] += r['success_rate']
        agg[k]['dur'] += r['duration']
        if r['planroute_called']:
            agg[k]['pr'] += 1

    for (v, lab), a in sorted(agg.items()):
        pr_pct = a['pr']/a['n']*100
        print(f'  {LABELS[v]:<18} {lab:<15} {a["n"]:<3} {a["succ"]/a["n"]:<8.1f} {pr_pct:<4.0f}% {a["dur"]/a["n"]:<8.1f}s')

    # Individual detail
    print(f'\n{"─"*70}')
    print(f'  {"Cmd":<40} {"V":<3} {"Run":<4} {"OK":<4} {"Total":<6} {"Err":<15} {"Dur":<6}')
    print(f'{"─"*70}')
    for r in results:
        ok = f'{r["success"]}/{r["total"]}' if r['total']>0 else '?/?'
        print(f'  {r["cmd"]:<40} {r["version"]:<3} {r["run"]:<4} {ok:<4} {r["total"]:<6} {r["errors"]:<15} {r["duration"]:<6}')

    # ── Save CSV ──
    exp_dir = os.path.join(DIR, 'experiments')
    os.makedirs(exp_dir, exist_ok=True)
    csv_path = os.path.join(exp_dir, 'ablation_prompt.csv')
    with open(csv_path, 'w') as f:
        f.write('scenario,version,run,total,success,success_rate_pct,planroute_called,has_submit,errors,duration_s\n')
        for r in results:
            f.write(f'{r["cmd"]},{r["version"]},{r["run"]},{r["total"]},{r["success"]},'
                    f'{r["success_rate"]:.1f},{r["planroute_called"]},{r["has_submit"]},'
                    f'"{r["errors"]}",{r["duration"]}\n')
    print(f'\n  CSV: {csv_path}')
    print('=== DONE ===')

if __name__ == '__main__':
    main()
