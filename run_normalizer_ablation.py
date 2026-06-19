#!/usr/bin/env python3
"""Normalizer 消融实验：ValidateAndFix 有无对比"""

import subprocess as sp, re, time, os

BIN = '/tmp/llm_agent_bin'
DIR = '/home/mofus/rmf_ws/llm_robot_agent'
GO = '/home/mofus/go1.23/bin/go'
EXPERIMENTS = [
    ('一号向前移动1米', 'forward'),
    ('一号去pantry，同时送货到coe', 'patrol'),
]

ENV = os.environ.copy()
ENV.update(dict(
    GOTOOLCHAIN='local', GOPROXY='https://goproxy.cn,direct',
    GOPATH='/home/mofus/go', GOMODCACHE='/home/mofus/go/pkg/mod',
    ARK_API_KEY='03978d91-d03d-463a-bf1f-488c6307727d',
    ARK_MODEL_ID='deepseek-v3-2-251201',
))

def build():
    print('Building binary...', flush=True)
    r = sp.run([GO, 'build', '-o', BIN, '.'], cwd=DIR, env=ENV,
               capture_output=True, text=True, timeout=60)
    if r.returncode:
        print('Build failed:', r.stderr)
        sys.exit(1)
    print('Build OK', flush=True)

def run_agent(command, no_normalizer=False, timeout=600):
    args = [BIN, '--simulator=go']
    if no_normalizer:
        args.append('--no-normalizer')
    input_text = f'{command}\n1\n/status\n/quit\n'
    start = time.time()
    r = sp.run(args, cwd=DIR, input=input_text, capture_output=True,
               text=True, timeout=timeout, env=ENV)
    dur = time.time() - start
    return r.stdout, dur

def parse_summary(text):
    m = re.search(r'Total:\s*(\d+)\s*\|\s*Success:\s*(\d+)\s*\|\s*Failed:\s*(\d+)\s*\|\s*Skipped:\s*(\d+)', text)
    if m:
        return {'total': int(m.group(1)), 'success': int(m.group(2)),
                'failed': int(m.group(3)), 'skipped': int(m.group(4)),
                'success_rate': int(m.group(2))/int(m.group(1))*100 if int(m.group(1))>0 else 0}
    return None

def main():
    build()
    results = []
    print('='*65)
    print('  Normalizer 消融实验')
    print('='*65)

    for cmd, label in EXPERIMENTS:
        for use_norm in [True, False]:
            mode_name = 'WITH Normalizer' if use_norm else 'WITHOUT Normalizer'
            for run_num in range(2):
                print(f'\n  [{label}] {mode_name} — Run {run_num+1}/2')
                print(f'    Cmd: {cmd}', flush=True)
                try:
                    output, dur = run_agent(cmd, no_normalizer=not use_norm)
                except sp.TimeoutExpired:
                    print(f'    TIMEOUT', flush=True)
                    results.append({'cmd':cmd,'mode':mode_name,'run':run_num+1,
                                    'total':0,'success':0,'rate':0,
                                    'corrections':'','errors':'timeout','duration':600})
                    continue

                summary = parse_summary(output)
                corr = re.findall(r'\[Corrections\].*?$|• [^\n]+', output, re.M)
                errs = re.findall(r'(?:[Ee]rror[^:\n]*|deadlock|cancelled|wrong format|unexpected)',
                                  output)

                if summary:
                    print(f'    ✅ Total={summary["total"]} S={summary["success"]} '
                          f'F={summary["failed"]} Sk={summary["skipped"]} '
                          f'({summary["success_rate"]:.0f}%)', flush=True)
                else:
                    print(f'    ⚠️  No summary (output last 400 chars)', flush=True)
                    print(f'    {output[-400:]}', flush=True)
                if corr:
                    for c in corr:
                        print(f'    🔧 {c[:120]}', flush=True)
                if errs:
                    for e in errs[:3]:
                        print(f'    ❌ {e[:100]}', flush=True)

                rec = {'cmd':cmd,'mode':mode_name,'run':run_num+1,
                       'corrections':'; '.join(corr) if corr else '',
                       'errors':'; '.join(errs[:3]) if errs else '',
                       'duration':round(dur,1)}
                if summary:
                    rec.update(summary)
                else:
                    rec.update({'total':0,'success':0,'failed':0,'skipped':0,'success_rate':0.0})
                results.append(rec)
                time.sleep(2)

    # ── Summary table ──
    print('\n\n'+'='*65)
    print('  汇总')
    print('='*65)
    print(f'  {"Cmd":<35} {"Mode":<20} {"N":<3} {"Succ%":<8} {"AvgDur":<8}')
    print('  '+'-'*74)
    agg = {}
    for r in results:
        k = (r['cmd'], r['mode'])
        agg.setdefault(k, {'n':0,'succ':0,'dur':0})
        agg[k]['n'] += 1
        agg[k]['succ'] += r['success_rate']
        agg[k]['dur'] += r['duration']
    for (c,m),v in sorted(agg.items()):
        print(f'  {c:<35} {m:<20} {v["n"]:<3} {v["succ"]/v["n"]:<8.1f} {v["dur"]/v["n"]:<8.1f}s')

    # ── CSV ──
    exp_dir = os.path.join(DIR, 'experiments')
    os.makedirs(exp_dir, exist_ok=True)
    p = os.path.join(exp_dir, 'ablation_normalizer.csv')
    with open(p,'w') as f:
        f.write('cmd,mode,run,total,success,success_rate_pct,corrections,errors,duration_s\n')
        for r in results:
            f.write(f'{r["cmd"]},{r["mode"]},{r["run"]},{r["total"]},{r["success"]},'
                    f'{r["success_rate"]:.1f},"{r["corrections"]}","{r["errors"]}",{r["duration"]}\n')
    print(f'\n  CSV: {p} ({len(results)} rows)')
    print('=== DONE ===')

if __name__ == '__main__':
    import sys
    main()
