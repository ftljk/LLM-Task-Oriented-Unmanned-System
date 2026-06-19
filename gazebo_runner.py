#!/usr/bin/env python3
"""Gazebo 验证实验采集器 — 连接到已有桥接，运行 LLM 链路"""

import os, sys, time, re, json, subprocess as sp
import websocket

WS = 'ws://localhost:9090'
AGENT_DIR = '/home/mofus/rmf_ws/llm_robot_agent'
GO = '/home/mofus/go1.23/bin/go'
LOG = '/tmp/gazebo_exp.csv'

ws = None
def connect():
    global ws
    ws = websocket.create_connection(WS, timeout=10)

def get_status():
    ws.send(json.dumps({'type': 'get_status'}))
    resp = json.loads(ws.recv())
    return resp.get('robots', {})

def parse_positions(raw):
    """Parse /status text output for positions"""
    pos = {}
    for m in re.finditer(r'(robot\d):\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s*\[(\w+)\]\s*scan front=([\d.]+)', raw, re.DOTALL):
        pos[m.group(1)] = {
            'x': float(m.group(2)), 'y': float(m.group(3)), 'theta': float(m.group(4)),
            'status': m.group(5), 'front': float(m.group(6))
        }
    return pos

def run_agent_cmd(command, timeout=300):
    """用 pexpect 运行 agent，发送指令"""
    import pexpect
    env = dict(os.environ, GOTOOLCHAIN='local', GOPROXY='https://goproxy.cn,direct',
               ARK_API_KEY='03978d91-d03d-463a-bf1f-488c6307727d')
    child = pexpect.spawn(f'{GO} run . --simulator=ros2 --ws=ws://localhost:9090',
                           cwd=AGENT_DIR, encoding='utf-8', timeout=timeout, env=env,
                           codec_errors='replace')
    try:
        child.expect('>>>', timeout=25)
        child.sendline('/status')
        child.expect('>>>', timeout=10)
        before = child.before + child.after

        child.sendline(command)
        i = child.expect_exact(['Select option', '[Executing]'], timeout=200)
        if i == 0:
            child.sendline('1')
            child.expect_exact('[Executing]', timeout=200)

        child.expect('>>>', timeout=timeout)
        child.sendline('/status')
        child.expect('>>>', timeout=10)
        after = child.before + child.after

        child.sendline('/quit')
        child.expect(pexpect.EOF, timeout=8)
        return {'before': before, 'after': after, 'full': str(child.before) + str(child.after),
                'error': None}
    except Exception as e:
        return {'before': '', 'after': '', 'full': '', 'error': str(e)}
    finally:
        try: child.close(force=True)
        except: pass

# ═══ 实验A：前进 1m ×3 ═══
def test_forward(runs=3):
    print('\n=== 实验A：前进 1m ===')
    data = []
    for i in range(runs):
        r = run_agent_cmd('一号向前移动1米', 180)
        b = parse_positions(r['before'])
        a = parse_positions(r['after'])
        r1b = b.get('robot1', {})
        r1a = a.get('robot1', {})
        if r1b and r1a:
            d = ((r1a['x']-r1b['x'])**2 + (r1a['y']-r1b['y'])**2)**0.5
            err = abs(d - 1.0)
            print(f'  [{i+1}] ({r1b["x"]:.3f},{r1b["y"]:.3f})→({r1a["x"]:.3f},{r1a["y"]:.3f}) d={d:.3f} err={err:.3f}')
            data.append({'run': i+1, 'bx': r1b['x'], 'by': r1b['y'], 'ax': r1a['x'], 'ay': r1a['y'],
                         'dist': d, 'err': err})
        else:
            print(f'  [{i+1}] ERROR: {r.get("error", "no position data")}')
        time.sleep(3)
    if data:
        errs = [d['err'] for d in data]
        print(f'  → 平均: {sum(errs)/len(errs):.4f}m  最大: {max(errs):.4f}m')
    return data

# ═══ 实验B：旋转 90° ×3 ═══
def test_rotate(runs=3):
    print('\n=== 实验B：旋转 90° ===')
    data = []
    cmds = ['一号向左转90度', '一号向右转90度']
    for i in range(runs):
        cmd = cmds[i % 2]
        r = run_agent_cmd(cmd, 180)
        b = parse_positions(r['before'])
        a = parse_positions(r['after'])
        r1b = b.get('robot1', {})
        r1a = a.get('robot1', {})
        if r1b and r1a:
            dt = r1a['theta'] - r1b['theta']
            dt = (dt + 3.14159) % (2*3.14159) - 3.14159
            err_rad = abs(abs(dt) - 1.5708)
            err_deg = err_rad * 180 / 3.14159
            print(f'  [{i+1}] {cmd}: θ {r1b["theta"]:.3f}→{r1a["theta"]:.3f} dθ={dt:.3f} err={err_deg:.2f}°')
            data.append({'run': i+1, 'cmd': cmd, 'tb': r1b['theta'], 'ta': r1a['theta'],
                         'dtheta': dt, 'err_deg': err_deg})
        else:
            print(f'  [{i+1}] ERROR: {r.get("error", "no position data")}')
        time.sleep(3)
    if data:
        errs = [d['err_deg'] for d in data]
        print(f'  → 平均角误差: {sum(errs)/len(errs):.2f}°  最大: {max(errs):.2f}°')
    return data

# ═══ 实验C：路径跟踪 ═══
def test_path():
    print('\n=== 实验C：路径跟踪 (pantry+coe) ===')
    r = run_agent_cmd('一号去pantry，同时送货到coe', 600)
    if r['error']:
        print(f'  ❌ {r["error"]}')
        return r
    a = parse_positions(r['after'])
    targets = {'robot1': (16.85, -5.40, 'pantry'), 'robot2': (5.35, -4.98, 'coe')}
    for name, (tx, ty, label) in targets.items():
        if name in a:
            err = ((a[name]['x']-tx)**2 + (a[name]['y']-ty)**2)**0.5
            s = '✅' if err < 0.5 else '⚠️'
            print(f'  {s} {name}→{label}: ({a[name]["x"]:.2f},{a[name]["y"]:.2f}) err={err:.3f}m')
        else:
            print(f'  ❌ {name}: no data')
    # Laser guard
    emergency = re.findall(r'EMERGENCY STOP[^\n]*', r.get('full', ''), re.IGNORECASE)
    if emergency:
        for e in emergency:
            print(f'  ⚠️  激光守卫触发: {e.strip()}')
    else:
        print(f'  ✅ 未触发紧急停（扫描距离持续正常）')
    return r

if __name__ == '__main__':
    print('='*55)
    print('  Gazebo 仿真功能验证 (LLM 全链路)')
    print('='*55)

    results = {}
    results['forward'] = test_forward(3)
    # Recenter after forward
    print('  → 回到起点...')
    run_agent_cmd('一号后退1米', 120)
    time.sleep(2)

    results['rotate'] = test_rotate(3)
    # Recenter heading
    print('  → 回正...')
    run_agent_cmd('一号向右转90度', 120)

    results['path'] = test_path()

    print('\n=== 完成 ===')
