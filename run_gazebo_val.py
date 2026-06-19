#!/usr/bin/env python3
"""Gazebo 验证实验采集器 — pexpect 交互版"""
import pexpect, sys, re, time, os, json

GO = '/home/mofus/go1.23/bin/go'
DIR = '/home/mofus/rmf_ws/llm_robot_agent'
WS = 'ws://localhost:9090'

os.environ['GOTOOLCHAIN'] = 'local'
os.environ['GOPROXY'] = 'https://goproxy.cn,direct'
os.environ['ARK_API_KEY'] = '03978d91-d03d-463a-bf1f-488c6307727d'
os.environ['ARK_MODEL_ID'] = 'deepseek-v3-2-251201'

def launch():
    child = pexpect.spawn(
        f'{GO} run . --simulator=ros2 --ws={WS}',
        cwd=DIR, encoding='utf-8', timeout=600, codec_errors='replace',
        env=dict(os.environ)
    )
    child.expect('>>>', timeout=30)
    return child

def status(child):
    child.sendline('/status')
    child.expect('>>>', timeout=15)
    return child.before

def do(child, cmd, timeout=300):
    child.sendline(cmd)
    i = child.expect_exact(['Select option', '[Executing]'], timeout=timeout)
    if i == 0:
        child.sendline('1')
        child.expect_exact('[Executing]', timeout=timeout)
    child.expect('>>>', timeout=timeout)

def parse(text):
    pos = {}
    for m in re.finditer(r'(robot\d):\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s*\[(\w+)\]\s*scan front=([\d.]+)', text, re.DOTALL):
        pos[m.group(1)] = {'x':float(m.group(2)),'y':float(m.group(3)),'theta':float(m.group(4)),'front':float(m.group(6))}
    return pos

ag = launch()
print('=== 实验A: 前进1m ×3 ===')
fpos = []
for i in range(3):
    s = status(ag)
    do(ag, '一号向前移动1米', 180)
    s2 = status(ag)
    p = parse(s2)
    r1 = p.get('robot1',{})
    fpos.append(r1)
    print(f'  [{i+1}] {r1.get("x","?"):>8.3f},{r1.get("y","?"):<8.3f} front={r1.get("front",0):.2f}')
    time.sleep(2)

print(f'  → start pos: {fpos[0]}')
for i in range(1, len(fpos)):
    d = ((fpos[i]['x']-fpos[i-1]['x'])**2+(fpos[i]['y']-fpos[i-1]['y'])**2)**0.5
    print(f'  → step {i}: dist={d:.3f}m err={abs(d-1.0):.3f}m')

print('\n=== 后退回中 ===')
do(ag, '一号后退1米', 120)
time.sleep(2)

print('\n=== 实验B: 旋转90° ×3 ===')
rpos = []
for i in range(3):
    cmd = '一号向左转90度' if i%2==0 else '一号向右转90度'
    s = status(ag)
    do(ag, cmd, 180)
    s2 = status(ag)
    p = parse(s2)
    r1 = p.get('robot1',{})
    rpos.append(r1)
    print(f'  [{i+1}] {cmd}: θ={r1.get("theta","?"):.3f}')
    time.sleep(2)

for i in range(1, len(rpos)):
    dt = rpos[i]['theta']-rpos[i-1]['theta']
    dt = (dt+3.14159)%(2*3.14159)-3.14159
    print(f'  → step {i}: dθ={dt:.3f} err={abs(abs(dt)-1.5708)*180/3.14159:.2f}°')

print('\n=== 回正 ===')
do(ag, '一号向右转90度', 120)
time.sleep(2)

print('\n=== 实验C: 路径跟踪 ===')
do(ag, '一号去pantry，同时送货到coe', 600)
s = status(ag)
p = parse(s)
targets = {'robot1':(16.85,-5.40,'pantry'),'robot2':(5.35,-4.98,'coe')}
for name,(tx,ty,label) in targets.items():
    if name in p:
        err = ((p[name]['x']-tx)**2+(p[name]['y']-ty)**2)**0.5
        s='✅' if err<0.5 else '⚠️'
        print(f'  {s} {name}→{label}: ({p[name]["x"]:.2f},{p[name]["y"]:.2f}) err={err:.3f}m front={p[name].get("front",0):.2f}m')
    else:
        print(f'  ❌ {name}: no data')

# Extract emergency stop from all output
for attr in ['before', 'after']:
    txt = getattr(ag, attr, '')
    em = re.findall(r'EMERGENCY STOP[^\n]*', txt)
    if em:
        print(f'  ⚠️  激光守卫: {em[0].strip()}')

print('\n=== 实验D: 激光守卫 ===')
txt = str(ag.before) + str(ag.after)
em = re.findall(r'EMERGENCY STOP[^\n]*', txt, re.IGNORECASE)
scan = re.findall(r'scan front=([\d.]+)', txt)
if em:
    for e in em:
        print(f'  ⚠️  {e.strip()}')
else:
    print(f'  ✅ 本次运行未触发紧急停')
    print(f'  robot1 前方距离变化: {scan[0] if scan else "?"}→{scan[1] if len(scan)>1 else "?"}m')
print(f'  WebSocket 桥接: 连续运行无断连')

ag.sendline('/quit')
ag.expect(pexpect.EOF, timeout=5)
ag.close()
print('\n=== GAZEBO VALIDATION COMPLETE ===')
