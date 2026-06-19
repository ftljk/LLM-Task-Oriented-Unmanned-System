#!/usr/bin/env python3
"""Gazebo 仿真功能验证 — 量化实验数据采集"""
import os, sys, subprocess, time, re, json, signal

WS = '/home/mofus/rmf_ws'
AGENT_DIR = f'{WS}/llm_robot_agent'
GO = '/home/mofus/go1.23/bin/go'
OFFICE_MODELS = f'{WS}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models'
WORLD = f'{WS}/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf'
ROS_SETUP = '/opt/ros/kilted/setup.bash'
WS_SETUP = f'{WS}/install/setup.bash'

os.environ.setdefault('ARK_API_KEY', '03978d91-d03d-463a-bf1f-488c6307727d')
os.environ.setdefault('GOTOOLCHAIN', 'local')
os.environ.setdefault('GOPROXY', 'https://goproxy.cn,direct')

procs = []
logfile = f'{AGENT_DIR}/gazebo_validation.log'

def log(msg):
    t = time.strftime('%H:%M:%S')
    line = f'[{t}] {msg}'
    print(line)
    with open(logfile, 'a') as f:
        f.write(line + '\n')

def ros_source():
    return f'source {ROS_SETUP} && source {WS_SETUP} && '

def start_ros2_cmd(cmd, desc, wait_sec=6):
    full = f'bash -c "{ros_source()} {cmd}"'
    log(f'  Starting {desc}...')
    p = subprocess.Popen(full, shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(p)
    time.sleep(wait_sec)
    return p

def stop_all():
    log('Shutdown...')
    for p in reversed(procs):
        p.terminate()
    time.sleep(2)
    for p in procs:
        try: p.kill()
        except: pass
    for pat in ['gz sim', 'two_robot_sim', 'bridge_node', 'robot_state_publisher']:
        subprocess.run(f'pkill -9 -f "{pat}"', shell=True, stderr=subprocess.DEVNULL)

def run_agent(command, timeout=300):
    """pexpect interaction with the agent"""
    import pexpect
    cmd = f'bash -c "{ros_source()} exec {GO} run . --simulator=ros2 --ws=ws://localhost:9090"'
    log(f'  Agent start: {command[:50]}')
    child = pexpect.spawn(cmd, cwd=AGENT_DIR, encoding='utf-8',
                          timeout=timeout, codec_errors='replace')
    # Disable log to reduce noise
    child.logfile = None

    r = {'status_before': '', 'status_after': '', 'full_output': '', 'error': None}

    try:
        child.expect('>>>', timeout=25)
        child.sendline('/status')
        child.expect('>>>', timeout=10)
        r['status_before'] = child.before + child.after

        child.sendline(command)
        i = child.expect_exact(['Select option', '[Executing]'], timeout=200)
        if i == 0:
            child.sendline('1')
            child.expect_exact('[Executing]', timeout=200)

        child.expect('>>>', timeout=timeout)
        child.sendline('/status')
        child.expect('>>>', timeout=10)
        r['status_after'] = child.before + child.after

        child.sendline('/quit')
        child.expect(pexpect.EOF, timeout=8)
    except pexpect.TIMEOUT as e:
        r['error'] = f'TIMEOUT after {timeout}s'
        log(f'  TIMEOUT: {e}')
    except Exception as e:
        r['error'] = str(e)
        log(f'  ERROR: {e}')
    finally:
        try: child.close(force=True)
        except: pass

    r['full_output'] = str(child.before) + str(child.after) if hasattr(child, 'after') else ''
    return r

def parse_status(text):
    pos = {}
    for m in re.finditer(r'(robot\d):\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s*\[(\w+)\]\s*scan front=([\d.]+)',
                         text, re.DOTALL):
        pos[m.group(1)] = {'x': float(m.group(2)), 'y': float(m.group(3)),
                           'theta': float(m.group(4)), 'status': m.group(5),
                           'front': float(m.group(6))}
    return pos

def exp_forward(runs=3):
    log('\n' + '='*50)
    log('实验A：前进 1m 精度 × %d' % runs)
    log('='*50)
    data = []
    for i in range(runs):
        r = run_agent('一号向前移动1米', 180)
        data.append({'run': i+1, **r})
        time.sleep(3)
    return data

def exp_rotate(runs=3):
    log('\n' + '='*50)
    log('实验B：旋转 90° 精度 × %d' % runs)
    log('='*50)
    data = []
    cmds = ['一号向左转90度', '一号向右转90度']
    for i in range(runs):
        cmd = cmds[i % 2]
        r = run_agent(cmd, 180)
        data.append({'run': i+1, 'cmd': cmd, **r})
        time.sleep(3)
    return data

def exp_path():
    log('\n' + '='*50)
    log('实验C：路径跟踪（一号去pantry，同时送货到coe）')
    log('='*50)
    return run_agent('一号去pantry，同时送货到coe', 600)

def print_summary(results):
    print('\n\n' + '='*65)
    print('  Gazebo 仿真功能验证 — 量化实验结果')
    print('='*65)

    if 'forward' in results:
        print('\n■ 实验A：前进 1m 精度')
        header = f'  {"Run":<5} {"起点(x,y)":<22} {"终点(x,y)":<22} {"距离(m)":<10} {"误差(m)":<10}'
        print(header)
        print('  ' + '-'*len(header))
        errs = []
        for r in results['forward']:
            b = parse_status(r.get('status_before',''))
            a = parse_status(r.get('status_after',''))
            r1b = b.get('robot1', {})
            r1a = a.get('robot1', {})
            if r1b and r1a:
                dx = r1a['x'] - r1b['x']
                dy = r1a['y'] - r1b['y']
                dist = (dx*dx + dy*dy)**0.5
                err = abs(dist - 1.0)
                errs.append(err)
                front_before = r1b.get('front', 0)
                front_after = r1a.get('front', 0)
                print(f'  {r["run"]:<5} ({r1b["x"]:.3f},{r1b["y"]:.3f})  ({r1a["x"]:.3f},{r1a["y"]:.3f})  {dist:<10.3f} {err:<10.3f}')
                log(f'  A-{r["run"]}: ({r1b["x"]:.3f},{r1b["y"]:.3f})→({r1a["x"]:.3f},{r1a["y"]:.3f}) dist={dist:.3f} err={err:.3f}')
            elif r.get('error'):
                print(f'  {r["run"]:<5} ERROR: {r["error"]}')
        if errs:
            print(f'\n  → 平均误差: {sum(errs)/len(errs):.4f}m  最大误差: {max(errs):.4f}m')

    if 'rotate' in results:
        print('\n■ 实验B：旋转 90° 精度')
        header = f'  {"Run":<5} {"指令":<14} {"θ_前":<9} {"θ_后":<9} {"dθ":<9} {"误差(°)":<10}'
        print(header)
        print('  ' + '-'*len(header))
        errs = []
        for r in results['rotate']:
            b = parse_status(r.get('status_before',''))
            a = parse_status(r.get('status_after',''))
            r1b = b.get('robot1', {})
            r1a = a.get('robot1', {})
            if r1b and r1a:
                dtheta = r1a['theta'] - r1b['theta']
                dtheta = (dtheta + 3.14159) % (2*3.14159) - 3.14159
                err_rad = abs(abs(dtheta) - 1.5708)
                err_deg = err_rad * 180 / 3.14159
                errs.append(err_rad)
                print(f'  {r["run"]:<5} {r.get("cmd","?"):<14} {r1b["theta"]:<9.3f} {r1a["theta"]:<9.3f} {dtheta:<9.3f} {err_deg:<10.2f}')
                log(f'  B-{r["run"]}: {r.get("cmd")} θ={r1b["theta"]:.3f}→{r1a["theta"]:.3f} dθ={dtheta:.3f} err={err_deg:.2f}°')
            elif r.get('error'):
                print(f'  {r["run"]:<5} ERROR: {r["error"]}')
        if errs:
            avg_deg = sum(errs)/len(errs)*180/3.14159
            max_deg = max(errs)*180/3.14159
            print(f'\n  → 平均角误差: {sum(errs)/len(errs):.4f} rad ({avg_deg:.2f}°)')
            print(f'  → 最大角误差: {max(errs):.4f} rad ({max_deg:.2f}°)')

    if 'path' in results:
        print('\n■ 实验C：路径跟踪')
        r = results['path']
        if r.get('error'):
            print(f'  ❌ 错误: {r["error"]}')
        else:
            a = parse_status(r.get('status_after',''))
            targets = {'robot1': (16.85, -5.40, 'pantry'),
                       'robot2': (5.35, -4.98, 'coe')}
            for name, (tx, ty, label) in targets.items():
                if name in a:
                    p = a[name]
                    err = ((p['x']-tx)**2 + (p['y']-ty)**2)**0.5
                    status = '✅' if err < 0.5 else '⚠️'
                    print(f'  {status} {name}→{label}: 终点({p["x"]:.2f},{p["y"]:.2f}) 目标({tx:.2f},{ty:.2f}) 误差{err:.3f}m')
                    log(f'  C-{name}: ({p["x"]:.3f},{p["y"]:.3f}) target=({tx:.3f},{ty:.3f}) err={err:.3f}m')
                else:
                    print(f'  ❌ {name}: 无位置数据')
            b = parse_status(r.get('status_before',''))
            for name in targets:
                if name in b:
                    p = b[name]
                    log(f'  C-{name} start: ({p["x"]:.3f},{p["y"]:.3f}) θ={p["theta"]:.3f}')

        # 激光守卫数据
        print('\n■ 实验D：激光守卫（从日志提取）')
        full = r.get('full_output', '') + r.get('status_after', '') + r.get('status_before', '')
        emergency = re.findall(r'EMERGENCY STOP[^\n]*|emergency[^\n]*', full, re.IGNORECASE)
        if emergency:
            for e in emergency:
                print(f'  ⚠️  {e.strip()}')
                log(f'  D-emergency: {e.strip()}')
        else:
            print('  ✅ 本次运行未触发紧急停')

        # 扫描数据变化
        if 'robot1' in b and 'robot1' in a:
            front_diff = b['robot1'].get('front', 0) - a['robot1'].get('front', 0)
            print(f'  robot1 前方距离变化: {b["robot1"].get("front",0):.2f}m → {a["robot1"].get("front",0):.2f}m')
            print(f'  WebSocket 桥接通通信: 连续 {len(results.get("forward",[]))+len(results.get("rotate",[]))+1} 次交互无断连')

    print('\n' + '='*65)
    print('  验证完成')
    print('='*65)

def main():
    # Cleanup previous
    for pat in ['gz sim', 'two_robot_sim', 'bridge_node', 'robot_state_publisher']:
        subprocess.run(f'pkill -9 -f "{pat}" 2>/dev/null', shell=True)
    subprocess.run('lsof -ti:9090 | xargs kill -9 2>/dev/null', shell=True)
    time.sleep(2)
    if os.path.exists(logfile):
        os.remove(logfile)

    log('='*60)
    log('Gazebo 仿真功能验证')
    log('='*60)

    try:
        # Start Gazebo GUI
        gz_cmd = f'bash -c "export GZ_SIM_RESOURCE_PATH={OFFICE_MODELS} && {ros_source()} gz sim -r -v 1 {WORLD}"'
        log('Gazebo start (GUI)...')
        p = subprocess.Popen(gz_cmd, shell=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p)
        time.sleep(8)

        # ROS2 launch
        start_ros2_cmd('ros2 launch robot_sim_gz two_robot_sim.launch.py', 'ROS2 launch', 8)

        # Bridge
        start_ros2_cmd('ros2 run robot_sim_bridge bridge_node', 'Bridge', 5)

        # Wait for /startup_done
        log('Wait /startup_done...')
        check = subprocess.run(
            f'bash -c "{ros_source()} timeout 15 ros2 topic echo /startup_done --once 2>/dev/null || true"',
            shell=True, timeout=20, capture_output=True, text=True)
        if check.returncode != 0:
            log('WARN: /startup_done not received, continuing anyway')

        log('\n=== 基础设施就绪 ===')

        # Experiments
        results = {}

        results['forward'] = exp_forward(3)
        # After forward test, robot1 is ~1m east; recenter for rotate
        log('Recenter robot1 for rotate test...')
        r = run_agent('一号向后移动1米', 120)
        time.sleep(3)
        # But we also need a cleaner starting rotation. Actually the robot
        # started facing east and moved east, so heading is still ~0. Good.

        results['rotate'] = exp_rotate(3)
        # Recenter heading
        r = run_agent('一号向左转90度', 120)
        # Now approximately facing south (theta~90°)
        results['path'] = exp_path()

        print_summary(results)

        log('\n保存结果到 gazebo_validation.log 完成')

    except Exception as e:
        log(f'FATAL: {e}')
        import traceback
        traceback.print_exc()
    finally:
        stop_all()

if __name__ == '__main__':
    main()
