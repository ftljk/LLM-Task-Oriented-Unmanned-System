#!/usr/bin/env python3
"""
Test: "一号去pantry，同时送货到coe"
- Starts Gazebo + ROS2 + bridge
- Runs agent with pexpect-driven I/O
- Captures output with timestamps
- Prints video segment descriptions
"""
import os, sys, subprocess, time, re, json, signal
import pexpect

WS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENT_DIR = os.path.join(WS_DIR, 'llm_robot_agent')
GO_BIN = os.path.expanduser('~/go1.23/bin/go')
ENV_FILE = os.path.join(AGENT_DIR, '.env')

os.environ.setdefault('GOTOOLCHAIN', 'local')
os.environ.setdefault('GOPROXY', 'https://goproxy.cn,direct')
os.environ.setdefault('ARK_API_KEY', '03978d91-d03d-463a-bf1f-488c6307727d')
os.environ.setdefault('ARK_MODEL_ID', 'deepseek-v4-pro-260425')

log = []  # (timestamp, text)

def ts():
    return time.strftime('%H:%M:%S')

def log_text(text):
    t = ts()
    log.append((t, text))
    print(f'[{t}] {text}', flush=True)

# ── Infrastructure ──
def start_infra(mode='--gui'):
    log_text('Starting infrastructure...')
    
    # Cleanup
    for p in ['gz sim', 'two_robot_sim', 'bridge_node']:
        subprocess.run(f'pkill -9 -f "{p}"', shell=True, stderr=subprocess.DEVNULL)
    subprocess.run('lsof -ti:9090 | xargs kill -9 2>/dev/null', shell=True)
    time.sleep(2)
    
    # Source ROS
    ros_setup = '/opt/ros/kilted/setup.bash'
    ws_setup = f'{WS_DIR}/install/setup.bash'
    
    # 1. Gazebo
    office_models = f'{WS_DIR}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models'
    world = f'{WS_DIR}/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf'
    
    gz_flag = '-s --headless-rendering' if mode == '--headless' else ''
    gz_cmd = f'bash -c "source {ros_setup} && source {ws_setup} && export GZ_SIM_RESOURCE_PATH={office_models} && gz sim -r -v 1 {gz_flag} {world}"'
    
    log_text('[1/4] Starting Gazebo...')
    gz_proc = subprocess.Popen(gz_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6)
    
    # 2. ROS2 launch
    log_text('[2/4] Spawning robots...')
    launch_cmd = f'bash -c "source {ros_setup} && source {ws_setup} && ros2 launch robot_sim_gz two_robot_sim.launch.py"'
    launch_proc = subprocess.Popen(launch_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6)
    
    # 3. Bridge
    log_text('[3/4] Starting WebSocket bridge...')
    bridge_cmd = f'bash -c "source {ros_setup} && source {ws_setup} && ros2 run robot_sim_bridge bridge_node"'
    bridge_proc = subprocess.Popen(bridge_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)
    
    log_text('[4/4] Infrastructure ready.')
    return gz_proc, launch_proc, bridge_proc

# ── Agent interaction ──
def run_agent():
    log_text('Starting agent (pexpect)...')
    
    child = pexpect.spawn(
        f'bash -c "source /opt/ros/kilted/setup.bash && source {WS_DIR}/install/setup.bash && exec {GO_BIN} run . --simulator=ros2 --ws=ws://localhost:9090"',
        cwd=AGENT_DIR,
        encoding='utf-8',
        timeout=600,
        codec_errors='replace',
    )
    child.logfile = sys.stdout  # live output
    
    # Wait for startup banner
    child.expect(['===.*LLM.*Agent.*===', '>>>', pexpect.TIMEOUT], timeout=15)
    log_text('Agent ready.')
    
    # ── Step 1: /status ──
    log_text('>>> /status')
    child.sendline('/status')
    child.expect('>>>', timeout=10)
    log_text('Status output captured.')
    
    # ── Step 2: Send task ──
    log_text('>>> 一号去pantry，同时送货到coe')
    child.sendline('一号去pantry，同时送货到coe')
    
    # ── Step 3: Wait for plan selection or execution ──
    # Patterns to watch for
    idx = child.expect([
        'Select option.*:',          # 0: plan selection prompt
        '\[Executing\]',              # 1: single-plan auto-selected
        '\[Timeout\]',                # 2: timeout
        pexpect.TIMEOUT,              # 3: timeout
        pexpect.EOF,                  # 4: EOF
    ], timeout=300)
    
    if idx == 0:
        log_text('Plan options presented. Selecting option 1...')
        child.sendline('1')
        child.expect(['\[Executing\]', pexpect.TIMEOUT, pexpect.EOF], timeout=120)
    elif idx == 1:
        log_text('Single plan auto-selected. Executing...')
    elif idx == 2 or idx == 3:
        log_text('TIMEOUT waiting for plan options')
        child.close()
        return
    elif idx == 4:
        log_text('Agent exited unexpectedly')
        return
    
    # ── Step 4: Wait for execution to finish ──
    log_text('Waiting for execution to complete...')
    idx = child.expect([
        '>>>',                        # 0: back to prompt
        'Error|FAIL|failed',          # 1: error
        pexpect.TIMEOUT,              # 2: timeout
    ], timeout=300)
    
    if idx == 0:
        log_text('Execution complete.')
    elif idx == 1:
        log_text('Execution error detected.')
    else:
        log_text('Execution timeout.')
    
    # ── Step 5: Final /status ──
    log_text('>>> /status')
    child.sendline('/status')
    idx = child.expect(['>>>', pexpect.TIMEOUT], timeout=10)
    
    # ── Step 6: /quit ──
    log_text('>>> /quit')
    child.sendline('/quit')
    child.expect(pexpect.EOF, timeout=5)
    
    child.close()
    return child.exitstatus

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else '--gui'
    log_text(f'Mode: {mode}')
    log_text('=== Test: 一号去pantry，同时送货到coe ===')
    
    gz_proc = launch_proc = bridge_proc = None
    try:
        gz_proc, launch_proc, bridge_proc = start_infra(mode)
        run_agent()
    finally:
        log_text('Shutting down...')
        for p in [bridge_proc, launch_proc, gz_proc]:
            if p: p.terminate()
        time.sleep(2)
        for pname in ['gz sim', 'two_robot_sim', 'bridge_node']:
            subprocess.run(f'pkill -9 -f "{pname}"', shell=True, stderr=subprocess.DEVNULL)
        log_text('Done.')
    
    # Print video timeline
    print('\n\n═══════════════════════════════════════════════')
    print('  VIDEO SEGMENT DESCRIPTIONS')
    print('═══════════════════════════════════════════════')
    
    # Group log entries into segments
    segments = []
    current = []
    
    for t, text in log:
        if text.startswith('>>> '):
            if current:
                segments.append((current[0][0], '\n'.join(current)))
            current = [text]
        else:
            current.append(text)
    if current:
        segments.append((current[0][0], '\n'.join(current)))
    
    print()
    print('Suggested video timeline and subtitles:\n')
    
    seg_descs = {
        '>>> /status': 
            ('初始状态查询', '显示机器人 1 和 2 的初始位置、朝向和 LIDAR 扫描数据，确认系统就绪'),
        '>>> 一号去pantry，同时送货到coe': 
            ('下达多机任务', '用户输入自然语言指令，要求一号机器人前往 pantry、二号机器人送货到 coe'),
        '>>> 1': 
            ('选择规划方案', 'LLM 提交路径规划选项，用户确认选择方案 1'),
        '>>> /quit': 
            ('结束', '测试完成，退出程序'),
    }
    
    for t, text in segments:
        action = text.split('\n')[0] if '\n' in text else text
        desc = seg_descs.get(action, ('系统处理', text[:80]))
        print(f'  ┌─ T+{t} ── {desc[0]}')
        print(f'  │   {desc[1]}')
        print(f'  │   对应画面内容：')
        # Describe what would be on screen
        if '初始状态' in desc[0]:
            print(f'  │   • Gazebo 俯视图：两机器人静止在起始位置')
            print(f'  │   • 终端输出：显示 robot1 (10,-5)、robot2 (17,-5.5) 坐标')
            print(f'  │   • 字幕建议："系统启动完成，两机器人待命"')
        elif '下达任务' in desc[0]:
            print(f'  │   • 终端输出：用户输入中文指令')
            print(f'  │   • 字幕建议："一号机器人前往 pantry，二号机器人送货到 coe"')
        elif '选择规划方案' in desc[0]:
            print(f'  │   • 终端输出：LLM 生成路径计划，包含多组任务步骤')
            print(f'  │   • 字幕建议："LLM 规划路径，用户确认方案一"')
        elif '结束' in desc[0]:
            print(f'  │   • 终端输出：显示最终状态')
            print(f'  │   • 字幕建议："测试完成"')
        else:
            print(f'  │   • {text[:100]}')
        print()

if __name__ == '__main__':
    main()
