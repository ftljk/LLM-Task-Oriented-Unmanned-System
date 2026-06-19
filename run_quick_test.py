#!/usr/bin/env python3
"""Quick verification test - launches stack and runs one command."""
import sys, os, time, subprocess, signal

WS_DIR = "/home/mofus/rmf_ws"
AGENT_DIR = f"{WS_DIR}/llm_robot_agent"
BINARY = f"{AGENT_DIR}/llm_robot_agent"
NAV_GRAPH = f"{WS_DIR}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"
ARK_KEY = "03978d91-d03d-463a-bf1f-488c6307727d"

# Build env
env = os.environ.copy()
env["ARK_API_KEY"] = ARK_KEY
env["ARK_MODEL_ID"] = "deepseek-v3-2-251201"
env["GOTOOLCHAIN"] = "local"
env["GOPROXY"] = "https://goproxy.cn,direct"
env["GZ_SIM_RESOURCE_PATH"] = f"{WS_DIR}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models"
# Source ROS
proc = subprocess.run(
    "source /opt/ros/kilted/setup.bash && source {}/install/setup.bash && env".format(WS_DIR),
    shell=True, capture_output=True, text=True, executable="/bin/bash"
)
for line in proc.stdout.splitlines():
    if "=" in line:
        k, v = line.split("=", 1)
        env[k] = v

procs = []

def launch():
    print("[1/4] Starting Gazebo...")
    p = subprocess.Popen(["gz", "sim", "-r", "-v", "1",
        f"{WS_DIR}/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    procs.append(p)
    time.sleep(5)

    print("[2/4] Spawning robots...")
    p = subprocess.Popen(["ros2", "launch", "robot_sim_gz", "two_robot_sim.launch.py"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    procs.append(p)
    time.sleep(5)

    print("[3/4] Starting bridge...")
    p = subprocess.Popen(["ros2", "run", "robot_sim_bridge", "bridge_node"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    procs.append(p)
    time.sleep(3)

    print("[4/4] Starting Go agent (background, stdin piped)...")
    return p

def cleanup():
    print("\nCleaning up...")
    for p in reversed(procs):
        try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except: pass
    time.sleep(1)
    for p in reversed(procs):
        try: p.kill()
        except: pass
    subprocess.run("timeout 3 killall gz-server bridge_node 2>/dev/null", shell=True)

if __name__ == "__main__":
    try:
        launch()

        # Now run agent and interact
        import pexpect
        cmd = f"{BINARY} --simulator=ros2 --ws=ws://localhost:9090 --nav-graph={NAV_GRAPH}"
        child = pexpect.spawn(cmd, cwd=AGENT_DIR, env=env, timeout=300,
                              encoding='utf-8', codec_errors='replace')
        child.setwinsize(40, 200)

        print("Waiting for agent prompt...")
        child.expect(r'>>>', timeout=60)
        print("Agent ready!")

        # Test command: dual robot different destinations (worked before)
        test_cmd = "一号机器人去pantry，2号去coe"
        print(f"\nSending: {test_cmd}")
        child.sendline(test_cmd)

        # Monitor output
        last_output = ""
        while True:
            idx = child.expect([
                r'>>>\s*',
                r'Select option \(1-\d+\) or 0 to cancel:',
                pexpect.TIMEOUT
            ], timeout=300)
            chunk = child.before + child.after
            last_output += chunk
            print(chunk[:500], end='')

            if idx == 0:
                print("\n[Done - prompt returned]")
                break
            elif idx == 1:
                print("\n[Auto-selecting option 1]")
                child.sendline("1")
                continue
            elif idx == 2:
                print("\n[Timeout]")
                break

        # Check results
        output = last_output
        completed = output.lower().count("[completed]")
        failed = output.lower().count("[failed]")
        skipped = output.lower().count("[skipped]")
        crashed = "panic" in output.lower() or "fatal" in output.lower()
        timed_out = "timeout" in output.lower()

        print(f"\n{'='*50}")
        print(f"RESULTS:")
        print(f"  completed: {completed}")
        print(f"  failed: {failed}")
        print(f"  skipped: {skipped}")
        print(f"  crash: {crashed}")
        print(f"  timeout: {timed_out}")
        print(f"  OVERALL: {'PASS' if (failed==0 and skipped==0 and not crashed) else 'FAIL'}")
        print(f"{'='*50}")

        # Save full output for inspection
        with open("/tmp/test_output.txt", "w") as f:
            f.write(output)
        print("Full output saved to /tmp/test_output.txt")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()
