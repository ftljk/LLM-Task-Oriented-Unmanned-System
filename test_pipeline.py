#!/usr/bin/env python3
"""
Automated multi-phase test pipeline for LLM robot agent.
Launches Gazebo+ROS2+bridge, then feeds commands via pexpect.
Phases:
  A: Single robot basic move
  B: Single robot nav graph route
  C: Dual robot different zones
  D: Dual robot same-area conflict resolution
  E: Patrol + delivery concurrent
"""

import pexpect
import subprocess
import time
import os
import signal
import sys
import re

WS_DIR = "/home/mofus/rmf_ws"
AGENT_DIR = f"{WS_DIR}/llm_robot_agent"
BINARY = f"{AGENT_DIR}/llm_robot_agent"
NAV_GRAPH = f"{WS_DIR}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"

# Build environment
BASE_ENV = os.environ.copy()
BASE_ENV["GOTOOLCHAIN"] = "local"
BASE_ENV["GOPROXY"] = "https://goproxy.cn,direct"
BASE_ENV["ARK_API_KEY"] = "03978d91-d03d-463a-bf1f-488c6307727d"
BASE_ENV["ARK_MODEL_ID"] = "deepseek-v3-2-251201"
BASE_ENV["GZ_SIM_RESOURCE_PATH"] = f"{WS_DIR}/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models:{BASE_ENV.get('GZ_SIM_RESOURCE_PATH', '')}"
BASE_ENV["RUST_BACKTRACE"] = "0"

def source_ros():
    """Source ROS2 and workspace into environment."""
    import shlex, subprocess as sp
    cmd = "source /opt/ros/kilted/setup.bash && source {}/install/setup.bash && env".format(WS_DIR)
    proc = sp.run(["bash", "-c", cmd], capture_output=True, text=True)
    for line in proc.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            BASE_ENV[k] = v

class TestPipeline:
    def __init__(self):
        self.procs = []  # background processes
        self.agent = None  # pexpect child
        self.phase_results = {}  # phase_name -> (pass_count, total_count, errors)

    def launch_background(self):
        """Launch Gazebo, ROS2 launch, bridge in background."""
        print("[Launch] Starting Gazebo server+GUI...")
        proc = subprocess.Popen(
            ["gz", "sim", "-r", "-v", "1",
             f"{WS_DIR}/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf"],
            env=BASE_ENV,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        self.procs.append(proc)
        print("[Launch] Gazebo PID:", proc.pid)
        time.sleep(5)

        print("[Launch] Spawning robots and ROS2 bridges...")
        proc = subprocess.Popen(
            ["ros2", "launch", "robot_sim_gz", "two_robot_sim.launch.py"],
            env=BASE_ENV,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        self.procs.append(proc)
        print("[Launch] Launch PID:", proc.pid)
        time.sleep(5)

        print("[Launch] Starting WebSocket bridge...")
        proc = subprocess.Popen(
            ["ros2", "run", "robot_sim_bridge", "bridge_node"],
            env=BASE_ENV,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        self.procs.append(proc)
        print("[Launch] Bridge PID:", proc.pid)
        time.sleep(3)

    def start_agent(self):
        """Start Go agent as pexpect child."""
        print("[Agent] Starting Go agent...")
        cmd = f"{BINARY} --simulator=ros2 --ws=ws://localhost:9090 --nav-graph={NAV_GRAPH}"
        self.agent = pexpect.spawn(
            cmd,
            cwd=AGENT_DIR,
            env=BASE_ENV,
            timeout=300,
            encoding='utf-8',
            codec_errors='replace'
        )
        # Set terminal size to avoid wrapping issues
        self.agent.setwinsize(40, 200)
        self._log_progress("Agent started, waiting for prompt...")
        idx = self.agent.expect([r'>>>\s*$', r'>>>', pexpect.TIMEOUT], timeout=120)
        if idx == 2:
            # Print what we got so far
            print(f"[Agent] TIMEOUT waiting for prompt. Output so far:\n{self.agent.before}")
            # Try to send newline to trigger prompt
            self.agent.sendline('')
            idx = self.agent.expect([r'>>>\s*$', r'>>>', pexpect.TIMEOUT], timeout=30)
            if idx == 2:
                raise RuntimeError("Agent failed to start in time")
        self._log_progress("Agent prompt ready!")

    def _log_progress(self, msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _read_until_prompt(self, timeout=180):
        """Read all output until next >>> prompt."""
        try:
            idx = self.agent.expect([r'>>>\s*$', r'>>>'], timeout=timeout)
            output = self.agent.before + self.agent.after
            return output
        except pexpect.TIMEOUT:
            output = self.agent.before
            return output

    def send_command(self, cmd, timeout=240):
        """Send a command and wait for completion (prompt or plan selection)."""
        self._log_progress(f"Sending: {cmd}")
        self.agent.sendline(cmd)

        # Read output until we get back to the >>> prompt
        full_output = ""
        while True:
            idx = self.agent.expect([
                r'>>>\s*$',                          # 0: back to prompt
                r'Select option \(1-\d+\) or 0 to cancel:',  # 1: plan selection prompt
                r'>>>\s*$',                          # 2: prompt (duplicate)
                pexpect.TIMEOUT                       # 3: timeout
            ], timeout=timeout)

            chunk = self.agent.before + self.agent.after
            full_output += chunk

            if idx == 0 or idx == 2:
                # Back at prompt - command finished
                break
            elif idx == 1:
                # Plan selection prompt - select option 1
                print("[AutoSelect] Choosing option 1")
                self.agent.sendline("1")
                full_output += "\n[Auto-selected option 1]\n"
                timeout = 180  # shorter timeout for execution
                continue
            elif idx == 3:
                print(f"[Timeout] No prompt after {timeout}s")
                full_output += "\n[TIMEOUT]\n"
                break

        return full_output

    def check_results(self, output, phase_name):
        """Parse results from output and check for success/failure."""
        # Count tasks and their statuses
        completed = output.count("[completed]") + output.count("completed]") + output.count("completed,")
        failed = output.count("[failed]") + output.count("failed]")
        skipped = output.count("[skipped]") + output.count("skipped]")

        # Look for explicit error indicators
        has_errors = "error" in output.lower() and "no error" not in output.lower()
        has_timeout = "timeout" in output.lower()
        has_crash = "panic" in output.lower() or "fatal" in output.lower()

        # Log results
        print(f"\n=== Phase '{phase_name}' Results ===")
        print(f"  Completed tasks: {completed}")
        print(f"  Failed tasks: {failed}")
        print(f"  Skipped tasks: {skipped}")
        print(f"  Other issues: errors={has_errors}, timeout={has_timeout}, crash={has_crash}")

        # Print the last portion of output for diagnosis
        lines = output.strip().split('\n')
        print(f"  Last 20 lines:")
        for line in lines[-20:]:
            print(f"    {line.strip()}")

        success = (failed == 0 and skipped == 0 and not has_crash and not has_timeout)
        self.phase_results[phase_name] = {
            "success": success,
            "completed": completed,
            "failed": failed,
            "skipped": skipped,
            "errors": has_errors,
            "timeout": has_timeout
        }
        return success

    def run_phase_A_single_move(self):
        """Phase A: Single robot basic movement."""
        print("\n" + "="*60)
        print("PHASE A: Single Robot Forward Move")
        print("="*60)

        cmd = "一号向前移动1米"
        output = self.send_command(cmd)

        return self.check_results(output, "A_single_move")

    def run_phase_B_single_route(self):
        """Phase B: Single robot nav graph route."""
        print("\n" + "="*60)
        print("PHASE B: Single Robot Route (robot1 → pantry)")
        print("="*60)

        cmd = "一号机器人去pantry"
        output = self.send_command(cmd, timeout=300)

        return self.check_results(output, "B_single_route")

    def run_phase_C_dual_different(self):
        """Phase C: Dual robot different zones."""
        print("\n" + "="*60)
        print("PHASE C: Dual Robot Different Zones (robot1 → pantry, robot2 → coe)")
        print("="*60)

        cmd = "一号机器人去pantry，2号去coe"
        output = self.send_command(cmd, timeout=300)

        return self.check_results(output, "C_dual_different")

    def run_phase_D_dual_conflict(self):
        """Phase D: Dual robot same-area conflict (pantry + coe delivery)."""
        print("\n" + "="*60)
        print("PHASE D: Dual Robot Same Area (robot1 → pantry, robot2 → coe delivery)")
        print("="*60)

        cmd = "一号去pantry，同时送货到coe"
        output = self.send_command(cmd, timeout=300)

        return self.check_results(output, "D_dual_conflict")

    def run_phase_E_patrol(self):
        """Phase E: Patrol route."""
        print("\n" + "="*60)
        print("PHASE E: Patrol Route")
        print("="*60)

        cmd = "一号开始巡逻"
        output = self.send_command(cmd, timeout=300)

        return self.check_results(output, "E_patrol")

    def cleanup(self):
        """Kill all processes."""
        print("\n[Cleanup] Terminating all processes...")
        if self.agent and self.agent.isalive():
            self.agent.sendline('/quit')
            time.sleep(1)
            self.agent.terminate(force=True)
        for proc in self.procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except:
                try:
                    proc.kill()
                except:
                    pass
        # Extra cleaning
        subprocess.run(["killall", "-9", "gz-server"], capture_output=True)
        subprocess.run(["killall", "-9", "bridge_node"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "gz sim"], capture_output=True)
        time.sleep(1)

    def run_all(self):
        """Run all phases and print summary."""
        all_passed = True
        try:
            source_ros()
            self.launch_background()
            self.start_agent()

            # Phase A
            if not self.run_phase_A_single_move():
                print("[Phase A] FAILED - but continuing")

            # Phase B
            if not self.run_phase_B_single_route():
                print("[Phase B] FAILED - but continuing")

            # Phase C
            if not self.run_phase_C_dual_different():
                print("[Phase C] FAILED - but continuing")

            # Phase D
            if not self.run_phase_D_dual_conflict():
                print("[Phase D] FAILED - but continuing")

            # Phase E
            self.run_phase_E_patrol()

        except Exception as e:
            print(f"\n[FATAL] Pipeline error: {e}")
            import traceback
            traceback.print_exc()
            all_passed = False
        finally:
            self.cleanup()
            self.print_summary()

    def print_summary(self):
        """Print final results."""
        print("\n" + "="*60)
        print("  TEST PIPELINE SUMMARY")
        print("="*60)
        all_ok = True
        for phase, result in self.phase_results.items():
            status = "✅ PASS" if result["success"] else "❌ FAIL"
            if not result["success"]:
                all_ok = False
            print(f"  {phase}: {status} (ok={result['completed']}, fail={result['failed']}, skip={result['skipped']})")
        print("="*60)
        if all_ok:
            print("  ALL PHASES PASSED!")
        else:
            print("  SOME PHASES FAILED - see details above")
        print("="*60)


if __name__ == "__main__":
    pipeline = TestPipeline()
    pipeline.run_all()
