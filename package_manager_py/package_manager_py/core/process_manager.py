#여기에서는 process 관련 함수만. 실행은 ros_node랑 main_window에서

from logging import info
from multiprocessing import process
from turtle import pos

from PyQt5.QtCore import QProcess, QObject, pyqtSignal, QProcessEnvironment, QTimer
from typing import Dict, Tuple, Optional
import os

from click import command
from ..package_settings.settings import PROCESS_STOP_TIMEOUT, PROCESS_KILL_TIMEOUT
import os, signal, subprocess
from ..utils.window_arrange import WindowArranger
from ..package_settings.settings import PACKAGE_GUI_SETTINGS
from ..core.runtime_state_manager import PackageState
class ProcessManager(QObject):
    process_started = pyqtSignal(int)
    process_stopped = pyqtSignal(int)
    process_error = pyqtSignal(int, str)
    log_received = pyqtSignal(str, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes: Dict[int, Dict]
        self.processes = {}
        self.arranger = WindowArranger()
        self.trace_timer = QTimer(self)
        self.trace_timer.timeout.connect(self._trace_processes)
        self.trace_timer.start(1000)

    def get_runtime_states(self) -> Dict[int, str]:
        result = {}

        for package_id, info in self.processes.items():
            state: PackageState = info.get("state", PackageState.STOPPED)

            if state == PackageState.STARTING:
                result[package_id] = "STARTING"
            elif state == PackageState.RUNNING:
                result[package_id] = "RUNNING"
            elif state == PackageState.STOPPING:
                result[package_id] = "STOPPING"
            else:
                result[package_id] = "STOPPED"

        return result
    
    # def start_process(self, package_id: str, command: str, arguments: list) -> Tuple[bool, str]:
    #     if package_id in self.processes:
    #         return False, f"Package '{package_id}' is already running"

    #     # 초기값 설정
    #     geo_str = "Default"
    #     run_args = list(arguments)
    #     pid_file = f"/tmp/{package_id}.pid"

    #     cmd_str = " ".join([command] + arguments)
    #     wrapped_cmd = f"echo $$ > {pid_file}; exec {cmd_str}"

    
    #     #GUI 설정 처리
    #     pkg_set = PACKAGE_GUI_SETTINGS.get(package_id)
    
    #     if pkg_set:
    #         w = pkg_set.get("width", 800)
    #         h = pkg_set.get("height", 600)
        
    #         geo_str = self.arranger.get_next_geometry(w, h)
    #         run_args.extend(["-geometry", geo_str])
    #         status_msg = f"Started {package_id} at {geo_str}"
    #     else:
    #         status_msg = f"Started {package_id} (No GUI)"

    #     #QProcess 설정
    #     process = QProcess(self)
    #     process.finished.connect(lambda code, status: self._on_finished(code, status, package_id))
    #     process.errorOccurred.connect(lambda err: self._on_error(err, package_id))
    #     process.started.connect(lambda: self._on_started(package_id))

    #     process.start(
    #         "terminator",
    #         [
    #             "--new-tab",
    #             f"--command={wrapped_cmd}"
    #         ]
    #     )
    
    #     self.processes[package_id] = {
    #         "QProcess": process,
    #         "pid_file": pid_file
    #     }
    
    #     return True, status_msg


    def _is_pid_alive(self, pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except OSError:
            return False
        else:
            return True
        
    def check_process_state(self, package_id: int) -> str:
        pid_file = f"/tmp/{package_id}.pid"

        if not os.path.exists(pid_file):
            return "STOPPED"

        try:
            with open(pid_file, "r") as f:
                pid = int(f.read().strip())
        except:
            return "STOPPED"

        if self._is_pid_alive(pid):
            return "RUNNING"
        else:
            os.remove(pid_file)  # 죽은파일정리
            return "CRASHED"
        
    def _trace_processes(self):
        for package_id, info in list(self.processes.items()):
            state = info["state"]
            pid_file = info.get("pid_file")

            #STARTING
            if state == PackageState.STARTING:
                if not pid_file or not os.path.exists(pid_file):
                    continue

                try:
                    with open(pid_file, "r") as f:
                        pid = int(f.read().strip())
                        info["pid"] = pid
                except Exception:
                    continue

                if self._is_pid_alive(pid):
                    info["state"] = PackageState.RUNNING
                    self.process_started.emit(package_id)

            #RUNNING
            elif state == PackageState.RUNNING:
                pid = info.get("pid")
                if not pid or not self._is_pid_alive(pid):
                    info["state"] = PackageState.STOPPED
                    self._handle_real_exit(package_id)

            #STOPPING
            elif state == PackageState.STOPPING:
                pid = info.get("pid")

            if not pid or not self._is_pid_alive(pid):
                info["state"] = PackageState.STOPPED
                self._handle_real_exit(package_id)


    def _handle_real_exit(self, package_id: int):
        info = self.processes.pop(package_id, None)

        if info:
            pid_file = info.get("pid_file")
            if pid_file and os.path.exists(pid_file):
                os.remove(pid_file)

        self.process_stopped.emit(package_id)


    def start_process(self, package_id: int, command: str, arguments: list):

        if package_id in self.processes:
            return False, f"{package_id} already running"
        
        pkg_set = PACKAGE_GUI_SETTINGS.get(package_id)
        if pkg_set:
            w = pkg_set.get("width", 800)
            h = pkg_set.get("height", 600)

            pos = self.arranger.get_next_position(w, h)

            x = pos["x"]
            y = pos["y"]

            geometry = f"{w}x{h}+{x}+{y}"

            arguments = list(arguments) + [f"--pm-geometry={geometry}"]

            status_msg = f"Starting {package_id} at {geometry}"
        else:
            status_msg = f"Starting {package_id} (No GUI)"

        import shlex
        pid_file = f"/tmp/{package_id}.pid"

        # 안전한 쉘 문자열 만들기 (공백/따옴표 포함 인자 깨짐 방지)
        cmd_exec = " ".join([shlex.quote(command)] + [shlex.quote(a) for a in arguments])

        cmd_chain = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/colcon_ws/install/setup.bash && "
            f"echo $$ > {shlex.quote(pid_file)} && "
            f"exec {cmd_exec}"
        )

        process = QProcess(self)
        process.errorOccurred.connect(lambda err: self._on_error(err, package_id))

        # terminator의 --command는 "한 개의 문자열"을 기대하는 경우가 많아서,
        # bash -lc 전체를 하나의 인자로 넘김
        process.start(
            "terminator",
            [
                "--new-tab",
                "--command",
                f"bash -lc {shlex.quote(cmd_chain)}"
            ]
        )

        self.processes[package_id] = {
            "QProcess": process,
            "pid_file": pid_file,
            "state": PackageState.STARTING,
            "pid": None
        }

        return True, status_msg

    
    # def _handle_stdout(self, package_id):
    #     process = self.processes.get(package_id)
    #     if process:
    #         data = process.readAllStandardOutput().data().decode('utf-8').strip()
    #     if data:
    #         self.log_received.emit(package_id, data)

    # def _handle_stderr(self, package_id):
    #     process = self.processes.get(package_id)
    #     if process:
    #         data = process.readAllStandardError().data().decode('utf-8').strip()
    #     if data:
    #         self.log_received.emit(package_id, f"ERROR: {data}")

    def stop_process(self, package_id: int) -> Tuple[bool, str]:
        if package_id not in self.processes:
            return False, f"Package '{package_id}' is not running"
        
        info = self.processes[package_id]
        qt_process: QProcess = info.get("QProcess")
        pid_file = info.get("pid_file")

        try:
            with open(pid_file, "r") as f:
                pid = int(f.read().strip())
            info["state"] = PackageState.STOPPING
            os.killpg(pid, signal.SIGINT)
            if qt_process and not qt_process.waitForFinished(PROCESS_STOP_TIMEOUT):
                os.killpg(pid, signal.SIGKILL)
            os.remove(pid_file)
            del self.processes[package_id]

            if package_id == 5 or package_id == 6:
                #web_controller_bridge등등 프로세스 종료후에도 백그라운드에 남아있는 경우가 있어서 강제 종료
                subprocess.run(["pkill", "-9", "-f", "sender"])
                subprocess.run(["pkill", "-9", "-f", "receiver"])

            if os.path.exists(pid_file):
                os.remove(pid_file)

            self._on_stopped(package_id)
            return True, f"Package '{package_id}' stopped"
            
        except Exception as e:
            return False, f"Error stopping package '{package_id}': {str(e)}"
    
    def is_running(self, package_id: int) -> bool:
        return package_id in self.processes
    
    def get_running_packages(self) -> list:
        return list(self.processes.keys())
    
    def stop_all(self):
        package_ids = list(self.processes.keys())
        for package_id in package_ids:
            self.stop_process(package_id) 
    # ========== Private Signal Handlers ==========
    
    def _on_finished(self, code, status, package_id):
        if package_id in self.processes:
            del self.processes[package_id]

        self.process_stopped.emit(package_id)

        if code == 0 and status == QProcess.NormalExit:
            msg = f"Process stopped normally (exit code: 0)"
        elif status == QProcess.CrashExit:
            msg = f"Process crashed (exit code: {code})"
            self.process_error.emit(package_id, msg)
        else:
            msg = f"Process stopped with error (exit code: {code})"
            self.process_error.emit(package_id, msg)
    
    def _on_error(self, error: QProcess.ProcessError, package_id: int):
        error_messages = {
            QProcess.FailedToStart: "Failed to start - command not found or insufficient permissions",
            QProcess.Crashed: "Process crashed",
            QProcess.Timedout: "Process timed out",
            QProcess.WriteError: "Write error",
            QProcess.ReadError: "Read error",
            QProcess.UnknownError: "Unknown error",
        }
        
        error_msg = error_messages.get(error, "Unknown error occurred")
        self.process_error.emit(package_id, error_msg)
        
        if error == QProcess.FailedToStart and package_id in self.processes:
            del self.processes[package_id]
            self.process_stopped.emit(package_id)
    
    # def _on_started(self, package_id: int):
    #     self.process_started.emit(package_id)

    def _on_stopped(self, package_id: int):
        self.process_stopped.emit(package_id)