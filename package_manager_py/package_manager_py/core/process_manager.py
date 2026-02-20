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
class ProcessManager(QObject):
    process_started = pyqtSignal(str)  # package_name
    process_stopped = pyqtSignal(str)  # package_name
    process_error = pyqtSignal(str, str)  # package_name, error_message
    log_received = pyqtSignal(str, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes: Dict[str, QProcess] = {}
        self.arranger = WindowArranger()
        self.trace_timer = QTimer(self)
        self.trace_timer.timeout.connect(self._trace_processes)
        self.trace_timer.start(1000)
    
    # def start_process(self, package_name: str, command: str, arguments: list) -> Tuple[bool, str]:
    #     if package_name in self.processes:
    #         return False, f"Package '{package_name}' is already running"

    #     # 초기값 설정
    #     geo_str = "Default"
    #     run_args = list(arguments)
    #     pid_file = f"/tmp/{package_name}.pid"

    #     cmd_str = " ".join([command] + arguments)
    #     wrapped_cmd = f"echo $$ > {pid_file}; exec {cmd_str}"

    
    #     #GUI 설정 처리
    #     pkg_set = PACKAGE_GUI_SETTINGS.get(package_name)
    
    #     if pkg_set:
    #         w = pkg_set.get("width", 800)
    #         h = pkg_set.get("height", 600)
        
    #         geo_str = self.arranger.get_next_geometry(w, h)
    #         run_args.extend(["-geometry", geo_str])
    #         status_msg = f"Started {package_name} at {geo_str}"
    #     else:
    #         status_msg = f"Started {package_name} (No GUI)"

    #     #QProcess 설정
    #     process = QProcess(self)
    #     process.finished.connect(lambda code, status: self._on_finished(code, status, package_name))
    #     process.errorOccurred.connect(lambda err: self._on_error(err, package_name))
    #     process.started.connect(lambda: self._on_started(package_name))

    #     process.start(
    #         "terminator",
    #         [
    #             "--new-tab",
    #             f"--command={wrapped_cmd}"
    #         ]
    #     )
    
    #     self.processes[package_name] = {
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
        
    def _trace_processes(self):
        for package_name, info in list(self.processes.items()):
            pid_file = info.get("pid_file")

            if not pid_file or not os.path.exists(pid_file):
                continue

            try:
                with open(pid_file, "r") as f:
                    pid = int(f.read().strip())
            except Exception:
                continue

            if not self._is_pid_alive(pid):
                self._handle_real_exit(package_name)

    def _handle_real_exit(self, package_name: str):
        info = self.processes.pop(package_name, None)

        if info:
            pid_file = info.get("pid_file")
            if pid_file and os.path.exists(pid_file):
                os.remove(pid_file)

        self.process_stopped.emit(package_name)


    def start_process(self, package_name: str, command: str, arguments: list):

        if package_name in self.processes:
            return False, f"{package_name} already running"
        
        pkg_set = PACKAGE_GUI_SETTINGS.get(package_name)
        if pkg_set:
            w = pkg_set.get("width", 800)
            h = pkg_set.get("height", 600)

            pos = self.arranger.get_next_position(w, h)

            x = pos["x"]
            y = pos["y"]

            geometry = f"{w}x{h}+{x}+{y}"

            arguments = list(arguments) + [f"--pm-geometry={geometry}"]

            status_msg = f"Started {package_name} at {geometry}"
        else:
            status_msg = f"Started {package_name} (No GUI)"

        pid_file = f"/tmp/{package_name}.pid"
        cmd_str = " ".join([command] + arguments)
        wrapped_cmd = f"""
        source /opt/ros/humble/setup.bash
        source ~/colcon_ws/install/setup.bash
        echo $$ > {pid_file}
        exec {cmd_str}
        """
        process = QProcess(self)

        #process.finished.connect(lambda code, status: self._on_finished(code, status, package_name))
        process.errorOccurred.connect(lambda err: self._on_error(err, package_name))
        process.started.connect(lambda: self._on_started(package_name))

        process.start(
            "terminator",
            [
                "--new-tab",
                "--command",
                f"bash -lc '{wrapped_cmd}'"
            ] 
        )


        self.processes[package_name] = {
            "QProcess": process,
            "pid_file": pid_file
        }

        return True, f"{status_msg}"

    
    # def _handle_stdout(self, package_name):
    #     process = self.processes.get(package_name)
    #     if process:
    #         data = process.readAllStandardOutput().data().decode('utf-8').strip()
    #     if data:
    #         self.log_received.emit(package_name, data)

    # def _handle_stderr(self, package_name):
    #     process = self.processes.get(package_name)
    #     if process:
    #         data = process.readAllStandardError().data().decode('utf-8').strip()
    #     if data:
    #         self.log_received.emit(package_name, f"ERROR: {data}")

    def stop_process(self, package_name: str) -> Tuple[bool, str]:
        if package_name not in self.processes:
            return False, f"Package '{package_name}' is not running"
        
        process = self.processes[package_name]
        info = self.processes[package_name]
        pid_file = info.get("pid_file")

        try:
            with open(pid_file, "r") as f:
                pid = int(f.read().strip())
            os.kill(-pid, signal.SIGINT)
            if not process.waitForFinished(PROCESS_STOP_TIMEOUT):
                os.kill(-pid, signal.SIGKILL)
            os.remove(pid_file)
            del self.processes[package_name]

            if package_name == "web_controller_bridge" or package_name == "udpcom":
                #web_controller_bridge등등 프로세스 종료후에도 백그라운드에 남아있는 경우가 있어서 강제 종료
                subprocess.run(["pkill", "-9", "-f", "sender"])
                subprocess.run(["pkill", "-9", "-f", "receiver"])
            return True, f"Package '{package_name}' stopped"
            
        except Exception as e:
            return False, f"Error stopping package '{package_name}': {str(e)}"
    
    def is_running(self, package_name: str) -> bool:
        return package_name in self.processes
    
    def get_running_packages(self) -> list:
        return list(self.processes.keys())
    
    def stop_all(self):
        package_names = list(self.processes.keys())
        for package_name in package_names:
            self.stop_process(package_name) 
    # ========== Private Signal Handlers ==========
    
    def _on_finished(self, exit_code: int, exit_status: QProcess.ExitStatus, package_name: str):
        if package_name in self.processes:
            del self.processes[package_name]
        
        self.process_stopped.emit(package_name)

        if exit_code == 0 and exit_status == QProcess.NormalExit:
            msg = f"Process stopped normally (exit code: 0)"
        elif exit_status == QProcess.CrashExit:
            msg = f"Process crashed (exit code: {exit_code})"
            self.process_error.emit(package_name, msg)
        else:
            msg = f"Process stopped with error (exit code: {exit_code})"
            self.process_error.emit(package_name, msg)
    
    def _on_error(self, error: QProcess.ProcessError, package_name: str):
        error_messages = {
            QProcess.FailedToStart: "Failed to start - command not found or insufficient permissions",
            QProcess.Crashed: "Process crashed",
            QProcess.Timedout: "Process timed out",
            QProcess.WriteError: "Write error",
            QProcess.ReadError: "Read error",
            QProcess.UnknownError: "Unknown error",
        }
        
        error_msg = error_messages.get(error, "Unknown error occurred")
        self.process_error.emit(package_name, error_msg)
        
        if error == QProcess.FailedToStart and package_name in self.processes:
            del self.processes[package_name]
            self.process_stopped.emit(package_name)
    
    def _on_started(self, package_name: str):
        self.process_started.emit(package_name)
