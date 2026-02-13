"""
Process Manager - Handles QProcess lifecycle
"""
#여기에서는 process 관련 함수만. 실행은 ros_node랑 main_window에서

from PyQt5.QtCore import QProcess, QObject, pyqtSignal
from typing import Dict, Tuple, Optional
from ..package_settings.settings import PROCESS_STOP_TIMEOUT, PROCESS_KILL_TIMEOUT
import os, signal, subprocess
from ..utils.window_arrange import WindowArranger
from ..package_settings.settings import PACKAGE_GUI_SETTINGS
class ProcessManager(QObject):
    """
    Manages QProcess instances for ROS2 packages
    
    Signals:
        process_started(str): Emitted when a process starts successfully
        process_stopped(str): Emitted when a process stops
        process_error(str, str): Emitted on error (package_name, error_message)
    """
    
    process_started = pyqtSignal(str)  # package_name
    process_stopped = pyqtSignal(str)  # package_name
    process_error = pyqtSignal(str, str)  # package_name, error_message
    log_received = pyqtSignal(str, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes: Dict[str, QProcess] = {}
        self.arranger = WindowArranger()
    
    def start_process(self, package_name: str, command: str, arguments: list) -> Tuple[bool, str]:
        if package_name in self.processes:
            return False, f"Package '{package_name}' is already running"

        # 초기값 설정
        geo_str = "Default"
        run_args = list(arguments)
    
        #GUI 설정 처리
        pkg_set = PACKAGE_GUI_SETTINGS.get(package_name)
    
        if pkg_set:
            w = pkg_set.get("width", 800)
            h = pkg_set.get("height", 600)
        
            geo_str = self.arranger.get_next_geometry(w, h)
            run_args.extend(["-geometry", geo_str])
            status_msg = f"Started {package_name} at {geo_str}"
        else:
            status_msg = f"Started {package_name} (No GUI)"

        #QProcess 설정
        process = QProcess(self)
        actual_command = "setsid"
        actual_args = [command] + run_args
    
        # 시그널 연결
        process.finished.connect(lambda code, status: self._on_finished(code, status, package_name))
        process.errorOccurred.connect(lambda err: self._on_error(err, package_name))
        process.started.connect(lambda: self._on_started(package_name))
    
        # 프로세스 시작
        process.start(actual_command, actual_args)
    
        self.processes[package_name] = process
    
        return True, status_msg
    
    def stop_process(self, package_name: str) -> Tuple[bool, str]:
        """
        Stop a running process
        
        Args:
            package_name: Package identifier
            
        Returns:
            (success, message): Tuple indicating success and status message
        """
        if package_name not in self.processes:
            return False, f"Package '{package_name}' is not running"
        
        process = self.processes[package_name]
        pid = process.processId()

        try:
            #QProcess 종료(systemd 서비스 x)
            os.killpg(os.getpgid(pid), signal.SIGINT)
            
            if not process.waitForFinished(PROCESS_STOP_TIMEOUT):
                os.killpg(os.getpgid(pid), signal.SIGTERM)

                if package_name in self.processes:
                    del self.processes[package_name]
            return True, f"Package '{package_name}' stopped"
            
            
        except Exception as e:
            return False, f"Error stopping package '{package_name}': {str(e)}"
    
    def is_running(self, package_name: str) -> bool:
        """Check if a package process is running"""
        return package_name in self.processes
    
    def get_running_packages(self) -> list:
        """Get list of currently running package names"""
        return list(self.processes.keys())
    
    def stop_all(self):
        """Stop all running processes"""
        package_names = list(self.processes.keys())
        for package_name in package_names:
            self.stop_process(package_name) #stop은 process manager에서 단독으로 처리 가능함 --> import 꼬이지 않음
            #근데 start하는거는 ros_node에서 처리해야함 --> 그걸 process manager에서 처리하면 import 꼬임 파일이 용도에 맞지가 않음...  
    

    # ========== Private Signal Handlers ==========
    
    def _on_finished(self, exit_code: int, exit_status: QProcess.ExitStatus, package_name: str):
        """Handle process finished signal"""
        # Remove from tracking
        if package_name in self.processes:
            del self.processes[package_name]
        
        # Emit stopped signal
        self.process_stopped.emit(package_name)
        
        # Log exit status
        if exit_code == 0 and exit_status == QProcess.NormalExit:
            msg = f"Process stopped normally (exit code: 0)"
        elif exit_status == QProcess.CrashExit:
            msg = f"Process crashed (exit code: {exit_code})"
            self.process_error.emit(package_name, msg)
        else:
            msg = f"Process stopped with error (exit code: {exit_code})"
            self.process_error.emit(package_name, msg)
    
    def _on_error(self, error: QProcess.ProcessError, package_name: str):
        """Handle process error signal"""
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
        
        # Remove from tracking if it failed to start
        if error == QProcess.FailedToStart and package_name in self.processes:
            del self.processes[package_name]
            self.process_stopped.emit(package_name)
    
    def _on_started(self, package_name: str):
        """Handle process started signal"""
        self.process_started.emit(package_name)
