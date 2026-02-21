from enum import IntEnum
from dataclasses import dataclass
from typing import Dict, Callable


class PackageState(IntEnum):
    STOPPED = 0
    STARTING = 1
    RUNNING = 2
    STOPPING = 3
    ERROR = 4


@dataclass
class RuntimePackage:
    id: int
    state: PackageState = PackageState.STOPPED
    last_error: str = ""


class RuntimeStateManager:

    def __init__(self):
        self.packages: Dict[int, RuntimePackage] = {}

    def register(self, package_id: int):
        if package_id not in self.packages:
            self.packages[package_id] = RuntimePackage(package_id)

    def can_start(self, package_id: int) -> bool:
        return self.packages[package_id].state in (
            PackageState.STOPPED,
            PackageState.ERROR
        )

    def can_stop(self, package_id: int) -> bool:
        return self.packages[package_id].state == PackageState.RUNNING

    def set_starting(self, package_id: int):
        self.packages[package_id].state = PackageState.STARTING

    def set_running(self, package_id: int):
        self.packages[package_id].state = PackageState.RUNNING

    def set_stopping(self, package_id: int):
        self.packages[package_id].state = PackageState.STOPPING

    def set_stopped(self, package_id: int):
        self.packages[package_id].state = PackageState.STOPPED

    def set_error(self, package_id: int, msg: str):
        pkg = self.packages[package_id]
        pkg.state = PackageState.ERROR
        pkg.last_error = msg

    def get_state(self, package_id: int) -> PackageState:
        return self.packages[package_id].state

    def get_all_states(self):
        return {pid: pkg.state for pid, pkg in self.packages.items()}