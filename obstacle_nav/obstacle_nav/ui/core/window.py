from PyQt5.QtWidgets import QMainWindow

from ..designer.ui_main_window import Ui_MainWindow
from ..controllers.slot_controller import SlotController
from ..controllers.param_controller import ParamController
from ..controllers.state_controller import StateController


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ros_node = ros_node

        self.param_controller = ParamController(self.ui)
        self.state_controller = StateController(self.ui, self.ros_node)
        self.slot_controller = SlotController(
            window=self,
            ui=self.ui,
            node=self.ros_node,
            param_controller=self.param_controller,
        )