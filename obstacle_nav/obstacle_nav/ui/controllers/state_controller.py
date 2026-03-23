from PyQt5.QtCore import QTimer


class StateController:
    def __init__(self, ui, node):
        self.ui = ui
        self.node = node

        self.timer = QTimer()