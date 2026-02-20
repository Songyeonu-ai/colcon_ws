from PyQt5.QtGui import QGuiApplication

class WindowArranger:
    def __init__(self):
        screen = QGuiApplication.primaryScreen()
        size = screen.size()

        self.monitor_w = size.width()
        self.monitor_h = size.height()

        self.start_x = 0
        self.start_y = 0

        self.current_x = self.start_x
        self.current_y = self.start_y
        self.max_row_h = 0

    def get_next_position(self, width, height):
        # 다음 줄
        if self.current_x + width > self.start_x + self.monitor_w:
            self.current_x = self.start_x
            self.current_y += self.max_row_h
            self.max_row_h = 0

        if self.current_y + height > self.monitor_h:
            self.current_y = self.start_y

        x = self.current_x
        y = self.current_y

        # 다음 창 위치 계산
        self.current_x += width
        self.max_row_h = max(self.max_row_h, height)

        return {
            "x": x,
            "y": y,
            "w": width,
            "h": height
        }

    def reset(self):
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.max_row_h = 0
