from PyQt5.QtGui import QGuiApplication

class WindowArranger:
    def __init__(self):
        screen = QGuiApplication.primaryScreen()
        size = screen.size()

        width = size.width()
        height = size.height()

        self.start_x = 0  #모니터 시작점
        self.monitor_w = width
        self.monitor_h = height
        
        self.current_x = self.start_x
        self.current_y = 0
        self.max_row_h = 0

    def get_next_geometry(self, width, height):
        if self.current_x + width > self.start_x + self.monitor_w:
            self.current_x = self.start_x
            self.current_y += self.max_row_h
            self.max_row_h = 0
            

        if self.current_y + height > self.monitor_h:
            self.current_y = 0

        geometry_str = f"{width}x{height}+{self.current_x}+{self.current_y}"

        self.current_x += width
        self.max_row_h = max(self.max_row_h, height)
        
        return geometry_str