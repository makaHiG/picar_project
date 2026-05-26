
import sys
import numpy as np
from multiprocessing import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class LidarWindow(pg.GraphicsLayoutWidget):

    def __init__(self, que):
        super().__init__()

        self.que = que

        self.setWindowTitle("LiDAR Monitor")
        self.resize(800, 800)

        self.plot = self.addPlot()

        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)

        self.plot.setXRange(-300, 300)
        self.plot.setYRange(-300, 300)

        # Scatter plot item
        self.scatter = pg.ScatterPlotItem(size=5)
        self.plot.addItem(self.scatter)

        # Robot body circle
        robot_radius = 24

        circle = QtWidgets.QGraphicsEllipseItem(
            -robot_radius,
            -robot_radius,
            robot_radius * 2,
            robot_radius * 2
        )

        self.plot.addItem(circle)

        # Update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(30)

    def update_plot(self):

        latest = None

        while not self.que.empty():
            latest = self.que.get_nowait()

        if latest is None:
            return

        spots = []

        for point in latest:

            x, y = point['pos']

            spots.append({
                'pos': (x, y)
            })

        self.scatter.setData(spots)


if __name__ == '__main__':

    q = Queue()

    # Fake data generator for testing
    import math
    import random

    def fake_scan():

        points = []

        for angle in range(360):

            r = 150 + random.uniform(-10, 10)

            x = r * math.cos(math.radians(angle))
            y = r * math.sin(math.radians(angle))

            points.append({
                'pos': (x, y)
            })

        return points

    app = QtWidgets.QApplication(sys.argv)

    win = LidarWindow(q)
    win.show()

    def push_fake_data():
        q.put(fake_scan())

    fake_timer = QtCore.QTimer()
    fake_timer.timeout.connect(push_fake_data)
    fake_timer.start(100)

    sys.exit(app.exec_())
