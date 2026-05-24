import math
import sys

from rplidar import RPLidar

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from serial.tools import list_ports

for p in list_ports.comports():
    print(p.device, p.description)

PORT_NAME = "/dev/ttyUSB0"

lidar = RPLidar(PORT_NAME)

# Qt app
app = QtWidgets.QApplication(sys.argv)

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("RPLidar A1")

plot = win.addPlot()
plot.setAspectLocked(True)

# Scatter plot item
scatter = pg.ScatterPlotItem(size=3)
plot.addItem(scatter)

plot.setXRange(-4000, 4000)
plot.setYRange(-4000, 4000)

scan_iterator = lidar.iter_scans()


def update():
    scan = next(scan_iterator)

    spots = []

    for quality, angle, distance in scan:

        # Ignore invalid readings
        if distance <= 0:
            continue

        rad = math.radians(angle)

        x = distance * math.cos(rad)
        y = distance * math.sin(rad)

        spots.append({
            'pos': (x, y)
        })

    scatter.setData(spots)


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)


try:
    QtWidgets.QApplication.instance().exec()

finally:
    lidar.stop()
    lidar.disconnect()