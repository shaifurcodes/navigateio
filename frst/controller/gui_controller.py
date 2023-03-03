from  controller import FRSTController
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QVBoxLayout, QApplication
from PyQt5.QtGui import  QFont
from PyQt5 import QtCore
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import time
import sys

class GUI_Controller(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # READ from file
        self.gui_origin_node = 2
        self.gui_x_axis_node = 4
        self.gui_mobile_node = 6
        self.gui_y_dir = 1

        self.gui_node_list = [self.gui_origin_node,
                              self.gui_x_axis_node,
                              self.gui_mobile_node
                              ]
        self.frst_controller = FRSTController(
                              self.gui_origin_node,
                              self.gui_x_axis_node,
                              self.gui_mobile_node,
                              self.gui_y_dir)
        self.canvas = None
        self.fig = None
        self.axes = None
        self.gui_node_list = {}
        self.mobile_node_z_label = "  "

        self.show()
        self.setupGUI()
        return

    def setupGUI(self):
        self.showMaximized()
        self.setWindowTitle("NavigateIO Desktop App")

        #-------------set main window--------#
        self.layout = QGridLayout()
        self.setLayout(self.layout)

        # ------------- set title------------------------------------#
        cur_font = QFont('Arial', 30, QFont.Bold)
        cur_label = QLabel("NavigateIO")
        cur_label.setStyleSheet("QLabel {color: white; background-color: blue;}")
        cur_label.setAlignment(QtCore.Qt.AlignCenter)
        cur_label.setFont(cur_font)
        title_layout = QVBoxLayout()
        title_layout.addWidget(cur_label)
        title_layout.addStretch()

        self._initialize_floormap_gui()
        #----------------setup center map window-------------------------#
        self.layout.addLayout(title_layout,     0, 0,  2, 30 )
        self.layout.addWidget(self.canvas,      2, 0,  15, 30)
        self.repaint()
        return

    def _initialize_floormap_gui(self):
        #---------------floor map panel-----------#
        self.fig, self.axes = plt.subplots()
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        # self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        # self.canvas.setFocus()
        cur_axis = self.axes
        cur_axis.cla()
        cur_axis.set_xlim(-50, 50)
        cur_axis.set_ylim(0, 100)
        cur_axis.margins(0)
        cur_axis.set_xticks([], [])
        cur_axis.set_yticks([], [])

        for n1_indx, n1 in enumerate(self.frst_controller.node_list):
            backgroundcolor = 'red'
            if n1==self.frst_controller.mobile_node:
                backgroundcolor='blue'
            self.gui_node_list[n1] =cur_axis.text(-1000., -1000.,
                          str(n1),
                          fontsize=20,
                          fontweight='bold',
                          color='w',
                          bbox=dict(boxstyle='square', fc=backgroundcolor, ec='none') )

        #mobile_node_z  = self.frst_controller.get_z_of_mobile_node()
        self.mobile_node_z_label = cur_axis.text(-50, 50,  #TODO: bring within frame
                                               str(self.mobile_node_z_label), fontsize=20, fontweight='bold', color='red',
                                               bbox=dict(boxstyle='square', fc="cyan", ec='none'))

        self.canvas.draw()
        return

    def run_controller(self):
        while True:
            self.frst_controller.run_controller_single_iteration()
            for n1_indx, n1 in enumerate(self.frst_controller.node_list):
                x, y = self.frst_controller.x[n1_indx], self.frst_controller.y[n1_indx]
                self.gui_node_list[n1].set_position((x, y))
                if n1 == self.frst_controller.mobile_node:
                    mobile_node_z = self.frst_controller.get_z_of_mobile_node()
                    self.mobile_node_z_label.set_text(str(mobile_node_z))
            self.canvas.draw_idle()
            time.sleep(0.5)
        return

if __name__ == '__main__':
    if __name__ == '__main__':
        app = QApplication(sys.argv)
        trackio = GUI_Controller()
        app.exec_()