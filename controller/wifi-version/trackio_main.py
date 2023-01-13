
from PyQt5.QtWidgets import  QGridLayout, QHBoxLayout, QVBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QPushButton, QComboBox, QInputDialog, QMessageBox
from PyQt5.QtGui import  QFont
from PyQt5 import QtCore
import  sys
from PIL import Image
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import matplotlib.pyplot as plt
import math
import configparser


from controller import UDPSolver
import  time
import  threading


from _collections import deque

#----------the folllowing include are database related---------#
#from datetime import datetime
#import json
#from influxdb import InfluxDBClient
#from pymemcache.client import base
#---------------------------------------------------------------#

class TrackIO_Interface(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node_floor = {}

        self._parse_config_file()
        self._init_floors() #need to call after loading floor labels in _parse_config_file() method

        self.solver_thread = None
        self.viz_thread = None
        self.is_viz = False
        self.checker_thread = None

        self.viz_selected_nodes = []
        self.active_nodes = []
        self.controller = None

        #self._configure_server_connections()
        self.flip_x = [2, 3] #TODO: orientation control for individual floors
        self.flip_y = [2, 3] #TODO: orientation control for individual floors

        self.show()
        self.setup_gui()
        #self._show_bread_crumb(9)
        return

    def _init_floors(self):
        for i in range(1, 15):
            if not i in self.node_floor.keys():
                self.node_floor[i] = min(self.floor_labels)
        return

    def _parse_config_file(self):
        self.config = configparser.ConfigParser()
        self.config.read('./trackio.ini')

        #--------------network-related----------------------#
        self.udp_port = int(self.config['NETWORK']['port'])
        self.udp_timeout_msec = int(self.config['NETWORK']['udp_timeout_milliseconds'])

        #----------------ranging protocol related--------------------#
        self.twr_frame_time_msec = int(self.config['PROTOCOL']['twr_tdma_slot_milliseconds'])

        #-----------atmosphere-related-------------------------------#
        self.sea_level_pressure_inch_hg = float( self.config['ATMOSPHERE']['sea_level_pressure_inch_hg'] )
        self.floormap_folder = self.config['BUILDING']['floormap_folder']


        #-----------------floor-labels--------------------------------#
        config_val = self.config['BUILDING']['floor_labels'].split(',')
        self.floor_labels =  []
        for v in sorted(config_val):
            self.floor_labels.append(int(v))
        self.floor_count = len(self.floor_labels)

        # -----------------per-floor-pressure-change--------------------------------#
        self.floor_change_threshold_meter = float(self.config['BUILDING']['floor_change_threshold_meter'])

        self.anchor_nodes = {}
        self.fixed_node_global_loc_meter = {}
        #-----------nodes-per-floor-related-----------------------------#
        self.per_floor_pixel_per_meter = {}

        for cur_floor_indx, cur_floor in enumerate(self.floor_labels):
            config_label = 'BUILDING-FLOOR-'+str(cur_floor)
            self.per_floor_pixel_per_meter[cur_floor] = \
                float(self.config[config_label]['pixel_per_meter'])
            #---set the anchor nodes----#
            origin_node, x_axis_node = int(self.config[config_label]['origin_node']),\
                                       int(self.config[config_label]['x_axis_node'])
            self.anchor_nodes[cur_floor] = [origin_node, x_axis_node]
            self.node_floor[origin_node] = cur_floor
            self.node_floor[x_axis_node] = cur_floor

            origin_node_xy = self.config[config_label]['origin_node_offset_meter'].split(',')
            x_axis_node_xy = self.config[config_label]['x_axis_node_offset_meter'].split(',')

            self.fixed_node_global_loc_meter[origin_node] = (float(origin_node_xy[0]), \
                                                             float(origin_node_xy[1]))
            self.fixed_node_global_loc_meter[x_axis_node] = (float(x_axis_node_xy[0]), \
                                                             float(x_axis_node_xy[1]))
        self.fixed_nodes = {}
        if 'FIXED-NODES' in self.config.sections():
            for cur_node_str in self.config['FIXED-NODES'].keys():
                cur_node = int(cur_node_str)
                xyz_str = self.config['FIXED-NODES'][cur_node_str].split(',')
                x, y, cur_floor = float(xyz_str[0]), float(xyz_str[1]), int(xyz_str[2])
                self.fixed_node_global_loc_meter[cur_node] = (x, y)
                self.node_floor[cur_node] = cur_floor


        #-----------load all node ips--------------------#
        self.node_ips = {}
        for n1 in range(1,15):
            if str(n1) in self.config['NODE-IPS'].keys():
                self.node_ips[n1] = self.config['NODE-IPS'][str(n1)].strip()

        #-----------load node labels & colors if any-----------#
        self.node_names = {}
        self._node_colors = {}
        for n1 in range(1, 15):
            if str(n1) in self.config['NODE-LABELS'].keys():
                self.node_names[n1] = self.config['NODE-LABELS'][str(n1)].strip()
            if str(n1) in self.config['NODE-COLORS'].keys():
                self._node_colors[n1] = self.config['NODE-COLORS'][str(n1)].strip()

        #------------display refresh-rate and pixel mapping related-----#
        self.display_interval = 1./float(self.config['DISPLAY']['refresh_rate_hz'])
        return

    def _show_anchor_nodes_from_config_file(self):
        for anchor_ax_indx, anchor_floor in enumerate(self.floor_labels):
            n1_axis = self.axes[anchor_ax_indx]
            anchor_floor_pix_per_m = float(self.config['BUILDING-FLOOR-' + str(anchor_floor)]['pixel_per_meter'])

            n1 = int(self.config['BUILDING-FLOOR-' + str(anchor_floor)]['origin_node'])
            n1_xy = self.config['BUILDING-FLOOR-' + str(anchor_floor)]['origin_node_offset_meter'].split(',')
            n1_x, n1_y = float(n1_xy[0]) * anchor_floor_pix_per_m, float(n1_xy[1]) * anchor_floor_pix_per_m
            n1_axis.text(n1_x, n1_y, str(n1), fontsize=20, fontweight='bold', color='white',
                         bbox=dict(boxstyle='square', fc='g', ec='none'))

            n1 = int(self.config['BUILDING-FLOOR-' + str(anchor_floor)]['x_axis_node'])
            n1_xy = self.config['BUILDING-FLOOR-' + str(anchor_floor)]['x_axis_node_offset_meter'].split(',')
            n1_x, n1_y = float(n1_xy[0]) * anchor_floor_pix_per_m, float(n1_xy[1]) * anchor_floor_pix_per_m
            n1_axis.text(n1_x, n1_y, str(n1), fontsize=20, fontweight='bold', color='white',
                         bbox=dict(boxstyle='square', fc='b', ec='none'))
        self.canvas.draw()

    def _on_map_click(self, event):
         if event.xdata is None:
             return
         ix, iy = round(float(event.xdata), 3), round(float(event.ydata), 3)
         cur_floor = 0
         for indx, cur_axis in enumerate(self.axes):
             if cur_axis.in_axes(event):
                cur_floor = self.floor_labels[indx]  # len(self.axes)-indx
                pix_per_m = float(self.config['BUILDING-FLOOR-' + str(cur_floor)]['pixel_per_meter'])

                if self.mouse_click_input_status == 'anchor':
                    if event.button == 1:
                        self.config['BUILDING-FLOOR-' + str(cur_floor)]['origin_node_offset_meter'] = \
                            str(round(1. * ix / pix_per_m, 3)) + ", " + str(round(1. * iy / pix_per_m, 3))
                    elif event.button == 3:
                        self.config['BUILDING-FLOOR-' + str(cur_floor)]['x_axis_node_offset_meter'] = \
                            str(round(1. * ix / pix_per_m, 3)) + ", " + str(round(1. * iy / pix_per_m, 3))
                    self._setup_floormaps(is_drawn=False)
                    self._show_anchor_nodes_from_config_file()
                elif self.mouse_click_input_status == 'fixed':
                    node_id, okPressed = QInputDialog.getInt(self, "Fixed Node Input", "Fixed Node ID:", 13, 0, 100, 1)
                    if okPressed :
                        self.config['FIXED-NODES'][str(node_id)] = str(round(1. * ix / pix_per_m, 3)) + \
                                                                   ", " + str(round(1. * iy / pix_per_m, 3)) + \
                                                                   ", " + str(cur_floor)
                        self._setup_floormaps(is_drawn=False)
                        self._show_anchor_nodes_from_config_file()
                        for n1_str in self.config['FIXED-NODES'].keys():
                            xyz_str = self.config['FIXED-NODES'][n1_str].split(',')
                            n1_floor = int(xyz_str[2])
                            n1_floor_indx = self.floor_labels.index(n1_floor)
                            n1_axis = self.axes[n1_floor_indx]
                            n1_floor_pix_m = \
                                float(self.config['BUILDING-FLOOR-' + str(n1_floor)]['pixel_per_meter'])
                            n1_x, n1_y = float(xyz_str[0])*n1_floor_pix_m, float(xyz_str[1])*n1_floor_pix_m
                            n1_axis.text(n1_x, n1_y,
                                          n1_str, fontsize=20, fontweight='bold', color='w',
                                          bbox=dict(boxstyle='square', fc='black', ec='none'))
                        self.canvas.draw()
                elif self.mouse_click_input_status == 'scaling':
                    if self.map_scaling_init_pixel is None:
                        self.map_scaling_init_pixel = [ix, iy]
                        cur_axis.scatter([ix], [iy],s=100, c='b')
                        self.canvas.draw()
                    else:
                        x1, y1 = self.map_scaling_init_pixel[0], self.map_scaling_init_pixel[1]
                        d_pix = math.sqrt((ix- x1)**2. + (iy - y1)**2.)
                        cur_axis.scatter([ix], [iy], s=100, c='b')
                        cur_axis.scatter([x1], [y1], s=100, c='b')
                        cur_axis.plot([ix, x1], [iy, y1], linewidth=4, c = 'b')
                        self.canvas.draw()
                        d_m, okPressed = \
                            QInputDialog.getDouble(self, "Distance Input", "Distance between the points in meters:", 1., 0, 100, 2)
                        self.config['BUILDING-FLOOR-' + str(cur_floor)]['pixel_per_meter'] = \
                            str(round(1.*d_pix/d_m ,3))
                        self.map_scaling_init_pixel = None
                break
         return

    def _setup_floormaps(self, is_drawn = True):
        #self.fig.clf()
        for cur_floor_indx, cur_floor in enumerate(self.floor_labels):
            cur_axis = self.axes[cur_floor_indx]
            cur_axis.cla()
            x_extent, y_extent = self.img[cur_floor_indx].size
            cur_axis.set_xlim(0, x_extent)
            cur_axis.set_ylim(0, y_extent)
            cur_axis.margins(0)
            #plt.rcParams.update({'axes.titlesize':'x-large'})
            cur_axis.set_title("Floor: " + str(self.floor_labels[cur_floor_indx]), fontsize=24, fontweight = 'bold', color='blue')
            cur_axis.imshow(self.img[cur_floor_indx], extent=[0, x_extent, 0, y_extent], cmap='gray')
            cur_axis.set_xticks([], [])
            cur_axis.set_yticks([], [])
        if is_drawn:
            self.canvas.draw()
        return

    def _initialize_floormap_gui(self):
        self.mpl_cid = None
        #---------------floor map panel-----------#
        self.fig, self.axes = plt.subplots(1, self.floor_count)
            #TODO: above, swap 1 and self.floor_count to switch horiontal/vertical map placement
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()
        self.fig.canvas.mpl_connect('button_press_event', self._on_map_click)

        self.img = []
        for cur_floor_label in self.floor_labels:
            self.img.append( Image.open(self.floormap_folder+"/floor-"+str(cur_floor_label)+".jpg"))
        self._setup_floormaps(is_drawn=True)
        return

    def _start_anchor_input(self):
        self.mouse_click_input_status = 'anchor'
        self.status_label.setText("Status: Anchor Input Started")
        self.repaint()
        return

    def _stop_anchor_input(self):
        self.mouse_click_input_status = ''
        self._save_config_file()
        self.status_label.setText("Status: Anchor Input Saved")
        self._setup_floormaps(is_drawn=True)
        self.repaint()
        return

    def _start_fixed_node_input(self):
        self.mouse_click_input_status = 'fixed'
        self.status_label.setText("Status: Fixed Node Input Started")
        self._show_anchor_nodes_from_config_file()
        self.repaint()
        return

    def _stop_fixed_node_input(self):
        self.mouse_click_input_status = ''
        self._save_config_file()
        self.status_label.setText("Status: Fixed Node Input Saved")
        self._setup_floormaps(is_drawn=True)
        self.repaint()
        return

    def _start_map_scaling(self):
        self.mouse_click_input_status = 'scaling'
        self.map_scaling_init_pixel = None
        self.status_label.setText("Status: Map Scaling Started")
        self.repaint()
        return

    def _stop_map_scaling(self):
        self.mouse_click_input_status = ''
        self._save_config_file()
        self.status_label.setText("Status: Map Scaling Stopped")
        self._setup_floormaps(is_drawn=True)
        self.repaint()
        return

    def _save_config_file(self):
        with open('./trackio.ini', 'w') as f:
            self.config.write(f)
        return

    def setup_gui(self):
        self.showMaximized()
        self.setWindowTitle("NavigateIO Desktop App")
        self.layout = QGridLayout()
        self.setLayout(self.layout)

        self.viz_checkbox = []
        self.gui_node_list = []
        self.start_buttons = []
        self.stop_buttons = []
        self.floor_combo_box = []
        self.mouse_click_input_status = ''
        #------------- set title------------------------------------#
        cur_font = QFont('Arial', 30, QFont.Bold)
        cur_label = QLabel("NavigateIO")
        cur_label.setStyleSheet("QLabel {color: white; background-color: green;}")
        cur_label.setAlignment(QtCore.Qt.AlignCenter)
        cur_label.setFont(cur_font)
        title_layout = QVBoxLayout()
        title_layout.addWidget(cur_label)
        title_layout.addStretch()

        #--------------------node panel------------------------------------------#
        self.node_widget = QWidget()
        # self.node_widget.setStyleSheet("border: 10px; ")
        self.node_layout = QVBoxLayout()
        self.node_widget.setLayout(self.node_layout)

        cur_font = QFont('Arial', 15, QFont.Bold)
        cur_label = QLabel("Program Control")
        cur_label.setFont(cur_font)
        cur_layout = QHBoxLayout()
        cur_layout.addWidget(cur_label)
        self.node_layout.addLayout(cur_layout)

        cur_layout = QHBoxLayout()
        cur_start_button = QPushButton("Start Map Scaling")
        cur_start_button.clicked.connect(self._start_map_scaling)
        cur_layout.addWidget(cur_start_button)

        cur_stop_button = QPushButton("Stop Map Scaling")
        cur_stop_button.clicked.connect(self._stop_map_scaling)
        cur_layout.addWidget(cur_stop_button)
        self.node_layout.addLayout(cur_layout)

        cur_layout = QHBoxLayout()
        cur_start_button = QPushButton("Start Anchor Input")
        cur_start_button.clicked.connect(self._start_anchor_input)
        cur_layout.addWidget(cur_start_button)

        cur_stop_button = QPushButton("Stop Anchor Input")
        cur_stop_button.clicked.connect(self._stop_anchor_input)
        cur_layout.addWidget(cur_stop_button)
        self.node_layout.addLayout(cur_layout)

        cur_layout = QHBoxLayout()
        cur_start_button = QPushButton("Start Fixed Node Input")
        cur_start_button.clicked.connect(self._start_fixed_node_input)
        cur_layout.addWidget(cur_start_button)

        cur_stop_button = QPushButton("Stop Fixed Node Input")
        cur_stop_button.clicked.connect(self._stop_fixed_node_input)
        cur_layout.addWidget(cur_stop_button)
        self.node_layout.addLayout(cur_layout)

        cur_layout = QHBoxLayout()
        cur_start_button = QPushButton("Start Tracking")
        cur_start_button.clicked.connect(self._start_tracking)
        cur_layout.addWidget(cur_start_button)

        cur_stop_button = QPushButton("Stop Tracking")
        cur_stop_button.clicked.connect(self._stop_tracking)
        cur_layout.addWidget(cur_stop_button)
        self.node_layout.addLayout(cur_layout)

        cur_layout = QHBoxLayout()
        cur_start_button = QPushButton("Show BreadCrumb")
#        cur_start_button.clicked.connect(self._show_bread_crumb)
        cur_layout.addWidget(cur_start_button)
        self.node_layout.addLayout(cur_layout)

        # cur_stop_button = QPushButton("SRemove Breadcrumb")
        # cur_stop_button.clicked.connect(self._stop_tracking)
        # cur_layout.addWidget(cur_stop_button)
        # self.node_layout.addLayout(cur_layout)

        for cur_indx in range(15):
            # for each node, add a horizontal layout, add check box, label, radio buttons, button
            cur_layout = QHBoxLayout()
            if cur_indx == 0:
                cur_font = QFont('Arial', 15, QFont.Bold)
                cur_label = QLabel("Node Control")
                cur_label.setFont(cur_font)
                cur_layout = QHBoxLayout()
                cur_layout.addWidget(cur_label)
            else :
                cur_checkbox = QCheckBox('')
                cur_checkbox.setChecked(True)
                cur_layout.addWidget(cur_checkbox)
                cur_checkbox.stateChanged.connect(self._checkbox_callback)
                self.viz_checkbox.append(cur_checkbox)
                self.viz_selected_nodes.append(cur_indx)

                cur_node_label = QLabel('{0:02}'.format(cur_indx))
                cur_node_label.setStyleSheet("border: 3px solid red;")
                cur_layout.addWidget(cur_node_label)
                self.gui_node_list.append(cur_node_label)

                cur_start_button = QPushButton("Start")
                # cur_reset_button.setStyleSheet("background-color : yellow")
                cur_start_button.clicked.connect(lambda : self._start_button_handler(cur_start_button))
                cur_layout.addWidget(cur_start_button)
                self.start_buttons.append(cur_start_button)


                cur_stop_button = QPushButton("Stop")
                cur_stop_button.clicked.connect(lambda: self._stop_button_handler(cur_stop_button))
                cur_layout.addWidget(cur_stop_button)
                self.stop_buttons.append(cur_stop_button)

                cur_layout.addWidget(QLabel(" Floor:"))

                cur_combo = QComboBox(self)
                for cur_floor in self.floor_labels:
                    cur_combo.addItem(str(cur_floor))
                cur_combo.setCurrentIndex(self.floor_labels.index(self.node_floor[cur_indx]))
                cur_combo.currentTextChanged.connect(lambda : self._floor_combobox_handler( cur_combo))
                cur_layout.addWidget(cur_combo)
                self.floor_combo_box.append(cur_combo)
            self.node_layout.addLayout(cur_layout)

        self.status_label = QLabel('Status: Waiting for command')
        self.status_label.setStyleSheet("QLabel {color: red; font-size: 12pt; font: bold }")
        cur_layout = QHBoxLayout()
        cur_layout.addWidget(self.status_label)
        self.node_layout.addLayout(cur_layout)

        self.node_layout.addStretch()
        #---------------map-panel-----------------------#
        self._initialize_floormap_gui()

        #----------------setup center map window-------------------------#
        self.layout.addLayout(title_layout,     0, 0,  2, 31 )
        self.layout.addWidget(self.node_widget, 2, 0,  2,  1)
        self.layout.addWidget(self.canvas,      2, 1,  10, 30)
        self.repaint()
        return

    def _checkbox_callback(self, cbox):
        self.viz_selected_nodes = []
        for indx, i in enumerate(self.viz_checkbox):
            if i.isChecked():
                n1 = indx+1
                if not n1 in self.viz_selected_nodes:
                    self.viz_selected_nodes.append(n1)
        return

    def _start_button_handler(self, cur_button):
        for indx, i in enumerate(self.start_buttons):
            if cur_button.sender() == i:
                n1 = indx+1
                if not self.controller is None:
                    self.controller.trackio_add_active_node(n1, self.node_floor[n1])
                if not n1 in self.active_nodes:
                    self.active_nodes.append(n1)
                self.gui_node_list[indx].setStyleSheet("border: 3px solid green;")
                if not self.is_viz:
                    self.repaint()
                return
        return

    def _stop_button_handler(self, cur_button):
        for indx, i in enumerate(self.stop_buttons):
            if cur_button.sender() == i:
                n1 = indx+1
                if n1 in self.active_nodes:
                    self.active_nodes.remove(n1)
                if not self.controller is None:
                    self.controller.trackio_remove_active_node(n1)
                if not self.is_viz:
                    self.gui_node_list[indx].setStyleSheet("border: 3px solid red;")
                    self.repaint()
                return
        return

    def _floor_combobox_handler(self, cur_combo):
        for indx, i  in enumerate(self.floor_combo_box):
            if cur_combo.sender() == i:
                cur_txt = str(i.currentText())
                n1, n1_floor = indx + 1, int(cur_txt)
                if self.controller is None:
                    self.node_floor[n1] = n1_floor
                else:
                    self.controller.trackio_change_node_floor(n1, n1_floor)
                return
        return

    def _init_tracking_state(self):
        # self.loc_history = {}
        # for n1 in range(1, 15):
        #     self.loc_history[n1] =  deque([], maxlen=3)

        self.cached_locations = {}
        self.is_viz = True
        self.rot_angle = {}
        #self.inv_rot_angle = {}
        for cur_floor in self.floor_labels:
            n1, n2 = self.anchor_nodes[cur_floor]
            loc1, loc2 = self.fixed_node_global_loc_meter[n1], \
                         self.fixed_node_global_loc_meter[n2]
            self.rot_angle[cur_floor] =  self._angle(loc2, loc1)
            #self.inv_rot_angle[cur_floor] = -self._angle((loc2[0] -loc1[0], loc2[1]-loc2[1]), (0., 0.))

        self.tranformed_fixed_node_loc = {}
        # offset first
        for n1, n1_loc in self.fixed_node_global_loc_meter.items():
            n1_x, n1_y = n1_loc[0], n1_loc[1]
            n1_floor = self.node_floor[n1]
            n1_origin = self.anchor_nodes[n1_floor][0]
            n1_origin_x, n1_origin_y = self.fixed_node_global_loc_meter[n1_origin][0], \
                                       self.fixed_node_global_loc_meter[n1_origin][1]
            self.tranformed_fixed_node_loc[n1] = (n1_x - n1_origin_x, n1_y - n1_origin_y)
        req_angle = {}
        for cur_floor in self.floor_labels:
            x_axis_node = self.anchor_nodes[cur_floor][1]
            x_axis_node_x, x_axis_node_y = self.tranformed_fixed_node_loc[x_axis_node][0],\
                                                 self.tranformed_fixed_node_loc[x_axis_node][1]
            req_angle[cur_floor] = math.atan2(x_axis_node_y, x_axis_node_x)

        for n1, n1_loc in self.tranformed_fixed_node_loc.items():
            n1_floor = self.node_floor[n1]
            n1_x, n1_y = self._rotate_point(n1_loc, -req_angle[n1_floor])
            if n1_y < 0.:
                n1_y = -n1_y #TODO: automatically add cur_floor to flip_x or flip_y
            self.tranformed_fixed_node_loc[n1] = (round(n1_x, 4), round(n1_y, 4) )
        return

    def _thread_checker(self):
        if self.solver_thread is None or self.viz_thread is None:
            return
        while self.is_viz:
            time.sleep(3)
            if not self.solver_thread.is_alive():
                self.is_viz = False
                #QMessageBox.about(self, "Exception Alert", "Solver thread terminated")
                #print("Solver thread terminated!!")
                break
            if not self.viz_thread.is_alive():
                self.controller.trackio_stop()
                #QMessageBox.about(self, "Exception Alert", "Viz thread terminated")
                #print("Viz thread terminated!!")
                break
            return

    def _start_tracking(self):
        self._parse_config_file()
        self._init_tracking_state()
        self.controller = UDPSolver(
            udp_port = self.udp_port,
            udp_timeout_milliseconds = self.udp_timeout_msec,
            node_ips = self.node_ips,
            twr_tdma_slot_milliseconds = self.twr_frame_time_msec,
            anchor_nodes = self.anchor_nodes,
            fixed_node_loc = self.tranformed_fixed_node_loc,
            node_floor = self.node_floor,
            active_nodes = self.active_nodes,
            sea_leve_pressure_in_hg = self.sea_level_pressure_inch_hg,
            floor_change_threshold_meter = self.floor_change_threshold_meter
        )
        self.solver_thread = threading.Thread(name='Trackio-solver-thread',
                                              target=self.controller.trackio_start,
                                              daemon=True)
        self.viz_thread = threading.Thread(name='Vizualizer-thread', target=self.visualize_loc,
                                           daemon=True)
        self.checker_thread = threading.Thread(name='Checker-thread', target=self._thread_checker, daemon=True)
        self.solver_thread.start()
        self.viz_thread.start()
        self.checker_thread.start()
        self.status_label.setText("Status: Tracking Started")
        return

    def visualize_loc(self):
        self.gui_frame_count = 0
        self.gui_node_list = {}
        self.gui_frame_count_text = None
        # self.debug_gui_test_node = None
        # self.debug_gui_loc_offset = 15
        while self.is_viz:
            self._update_location_plots()
            self._gui_heartbit()
            self.canvas.draw_idle()
            time.sleep(self.display_interval)
        return

    def _stop_tracking(self):
        if not self.controller is None:
            self.controller.trackio_stop()
        self.is_viz = False
        time.sleep(1)
        self.controller = None
        self.status_label.setText("Status: Tracking Stopped")
        self.repaint()
        return

    def _angle(self, v1, v2):
        x1, y1, x2, y2 = v1[0], v1[1], v2[0], v2[1]
        return math.atan2(y2-y1, x2-x1)

    def _rotate_point(self, point, angle):
        px, py = point
        qx = math.cos(angle) * (px) - math.sin(angle) * (py)
        qy = math.sin(angle) * (px) + math.cos(angle) * (py)
        return qx, qy

    def _tranform_point(self, point, translation, angle, is_rot_first = True):
        x , y  = point
        del_x, del_y = translation
        if is_rot_first:
            x, y =self._rotate_point((x, y), angle)
            x += del_x
            y += del_y
        else:
            x += del_x
            y += del_y
            x, y = self._rotate_point((x, y), angle)
        return (x, y)

    def _cache_location_data(self, cur_loc_indexed_by_floor):
        nodes_with_loc_update = []
        for cur_floor, cur_locs in cur_loc_indexed_by_floor.items():
            for n1, n1_loc in cur_locs.items():
                self.node_floor[n1] = cur_floor
                self.floor_combo_box[n1 - 1].setCurrentIndex(self.floor_labels.index(cur_floor))
                n1_x, n1_y = n1_loc[0], n1_loc[1]
                origin_node, x_axis_node = self.anchor_nodes[cur_floor]
                if cur_floor in self.flip_x:
                    n1_x = -n1_x
                if cur_floor in self.flip_y:
                    n1_y = -n1_y

                n1_x, n1_y = self._tranform_point((n1_x, n1_y),
                                                angle=self.rot_angle[cur_floor],
                                                translation=self.fixed_node_global_loc_meter[origin_node])
                # ------------------------------------------------------------------------------------#
                n1_x_pixel, n1_y_pixel = (n1_x) * self.per_floor_pixel_per_meter[cur_floor], \
                                   (n1_y) * self.per_floor_pixel_per_meter[cur_floor]

                self.cached_locations[n1] = (n1_x_pixel, n1_y_pixel)
                nodes_with_loc_update.append(n1)
        return nodes_with_loc_update

    def _gui_heartbit(self):
        self.gui_frame_count += 1
        cur_frame_count_text = "Update# "+str(self.gui_frame_count)
        if self.gui_frame_count_text is None:
            self.gui_frame_count_text = self.axes[0].text(-210,-60,
                      cur_frame_count_text, fontsize=20, fontweight='bold', color='red',
                      # TODO: adjust node font size
                      bbox=dict(boxstyle='square', fc="cyan", ec='none'))
        else:
            self.gui_frame_count_text.set_text(cur_frame_count_text)

        # if self.debug_gui_test_node is None:
        #     self.debug_gui_test_node = self.axes[1].text(500,
        #                   500,
        #                   "T1", fontsize=20, fontweight='bold', color='w',
        #                   # TODO: adjust node font size
        #                   bbox=dict(boxstyle='square', fc="red", ec='none'))
        # else:
        #     x, y = self.debug_gui_test_node.get_position()
        #     x -= self.debug_gui_loc_offset
        #     y -= self.debug_gui_loc_offset
        #     if x < 0 or x>1000:
        #         self.debug_gui_loc_offset = -self.debug_gui_loc_offset
        #
        #     self.debug_gui_test_node.set_position((
        #             max(x-self.debug_gui_loc_offset, 0),
        #             max(y-self.debug_gui_loc_offset, 0)))
        return

    def _add_node_to_gui(self, n1):
        print("debug: add node to gui ", n1)
        n1_floor = self.node_floor[n1]
        n1_floor_index = self.floor_labels.index(n1_floor)
        cur_axis = self.axes[n1_floor_index]
        if not n1 in self.cached_locations.keys():
            return
        backgroundcolor = 'darkred'
        if n1 in self._node_colors.keys():
            backgroundcolor = self._node_colors[n1]

        n1_label = str(n1)
        if n1 in self.node_names.keys():
            n1_label = self.node_names[n1]

        self.gui_node_list[n1] =cur_axis.text(self.cached_locations[n1][0],
                      self.cached_locations[n1][1],
                      str(n1_label), fontsize=20, fontweight='bold', color='w',
                      # TODO: adjust node font size
                      bbox=dict(boxstyle='square', fc=backgroundcolor, ec='none'))
        return

    def _update_location_plots(self):
        #cur_active_nodes = self.controller.trackio_get_current_active_nodes()
        if not self.controller.trackio_is_new_location_available():
            return

        nodes_with_loc_update = self._cache_location_data( self.controller.trackio_get_locations() )

        #self._setup_floormaps(is_drawn=False) #TODO: uncomment if needed
        for n1, n1_floor in self.node_floor.items():
            if not n1 in self.viz_selected_nodes:
                if n1 in self.gui_node_list.keys():
                    self.gui_node_list[n1].remove()
                continue

            if not n1 in nodes_with_loc_update:
                continue

            if not n1 in self.gui_node_list.keys():
                self._add_node_to_gui(n1)
            else:
                self.gui_node_list[n1].set_position((
                    self.cached_locations[n1][0],
                    self.cached_locations[n1][1]  ))

        # for n1 in range(1, 15):
        #     if n1 in cur_active_nodes:
        #         self.gui_node_list[n1 - 1].setStyleSheet("border: 3px solid green;")
        #     else:
        #         self.gui_node_list[n1 - 1].setStyleSheet("border: 3px solid red;")
        #self.canvas.draw_idle()
        # self.repaint()
        return


if __name__ == '__main__':
    app = QApplication(sys.argv)
    trackio = TrackIO_Interface()
    app.exec_()

'''
    #-------------------------backup----------------------------------------------#
    def _cache_location_data(self, cur_loc_indexed_by_floor):
        for cur_floor, cur_locs in cur_loc_indexed_by_floor.items():
            for n1, n1_loc in cur_locs.items():
                self.node_floor[n1] = cur_floor
                self.floor_combo_box[n1 - 1].setCurrentIndex(self.floor_labels.index(cur_floor))
                n1_x, n1_y = n1_loc[0], n1_loc[1]
                origin_node, x_axis_node = self.anchor_nodes[cur_floor]
                if cur_floor in self.flip_x:
                    n1_x = -n1_x
                if cur_floor in self.flip_y:
                    n1_y = -n1_y

                n1_x, n1_y = self._tranform_point((n1_x, n1_y),
                                                angle=self.rot_angle[cur_floor],
                                                translation=self.fixed_node_global_loc_meter[origin_node])
                # ------------------------------------------------------------------------------------#
                n1_x_pixel, n1_y_pixel = (n1_x) * self.per_floor_pixel_per_meter[cur_floor], \
                                   (n1_y) * self.per_floor_pixel_per_meter[cur_floor]

                self.loc_history[n1].append((cur_floor, time.time(), n1_x_pixel, n1_y_pixel))

                if n1 in self.fixed_node_global_loc_meter.keys()\
                        or (not n1 in self.cached_locations.keys()) or len(self.loc_history[n1])==1 :
                    self.cached_locations[n1] = (n1_x_pixel, n1_y_pixel)
                else:
                    x_vals, y_vals = [], []
                    x1 , y1 = self.cached_locations[n1][0],self.cached_locations[n1][1]
                    distances = []
                    for v in self.loc_history[n1]:
                        if time.time() - v[0] < 10:
                            d = math.sqrt((x1-v[1])**2. + (y1-v[2])**2.)
                            distances.append(d)
                            x_vals.append(v[1])
                            y_vals.append(v[2])

                    if len(distances) == 2:
                        self.cached_locations[n1] =(sum(x_vals)/2., sum(y_vals)/2.)
                    elif len(distances) == 3:
                        median_distance = sorted(distances)[1]
                        for indx, cur_d in enumerate(distances):
                            if cur_d == median_distance:
                                self.cached_locations[n1] = (x_vals[indx], y_vals[indx])
                                break

        transformed_locs_by_floor = {}
        for cur_floor in cur_loc_indexed_by_floor.keys():
            transformed_locs_by_floor[cur_floor] = {}
            for n1 in cur_loc_indexed_by_floor[cur_floor].keys():
                self.node_floor[n1] = cur_floor
                x,y=self.cached_locations[n1]
                transformed_locs_by_floor[cur_floor][n1] = (x,y)
        self._save_to_servers(transformed_locs_by_floor)
        return
'''

'''
    #-----------------server-related-------------------------#
    def _configure_server_connections(self):
        self.db_client = InfluxDBClient('localhost', 8086, 'root', 'root', 'trackio')
        #self.cache_client = base.Client(('localhost', 11211))
        return

    def _save_to_servers(self, node_locs_all_floor):
        # try:
        db_json = []
        cached_locations = []
        loc_time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
        for cur_floor, locs in node_locs_all_floor.items():
            for n1, n1_loc in locs.items():
                cached_locations.append({"id": str(n1),
                                         "x": float(n1_loc[0]),
                                         "y": float(n1_loc[1]),
                                         "floor": cur_floor})
                db_json.append({"time": loc_time,
                                "measurement": "location",
                                "tags": {"nodeid": n1},
                                "fields": {
                                    "x": float(n1_loc[0]),
                                    "y": float(n1_loc[1]),
                                    "floor": cur_floor}})

        self.db_client.write_points(db_json)
        #self.cache_client.set('locations', json.dumps({"time": loc_time, "locations": cached_locations}))

        # except Exception as e:
        #     print("Exception: ", str(e))
        return
    #------------------------breadcrumb-related---------#
    def _show_bread_crumb(self):
        node_id, okPressed = QInputDialog.getInt(self, "BreadCrumb Node Input", "Fixed Node ID:", 5, 0, 100, 1)
        if not (okPressed and 1 <= node_id <= 14):
            return

        node_id = int(node_id)
        self._setup_floormaps(is_drawn=False)
        queryString = "select x,y,floor from location where nodeid = \'" + str(node_id) + "\'"
        query_result_set = self.db_client.query(queryString)
        result_vals = list(query_result_set.get_points(measurement='location'))
        location = []
        for val in result_vals:
            location.append((val['x'],val['y'],val['floor']))

        if not location:
            return

        avg_window = 5
        prev_x = {}
        prev_y = {}
        prev_plotted_x = {}
        prev_plotted_y = {}
        for cur_floor in self.floor_labels:
            prev_plotted_x[cur_floor] = -1.
            prev_plotted_y[cur_floor] = -1.
        starting_x, starting_y = -1., -1.

        for v in location:
            x, y, flr = float(v[0]), float(v[1]), int(v[2])
            cur_axis = self.axes[self.floor_labels.index(flr)]

            if not flr in prev_x.keys():
                prev_x[flr] = deque(maxlen=avg_window)
                prev_y[flr] = deque(maxlen=avg_window)
            prev_x[flr].append(x)
            prev_y[flr].append(y)

            if len(prev_x[flr]) >=2:
                avg_x = 1.*sum(prev_x[flr])/len(prev_x[flr])
                avg_y = 1. * sum(prev_y[flr] )/ len(prev_y[flr])
                if math.sqrt( (avg_x-prev_plotted_x[flr])**2. + (avg_x-prev_plotted_x[flr])**2.)>5.:
                    cur_axis.plot([prev_plotted_x[flr], avg_x], [prev_plotted_y[flr], avg_y], c='b', lw=4.5, alpha=0.25)
                    prev_plotted_x[flr], prev_plotted_y[flr] = avg_x, avg_y
            else:

                prev_plotted_x[flr], prev_plotted_y[flr] = x, y
                starting_x, starting_y = x,y

        start_floor_indx = self.floor_labels.index( int(location[0][2]) )
        end_floor_indx = self.floor_labels.index( int(location[-1][2]) )

        self.axes[start_floor_indx].scatter(starting_x, starting_y, s=50, c='r', marker="s")
        self.axes[end_floor_indx].scatter(prev_plotted_x[flr], prev_plotted_y[flr], s=50, c='b', marker="s")
        self.canvas.draw()
        return
'''


