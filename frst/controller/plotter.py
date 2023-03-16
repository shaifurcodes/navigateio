
import sys
import numpy as np
#from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import configparser

class Plotter(object):
    def __init__(self, exp_name):
        '''
        #TODO:
            1. load config.ini file from exp_name folder
            2. set origin, xaxis_node, mobile_node and fixed_nodes
            3. load the range-log file
            4. process each range-event at a time
                A. filter range
                B. localize
                C. plot
        '''
        self.exp_name = str(exp_name)

        self.origin_node = None
        self.xaxis_node = None
        self.mobile_node = None
        self.node_list = []
        self.fixed_nodes = []
        self.load_config()

        self.x = [None for _ in self.node_list]
        self.y = [None for _ in self.node_list]
        self.z = [0. for _ in self.node_list]

        self.x[0] = 0. #origin node x-coord zero
        self.y[0] = 0. #origin node y-coord zero

        self.y[1] = 0. #xaxis node y-coord zero

        self.mobile_xs = []
        self.mobile_ys = []
        self.xaxis_xs = []
        self.mobile_ts = []

        n = len(self.node_list)
        self.edm = np.zeros((n, n), dtype='f8')
        with open('./'+exp_name+'/location_data.txt',"w") as f:
            pass

        vals = np.genfromtxt('./'+exp_name+'/ranges_log.txt', delimiter=',', dtype='f8,i4,i4,f8') #'f8,i4,i4,f8'

        self.fig, self.ax = plt.subplots()

        for v in vals:
            ts, n1, n2, r = v[0], v[1], v[2], v[3]/100.
            if r <= 0. or n1==n2: continue
            if not n1 in self.node_list or not n2 in self.node_list: continue

            n1_indx, n2_indx = self.node_list.index(n1), self.node_list.index(n2)
            self.edm[n1_indx, n2_indx] = r
            self.edm[n2_indx, n1_indx] = r

            self.localize(n1)
            self.localize(n2)

            if self.mobile_node in [n1, n2]:
                if not self.x[2] is None and not self.y[2] is None:
                    self.mobile_xs.append(self.x[2])
                    self.mobile_ys.append(self.y[2])
                    self.mobile_ts.append(ts)
                    self.xaxis_xs.append(self.x[1])
                    ftext = ""

                    for nxy in zip(self.node_list, self.x, self.y):
                        ftext = str(ts)+", "+str(nxy[0])+", "+str(round(nxy[1], 3))+",  "+str(round(nxy[2], 3))
                        print(ftext)
                        print("=======================================")
                        with open('./' + exp_name + '/location_data.txt', "a") as f:
                            f.write(ftext+"\n")

        anim = FuncAnimation(self.fig, self.animate, frames=len(self.mobile_ts), interval=10, repeat=False)
        anim.save('./'+self.exp_name+'/triangle.mp4', writer='ffmpeg', fps=30)
        plt.show()
        return

    def animate(self, i):
        if i > len(self.mobile_ts):
            return
        plt.cla()
        xmin, xmax, ymin, ymax  = -5+min(self.mobile_xs), 5+max(self.mobile_xs), -5, 5+max(self.mobile_ys)
        self.ax.set_xlim(xmin, xmax )
        self.ax.set_ylim(ymin, ymax)
        self.ax.grid(linestyle='--')
        self.ax.set_xticks(range(int(xmin), int(xmax), 5))
        self.ax.set_yticks(range(int(ymin), int(ymax), 5))
        plt.title(str(self.mobile_ts[i]))
        xs = [ self.mobile_xs[i], 0., self.xaxis_xs[i], self.mobile_xs[i] ]
        ys = [ self.mobile_ys[i], 0., 0., self.mobile_ys[i] ]
        plt.plot(xs , ys)
        plt.scatter(self.mobile_xs[max(0, i-10):i], self.mobile_ys[max(0, i-10):i])
        return

    def load_config(self):
        config = configparser.ConfigParser()
        config.read('./controller.ini')

        self.origin_node = int(config['BEACON']['origin'])
        self.xaxis_node = int(config['BEACON']['xaxis'])
        self.mobile_node = int(config['BEACON']['mobile'])

        self.node_list = [self.origin_node, self.xaxis_node, self.mobile_node]

        self.fixed_nodes = []
        if 'fixed' in config['BEACON'].keys():
            self.fixed_nodes = \
                [int(i) for i in str(config['BEACON']['fixed']).split(',') if not int(i) in self.node_list]
            if self.fixed_nodes:
                self.node_list = self.node_list+self.fixed_nodes
        return

    def localize(self, n1):
        if n1 == self.origin_node: return
        elif n1 == self.xaxis_node:
            if self.edm[0, 1] ==0.: return
            self.x[1], self.y[1] = self.edm[0, 1], 0.
        else:
            n1_indx = self.node_list.index(n1)
            if self.x[1] is None or self.edm[0, n1_indx] ==0. or self.edm[1, n1_indx] == 0.:
                return
            v = self.get_intersections(0.,        0., self.edm[0, n1_indx],
                                       self.x[1], 0., self.edm[1, n1_indx])
            if not v is None:
                    if v[1] >= 0.:
                        self.x[n1_indx], self.y[n1_indx] = v[0], v[1]
                    else:
                        self.x[n1_indx], self.y[n1_indx] = v[2], v[3]
        return

    def get_intersections(self, x0, y0, r0, x1, y1, r1):
        # circle 1: (x0, y0), radius r0
        # circle 2: (x1, y1), radius r1

        d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

        # non intersecting
        if d > r0 + r1:
            return None
        # One circle within other
        if d < abs(r0 - r1):
            return None
        # coincident circles
        if d == 0 and r0 == r1:
            return None

        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        try:
            h = math.sqrt(r0 ** 2 - a ** 2)
        except Exception as ex:
            print(r0, a)
            return None
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return (x3, y3, x4, y4)

if __name__ == '__main__':
    if not len(sys.argv) == 2:
        print(" usage: sudo python3 plotter.py exp_name")
    exp_name = str(sys.argv[1])

    pt = Plotter( exp_name=exp_name)