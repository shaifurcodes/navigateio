import  logging
import  time
from copy import  deepcopy
import numpy as np
from collections import  Counter
import localization as lx
import threading
from collections import  deque
from time import  sleep
#from feb_demo.controller.udp_data_collector import UDPDataCollector
from udp_data_collector import UDPDataCollector
from numpy.linalg import inv

import matplotlib.pyplot as plt
import matplotlib.animation as anim

class UDPSolver(object):
    def __init__(self,
                 udp_port,
                 udp_timeout_milliseconds,
                 node_ips,
                 twr_tdma_slot_milliseconds,
                 anchor_nodes,
                 fixed_node_loc,
                 node_floor,
                 active_nodes,
                 sea_leve_pressure_in_hg,
                 floor_change_threshold_meter
                 ):
        self.udp_data_collector = UDPDataCollector(
                udp_com_time_msec  = udp_timeout_milliseconds,
                twr_frame_time_msec = twr_tdma_slot_milliseconds,
                udp_port = udp_port
        )
        self.udp_data_collector.set_node_ips(node_ips)

        self.nodes_with_location_update = set()
        self.lock_nodes_with_location_update = threading.Lock()

        self.init_time = time.time()

        self.anchor_nodes = deepcopy(anchor_nodes)
        self.fixed_node_loc = deepcopy(fixed_node_loc)
        self.active_nodes = sorted(active_nodes)
        self.all_nodes = [i for i in range(1, 15)]

        self.sea_level_pressure_in_hg = float(sea_leve_pressure_in_hg)
        self.floor_change_threshold_meter = float(floor_change_threshold_meter)

        self.node_floor = deepcopy(node_floor)
        self.lock_node_floor = threading.Lock()

        self.max_floor = max(self.node_floor.values())
        self.min_floor = min(self.node_floor.values())

        self.DUMMY_ID = 64
        self.wifi_link_aliveness = 30

### --- RKS added this --- ##
        self.imu_locations = {}
        self.P = {}
        self.deltatime = {}
        self.floor_change_flag = {}

        return
#-------------------setter methods---------------------------------------------#
    def _save_link_quality(self, link_info):
        for n1, link_q_vals in link_info.items():
            n1_indx = self.all_nodes.index(n1)
            for n2, v in link_q_vals.items():
                if not n2 in self.all_nodes:
                    continue
                n2_indx = self.all_nodes.index(n2)
                lq_ts = max( [self._get_current_time() - v[0],  0.] )
                self.ts_link_quality[n1_indx, n2_indx] =\
                    self.ts_link_quality[n2_indx, n1_indx] = lq_ts
                self.link_quality[n1_indx, n2_indx] =\
                    self.link_quality[n2_indx, n1_indx] = v[1]
        return

    def _save_floor_val(self,cur_floor):
        with self.lock_node_floor:
            for n1, n1_floor in cur_floor.items():
                if(n1 in self.floor_change_flag.keys()):
                    if(self.node_floor[n1] == n1_floor):
                        self.floor_change_flag[n1] = False
                    else:
                        self.floor_change_flag[n1] = True
                else:
                    self.floor_change_flag[n1] = False

                self.node_floor[n1] = n1_floor
        return

    def _save_ranges_to_edm(self, all_range_vals):
        for n1, v in all_range_vals.items():
            n1_indx = self.all_nodes.index(n1)
            for n2, d in v.items():
                self.ts_tx[n1] = self._get_current_time()
                if n2 == self.DUMMY_ID:
                    continue
                n2_indx = self.all_nodes.index(n2)
                if d > 0.:
                    self.edm[n1_indx, n2_indx] = self.edm[n2_indx, n1_indx] = d
                    self.ts_edm[n1_indx, n2_indx] = self.ts_edm[n2_indx, n1_indx] = \
                        self._get_current_time()
                    self.ts_tx[n2] = self._get_current_time()
        return

    def _save_location(self, n1, n1_x, n1_y):
        '''
        TODO: before update of location
            1) add to history
            2) filter location based on floor, timestamp, and distances from previous locations
            3) save location
        '''

        cur_ts = self._get_current_time()
        new_loc_x, new_loc_y = None, None
        if  not self.location_history[n1] or n1 in self.fixed_node_loc.keys(): # if no history (first time localization), or fixed node,
                                                                                # save location regardless
            new_loc_x, new_loc_y = n1_x, n1_y
        else:
            locs = []
            for n1_tuple in self.location_history[n1]: # find the same floor locations in the last 10. seconds
                if n1_tuple[0] == self.node_floor[n1] and cur_ts - n1_tuple[1] < 5:
                    locs.append([n1_tuple[2], n1_tuple[3]])
            if not locs: # if no such locations found from history, save the current location regardless
                new_loc_x, new_loc_y = n1_x, n1_y
            else:
                new_successive_d = np.sqrt((locs[-1][0] - n1_x)**2. + (locs[-1][1] - n1_y)**2.)
                if new_successive_d < 10.: # check if new location is >10m apart from the last recorded location as above
                    # successive_d = []
                    # for i in range(1, len(locs)): # compute successive (gap) distances of locations
                    #     successive_d.append(    np.sqrt( (locs[i][0] - locs[i-1][0])**2.+ \
                    #                                      (locs[i][1] - locs[i-1][1])**2.  ) )
                    # median_successive_d = 10.
                    # if len(locs)>1:
                    #     median_successive_d = np.median(successive_d)
                    # if new_successive_d <= 2*median_successive_d: #TODO 2*median_d
                        # check if new location gap is within 2*standard dev. of above computed successive gap
                        #if yes, then use the median filter
                        #print("debug: new loc!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        new_loc_x = np.median([i[0] for i in locs[-2:]]+[n1_x])
                        new_loc_y = np.median([i[1] for i in locs[-2:]]+[n1_y])

        if not new_loc_x is None:
            self.location[n1] = [new_loc_x, new_loc_y]
            self.ts_location[n1] = self._get_current_time()
            with self.lock_nodes_with_location_update:
                self.nodes_with_location_update.add(n1)
            if not n1 in self.fixed_node_loc.keys(): #no need to build up location history for fixed nodes
                self.location_history[n1].append([self.node_floor[n1], self._get_current_time(), n1_x, n1_y])

        return

    def _save_imu_locations(self,all_imu_location):
        for node, loc in all_imu_location.items():
            self.imu_locations[node] = loc
        return

#----------------------getter methods---------------------------------------------------------------#
    def _get_current_time(self):
        self.cur_time = round(time.time() - self.init_time,3)
        return self.cur_time

    def _get_range_from_edm(self,n1, n2, freshness_sec=0):
        if freshness_sec == 0:
            freshness_sec = len(self.all_nodes)
        n1_indx, n2_indx= self.all_nodes.index(n1), self.all_nodes.index(n2)
        if self._get_current_time() - self.ts_edm[n1_indx, n2_indx] < freshness_sec:
            return self.edm[n1_indx, n2_indx]
        return 0.

#---------------------------node-data-collection-related-------------------------------------------------#
    def _inform_locations(self, node, new_loc, flag):
        self.udp_data_collector.inform_location(node, new_loc, flag)
        return

    def _query_ranges(self, cur_node, cur_nbr, freshness_for_ranging_skip):
        '''
        tasks:
            query range n->nbr, save ts, ranges, heading and a to file
        :param n:
        :param nbr:
        :return:
        '''

        excluded_nodes = []
        if freshness_for_ranging_skip != 0:
            for n2 in cur_nbr:
                if self._is_range_fresh(cur_node, n2, freshness_sec=freshness_for_ranging_skip):
                    excluded_nodes.append(n2)
        ranging_nbr = list(set(cur_nbr) - set(excluded_nodes))

        if len(ranging_nbr) <=0:
            return

        #all_range_vals, all_link_info, all_heading, all_accleration, all_cur_floors = \
        #    self.udp_data_collector.query_range(node=cur_node, nlist=ranging_nbr)

        all_range_vals, all_link_info, all_imu_locations, all_cur_floors = \
        self.udp_data_collector.query_range(node=cur_node, nlist=ranging_nbr)

        if all_cur_floors:
            for n1 in all_cur_floors.keys():
                self.wifi_rx_ts[n1] = self._get_current_time()
                #print("\t\tdebug: "+str(n1)+" got wifi ts updated to "+str(self.wifi_rx_ts[n1])  )
        if all_range_vals:
            self._save_ranges_to_edm(all_range_vals)
        if all_link_info:
            self._save_link_quality(all_link_info)
        if all_imu_locations:
            self._save_imu_locations(all_imu_locations)
        if all_cur_floors:
            self._save_floor_val(all_cur_floors)
        return

    def _send_discovery_frame(self, node_list, send_count = 1):
        for i in range(send_count):
            for n1 in node_list:
                _ = self._query_ranges(n1, [self.DUMMY_ID], freshness_for_ranging_skip=0)
        return

    def _send_params_to_nodes(self, node_list):
        common_paramlist = 'sea-level-pressure-inch-hg '+str(self.sea_level_pressure_in_hg)+', '\
                    +'floor-change-threshold-meter '+str(self.floor_change_threshold_meter)
        for cur_floor in self.anchor_nodes.keys():
            cur_anchors = self.anchor_nodes[cur_floor]
            common_paramlist += ', anchor '+str(cur_floor)+' '+str(cur_anchors[0])+' '+str(cur_anchors[1])

        for n1 in node_list:
            paramlist = common_paramlist+', current-floor '+str(self.node_floor[n1])
            self.udp_data_collector.send_params(node=n1, paramlist=paramlist)
        return


    def predict(self, x, P, A, Q):
        # Predict Step
        x = np.dot(A, x)
        At = np.transpose(A)
        P = np.add(np.dot(A, np.dot(P, At)), Q)

        return x, P

    def update(self, z, x, P, H, R, I):
        # Measurement update step
        Y = np.subtract(z, np.dot(H, x))
        Ht = np.transpose(H)
        S = np.add(np.matmul(H, np.dot(P, Ht)), R)
        K = np.dot(P, Ht)
        Si = inv(S)
        K = np.dot(K, Si)

        # New state
        x = np.add(x, np.dot(K, Y))
        P = np.dot(np.subtract(I ,np.dot(K, H)), P)

        return x, P


    def run_kalman(self, node, uwb_locs, imu_locs, cur_pos, dt):

        if(node not in self.P.keys()):
            self.P[node] = np.array([
                                    [1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 0]
                                    ])

        imu_x = imu_locs[0]
        imu_y = imu_locs[1]
        uwb_x = uwb_locs[0]
        uwb_y = uwb_locs[1]


        vx = np.zeros([2, 1])
        ux = np.zeros([2, 1])

        x = np.array([
            [float(cur_pos[0])],
            [float(cur_pos[1])],
            [0],
            [0]
            ])

        #Initialize matrices A
        A = np.array([
                [1.0, 0, 1.0, 0],
                [0, 1.0, 0, 1.0],
                [0, 0, 1.0, 0],
                [0, 0, 0, 1.0]
                ])
        H = np.array([
                [1.0, 0, 0, 0],
                [0, 1.0, 0, 0]
                ])
        I = np.identity(4)
        R = np.array([
                [0.01, 0],
                [0, 0.01]
                ])
        Q = np.zeros([4, 4])

        #Updating matrix A with dt value
        A[0][2] = dt
        A[1][3] = dt

        #Updating Q matrix
        Q[0][0] = dt**4/4
        Q[0][2] = dt**3/2
        Q[1][1] = dt**4/4
        Q[1][3] = dt**3/2
        Q[2][0] = dt**3/2
        Q[2][2] = dt**2
        Q[3][1] = dt**3/2
        Q[3][3] = dt**2

        #Call Kalman Filter Predict and Update functions.
        x, self.P[node] = self.predict(x, self.P[node], A, Q)

        vx[0][0] = imu_x
        vx[1][0] = imu_y
        ux[0][0] = uwb_x
        ux[1][0] = uwb_y

        x, self.P[node] = self.update(vx, x, self.P[node], H, R, I)
        x, self.P[node] = self.update(ux, x, self.P[node], H, R, I)

        final_pos = [x[0][0],x[1][0]]

        print("Node #, UWB, IMU, Final pos = " + str(node) +" -- "+str(uwb_locs) + " , "+ str(imu_locs) + " , "+ str(final_pos))

        return(final_pos)


#-------------initialization-relatd---------------------------------------------#
    def _reset_internal_states(self):
        # -----------------EDM-RELATED---------------#
        n = len(self.all_nodes)
        self.edm = np.zeros((n, n), dtype=np.float)
        self.ts_edm = np.zeros((n, n), dtype=np.float)
        # -----------------link-q related--------------------#
        self.link_quality = np.zeros((n, n), dtype=np.float)
        self.ts_link_quality = np.zeros((n, n), dtype=np.float)
        # ------------------location-related-------------------------#
        self.location = {}
        self.ts_location = {}

        self.location_history = {}
        for n1 in self.all_nodes:
            self.location_history[n1] = deque([], maxlen=3) #structure of entry [floor, timestamp, x, y]
        #-------------------tx.rx, timestamp-related---------------------------------#
        self.ts_tx = {}
        self.wifi_rx_ts = {}
        for n1 in self.all_nodes:
            self.wifi_rx_ts[n1] = self._get_current_time()
        #------------------front-end event related-------------------------#
        self.node_to_be_added = []
        self.node_to_be_deleted = []
        self.node_floor_change_request = []
        self.lock_add_delete_node = threading.Lock()

        self.remote_restart_node_list = []
        self.lock_remote_restart_node_list = threading.Lock()

        self.stop_run = False
        return

    def _initialize_fixed_node_locations(self):
        '''
        :return:
        '''
        for n1, n1_loc in self.fixed_node_loc.items():
            self._save_location(n1, n1_loc[0],n1_loc[1])
        return

    def _add_nodes_to_the_system(self, node_list):
        # for n1 in node_list:
        #     self.udp_data_collector.remote_restart_node_program(n1)

        self._send_params_to_nodes(node_list)

        self._send_discovery_frame(node_list)

        for n1 in node_list:
            if not n1 in self.active_nodes:
                self.active_nodes.append(n1)
        return

    def _initialize_trackio_algo(self):
        self._reset_internal_states()
        self._initialize_fixed_node_locations()
        self._add_nodes_to_the_system(self.active_nodes)
        return
#--------------------node addition deletion related-----------------------------------#
    def _change_node_floor_online(self, n1, n1_floor):
        if not n1 in self.active_nodes:
            self.active_nodes.append(n1)
        self.node_floor[n1] = n1_floor
        self._send_params_to_nodes([n1])
        return

    def _run_node_maintenance(self):
        # ---------------mark wifi-dead nodes--------------------------#
        # for n1, n1_tx_ts in self.ts_tx.items():
        #     if self._get_current_time() - n1_tx_ts > self.wifi_link_aliveness:
        #         if n1 in self.active_nodes:
        #             self.active_nodes.remove(n1)

        #------front-end button click and combo box requests----------------#
        new_active_node_data = []
        node_floor_change_data = []
        with self.lock_add_delete_node:
            # ---------------process active node stop request---------------------------#
            for n1 in self.node_to_be_deleted:
                if n1 in self.active_nodes:
                    self.active_nodes.remove(n1)
                self.node_to_be_deleted = []
            # ---------------retrieve other front-end requests---------------------------#
            new_active_node_data = list(self.node_to_be_added)
            self.node_to_be_added = []
            node_floor_change_data = list(self.node_floor_change_request)
            self.node_floor_change_request = []

        # ---------------process active node addition request---------------------------#
        new_nodes = []
        for n1_val in new_active_node_data:
            n1, n1_floor = n1_val[0], n1_val[1]
            new_nodes.append(n1)
            if n1_floor is None:
                continue
            self.node_floor[n1] = n1_floor
        self._add_nodes_to_the_system(new_nodes)

        #---------------process floor change request---------------------------#
        for nval in node_floor_change_data:
            n1, n1_floor = nval[0], nval[1]
            self._change_node_floor_online(n1, n1_floor)
        return

#------------------------------algo computation related---------------------------------#
    def _localize_node(self, n1, n2_ranges):
        if len(n2_ranges) <= 1:
            return

        mulitlat_solver = lx.Project(mode='2D', solver='LSE_GC')
        n1_loc_sovler, _ = mulitlat_solver.add_target()

        for v in n2_ranges:
            n2 = v[0]
            n2_x, n2_y = self.location[n2][0], self.location[n2][1]
            mulitlat_solver.add_anchor(str(n2), (n2_x, n2_y))

        for v in n2_ranges:
            n2, d = v[0], v[1]
            if d<=0.:
                return
            n1_loc_sovler.add_measure(str(n2), d)
        try:
            mulitlat_solver.solve()
            if n1_loc_sovler.loc is None:
                return
            n1_x, n1_y = n1_loc_sovler.loc.x, n1_loc_sovler.loc.y
            #if n1_y < 0.:
            #    n1_y = - n1_y

            new_loc = [n1_x, abs(n1_y)]

            if(n1 in self.deltatime.keys()):
                delta = float(time.time() - self.deltatime[n1])
                self.deltatime[n1] = time.time()
                new_loc = self.run_kalman(n1, new_loc, self.imu_locations[n1], self.location[n1],delta)
            else:
                self.deltatime[n1] = time.time()

            #print("Line: 476 : "+ str(n1) + " ->" + str(new_loc))
            self._save_location(n1, new_loc[0], new_loc[1])
            self._inform_locations(n1, new_loc, self.floor_change_flag[n1])
            #self._temp_viz() #TODO: remove from final version
            #self._save_location(n1, n1_x, n1_y)

        except Exception as e:
            logging.error("loc-solver error: "+str(e))
        return

    def _find_same_floor_nodes(self, n1):
        node_list = []
        n1_floor = self.node_floor[n1]
        for n2, n2_floor in self.node_floor.items():
            if n1 == n2: continue
            if n2 in self.active_nodes and n1_floor == n2_floor:
                    node_list.append(n2)
        logging.debug("\t\t:same-floor-node of "+str(n1)+" is :"+str(node_list))
        return node_list

    def _find_link_alive_nodes(self, node_list, aliveness):
        link_alive_nodes = []
        for n1 in node_list:
            if not n1 in self.active_nodes:
                continue
            if not n1 in self.wifi_rx_ts.keys():
                continue
            if self._get_current_time() - self.wifi_rx_ts[n1] > aliveness:
                continue
            link_alive_nodes.append(n1)
        return link_alive_nodes

    def _find_nodes_with_fresh_location(self, node_list, freshness):
        fresh_loc_node_list = []
        for n1 in node_list:
            if n1 in self.location.keys() and self._get_current_time() - self.ts_location[n1] < freshness:
                    fresh_loc_node_list.append(n1)
        logging.debug("\t\t:nodes-with-fresh-loc :"+str(fresh_loc_node_list))
        return fresh_loc_node_list

    def _is_link_fresh(self, n1, n2, min_link_quality, link_quality_freshness):
        n1_indx, n2_indx = self.all_nodes.index(n1), self.all_nodes.index(n2)
        diff_ts = round(self._get_current_time() - self.ts_link_quality[n1_indx, n2_indx], 3)
        logging.debug("\t\t\t\tLINKQ-Freshness-test: nodes: " + str(n1) + ", " + str(n2) +\
                      " ts-diff: " + str(diff_ts) + "<" + str(link_quality_freshness) \
                      +" linkq-q " + str(round(self.link_quality[n1_indx, n2_indx], 3)) + ">" + str(min_link_quality))
        if diff_ts <= link_quality_freshness:
            if self.link_quality[n1_indx, n2_indx] >= min_link_quality:
                return True
        return False

    def _find_nodes_by_linkq(self, n1, node_list, min_link_quality, link_quality_freshness, max_ranging_nodes):
        self.cur_time = time.time() - self.init_time
        n1_indx = self.all_nodes.index(n1)
        nbr_linkq = {}
        nbr_list = []

        for v in self.anchor_nodes.values():
            origin_node, x_axis_node = v[0], v[1]
            if n1 == origin_node:
                    return [x_axis_node]
            elif n1 == x_axis_node:
                    return [origin_node]

        for n2 in node_list:
            n2_indx = self.all_nodes.index(n2)
            if n2 == n1:
                continue
            if self._is_link_fresh(n1, n2, min_link_quality, link_quality_freshness):
                    nbr_linkq[n2] = self.link_quality[n1_indx, n2_indx]

        if len(nbr_linkq) == 2:
            n1_floor = self.node_floor[n1]
            if self.anchor_nodes[n1_floor][0] in nbr_linkq.keys() and \
               self.anchor_nodes[n1_floor][1] in nbr_linkq.keys():
                logging.debug("\t\t: nodes selected for " + str(n1) + " for link-q is " + str(nbr_list))
                return self.anchor_nodes[n1_floor]

        elif len(nbr_linkq) >= 3:
            sorted_linkq_vals = Counter(nbr_linkq)
            highest_vals = sorted_linkq_vals.most_common(max_ranging_nodes)
            for val in highest_vals:
                nbr_list.append(val[0])
        logging.debug("\t\t: nodes selected for "+str(n1)+" for link-q is "+str(nbr_list))
        return nbr_list

    def _run_trackio_main_loop(self):
        '''
        algo:
        while true:
            select node in round robin
            if fixed node:
                update fixed node location ts
                if node has not tx'ed in min_link-q_ts/4 s, make it send dummy frame (range with dummy node)
            else: non-fixed node
                select neighbors:
                    select same floor nodes,
                    select nodes with fresh location
                    select nodes with min. linkq and min. linkq ts
                    select only 4 nodes if > 4
                    select 2 nodes if origin node + x_axis node
        '''
        self._initialize_trackio_algo()
        #self._init_temp_viz()
        #----------------------------------------------#
        self.wifi_link_aliveness = 30
        fixed_node_heartbeat_interval = 8
        location_freshness = 2*fixed_node_heartbeat_interval
        link_quality_freshness = 4*fixed_node_heartbeat_interval
        min_link_quality = 10.0
        max_ranging_nodes = 4
        while not self.stop_run:
            #-------------------------------------------------------------------------------#
            self._run_node_maintenance()
            #--------------------------------------------------------------------------------#
            sleep(0) #----thread yield
            cur_node_list =  sorted(self.active_nodes)
            data_collection_nodes = []
            #print("=============================================================================================")
            #print("debug: "+str(self._get_current_time())+": cur_node_list: "+str(cur_node_list))

            for n1 in cur_node_list:
                if not n1 in self.ts_tx.keys():
                    self.ts_tx[n1] = 0
                if n1 in self.fixed_node_loc.keys(): #anchor or fixed nodes
                    cur_ts = self._get_current_time()
                    ##print("\t\tdebug: "+str(cur_ts)+" fixed node "+str(n1)+" has wifi ts "+str(self.wifi_rx_ts[n1])+\
                    #      " and loc-ts "+str(self.ts_location[n1])+" and tx_ts: "+str(self.ts_tx[n1]))
                    if cur_ts - self.ts_location[n1] > location_freshness:
                        self._save_location(n1, self.fixed_node_loc[n1][0], self.fixed_node_loc[n1][1])
                    if  cur_ts - self.ts_tx[n1] > fixed_node_heartbeat_interval or \
                         cur_ts - self.wifi_rx_ts[n1] > self.wifi_link_aliveness:
                        self._send_discovery_frame([n1])

                else:
                    n1_same_floor_nodes =  self._find_same_floor_nodes(n1)
                    if not n1_same_floor_nodes:
                        #print('\t\tdebug '+str(self._get_current_time())+" : "+str(n1)+" has no same floor nodes")
                        data_collection_nodes.append(n1)
                        continue
                    # n1_same_floor_alive_nodes = self._find_link_alive_nodes(n1_same_floor_nodes,
                    #                                                         aliveness=self.wifi_link_aliveness)
                    # if not n1_same_floor_alive_nodes:
                    #     print('\t\tdebug ' + str(self._get_current_time()) + " : " + str(n1) + " same floor nodes are not wifi-alive: "+str(self.wifi_rx_ts))
                    #     data_collection_nodes.append(n1)
                    #     continue
                    n1_ranging_candidate_nodes = self._find_nodes_with_fresh_location(n1_same_floor_nodes,
                                                                                      freshness=location_freshness)
                    if not n1_ranging_candidate_nodes:
                        #print('\t\tdebug ' + str(self._get_current_time()) + " : " + str(n1) + " has no nbr with fresh links/locs "+str(self.ts_location))
                        data_collection_nodes.append(n1)
                        continue
                    n1_ranging_nodes = self._find_nodes_by_linkq(n1,
                                                                 node_list = n1_ranging_candidate_nodes,
                                                                 min_link_quality = min_link_quality,
                                                                 link_quality_freshness=link_quality_freshness,
                                                                 max_ranging_nodes=max_ranging_nodes)
                    if not n1_ranging_nodes:
                        data_collection_nodes.append(n1)
                        continue
                    self._query_ranges(n1, n1_ranging_candidate_nodes, freshness_for_ranging_skip=0)
                    n2_ranges_for_loc = []
                    cur_ts = self._get_current_time()
                    for n2 in n1_ranging_candidate_nodes:
                        n1_indx, n2_indx = self.all_nodes.index(n1), self.all_nodes.index(n2)
                        n2_d, n2_d_ts = self.edm[n1_indx, n2_indx] , cur_ts - self.ts_edm[n1_indx, n2_indx]
                        if  n2_d_ts<= 1. and n2_d> 0.:
                            n2_ranges_for_loc.append([n2, n2_d])
                    self._localize_node(n1, n2_ranges_for_loc)

            if data_collection_nodes:
                self._send_discovery_frame(sorted(data_collection_nodes))
        return

#-----------interfacing with subclasses--------------------#
    def trackio_stop(self):
        self.stop_run = True
        return

    def trackio_start(self):
        self.stop_run = False
        logging.info("Solver program started...")
        print("Solver program started...")
        self._run_trackio_main_loop()
        return

    def trackio_add_active_node(self, n1, n1_floor = None):
        with self.lock_add_delete_node:
            if not n1 in self.node_to_be_added:
                self.node_to_be_added.append([n1, n1_floor])
        return

    def trackio_change_node_floor(self, n1, n1_floor):
        with self.lock_add_delete_node:
            self.node_floor_change_request.append([n1, n1_floor])
        return

    def trackio_remove_active_node(self, n1):
        with self.lock_add_delete_node:
            if not n1 in self.node_to_be_deleted:
                self.node_to_be_deleted.append(n1)
        return

    def trackio_is_new_location_available(self):
        if self.nodes_with_location_update:
            return  True
        return False

    def trackio_get_locations(self):
        cur_locations = {}
        for cur_floor in self.anchor_nodes.keys():
            cur_locations[cur_floor] = {}
        nodes_with_new_locations = []

        with self.lock_nodes_with_location_update:
            nodes_with_new_locations = list(self.nodes_with_location_update)
            self.nodes_with_location_update = set()

        #print("debug:", self.locations, nodes_with_new_locations)
        for n1 in nodes_with_new_locations:
                n1_floor = self.node_floor[n1]
                cur_locations[n1_floor][n1] = (self.location[n1][0], self.location[n1][1])
        return cur_locations

    def trackio_get_current_active_nodes(self):
        return list(self.active_nodes)

    def trackio_restart_node_remotely(self, n1):
        with self.lock_remote_restart_node_list:
            self.remote_restart_node_list.append(n1)
        return


#--------------temporary vizualizer-----------------------#
'''
    def _init_temp_viz(self):
        C = ['g', 'g', 'b', 'b']
        xlim = 10
        ylim = 10
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-20, xlim + 5)
        self.ax.set_ylim(-20, ylim + 5)
        self.ax.grid()

        self.sp = self.ax.scatter([1, 2, 3, 4], [1, 2, 3, 4], s=100, c=C)
        self.ax.set_xticks(range(-5, xlim + 5, 5))
        self.ax.set_yticks(range(-5, ylim + 5, 5))
        plt.show(block=False)
        return

    def _temp_viz(self):
        X, Y, Labels = [], [], []
        for n1, n1_loc in self.location.items():
            Labels.append(n1)
            X.append(n1_loc[0])
            Y.append(n1_loc[1])
        self.sp.set_offsets(np.c_[X, Y])
        self.fig.canvas.draw()
        return
'''

## ------------ RKS added this -------- ##
'''
if __name__ == '__main__':

   node_ips = {}
   anchor_nodes = {}
   fixed_nodes = {}
   node_floor = {}
   active_nodes = []

   anchor_nodes[1] = [1,2]
   anchor_nodes[2] = [3,4]


   node_ips[1] = "138.15.20.90"
   node_ips[2] = "138.15.20.89"
   node_ips[3] = "138.15.20.91"
   node_ips[4] = "138.15.20.92"


   node_floor[1] = 1 
   node_floor[2] = 1 
   node_floor[3] = 1
   node_floor[4] = 1 


   fixed_nodes[1] = [0.0, 0.0]
   fixed_nodes[2] = [18.4, 0.0]
   fixed_nodes[3] = [0.0, 10.4]


   active_nodes = [1,2,3,4]

   controller = UDPSolver(
            udp_port = 12345,
            udp_timeout_milliseconds = 120,
            node_ips = node_ips,
            twr_tdma_slot_milliseconds = 8,
            anchor_nodes = anchor_nodes,
            fixed_node_loc = fixed_nodes,
            node_floor = node_floor,
            active_nodes = active_nodes,
            sea_leve_pressure_in_hg = 29.94,
            floor_change_threshold_meter = 2.8
   )
   controller.trackio_start()
'''
