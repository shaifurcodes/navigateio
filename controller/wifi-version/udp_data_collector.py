import logging
import  socket
import  time
from subprocess import  check_output, CalledProcessError, check_call

logging.basicConfig(
    filename='./udp_data_collector.log',
    filemode='w',
    level=logging.ERROR, #TODO: logging level error
    format='%(asctime)s.%(msecs)03d, %(threadName)-10s: %(message)s',
    datefmt= '%H:%M:%S' #'%Y-%m-%d %H:%M:%S'
)

class UDPDataCollector(object):
    def __init__(self,
                     udp_com_time_msec,
                     twr_frame_time_msec,
                     udp_port
                 ):
        self.init_time = time.time()
        self.all_node_ips = {}
        self.udp_com_time_msec = udp_com_time_msec
        self.twr_slot_time_msec = int(twr_frame_time_msec)
        self.udp_port = int(udp_port)

        self.cmd_counter = {}

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("", self.udp_port))
        return

    def get_data_collection_latency_sec(self, nbr_count):
        return (2.*self.udp_com_time_msec+(2.*nbr_count+2.)*self.twr_slot_time_msec)/1000.

    def set_node_ips(self, node_ips):
        self.all_node_ips = dict(node_ips)
        self.cmd_counter = {}
        for i in self.all_node_ips.keys():
            self.cmd_counter[i] = 1
        return

    def start_node_program(self, node_id):
        if node_id in self.all_node_ips.keys():
            node_ip = self.all_node_ips[node_id]
            rx_timeout_msec = 2 * self.udp_com_time_msec
            self.run_udp_query("start", node_id, " ", node_ip, rx_timeout_msec)
        return

    def run_udp_query(self, cmd_str, node, cmd_arg, node_ip, rx_timeout_msec):
        '''

        :param cmd_str:
        :param node:
        :param cmd_arg:
        :return:
        '''
        # if node == 10:
        #     print("debug trap")
        resp_msg = ''
        unexpected_resp_msg = {}
        try:
            cur_cmd_counter = -1

            if node in self.cmd_counter.keys():
                cur_cmd_counter = self.cmd_counter[node]
            tx_data = str(cur_cmd_counter) + " : " + cmd_str \
                      + " " + str(node) + \
                      " = " + cmd_arg
            logging.debug("\ttx data: " + tx_data)
            _ = self.udp_sock.sendto(str.encode(tx_data), (node_ip, self.udp_port))
            deadline_ts = time.time()+rx_timeout_msec/1000.
            while(time.time()<deadline_ts):
                remaining_time_sec = deadline_ts-time.time()
                if remaining_time_sec <=0.:
                    break
                self.udp_sock.settimeout(remaining_time_sec)
                rx_data, src_addr = self.udp_sock.recvfrom(1024)
                rx_data = rx_data.decode('ascii').strip()
                logging.debug("\t\tUDP-RX-DATA: " + str(rx_data))

                #if not(src_addr[1] == self.udp_port) or \
                if        len(rx_data) == 0 or \
                        not(':' in rx_data) or \
                        not('=' in rx_data):
                    continue

                resp_cmd_seq_no, rx_data = rx_data.split(':')
                resp_hdr, resp_payload = rx_data.split('=')
                resp_cmd_str, resp_node = resp_hdr.split()
                resp_cmd_seq_no, resp_cmd_str, resp_node = int(resp_cmd_seq_no), \
                                                           resp_cmd_str.strip(), \
                                                           int(resp_node)

                if src_addr[0] in node_ip and \
                        resp_cmd_seq_no == cur_cmd_counter and \
                        cmd_str in resp_cmd_str and (resp_node == node or node == -1):
                            resp_msg = str(resp_payload.strip())
                            break
                else:
                    unexpected_resp_msg[resp_node] = [ str(resp_cmd_str), str(resp_payload.strip())]
        except Exception as e:
            logging.debug("\ttimeout for tx " + str(cur_cmd_counter)+" Exception-msg:"+str(e))
        finally:
            if node in self.cmd_counter.keys():
                self.cmd_counter[node] = (self.cmd_counter[node] + 1) % 256
        return resp_msg, unexpected_resp_msg

    def process_link_info(self, link_info_str):
        link_quality = {}
        link_info_str = link_info_str.strip()
        if  link_info_str:
            if ',' in link_info_str:
                link_info_str_pairs = link_info_str.split(',')
            else:
                link_info_str_pairs = [link_info_str]
            for cur_link_info_str in link_info_str_pairs:
                vals = cur_link_info_str.split()
                if len(vals) != 3:
                    continue
                link_quality[ int(vals[0])] = [ int(vals[1]), float(vals[2]) ]
        return link_quality

    def process_heading_acceleration(self, heading_a_str):
        heading, a = float('inf'), float('inf')
        if ',' in heading_a_str:
            heading_a = heading_a_str.split(',')
            heading, a = float(heading_a[0]), float(heading_a[1])
        return heading, a

    def process_range_response_string(self, resp_msg):
        range_vals = {}
        link_quality = {}
        #heading = float('inf')
        #acceleration = float('inf')
        altitude = float('inf')
        imu_location = [0.0,0.0]

        if len(resp_msg) > 0:
            range_msg = ''
            if ';' in resp_msg:
                vals =resp_msg.split(';')
                if not len(vals) >= 4:
                    #return range_vals, link_quality, heading, acceleration, altitude
                    return range_vals, link_quality, imu_location, altitude
                #range_msg, link_info_str, heading_a_str, altitude = vals[0], vals[1], vals[2], vals[3]
                range_msg, link_info_str, imu_location_str, altitude = vals[0], vals[1], vals[2], vals[3]
                link_quality = self.process_link_info(link_info_str)
                #heading, acceleration = self.process_heading_acceleration(heading_a_str)
                imu_location = [float(i) for i in imu_location_str.split(',')]
                altitude = float(altitude)

            node_range_pairs = []
            if ',' in range_msg:
                node_range_pairs = range_msg.split(',')
            else:
                node_range_pairs = [range_msg]

            for val in node_range_pairs:
                if len(val) == 0 or val.isspace():
                    continue
                i,d= val.split()
                range_vals[ int(i)] = round(float(d)/100. , 2 )
        #return range_vals, link_quality, heading, acceleration, altitude
        return range_vals, link_quality, imu_location, altitude

    def query_node_id(self, ip_address):
        node = -1
        cmd_str = 'send-node-id'
        cmd_arg = ''
        node_ip = ip_address
        rx_timeout_msec = 2*self.udp_com_time_msec
        resp_msg, _ = self.run_udp_query(cmd_str=cmd_str,
                                                 node=node,
                                                 cmd_arg=cmd_arg,
                                                 node_ip=node_ip,
                                                 rx_timeout_msec= rx_timeout_msec)
        return resp_msg

    def inform_location(self, node, location, flag):
        if(flag == True):
            cmd_str = 'force_loc'
        else:
            cmd_str = 'pos'

        cmd_arg = str(location[0]) + ',' + str(location[1])
        node_ip = self.all_node_ips[node]
        rx_timeout_msec = self.udp_com_time_msec
        resp_msg, _ = self.run_udp_query(cmd_str=cmd_str,
                                                 node=node,
                                                 cmd_arg=cmd_arg,
                                                 node_ip=node_ip,
                                                 rx_timeout_msec= rx_timeout_msec)
    
        return

    def query_range( self, node, nlist ):
        '''

        :param node:
        :param nlist:
        :return:
        '''
        # if node == 10:
        #     print("debug trap")
        range_vals = {}
        link_info = {}
        #heading = {}
        #acceleration = {}
        altitude = {}
        imu_location = {}

        #find the node ip if any
        if node not in self.all_node_ips.keys():
            logging.debug("\tnode ip not found")
            logging.debug('END query range of node ' + str(node))
            #return range_vals, link_info, heading, acceleration, altitude
            return range_vals, link_info, imu_location, altitude
        logging.debug('BEGIN query range of node ' + str(node) + ' w/ nodes ' + str(nlist) + " cmd# " + str(
            self.cmd_counter[node]))
        node_ip = self.all_node_ips[node]
        nlist_str = ''
        for i in nlist:
            nlist_str += " "+str(i)

        cmd_str, node, cmd_arg = 'range', node, nlist_str
        rx_timeout_msec = 2 * self.udp_com_time_msec \
                          + (2*len(nlist)+2)*self.twr_slot_time_msec
        resp_msg, unexpected_resp_msg = self.run_udp_query(cmd_str=cmd_str,
                                      node=node,
                                      cmd_arg=cmd_arg,
                                      node_ip=node_ip,
                                      rx_timeout_msec=rx_timeout_msec)
        # if unexpected_resp_msg:
        #     print("debug: unexpected msg!!")
        #range_vals[node], link_info[node], heading_val, acceleration_val, altitude_val = \
        #    self.process_range_response_string(resp_msg=resp_msg) #
        #if 0<=heading_val < float('inf'):
        #    heading[node] = heading_val
        #if acceleration_val != float('inf'):
        #    acceleration[node] = acceleration_val
        range_vals[node], link_info[node], imu_location[node], altitude_val = \
            self.process_range_response_string(resp_msg=resp_msg) #

        if altitude_val != float('inf'):
            altitude[node] = altitude_val

        for prev_node, prev_cmd_msgs in unexpected_resp_msg.items():
            prev_cmd, prev_msg = prev_cmd_msgs[0], prev_cmd_msgs[1]
            if not 'range' in prev_cmd:
                continue
            #range_vals[prev_node], link_info[prev_node], heading_val, acceleration_val, altitude_val = \
            #    self.process_range_response_string(resp_msg=prev_msg)
            #if 0 <= heading_val < float('inf'):
            #    heading[prev_node] = heading_val
            #if acceleration_val != float('inf'):
            #    acceleration[prev_node] = acceleration_val
            range_vals[prev_node], link_info[prev_node], imu_location[node], altitude_val = \
                self.process_range_response_string(resp_msg=prev_msg)
            if altitude_val != float('inf'):
                altitude[prev_node] = altitude_val

        logging.debug('END query range of node '+str(node))
        #return range_vals, link_info, heading, acceleration, altitude
        return range_vals, link_info, imu_location, altitude

    def send_params(self, node, paramlist):
        if node not in self.all_node_ips.keys():
            return
        node_ip = self.all_node_ips[node]
        cmd_str = 'param'
        rx_timeout_msec = 120 #self.udp_com_time_msec #TODO: tune this delay carefully
        _, _ = self.run_udp_query(cmd_str=cmd_str,
                                  node=node,
                                  cmd_arg=paramlist,
                                  node_ip=node_ip,
                                  rx_timeout_msec= rx_timeout_msec)
        return

    def get_node_ip(self, node):
        if node in self.all_node_ips.keys():
            return self.all_node_ips[node]
        return '0.0.0.0'

    def get_node_id_from_ip(self, lookup_ip):
        for node, ip in self.all_node_ips.items():
                if lookup_ip in ip:
                    return  node
        return -1

    def remote_restart_node_program(self, node_no):
        username = 'pi'
        password = 'Labs12#$'
        ip = self.all_node_ips[node_no]
        script_file = './scripts/restart_trackio_service.sh'
        try:
            check_call([script_file, username, password, ip])
        except Exception as e:
            logging.error('failed to start trackio service in node: '+str(e))
        return
