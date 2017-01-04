import bluetooth
import re
import time
from threading import Thread, Lock

class RobotInterface:
    def __init__(self, log = None):
        self._log = log
        self._angle=1
        self._pos=(0,0)
        self._data_for_sending = ""
        self._input_data=""
        self._distination = None
        self._left_obst = None
        self._center_obst = None
        self._right_obst = None
        self._stop_loop = False
        self.lock = Lock()
        if self._log: self._log("> "+ "starting communication")
        self.start_communication_loop()

    def __getattr__(self, name):
        self.lock.acquire()
        avalible_attrs = {"angle":self._angle,"pos":self._pos,"distination": self._distination,
            "left_obst":self._left_obst,"center_obst":self._center_obst,
            "right_obst":self._right_obst}
        if name not in avalible_attrs:
            self.lock.release()
            raise AttributeError("no such attr %s"%name)
        self.lock.release()
        return avalible_attrs[name]

    def start_communication_loop(self):
        self.loot_thread = Thread(target = self.robot_commutication_loop)
        self.loot_thread.start()

    def robot_commutication_loop(self):
        while not self._stop_loop:
            try:
                if self._log: self._log("> "+ "connecting....")
                sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
                sock.connect(('20:15:10:12:33:55', 1))
                if self._log: self._log("> "+ "connected")

                while not self._stop_loop:
                    data = sock.recv(1024)
                    if len(data):
                        try:
                            self.lock.acquire()
                            self.process_raw_chunk(data)
                        finally:
                            self.lock.release()

                    if len(self._data_for_sending):
                        self._distination = None
                        sock.send("+%s-"%self._data_for_sending)
                        if self._log: self._log("< "+ self._data_for_sending)
                        self._data_for_sending = ""
                sock.close()
            except Exception as ex:
                if self._log: self._log("> " + str(ex))

    def set_log_func(self,log):
        self._log = log

    def process_raw_chunk(self,new_chunk):
        self._input_data += new_chunk.replace("\r","").replace("\n","")

        m = re.search(r"log:GOING TO X:([-0-9\.]+) Y:([-0-9\.]+);", self._input_data)
        if m:
            self._distination = (int(m.groups(0)[0]),int(m.groups(0)[1]))

        m = re.search(r'log:(.*);', self._input_data)
        if m:
            if self._log: self._log("> "+m.groups(0)[0])

        m = re.search(r'pos:([-0-9]+):([-0-9]+)eangl:([0-9\.]+)eLDist:([0-9\.]+)eCDist:([0-9\.]+)eRDist:([0-9\.]+)e', self._input_data)
        if m:
            self._pos = (float(m.groups(0)[0]),float(m.groups(0)[1]))
            self._angle = float(m.groups(0)[2])
            self._left_obst = ( int(float(m.groups(0)[3])) )
            self._center_obst = ( int(float(m.groups(0)[4])) )
            self._right_obst = ( int(float(m.groups(0)[5])) )
            self._input_data = ""

    def go_forward(self):
        self._data_for_sending = "F"

    def go_back(self):
        self._data_for_sending = "B"

    def tern_left(self):
        self._data_for_sending = "L"

    def tern_right(self):
        self._data_for_sending = "R"

    def stop(self):
        self._data_for_sending = "S"

    def go_to_point(self,pos):
        self._data_for_sending = "GOTO:%s:%s;"%pos

    def stop_communication(self):
        self._stop_loop = True
        self.loot_thread.join()

    def __del__(self):
        if self.loot_thread.is_alive():
            self._stop_loop = True
            self.loot_thread.join()
