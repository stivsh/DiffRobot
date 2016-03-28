import re
import pygame
import bluetooth
from Graphics.graphical_objs import *

pygame.init()

size=[700,600]
screen=pygame.display.set_mode(size)
pygame.display.set_caption("2WRobot")
backgraund = Backgraund(screen,size)
ship = Ship(screen)

point = None

def go_forward():
    global point
    interf.send("F")
    point = None

def go_back():
    global point
    interf.send("B")
    point = None

def tern_left():
    global point
    interf.send("L")
    point = None

def tern_right():
    global point
    interf.send("R")
    point = None

def stop():
    global point
    interf.send("S")
    point = None

def go_to_point(pos):
    interf.send("GOTO:%s:%s;"%( (pos[0]-350)/2,-1*(pos[1]-300)/2 ))


def create_buttons_group(x,y):
    buttons = \
        [Button(screen,(x,y),'l',action=tern_left),Button(screen,(x+2*44,y),'r',action=tern_right),
            Button(screen,(x+44,y-34),'u',action=go_forward),Button(screen,(x+44,y+34),'d',action=go_back)]
    return buttons

buttons = create_buttons_group(25,500)
logWindow = LogWindow(screen,(200,470))

def log(str):
    logWindow.add_line(str)

class RobotInterface:
    def __init__(self):
        self.angle=1
        self.pos=(0,0)
        self.data=""
        self.dist = None
        sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        sock.connect(('20:15:10:12:33:55', 1))
        self.sock = sock
        try:
            while len(sock.recv(1024)) > 10 :pass
        except:
            pass

    def send(self,comand):
        self.dist = None
        try:
            self.sock.send("+%s-"%comand)
            log("< "+ comand)
        except:
            pass

    def read_raw_data(self):
        try:
            return self.sock.recv(1024)
        except:
            return ""

    def read_data(self):
        data = self.read_raw_data()
        if len(data) == 0: return
        self.data += data
        pos_readed = False
        angle_readed = False
        new_pos = (0,0)
        new_angle = 0
        self.data = self.data.replace("\r","").replace("\n","")
        print self.data

        m = re.search(r"log:GOING TO X:([-0-9\.]+) Y:([-0-9\.]+);", self.data)
        if m:
            self.dist = (int(m.groups(0)[0]),int(m.groups(0)[1]))


        m = re.search(r'log:(.*);', self.data)
        if m:
            log("> "+m.groups(0)[0])

        m = re.search(r'.*pos:([-0-9]+):([-0-9]+)eangl:([0-9\.]+)e.*', self.data)
        if m:
            new_pos = (float(m.groups(0)[0]),-1*float(m.groups(0)[1]))
            new_angle = float(m.groups(0)[2])
            self.data = ""
            self.pos = new_pos
            self.angle = new_angle

    def __del__(self):
            self.sock.close()

interf = RobotInterface()

done=False
clock=pygame.time.Clock()

while done==False:
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = pygame.mouse.get_pos()
            clicked = False
            for button in buttons:
                clicked = clicked or button.check_click(pos)
            if not clicked:
                go_to_point(pos)
                point = pos

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if point is None: stop()

    interf.read_data()
    backgraund.paint()

    font = pygame.font.Font(None, 25)
    text = font.render("pos:%s:%s, ang:%s"%(interf.pos[0],interf.pos[1],interf.angle),True,blue)
    screen.blit(text, [470,10])

    if interf.dist is not None:
        text = font.render("destination:%s:%s "%(interf.dist[0],interf.dist[1]),True,blue)
        screen.blit(text, [470,30])

    if point is not None:
        pygame.draw.circle(screen, red, point , 8)
    ship.paint( (2*interf.pos[0]+350,2*interf.pos[1]+300) ,interf.angle+90)
    for button in buttons: button.paint()
    logWindow.paint()
    pygame.display.flip()
    clock.tick(20)
