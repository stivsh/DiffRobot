import pygame
import math
from bluetooth_robot_interface import RobotInterface
from Graphics.graphical_objs import *

def transform_distanses(x):
    if x < 4 : x=0
    if x > 25 : x=0
    x = (x+8)*2
    if x>180 : x=None
    return x

def transform_coordinate(pos):
    return (float(pos[0])*2+350,-2*float(pos[1])+300)

draw_obst = True
pygame.init()

size=[700,600]
screen=pygame.display.set_mode(size)
pygame.display.set_caption("2WRobot")
backgraund = Backgraund(screen,size)
ship = Ship(screen)
obstacles = [ [False for x in range(700/10)] for x in range(600/10) ]

def nearest_obstacle_point(point):
    x = int(point[0])
    y = int(point[1])
    X = x/10 + 1 if x%10 > 10 else x/10
    Y = y/10 + 1 if y%10 > 10 else y/10
    if X > 700/10 -1 : X = 700/10 -1
    if Y > 600/10 -1: Y = 600/10 -1
    if X < 0 : X = 0
    if Y < 0 : Y = 0
    return (X,Y)

point = None
def process_obstacles():
    get_dy = lambda rad,dist: -1*dist*math.sin(rad)
    get_dx = lambda rad,dist: dist*math.cos(rad)
    get_endp = lambda rad,dist,spoint: (spoint[0]+get_dx(rad,dist),spoint[1]+get_dy(rad,dist))
    rad = rob_interface.angle*math.pi/180
    lrad = (rob_interface.angle+45)%360*math.pi/180
    rrad = (rob_interface.angle-45)%360*math.pi/180
    def process_obst_from_rad(rad,dist):
        end_dist = 216
        if dist is not None:
            end_dist = dist
        sdist = 0
        while sdist < end_dist:
            endp = get_endp(rad,sdist,transform_coordinate(rob_interface.pos))
            obst_point = nearest_obstacle_point(endp)
            obstacles[obst_point[1]][obst_point[0]] = False
            sdist = sdist + 1

        if dist is not None:
            endp = get_endp(rad,dist,transform_coordinate(rob_interface.pos))
            obst_point = nearest_obstacle_point(endp)
            obstacles[obst_point[1]][obst_point[0]] = True

    process_obst_from_rad(rad,transform_distanses(rob_interface.center_obst))
    process_obst_from_rad(rrad,transform_distanses(rob_interface.right_obst))
    process_obst_from_rad(lrad,transform_distanses(rob_interface.left_obst))

logWindow = LogWindow(screen,(200,470))
def log(str):
    logWindow.add_line(str)

rob_interface = RobotInterface(log)

def create_buttons_group(x,y):
    buttons = \
        [Button(screen,(x,y),'l',action=rob_interface.tern_left),Button(screen,(x+2*44,y),'r',action=rob_interface.tern_right),
            Button(screen,(x+44,y-34),'u',action=rob_interface.go_forward),Button(screen,(x+44,y+34),'d',action=rob_interface.go_back)]
    return buttons

buttons = create_buttons_group(25,500)

done=False
clock=pygame.time.Clock()

while done==False:
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = pygame.mouse.get_pos()
            point = None
            clicked = False
            for button in buttons:
                clicked = clicked or button.check_click(pos)
            if not clicked:
                rob_interface.go_to_point(((pos[0]-350)/2,-1*(pos[1]-300)/2))
                point = pos

        if (event.type == pygame.MOUSEBUTTONUP and event.button == 1) or event.type == pygame.KEYUP:
            if point is None: rob_interface.stop()

        if event.type == pygame.KEYDOWN:
            {pygame.K_UP:rob_interface.go_forward,pygame.K_DOWN:rob_interface.go_back,
                pygame.K_RIGHT:rob_interface.tern_right,
                pygame.K_LEFT:rob_interface.tern_left
            }.get(event.key,rob_interface.stop)()

    backgraund.paint()

    font = pygame.font.Font(None, 25)
    text = font.render("mouse pos:%s:%s"%((pygame.mouse.get_pos()[0]-350)/2,(pygame.mouse.get_pos()[1]-300)/-2),True,blue)
    screen.blit(text, [20,10])
    pos = transform_coordinate(rob_interface.pos)
    text = font.render("pos:%s:%s, ang:%s"%(pos[0],pos[1],rob_interface.angle),True,blue)
    screen.blit(text, [470,10])

    if rob_interface.distination is not None:
        text = font.render("destination:%s:%s "%(rob_interface.distination[0],rob_interface.distination[1]),True,blue)
        screen.blit(text, [470,30])

    if point is not None:
        pygame.draw.circle(screen, green, point , 8)
    ship.paint( pos ,rob_interface.angle+90)
    process_obstacles()
    #draw obstacles
    if draw_obst:
        for linx in range(len(obstacles)):
            for rinx in range(len(obstacles[linx])):
                if obstacles[linx][rinx]:
                    pygame.draw.circle(screen, red, (rinx*10,linx*10) , 10)

    for button in buttons: button.paint()
    logWindow.paint()
    pygame.display.flip()
    clock.tick(20)
rob_interface.stop_communication()
