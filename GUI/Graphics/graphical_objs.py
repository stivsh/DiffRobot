__author__ = 'stiv'

black = ( 0, 0, 0)
white = ( 255, 255, 255)
blue = (49,49,79)
yellow = (255,255,0)
green = ( 0, 255, 0)
red = ( 255, 0, 0)

import pygame

#grafical objects
class Explosion:
    load_pics=False
    pics=[]
    def LoadPics(self):
        image=pygame.image.load("./Graphics/img/expl.bmp")
        Explosion.pics=[]
        for row in range(2):
            for col in range(4-row):
                Explosion.pics.append(image.subsurface((48*(col),48*row,48,48)))
        for inx in range(len(Explosion.pics)):
            Explosion.pics[inx].set_colorkey((0x45,0x4e,0x5b))
        Explosion.big_pics=list(Explosion.pics)
        for inx in range(len(Explosion.pics)):
            Explosion.pics[inx]=pygame.transform.scale(Explosion.pics[inx],(25,25))
        Explosion.load_pics=True

    def __init__(self,screen,pos,big=False):
        self.Big=big
        if big:self.pos=[pos[0]-24,pos[1]-24]
        else:self.pos=[pos[0]-12,pos[1]-12]
        self.phase=0
        self.end=False
        self.screen=screen
        if not Explosion.load_pics:
            Explosion.LoadPics(self)

    def paint(self):
        if not self.end:

            pic=Explosion.big_pics[self.phase/2] if self.Big else Explosion.pics[self.phase/2]
            self.screen.blit(pic,self.pos)
            self.phase+=1
            if self.phase/2==len(Explosion.pics):self.end=True

def do_nothing():
    pass

class Button:
    def __init__(self,screen,pos,img,action=do_nothing):
        self.LoadPics(img)
        self.screen=screen
        self.pos=pos
        self.action=action

    def LoadPics(self,img):
        pic=pygame.image.load("./Graphics/img/"+img+".png")
        pic.set_colorkey(white)
        self.pic=pic

    def check_click(self,pos):
        if ((self.pos[0]+22-pos[0])**2 + (self.pos[1]+22-pos[1])**2)**(0.5) < 22:
            self.action()
            return True
        return False

    def paint(self):
        self.screen.blit(self.pic,self.pos)
        #pygame.draw.rect(self.screen,red,[pos[0]-25,pos[1]-25,50,50],2)
        #pygame.draw.rect(self.screen,red,[pos[0]-2,pos[1]-2,4,4],2)

class LogWindow:
    def __init__(self,screen,pos):
        self.num = 0
        self.pos=pos
        self.screen=screen
        self.lines = []
    def paint(self):
        pygame.draw.rect(self.screen,black,[self.pos[0],self.pos[1],480,110],0)

        font = pygame.font.Font(None, 25)
        for inx,l in enumerate(self.lines):
            text = font.render("  "+l,True,green)
            self.screen.blit(text, [self.pos[0],self.pos[1]+inx*17])

    def add_line(self,line):
        if len(self.lines) and line == self.lines[-1]: return
        self.lines.append("%s %s"%(self.num,  line))
        self.num += 1
        self.lines = self.lines[-5:]


class Ship:
    load_pics=False
    def LoadPics(self):
        pic=pygame.image.load("./Graphics/img/ship.bmp")
        pic.set_colorkey((0x45,0x4e,0x5b))
        Ship.pic=pic
        Ship.load_pics=True

    def __init__(self,screen):
        if not Ship.load_pics:
            Ship.LoadPics(self)
        self.end=False
        self.screen=screen
    def paint(self,pos,angle):
        pic=pygame.transform.rotate(Ship.pic,angle)
        pos1=map(lambda x,x2:x2-x/2,pic.get_size(),pos)
        self.screen.blit(pic,pos1)
        #pygame.draw.rect(self.screen,red,[pos[0]-25,pos[1]-25,50,50],2)
        #pygame.draw.rect(self.screen,red,[pos[0]-2,pos[1]-2,4,4],2)


class Shot:
    load_pics=False
    def LoadPics(self):
        pic=pygame.image.load("./Graphics/img/expl.bmp").subsurface(0,0,48,48)
        pic.set_colorkey((0x45,0x4e,0x5b))
        Shot.pic=pygame.transform.scale(pic,(25,25))
        Shot.load_pics=True

    def __init__(self,screen):
        if not Shot.load_pics:
            Shot.LoadPics(self)
        self.screen=screen
    def paint(self,pos):
        self.screen.blit(Shot.pic,pos)

class Backgraund:
    def __init__(self,screen,size):
        background_image=pygame.image.load("./Graphics/img/back.jpg").convert()
        self.background_image=pygame.transform.scale(background_image,size)
        self.screen=screen
    def paint(self):
        self.screen.blit(self.background_image,[0,0])
