from platform import node
import sys,random,math,pygame
from pygame.locals import *
import math

class Node():
    cost=0
    parent=None
    def __init__(self,x,y):
        self.x=x
        self.y=y
    
def orinetation_check(a,b,c):
    return (b[1]-a[1])*(c[0]-a[0])>(c[1]-a[1])*(b[0]-a[0])

def intersect(nodea,nodeb,obstacle_space):
    a=(nodea.x,nodea.y)
    b=(nodeb.x,nodeb.y)
    for i in obstacle_space:
        rect_i=(i[0],i[1],i[0]+i[2],i[1]+i[3])
        rect_top_left=(rect_i[0],rect_i[1])
        rect_bottom_left=(rect_i[0],rect_i[3])
        rect_top_right=(rect_i[2],rect_i[1])
        rect_bottom_right=(rect_i[2],rect_i[3])
        case_1=orinetation_check(a,rect_top_left,rect_bottom_left)!=orinetation_check(b,rect_top_left,rect_bottom_left) and orinetation_check(a,b,rect_top_left)!=orinetation_check(a,b,rect_bottom_left)
        case_2=orinetation_check(a,rect_top_left,rect_top_right)!=orinetation_check(b,rect_top_left,rect_top_right) and orinetation_check(a,b,rect_top_left)!=orinetation_check(a,b,rect_top_right)
        case_3=orinetation_check(a,rect_top_right,rect_bottom_right)!=orinetation_check(b,rect_top_right,rect_bottom_right) and orinetation_check(a,b,rect_top_right)!=orinetation_check(a,b,rect_bottom_right)
        case_4=orinetation_check(a,rect_bottom_left,rect_bottom_right)!=orinetation_check(b,rect_bottom_left,rect_bottom_right) and orinetation_check(a,b,rect_bottom_left)!=orinetation_check(a,b,rect_bottom_right)
        if case_1==False and case_2==False and case_3==False and case_4==False:
            continue
        else:
            return False
    return True

XDIM=400
YDIM=200
WINDOW_SIZE=[XDIM,YDIM]
stepsize=7
iterations=2000
search_radius=15
obstacle_space=[(75,65,15,15),(75,115,15,15),(75,165,15,15),(255,65,15,15),(255,115,15,15),(255,165,15,15),(165,65,15,15),(165,140,15,15)]

def draw_obstacle_space(pygame,screen):
    for i in obstacle_space:
        pygame.draw.rect(screen,(0,0,255),i)

def euclidean_dist(point1,point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def steer(point1,point2):
    if (euclidean_dist(point1,point2))<stepsize:
        return point2
    else:
        theta=math.atan2(point2[1]-point1[1],point2[0]-point1[0])
        return point1[0]+stepsize*math.cos(theta),point1[1]+stepsize*math.sin(theta)
    
def get_parent(nearest_node,new_node,nodes):
    for i in nodes:
        if intersect(i,new_node,obstacle_space) and euclidean_dist([i.x,i.y],[new_node.x,new_node.y])<search_radius and i.cost+euclidean_dist([i.x,i.y],[new_node.x,new_node.y])<nearest_node.cost+euclidean_dist([nearest_node.x,nearest_node.y],[new_node.x,new_node.y]):
            nearest_node=i
        new_node.cost=nearest_node.cost+euclidean_dist([nearest_node.x,nearest_node.y],[new_node.x,new_node.y])
        new_node.parent=nearest_node
        return new_node,nearest_node
    
def rewire(nodes,new_node,pygame,screen):
    for i in nodes:
        if intersect(i,new_node,obstacle_space) and i!=new_node.parent and euclidean_dist([i.x,i.y],[new_node.x,new_node.y])<search_radius and new_node.cost+euclidean_dist([i.x,i.y],[new_node.x,new_node.y])<i.cost:
            pygame.draw.line(screen,(255,255,255),[i.x,i.y],[i.parent.x,i.parent.y])
            i.parent=new_node
            i.cost=new_node.cost+euclidean_dist([i.x,i.y],[new_node.x,new_node.y])
            pygame.draw.line(screen,(0,0,0),[i.x,i.y],[new_node.x,new_node.y])
    return nodes

def back_track(start,goal,nodes,pygame,screen):
    node=nodes[0]
    for i in nodes:
        if euclidean_dist([i.x,i.y],[goal.x,goal.y])<euclidean_dist([node.x,node.y],[goal.x,goal.y]):
            node=i
    while node!=start:
        pygame.draw.line(screen,(255,0,0),[node.x,node.y],[node.parent.x,node.parent.y],5)
        node=node.parent

def main():
    pygame.init()
    screen=pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption('RRTstar')
    pygame.display.flip()
    screen.fill((255,255,255))
    draw_obstacle_space(pygame,screen)
    nodes=[]
    nodes.append(Node(5,5))
    start=nodes[0]
    goal=Node(300,100)
    for i in range(iterations):
        random_node=Node(random.random()*XDIM,random.random()*YDIM)
        nearest_node=nodes[0]
        for j in nodes:
            if euclidean_dist([j.x,j.y],[random_node.x,random_node.y])<euclidean_dist([nearest_node.x,nearest_node.y],[random_node.x,random_node.y]):
                nearest_node=j
        sampled_node=steer([nearest_node.x,nearest_node.y],[random_node.x,random_node.y])
        new_node=Node(sampled_node[0],sampled_node[1])
        if intersect(nearest_node,new_node,obstacle_space):
            [new_node,nearest_node]=get_parent(nearest_node,new_node,nodes)
            nodes.append(new_node)
            pygame.draw.line(screen,(0,0,0),[nearest_node.x,nearest_node.y],[new_node.x,new_node.y])
            nodes=rewire(nodes,new_node,pygame,screen)
            pygame.display.update()
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Qutting...")
    back_track(start,goal,nodes,pygame,screen)
    pygame.display.update()
        
main()
running=True
while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False