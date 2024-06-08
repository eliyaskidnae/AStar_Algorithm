# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
from random import random , randint  

# class 
class Node:
    def __init__(self , x , y  ):
        self.x = x # x-position
        self.y = y # y-postion
        self.id = 0 # vertices id 
        self.f_score = float('inf') # initialize as inifinity 
        self.g_score = float('inf') # initialize as inifinity 
        self.parent  = None
       
    def calcu_huristic(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance   
    def get_distance(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    def find_nearest_node(self,nodes):
        min_distance = float('inf')
        nearest_node = None
        for node in nodes:
            distance = self.get_distance(node)
            if(distance < min_distance):
                nearest_node = node
                min_distance = distance
            
        return nearest_node
    def find_nodes_with_in_radius(self,nodes,radius):
        # print(nodes)
        nodes_with_in_radius =[]
        for node in nodes:
            
            distance = self.get_distance(node)
            # print(distance)
            if(distance <= radius):
              nodes_with_in_radius.append(node)
        return nodes_with_in_radius
    def find_costopt_nearest_node(self,nodes ,radius):
        

        # filter a nodes with a given radius 
        nodes_with_in_radius = self.find_nodes_with_in_radius(nodes,radius)

        if(not nodes_with_in_radius): # return the nearest node 
            return self.find_nearest_node(nodes)
        else : 
            # find a node with minimum cost from start to the given node
            optimam_node = self.find_nearest_node(nodes) 
            min_cost = float("inf")
            # print(nodes_with_in_radius)

            for node in  nodes_with_in_radius:
                
                cost = node.g_score + self.get_distance(node)
                if(cost < min_cost):
                    optimam_node = node
                    min_cost = cost
            
            return optimam_node 
    # checks if a node already exist in the list of nodes
    def is_already_exist(self , list):
        for node in list:
            if(node.x == self.x and node.y == self.y):
                return True
        return False
    # def __str__(self):
    #     return str(self.id)
    def __eq__(self, other):
        # print("ob",self.x, self.y)
        return self.x == other.x and self.y == other.y   
class AStar:
    def __init__(self, goal,start ,grid_map):
        self.goal = goal
        self.start = start
       
        self.grid_map = grid_map
        self.open_list = []
        self.closed_list = []
        self.path =[]
        self.height ,self.width = grid_map.shape[0] , grid_map.shape[1]
    
    # returns the a node with smallest g_score from openlist
    def lowest_f_score(self):
            min_node = self.open_list[0]
            for i in range(1,len(self.open_list)):
                if(self.open_list[i].f_score < min_node.f_score):
                    min_node = self.open_list[i]
            return min_node    
        
    # reconstract a path from closed list
    def reconstruct_path(self):

        self.path = [self.goal]
        current = self.goal
        length = len(self.closed_list)
        parent=  current.parent   
        while current != self.start:
            
            current =  current.parent   
            self.path.append(current)  
        self.path[::-1]
        return self.get_path()
    
    def get_path(self):

        path  = [ (node.x, node.y) for node in self.path ]
        return  path 

    # chick if node is in side the map area 
    def is_onboard( self , node ):       
        if( 0<=node.x <self.height and 0<= node.y < self.width):
            return True        
        else:
            return False 
    # check wither the node is fee of obstacle
    def isValid(self,node): 
        if self.is_onboard(node) and self.grid_map[node.x,node.y] != 1 and  self.grid_map[node.x,node.y] == 0 :
            return True
   
    # getes child nodes of the current node 
    def get_childrens(self,current):
        motions = [(0,-1),(-1, 0),(0, 1),(1, 0)]
                #    ,(1,1),(-1,-1),(1,-1),(-1,1) ] # 8-point connectivity 
        childrens= []
        for mot in motions:
            x = current.x + mot[0]
            y = current.y + mot[1]
            if self.isValid(Node(x,y)):
                childrens.append(Node(x,y))
            
        return childrens
    
    # compute a star path
    def compute_path(self):
          
            self.open_list.append(self.start)
            # onitiializa open list wit the start node 
            self.start.g_score = 0
            self.start.h_score = self.start.calcu_huristic(self.goal)
            self.start.f_score = self.start.g_score + self.start.h_score
            self.start.parent  = None # set the parent of the start node as None 


            while len(self.open_list) > 0 :       
                # get a node with lowest f_score from open list
                current  = self.lowest_f_score()   # 
                self.closed_list.append(current)
                self.open_list.remove(current)

                if current == self.goal:  # if goal is found reconstract the path  
                    self.goal = current
                    
                    return self.reconstruct_path()  
                
                childrens = self.get_childrens(current)  # gt valid child nodes
                # expand the node to childs 
                for child in childrens:
                    tentative_g = current.g_score + child.get_distance(current)
                    if(tentative_g < child.g_score and not child.is_already_exist(self.closed_list) ):
                        
                        child.g_score = tentative_g
                        child.f_score = tentative_g + child.calcu_huristic(self.goal)
                        child.parent = current

                        if(not child.is_already_exist(self.open_list) ):
                            # add to the open list
                            self.open_list.append(child)
            return self.path
   
    def plot(self):
        # plot  both the map and the path 
        x_path  = [v.x for v in self.path ]
        y_path  = [v.y for v in self.path ]
        
        plt.matshow(self.grid_map)  
        plt.plot(self.start.y,self.start.x,color='green', marker = '*')
        plt.plot(self.goal.y,self.goal.x,color='green', marker = '*')
         
        plt.plot(y_path,x_path,color='green')
        
        plt.show()
 
def main():
    
    # Load grid map 
    image = Image.open("./gridMap/map0.png").convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255 
    # binarize the image 
    grid_map[grid_map >  0.5] = 1
    grid_map[grid_map <= 0.5] = 0 
    # Invert colors to make 0 -> free and 1 -> occupied 
    grid_map = (grid_map * -1) + 1 # Show grid map 
    
    start = Node(10,10)   
    # print("start", start)
    goal =  Node(90,70)
    astar = AStar(start,goal,grid_map )
    
    path = astar.compute_path()
    print("path", path)
    print("path_distance", astar.goal.f_score)
    astar.plot()
    
main()
