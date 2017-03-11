import numpy as np
import random
from maze import Maze
import sys

# Create a maze based on input argument on command line.
testmaze = Maze( str(sys.argv[1]) )

maps = {'u': [0,1], 'r':[1, 0], 'd': [0, -1], 'l': [-1, 0]}

reverse_maps = {(0,1):"u", (1,0):"r", (0,-1):"d", (-1,0): "l"}

dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        # for search algorithm
        self.location = (0, 0)
        self.heading = 'up'
        self.maze_dim = maze_dim
        # for actual movement
        self.temp = (0,0)
        # for the first and second run
        self.flag = 0
        self.score = 0
        # record score map
        self.map = np.full((self.maze_dim, self.maze_dim),1000)
        # for retreiving path
        self.path = []

    def reconstruct_path(self, parent, current_node):
        '''
        Find the path from the goal to the start point
        '''
        #if parent[current_node]:
        if current_node in parent.keys():
            p = self.reconstruct_path(parent, parent[current_node])
            p.append(current_node)
            return p
        else:
            return [(0,0)]
        
    def AStar(self, start, goal):

        #If all children of one node have been traversed, put the node into close set
        closeset = []
        #openset = [[i,j] for i in range(self.maze_dim) for j in range(self.maze_dim)]
        maze_dim = self.maze_dim

        #Initial the map with total scores in it
        current_map = self.InitialMap()
        current_map[(0,0)] = self.maze_dim

        #nodes in activeset have been reached and their children have not been searched.
        activeset = [start]

        #nodes with the minimum score
        min_node = start

        #g score and h score for A* algorithm
        g_score = self.InitialMap()
        g_score[(0,0)] = 0
        h_score = self.DistToTarget(self.location)

        #record parents for each nodes
        parent = {}

        #iterate all nodes
        while activeset:
            #find nodes with the minimum scores in the activeset
            min_score = 10000
            for node in activeset:
                if current_map[node] < min_score:
                    min_node = node
                    min_score = current_map[node]
            self.location = min_node 

            #If the node is the goal, return the path and break
            if self.ReachTarget():
                return self.reconstruct_path(parent, goal), np.array(g_score);
                #return current_map


            #Look around its children at four directions
            for direction in ["u","r","d","l"]:
                #find one child
                current_node = self.Move(self.location, direction, 1)
                #If this child has been searched or it was not accessible, pass
                if closeset and current_node in closeset or testmaze.is_permissible([self.location[0], self.location[1]], direction) == False:
                    continue;
                #If this child was not in close set and it is accessible, count the score
                g_score[current_node] = g_score[self.location] + 1.0
                h_score = self.DistToTarget(current_node)
                score = g_score[current_node] + h_score
                #If the child was not in activeset, it had not been searched, so the score should be the minimum
                if activeset and current_node not in activeset:
                    activeset.append(current_node)
                    better = True
                #if the node is in active set and the new score is smaller than previous number, replace it by the new one.
                elif score < current_map[current_node]:
                    better = True
                #Otherwise, do not change ethe score
                else:
                    better = False

                if better is True:
                    parent[current_node] = self.location
                    current_map[current_node] = score

            #Child of this node will be searched. Place it to closeset and remove it from activeset
            closeset.append(self.location)
            activeset.remove(self.location)

    def Dijkstra(self, start, goal, maps):

        #If all children of one node have been traversed, put the node into close set
        closeset = []
        #openset = [[i,j] for i in range(self.maze_dim) for j in range(self.maze_dim)]
        maze_dim = self.maze_dim

        #Initial the map with total scores in it
        current_map = maps

        #nodes in activeset have been reached and their children have not been searched.
        activeset = [start]

        #nodes with the minimum score
        min_node = start

        #record parents for each nodes
        parent = {}

        #iterate all nodes
        while activeset:
            #find nodes with the minimum scores in the activeset
            min_score = 10000
            for node in activeset:
                if current_map[node] < min_score:
                    min_node = node
                    min_score = current_map[node]
            self.location = min_node

            #If the node is the goal, return the path and break
            if self.ReachTarget():
                #return  parent, np.array(current_map);
                return self.reconstruct_path(parent, goal), np.array(current_map)

            #Determine the current heading direction
            if self.location == (0,0):
                head_direction = 'up'
            else:
                previous_location = parent[self.location]
                head_direction = self.heading2(previous_location, self.location)

            #Get the sensors value
            sensing = [testmaze.dist_to_wall([self.location[0],self.location[1]], heading)
                       for heading in dir_sensors[head_direction]]
                
            '''
            Look around its children
            '''
            # three directions. If 0, there should be a wall. Otherwise, get steps up to 3.
            for index, sensor in enumerate(sensing):
                if sensor == 0:
                    continue
                elif sensor > 3:
                    step = 3
                else:
                    step = sensor

                #get the direction it is heading for each child
                direction = dir_sensors[head_direction][index]
                #update score
                self.score = current_map[self.location] + 1

                #allocate values to them
                for step in range(step):
                    #find one child
                    current_node = self.Move(self.location, direction, step+1)
                    #no way to move to closeset
                    if current_node in closeset and closeset:
                        break
                    #Avoid entering into area that are blocked by the exploration step
                    elif maps[current_node] == 1000:
                        closeset.append(current_node)
                        break
                    #elif current_node in activeset and current_map[current_node] < self.score:
                     #   break
                    # if the child is in active set and it is the neighbor just next to its location, avoid this direction 
                    elif current_node in activeset and step == 0:
                        break
                    # if the child is not in active set, definitely it should be updated with current score. Record parents.
                    elif current_node not in activeset and activeset:
                        activeset.append(current_node)
                        parent[current_node] = self.location
                        current_map[current_node] = self.score
                    # if the child is in active set but it is not the right one next to its location, update it. 
                    elif current_node in activeset and current_map[current_node] >= self.score and step != 0:
                        parent[current_node] = self.location
                        current_map[current_node] = self.score

            #Child of this node have been searched. Place it to closeset and remove it from activeset
            closeset.append(self.location)
            activeset.remove(self.location)

    def create_dict(self, path):
        '''
        turn the path into dictionary. The previous node is the key, and the next node is the value
        '''
        new_dict = {}
        for index, item in enumerate(path):
            if index == 0:
                store_item = item
                continue
            new_dict[store_item] = item
            store_item = item

        return new_dict

    def direction(self, start, end, heading):
        '''
        Get the relative rotation based on starting point and ending point and its heading direction
        '''
        change = (end[0]-start[0], end[1]-start[1])
        #self.heading = reverse_maps[change]
        if change[0] == 0:
            if change[1] > 0:
                self.heading = "u"
            elif change[1] < 0:
                self.heading = "d"
        elif change[1] == 0:
            if change[0] > 0:
                self.heading = "r"
            elif change[0] < 0:
                self.heading = "l"
            
        direction = self.heading
        for index, direct in enumerate(dir_sensors[heading]):
            if direct == direction:
                num = index
        if num == 0:
            rotation = -90
        elif num == 1:
            rotation = 0
        elif num == 2:
            rotation = 90

        return rotation

    def heading2(self, start, end):
        '''
        Get the relative direction based on starting point and ending point
        '''
        change = (end[0]-start[0], end[1]-start[1])
        #self.heading = reverse_maps[change]
        if change[0] == 0:
            if change[1] > 0:
                heading = "up"
            elif change[1] < 0:
                heading = "down"
        elif change[1] == 0:
            if change[0] > 0:
                heading = "right"
            elif change[0] < 0:
                heading = "left"

        return heading

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        # start the first run
        if self.flag == 0:
            # get the map and path when it runs the first time
            if self.temp == (0,0):
                self.path, self.map = self.AStar((0,0), (self.maze_dim/2 - 1, self.maze_dim/2))
                self.location = (0,0)
                path1_dict = self.create_dict(self.path)
                rotation = 0
                movement = 1
                self.temp = path1_dict[self.temp]
            # for remaining steps
            else:
                path1_dict = self.create_dict(self.path)
                self.location = (0,0)
                previous_cell = self.temp
                if previous_cell == (self.maze_dim/2 - 1, self.maze_dim/2):
                    rotation = 'Reset'
                    movement = 'Reset'
                    self.temp = (0,0)
                    self.flag = 1
                    self.heading = "up"
                else:
                    self.temp = path1_dict[previous_cell]
                    rotation = self.direction(previous_cell, self.temp, self.heading)
                    movement = 1
        # start the second run
        else:
            if self.temp == (0,0):
                self.path, self.map = self.Dijkstra((0,0), (self.maze_dim/2 - 1, self.maze_dim/2), self.map)
                path2_dict = self.create_dict(self.path)
                rotation = 0
                movement = abs(path2_dict[self.temp][1] + path2_dict[self.temp][0])
                self.temp = path2_dict[self.temp]
            else:
                path2_dict = self.create_dict(self.path)
                previous_cell = self.temp
                if previous_cell == (self.maze_dim/2 - 1, self.maze_dim/2):
                    rotation = 'Reset'
                    movement = 'Reset'
                    self.temp = (0,0)
                else:
                    self.temp = path2_dict[previous_cell]
                    rotation = self.direction(previous_cell, self.temp, self.heading)
                    movement = abs(self.temp[0] + self.temp[1] - previous_cell[0] - previous_cell[1])                
        
        return rotation, movement
    
    def ActualMove(self, rotation, movement):
        '''
        Let robot move according to heading, rotation and movement pace
        Rotation:
        0 : left turn
        1: forward
        2: right turn
        '''

        self.heading = dir_sensors[self.heading][rotation]
        for i in range(movement):
            self.location += maps[self.heading]

        return self


    def Move(self, location, direction, step):
        '''
        Get the location of the next cell based on curent location and moving direction 
        '''
        x_label = location[0] + maps[direction][0] * step
        y_label = location[1] + maps[direction][1] * step

        return (x_label, y_label)

    def ReachTarget(self):
        '''
        Determine whether the robot reached the goal area
        '''

        return self.location in [(self.maze_dim/2-1, self.maze_dim/2)]
                                 #(self.maze_dim/2, self.maze_dim/2-1),(self.maze_dim/2 - 1, self.maze_dim/2 - 1)]

    def DistToTarget(self, location):
        '''
        Calculate the Manhattan distance between current location and target
        '''
        return self.maze_dim - location[1] - location[0]

    def InitialMap(self):
        '''
        Create an empty map with 100 in each box
        '''
        UpdatedMap = np.full((self.maze_dim, self.maze_dim),1000)

        return UpdatedMap

    




#if __name__ == '__main__':

    # Create a maze based on input argument on command line.
    #testmaze = Maze( str(sys.argv[1]) )

    # Intitialize a robot; robot receives info about maze dimensions.
    #testrobot = Robot(testmaze.dim)

    #path = testrobot.Dijkstra(testrobot.location,(testmaze.dim/2, testmaze.dim/2))

    #print(path)

                        
        
        
