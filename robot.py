import numpy as np
import random
from maze import Maze
import sys

# Create a maze based on input argument on command line.
testmaze = Maze( str(sys.argv[1]) )

maps = {'u': [0,1], 'r':[1, 0], 'd': [0, -1], 'l': [-1, 0]}

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

        self.location = (0, 0)
        self.heading = 'up'
        self.maze_dim = maze_dim

    def reconstruct_path(self, parent, current_node):
        '''
        Find the path from the goal to the start point
        '''
        if parent[current_node]:
            p = self.reconstruct_path(parent, parent[current_node])
            p.append(current_node)
            return p
        else:
            return []
        
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
                print g_score
                print "\n The optimal path is:"
                return self.reconstruct_path(parent, goal);
                #return current_map


            #Look around its children at four directions
            for direction in ["u","r","d","l"]:
                #find one child
                current_node = self.Move(self.location, direction)
                #If this child has been searched or it was not accessible, pass
                if closeset and current_node in closeset or testmaze.is_permissible([self.location[0], self.location[1]], direction) == False:
                    continue;
                #If this child was not in close set and it is accessible, count the score
                g_score[current_node] = g_score[self.location] + 1.0
                h_score = self.DistToTarget(current_node)
                score = g_score[current_node]
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


    def UpdateMap(self, CurrentMap):
        '''
        Update map information based on sensors, including open way and walls
        '''
        for direction in ["u","r","d","l"]:
            if testmaze.is_permissible(self.location, direction):
                CurrentMap[self.location[0] + maps[direction][0], self.location[1] + maps[direction][1]] = 0

        return CurrentMap

    def Move(self, location, direction):
        '''
        Get the location of the next cell based on curent location and moving direction 
        '''
        x_label = location[0] + maps[direction][0]
        y_label = location[1] + maps[direction][1]

        return (x_label, y_label)

    def ReachTarget(self):
        '''
        Determine whether the robot reached the goal area
        '''

        return self.location in [(self.maze_dim/2, self.maze_dim/2),(self.maze_dim/2-1, self.maze_dim/2),
                                 (self.maze_dim/2, self.maze_dim/2-1),(self.maze_dim/2 - 1, self.maze_dim/2 - 1)]

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

    




if __name__ == '__main__':

    # Create a maze based on input argument on command line.
    testmaze = Maze( str(sys.argv[1]) )

    # Intitialize a robot; robot receives info about maze dimensions.
    testrobot = Robot(testmaze.dim)

    path = testrobot.AStar(testrobot.location,(testmaze.dim/2, testmaze.dim/2))

    print(path)

                        
        
        
