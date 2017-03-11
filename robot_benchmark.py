import numpy as np
import random
from maze import Maze
import sys

# Create a maze based on input argument on command line.
testmaze = Maze( str(sys.argv[1]) )
maps = {'u': [0,1], 'r':[1, 0], 'd': [0, -1], 'l': [-1, 0]}
maps2 = {(0,1):"u", (1,0): "r", (0,-1): "l", (-1,0): "l"}
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
reverse = {'u':'d', 'd':'u', 'l':'r', 'r':'l'}

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
        self.map = np.full((self.maze_dim, self.maze_dim),1)
        self.flag = 0

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
        #for the first run
        if self.flag == 0:
            # if reaching target, set location to original point
            if self.ReachTarget():
                self.flag = 1
                self.location = (0,0)
                return 'Reset','Reset'
            valid_dir = []
            #save current location and heading
            previous_loc = self.location
            previous_head = self.heading
            #get accessible directions
            for index in range(3):
                if sensors[index] == 0:
                    continue
                else:
                    valid_dir.append(dir_sensors[self.heading][index])
            #if the node is surrounded by three walls, step back
            if len(valid_dir) == 0:
                movement = -1
                rotation = 0
                # remember to mark it as blocked
                self.map[self.location] = 1000
                self.location = self.Move(self.location, self.heading, -1)
            else:
                # if there is only one accessible direction
                if len(valid_dir) == 1:
                    direction2 = valid_dir[0]
                    current_node = self.Move(self.location, direction2, 1)
                    # if the only accessible node has been blocked, step back
                    if self.map[current_node] == 1000:
                        movement = -1
                        self.location = self.Move(self.location, self.heading, movement)
                        rotation = 0
                    # If the only accessible node is reachable, go ahead
                    else:
                        movement = 1
                        self.heading = direction2
                        self.location = self.Move(self.location, self.heading, movement)
                        rotation = self.direction(previous_loc, self.location, previous_head)
                #if there are multiple choices, then randomly select one direction
                else:
                    i = random.randint(0, len(valid_dir)-1)
                    direction2 = valid_dir[i]
                    current_node = self.Move(self.location, direction2, 1)
                    movement = 1
                    # keeping selecting until one accessible node has been selected
                    while self.map[current_node] == 1000:
                        valid_dir.remove(direction2)
                        i = random.randint(0, len(valid_dir)-1)
                        direction2 = valid_dir[i]
                        current_node = self.Move(self.location, direction2, 1)
                    change = (current_node[0] - self.location[0], current_node[1] - self.location[1])
                    self.heading = direction2
                    self.location = current_node
                    rotation = self.direction(previous_loc, self.location, previous_head)
        #for the second run
        else:
            #make sure that the first two special nodes have not been blocked
            self.map[(0,0)] = 1
            self.map[(0,1)] = 1
            # if reaching target, set location to original point
            if self.ReachTarget():
                self.flag = 1
                self.location = (0,0)
                return 'Reset','Reset'
            valid_dir = []
            #save current location and heading
            previous_loc = self.location
            previous_head = self.heading
            #get accessible directions
            for index in range(3):
                if sensors[index] == 0:
                    continue
                else:
                    valid_dir.append(dir_sensors[self.heading][index])
            #if the node is surrounded by three walls, step back
            if len(valid_dir) == 0:
                movement = -1
                rotation = 0
                # remember to mark it as blocked
                self.map[self.location] = 1000
                self.location = self.Move(self.location, self.heading, -1)
            else:
                # if there is only one accessible direction
                if len(valid_dir) == 1:
                    direction2 = valid_dir[0]
                    current_node = self.Move(self.location, direction2, 1)
                    # if the only accessible node has been blocked, step back
                    if self.map[current_node] == 1000:
                        movement = -1
                        self.location = self.Move(self.location, self.heading, movement)
                        rotation = 0
                    # If the only accessible node is reachable, go ahead
                    else:
                        movement = 1
                        self.heading = direction2
                        self.location = self.Move(self.location, self.heading, movement)
                        rotation = self.direction(previous_loc, self.location, previous_head)
                #if there are multiple choices, then randomly select one direction
                else:
                    i = random.randint(0, len(valid_dir)-1)
                    direction2 = valid_dir[i]
                    current_node = self.Move(self.location, direction2, 1)
                    movement = 1
                    # keeping selecting until one accessible node has been selected
                    while self.map[current_node] == 1000:
                        valid_dir.remove(direction2)
                        i = random.randint(0, len(valid_dir)-1)
                        direction2 = valid_dir[i]
                        current_node = self.Move(self.location, direction2, 1)
                    change = (current_node[0] - self.location[0], current_node[1] - self.location[1])
                    self.heading = direction2
                    self.location = current_node
                    rotation = self.direction(previous_loc, self.location, previous_head)

        return rotation, movement

    def Move(self, location, direction, step):
        '''
        Get the location of the next cell based on curent location and moving direction 
        '''
        x_label = location[0] + maps[direction][0] * step
        y_label = location[1] + maps[direction][1] * step

        return (x_label, y_label)

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
                d = index
        if d == 0:
            rotation = -90
        elif d == 1:
            rotation = 0
        elif d == 2:
            rotation = 90

        return rotation
    
    def ReachTarget(self):
        '''
        Determine whether the robot reached the goal area
        '''

        return self.location in [(self.maze_dim/2-1, self.maze_dim/2)]
