"""AstarPlanner controller."""

from controller import Robot, Supervisor, Node
import math
import matplotlib.pyplot as plt
import numpy 


# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0) 
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

pos = robot.getFromDef("get_pos")
trans_field = pos.getField("translation")

radius = 0.066
distance_between_wheels = 0.16
velocity = 2
x_pos = []
z_pos = []

class AstarPlanner:
    
    def get_path(self, grid, init, goal, cost, delta, delta_name, heuristic):
        closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
        expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
        action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
        plan = []
        closed[init[0]][init[1]] = 1
        x = init[0]
        y = init[1]
        g = 0
        h = heuristic[x][y]
        f = g + h
        
        open = [[f, g, x, y]]
        found = False
        resign = False
        count = 0
        
        while found is False and resign is False:
            if (len(open) == 0):
                resign = True
                print("No valid path from Start to Goal !")
                
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                g = next[1]
                x = next[2]
                y = next[3]
                expand[x][y] = count
                count += 1
                 
                if x == goal[0] and y == goal[1]:
                    found = True
                    print("No. of search: ", g)
                    
                else:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                         
                        if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                            if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                                g2 = g + cost
                                h2 = heuristic[x2][y2]
                                f2 = g2 + h2
                                open.append([f2, g2, x2, y2])
                                closed[x2][y2] = 1
                                action[x2][y2] = i
        
                                 
        path = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
        x_ = goal[0]
        y_ = goal[1]
        path[x_][y_] = '*'
        while x_ != init[0] or y_ != init[1]:
            x2_ = x_ - delta[action[x_][y_]][0]
            y2_ = y_ - delta[action[x_][y_]][1] 
            path[x2_][y2_] = delta_name[action[x_][y_]]
            plan.append(delta_name[action[x_][y_]])
            x_ = x2_
            y_ = y2_
       
        print("expand List")
        for i in range(len(expand)):
            print(expand[i]) 
                
        print("----------- PATH ------------")
        for i in range(len(path)):
            print(path[i])
        
        print("----------- PLAN ------------")
        plan.reverse()    
        plan.append('*')
        print(plan)
        
        return path, plan
        
    def getYaw(self):
        imuValues = imu.getRollPitchYaw()
        imuValues = math.degrees(imuValues[2])
        return abs(imuValues) if imuValues < 0 else 360 - imuValues
    
    def diff(self,inp, set):
        tmp = abs(inp - set);
        diff = min(tmp, abs(360- tmp));
        if ((set + diff) != inp and (set - diff) != inp):
            if ((inp + diff) >= 360):
                return -diff;
            else:
                return diff;
        return (inp - set)
    
    def rot90_r(self):
        
        while robot.step(timestep) != -1:
            current = self.getYaw()
            right_motor.setVelocity(2)
            left_motor.setVelocity(-2)
             
            while abs(self.diff(current, self.getYaw())) < 90:
                robot.step(timestep)
            break;
    
    def rot90_l(self):
        while robot.step(timestep) != -1:
            current = self.getYaw()
            right_motor.setVelocity(-2)
            left_motor.setVelocity(2)
            while abs(self.diff(current, self.getYaw())) < 90:
                robot.step(timestep)
            break;
        
    def move(self, last, now):
        store = ''
        if now == '*':
            store = 'stop'
            #print("stop")
            right_motor.setVelocity(0.0)
            left_motor.setVelocity(0.0)
        elif last == now:
            pass
        elif ((last=='^' and now=='>') or (last=='>' and now=='v') or (last=='v' and now=='<') or (last=='<' and now=='^')):
            #print("right")
            self.rot90_l()
        else:
            #print("left")
            self.rot90_r()
        if store != 'stop':    
            velo = 0.165
            move_duration = 0.5/velo
            start_time = robot.getTime()
            #print(start_time)
            current_time = robot.getTime()
            right_motor.setVelocity(5.0)
            left_motor.setVelocity(5.0)
            #print("straight")
            while robot.step(timestep) != -1:
                if(current_time - start_time > move_duration):
                    values = trans_field.getSFVec3f()
                    x_pos.append(values[0])
                    z_pos.append(values[2])
                    break
                current_time = robot.getTime()
                
                # x_pos.append(values[0])
                # z_pos.append(values[2])
                
                
            right_motor.setVelocity(0.0)
            left_motor.setVelocity(0.0)
    
    def follow_path(self, path, plan):
        
        dummy = 'v'
        for i in range(len(plan)):
            if i==0:
                self.move(dummy, plan[i])
              
            else:
                self.move(plan[i-1], plan[i])



if __name__ == "__main__":
    
    grid = [[0, 1, 1, 0, 0, 1, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 1, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
            

            
    heuristic = [[18, 17, 16, 15, 14, 13, 12, 11, 10, 9],
                 [17, 16, 15, 14, 13, 12, 11, 10, 9, 8],
                 [16, 15, 14, 13, 12, 11, 10, 9, 8, 7],
                 [15, 14, 13, 12, 11, 10, 9, 8, 7, 6],
                 [14, 13, 12, 11, 10, 9, 8, 7, 6, 5],
                 [13, 12, 11, 10, 9, 8, 7, 6, 5, 4],
                 [12, 11, 10, 9, 8, 7, 6, 5, 4, 3],
                 [11, 10, 9, 8, 7, 6, 5, 4, 3, 2],
                 [10, 9, 8, 7, 6, 5, 4, 3, 2, 1],
                 [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]]
            
    init = [0, 0]
    goal = [len(grid)-1, len(grid[0])-1]
    cost = 1
    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],   # go down
             [0, 1]]   # go right
         
    delta_name = ['^', '<', 'v', '>']
    Robot1 = AstarPlanner()
    path, plan = Robot1.get_path(grid, init, goal, cost, delta, delta_name, heuristic)
    Robot1.follow_path(path, plan)
    
    t_waypoints = []
    f_waypoints = []
    pos = [-2.25, -2.25]
    t_waypoints.append(pos)
    for i in range(1, len(plan)):
        #t_waypoints.append(pos)
        if plan[i-1] == 'v':
            pos = [pos[0], pos[1]+0.5]
        elif plan[i-1] == '>':
            pos = [pos[0]+0.5, pos[1]]
        elif plan[i-1] == '<':
            pos = [pos[0] - 0.5, pos[1]]
        elif plan[i-1] == '^':
            pos = [pos[0], pos[1]-0.5]
        t_waypoints.append(pos)
    print("------ temporary waypoints -----")
    print(t_waypoints)    
    
    pos = [-2.25, -2.25]
    f_waypoints.append(pos)
    for i in range(1, len(plan)):
        if i == len(plan)-1:
            f_waypoints.append(t_waypoints[i])
            break
        elif plan[i] != plan[i-1]:
            f_waypoints.append(t_waypoints[i-1])
            f_waypoints.append(t_waypoints[i+1])
            
    
    print("------ final waypoints -----")
    print(f_waypoints)
    
    z_pos_neg = [ -x for x in z_pos]
    fig = plt.figure()
    fig.suptitle('X - Z position', fontsize=16)
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('center')
    ax.spines['bottom'].set_position('center')

# Eliminate upper and right axes
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

# Show ticks in the left and lower axes only
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    
    plt.plot(x_pos, z_pos_neg)
    plt.xlabel("X axis")
    plt.ylabel("Z axis")
    #plt.plot(x_pos, z_pos, 'o', color='red')
    plt.show()

