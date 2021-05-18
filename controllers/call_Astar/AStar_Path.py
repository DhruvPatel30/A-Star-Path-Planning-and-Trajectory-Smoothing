"""AstarPlanner controller."""
class AstarPlanner:

    def __init__(self):
        self.grid = [[0, 1, 1, 0, 0, 1, 0, 0, 0, 0],
                    [0, 1, 1, 0, 0, 1, 0, 0, 0, 0],
                    [0, 1, 1, 1, 0, 1, 0, 0, 0, 0],
                    [0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
                    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
                    
        self.heuristic = [[18, 17, 16, 15, 14, 13, 12, 11, 10, 9],
                         [17, 16, 15, 14, 13, 12, 11, 10, 9, 8],
                         [16, 15, 14, 13, 12, 11, 10, 9, 8, 7],
                         [15, 14, 13, 12, 11, 10, 9, 8, 7, 6],
                         [14, 13, 12, 11, 10, 9, 8, 7, 6, 5],
                         [13, 12, 11, 10, 9, 8, 7, 6, 5, 4],
                         [12, 11, 10, 9, 8, 7, 6, 5, 4, 3],
                         [11, 10, 9, 8, 7, 6, 5, 4, 3, 2],
                         [10, 9, 8, 7, 6, 5, 4, 3, 2, 1],
                         [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]]
                    
        self.init = [0, 0]
        self.goal = [len(self.grid)-1, len(self.grid[0])-1]
        self.cost = 1
        self.delta = [[-1, 0],  # go up
                     [0, -1],  # go left
                     [1, 0],   # go down
                     [0, 1]]   # go right
             
        self.delta_name = ['^', '<', 'v', '>']
        
    
    def get_plan_and_waypoints(self):
        closed = [[0 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        expand = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        action = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        plan = []
        closed[self.init[0]][self.init[1]] = 1
        x = self.init[0]
        y = self.init[1]
        g = 0
        h = self.heuristic[x][y]
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
                 
                if x == self.goal[0] and y == self.goal[1]:
                    found = True
                    print("No. of search: ", g)
                    
                else:
                    for i in range(len(self.delta)):
                        x2 = x + self.delta[i][0]
                        y2 = y + self.delta[i][1]
                         
                        if x2 >= 0 and x2 < len(self.grid) and y2 >=0 and y2 < len(self.grid[0]):
                            if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                                g2 = g + self.cost
                                h2 = self.heuristic[x2][y2]
                                f2 = g2 + h2
                                open.append([f2, g2, x2, y2])
                                closed[x2][y2] = 1
                                action[x2][y2] = i
        
                                 
        path = [[' ' for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        x_ = self.goal[0]
        y_ = self.goal[1]
        path[x_][y_] = '*'
        while x_ != self.init[0] or y_ != self.init[1]:
            x2_ = x_ - self.delta[action[x_][y_]][0]
            y2_ = y_ - self.delta[action[x_][y_]][1] 
            path[x2_][y2_] = self.delta_name[action[x_][y_]]
            plan.append(self.delta_name[action[x_][y_]])
            x_ = x2_
            y_ = y2_
                
        print("----------- PATH ------------")
        for i in range(len(path)):
            print(path[i])
        
        #print("----------- PLAN ------------")
        plan.reverse()    
        plan.append('*')
        #print(plan)
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
        # print("------ temporary waypoints -----")
        # print(t_waypoints)    
        
        pos = [-2.25, -2.25]
        f_waypoints.append([pos,0])
        for i in range(1, len(plan)):
            if i == len(plan)-1:
                f_waypoints.append([t_waypoints[i],i])
                break
            elif plan[i] != plan[i-1]:
                
                f_waypoints.append([t_waypoints[i-1],i])
                f_waypoints.append([t_waypoints[i+1],i])
                
        
        # print("------ final waypoints -----")
        # print(f_waypoints)
        
        return plan, f_waypoints
   

#if __name__ == "__main__":
    
    #Robot1 = AstarPlanner()
    #path, plan, f_waypoints = Robot1.get_path_and_waypoints()
    
    
    
    

