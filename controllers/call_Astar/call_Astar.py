from AStar_Path import *
from controller import Robot, Supervisor, Node
import math
import matplotlib.pyplot as plt

robot = Supervisor()
#timestep = int(robot.getBasicTimeStep())
timestep = 32
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

pos = robot.getFromDef("get_pos")
trans_field = pos.getField("translation")

radius_of_wheel = 0.033
dist_between_wheels = 0.16

radius_of_curve = 0.5

x_pos = []
z_pos = []
x_waypoint = []
z_waypoint = []

Robot1 = AstarPlanner()
plan, f_waypoints = Robot1.get_plan_and_waypoints()

#print(f_waypoints)

x_waypoint.append(f_waypoints[0][0][0])
z_waypoint.append(f_waypoints[0][0][1])
T = [0, 8, 12, 16, 20, 24, 28, 34, 38, 44]
for i in range(len(f_waypoints)-1):
    if f_waypoints[i][1] != f_waypoints[i+1][1]:
        linear_distance = math.sqrt(pow(f_waypoints[i][0][0]-f_waypoints[i+1][0][0],2) + pow(f_waypoints[i][0][1]-f_waypoints[i+1][0][1],2))
        move_duration = T[i+1] - T[i]
        velocity_m = linear_distance/move_duration
        velocity_rw = velocity_m/radius_of_wheel
        velocity_lw = velocity_rw
        start_time = robot.getTime()
        current_time = robot.getTime()
        right_motor.setVelocity(velocity_rw)
        left_motor.setVelocity(velocity_lw)
        while robot.step(timestep) != -1:
            if(current_time - start_time > move_duration):
                values = trans_field.getSFVec3f()
                x_waypoint.append(values[0])
                z_waypoint.append(values[2])
                break
            current_time = robot.getTime()
            values = trans_field.getSFVec3f()
            x_pos.append(values[0])
            z_pos.append(values[2])
        right_motor.setVelocity(0)
        left_motor.setVelocity(0)
    else:
        circular_distance = (2*math.pi*radius_of_curve)/4
        move_duration = T[i+1] - T[i]
        linear_velocity = circular_distance/move_duration
        angular_velocity = linear_velocity/radius_of_curve
        velo_r = (2*linear_velocity + angular_velocity*dist_between_wheels)/(radius_of_wheel*2)
        velo_l = velo_r - (angular_velocity*dist_between_wheels)/radius_of_wheel
        print(velo_r, velo_l)
        if plan[f_waypoints[i][1]-1]=='v' and plan[f_waypoints[i][1]]=='<' or plan[f_waypoints[i][1]-1]=='^' and plan[f_waypoints[i][1]]=='>' or plan[f_waypoints[i][1]-1]=='>' and plan[f_waypoints[i][1]]=='v' or plan[f_waypoints[i][1]-1]=='<' and plan[f_waypoints[i][1]]=='^':
            start_time = robot.getTime()
            current_time = robot.getTime()
            right_motor.setVelocity(velo_l)
            left_motor.setVelocity(velo_r)
            while robot.step(timestep) != -1:
                if(current_time - start_time > move_duration):
                    values = trans_field.getSFVec3f()
                    x_waypoint.append(values[0])
                    z_waypoint.append(values[2])
                    break
                current_time = robot.getTime()
                values = trans_field.getSFVec3f()
                x_pos.append(values[0])
                z_pos.append(values[2])
            right_motor.setVelocity(0)
            left_motor.setVelocity(0)    
        else:
            dummy = velo_r
            velo_r = velo_l
            velo_l = dummy 
            start_time = robot.getTime()
            current_time = robot.getTime()
            right_motor.setVelocity(velo_l)
            left_motor.setVelocity(velo_r)
            while robot.step(timestep) != -1:
                if(current_time - start_time > move_duration):
                    values = trans_field.getSFVec3f()
                    x_waypoint.append(values[0])
                    z_waypoint.append(values[2])
                    break
                current_time = robot.getTime()
                values = trans_field.getSFVec3f()
                x_pos.append(values[0])
                z_pos.append(values[2])
            right_motor.setVelocity(0)
            left_motor.setVelocity(0)
            

z_pos_neg = [ -x for x in z_pos]   
z_waypoint_neg = [ -x for x in z_waypoint]         
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
plt.plot(x_waypoint, z_waypoint_neg, 'o', color='red')
plt.xlabel("X axis")
plt.ylabel("Z axis")
#plt.plot(x_pos, z_pos, 'o', color='red')
plt.show()        