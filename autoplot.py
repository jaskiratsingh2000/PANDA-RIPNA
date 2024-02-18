import numpy as np
import matplotlib.pyplot as plt
from numpy import e 
import math
import time
import pandas as pd
import os
#global position matrix
#global current velocity matrix
data_Array = np.zeros((7,13))

def reset():
    os.remove('time.csv')
    os.remove('priority.csv')
    os.remove('avg.csv')

def initialise_pos(n):
    start = []
    goal = []

    for i in range(1,n//4+1):
        one = (np.cos((spread + np.random.normal(0,noise))*(i/(n//4))), np.sin((spread + np.random.normal(0,noise))*(i/(n//4))))
        two = (-np.cos((spread + np.random.normal(0,noise))*(i/(n//4))), np.sin((spread + np.random.normal(0,noise))*(i/(n//4))))
        three = (-np.cos((spread + np.random.normal(0,noise))*(i/(n//4))), -np.sin((spread + np.random.normal(0,noise))*(i/(n//4))))
        four = (np.cos((spread + np.random.normal(0,noise))*(i/(n//4))), -np.sin((spread + np.random.normal(0,noise))*(i/(n//4))))
        
        start += [one, two, three, four]
        goal += [two, one, four, three]

    
    start, goal = radius*np.array(start), 1.2*radius*np.array(goal)

    return start, goal

def generator(n):
    u = np.zeros((n,2))
    v = np.zeros((n,2))
    vmax = maxv*np.ones(n)
    

    pos, goal = initialise_pos(n)
    #priority = np.ones(n)


    a = np.zeros((n,2))

    completed = np.zeros(n)
    clip = np.zeros(n)

    return u, v, pos, goal, a, completed, clip, vmax

reset()
rows = []
for ix, n in enumerate([4,8,12,16,20,24]):
    for priority_type in [False, "Gaussian"]:
        for seed in range(100):

            start=time.time()
            i=0
            from hyperparameters import noise, radius, delt, pert, b, sensor_range#, seed
            np.random.seed(seed)
            print("random seed:",seed)
            
            maxv = 3

            from hyperparameters import spread, noise

            u, v, pos, goal, a, completed, clip, vmax = generator(n)


            if priority_type=="Gaussian":
                priority = np.random.normal(3,1, n)
            elif priority_type == "Uniform":
                priority = np.random.uniform(1,6, n)
            else:
                priority = 3*np.ones(n)

            in_collision = [[i] for i in priority]

            completion = [0]*n
            priority_file = open("priority.csv",'a')
            priority_file.write(", ".join([str(x) for x in priority])+'\n')
            priority_file.close()

            # exec_file = open('exec.csv', 'a')
            """
            Now we calculate the forces to be applied

            attractive: a(1-e**(-bd^2))[unit dir vector]  a:max acceleration, b:reducing spread, d:distance to goal

            repulsive: ae**(-bd**2)[unit dir vector]  a:max repulsive force, b:repulsive spread, d:distance to obstacle
            """
            check = np.ones(n)
            min_dist = np.inf*np.ones((n,n))
            avg_dist = np.zeros(n)

            """
            two definitions for reached goal(completed trip), one checks wether distance to goal is less than 0.5

            """
            def reached_goal(i):
                global pos, goal, r
                if np.linalg.norm(pos[i]-goal[i]) <= radius/5:
                    #print(f"UAV {i} reached goal. Priority:{priority[i]}")
                    return True
                else:
                    return False
                # if np.linalg.norm(pos[i]) > r:
                #     return True
                

            def collision(i,j):#return true if collision eminenent between j and i. collision threshold = 0.1
                
                t = -np.dot(pos[i]-pos[j],v[i]-v[j])/(np.dot(v[i]-v[j], v[i]-v[j])+0.00001)

                if t<0:
                    return False
                else:
                    s_quared = np.dot(pos[i] + v[i]*t - pos[j] - v[j]*t, pos[i] + v[i]*t - pos[j] - v[j]*t) 
                    dist = np.linalg.norm(pos[i]-pos[j])

                    if np.sqrt(s_quared)<0.5 and dist < sensor_range:
                        in_collision[i].append(priority[j])
                        return True
                    else:
                        return False


            def clip_v(n):
                global v, clip
                #print("----------------")
                for i in range(n):
                    
                    if clip[i]:
                        vmax[i] = maxv*priority[i]/max(in_collision[i])
                    else:
                        vmax[i] = maxv
                    #vmax[i]=maxv
                    if np.linalg.norm(v[i])>vmax[i]:
                        v[i] = vmax[i]*v[i]/np.linalg.norm(v[i])

                    
                    #print(i,v[i], np.linalg.norm(v[i]), vmax[i])


            """
            Function to pertube angle of vector by a slight amount randomly. draw from N(0, pi/64)
            let's pertube the whole v matrix
            """


            def pertube(v):
                
                for i, vel in enumerate(v):
                    x = vel[0]
                    y = vel[1]

                    delta_theta = np.random.normal(0,np.pi/2**10.5)
                    theta = np.arctan2(y, x)

                    # Perturb the angle by a small amount
                    theta_perturbed = theta + delta_theta

                    # Calculate the perturbed vector components using the perturbed angle
                    x_perturbed = np.cos(theta_perturbed) * np.sqrt(x**2 + y**2)
                    y_perturbed = np.sin(theta_perturbed) * np.sqrt(x**2 + y**2)

                    v[i] = np.array([x_perturbed, y_perturbed])

            """
            test:
            v - pertube(v) != 0
            """

            """
            rotate vector v=x,y by angle theta
            """

            def rotate_vector(v, theta):
                x, y = v[0], v[1]
                # convert theta to radians
                #theta = math.radians(theta)
                
                # calculate sine and cosine of theta
                cos_theta = np.cos(theta)
                sin_theta = np.sin(theta)
                
                # apply rotation formula
                x_new = x * cos_theta - y * sin_theta
                y_new = x * sin_theta + y * cos_theta
                
                return np.array([x_new, y_new])

            r=0


            if __name__ == "__main__":

                file = open('out.csv','w')
                velocity_file = open('vel.csv', 'w')
                time_file = open('time.csv', 'a')
                acc_file = open('acc.csv', 'w')
                header = ""
                for m in range(n):
                    header += str(m)+"x,"+str(m)+"y,"
                file.write(header+"\n")
                velocity_file.write(header+"\n")
                acc_file.write(header+"\n")
                header2 = ", ".join([f"{i+1}" for i in range(n)])+"\n"
                #time_file.write(", ".join([str(x) for x in priority]))

                while not np.array_equal(completed, check):
                    # i+=1
                    r+=1
                    """
                    for each UAV:
                        if reached goal:
                            completed[self] = 1

                        attractive    
                        calculate goal a
                        add to a

                        repulsive
                        for each other UAV:
                            calc dist -> update min dist
                            if collision:
                                clip[self] = True
                                calculate repulsive a
                                add to a
                            else:
                                pass

                    
                    v = v + a*delt
                    clip v
                    to clip v, 
                    for each uav:
                        if clip[i] == True
                            vmax[i] = maxv*my_p/(max_p - lowest_p + 1)
                        else:
                            vmax[i] = maxv
                        if v>vmax:
                            v = maxvv[i]-v[j]
                    

                    completition check: within 0.1 of goal
                    """
                    clip = np.zeros(n)
                    a = np.zeros((n,2))
                    for i in range(n):
                        if reached_goal(i):
                            if completed[i] != 1:
                                completed[i] = 1
                                a[i] = 0
                                v[i] = 0
                                completion[i]=r
                            
                        else:
                            #attractive force 
                            dist_to_goal = np.linalg.norm(goal[i]-pos[i])
                            #attr = 2*(1-e**(-2*dist_to_goal**2))*np.array(goal[i]-pos[i])/dist_to_goal
                            attr = 2*np.array(goal[i]-pos[i])/dist_to_goal
                            a[i] = np.array(attr)
                            #print("a",i,a[i])

                            #repulsive potential
                            colliding=False
                            for j in range(n):
                                dist = np.linalg.norm(pos[j]-pos[i])
                                if dist < min_dist[i][j]:
                                    min_dist[i][j] = dist
                                if j != i:
                                    if collision(i, j):
                                        colliding=True
                                        #print("collision",i,j)
                                        #dist = np.linalg.norm(pos[j]-pos[i])
                                        rep = (priority[j]/priority[i])*10*(e**(-b*dist**2))*(pos[i]-pos[j])/dist
                                        rep = rotate_vector(rep, np.pi/2)
                                        #print(i,j,dist,rep,a[i])
                                        a[i] += rep
                                        
                            
                            if colliding:
                                clip[i]=1
                    if pert:
                        pertube(v)
                    v = v + a*delt
                    clip_v(n)
                    pos = pos + v*delt

                    file.write(",".join([str(x) for x in pos.flatten()])+"\n")
                    velocity_file.write(", ".join([str(x) for x in v.flatten()])+"\n")
                    acc_file.write(", ".join([str(x) for x in a.flatten()])+"\n")
                print(completed)
                print(check)
                time_file.write(", ".join([str(x) for x in completion])+'\n')
                time_file.close()

                file.close()
                velocity_file.close() 


                acc_file.close()

                for i in range(n):
                    avg_dist[i] = np.mean(min_dist[i])
                
                avg_file = open('avg.csv','a')
                avg_file.write(",".join([str(x) for x in avg_dist])+"\n")
                # end = time.time()
                # excec_time = end-start
                # exec_file.write(f'{i},{excec_time}')
                print("n:",n, "priority: ",priority)
                avg_file.close()
        #row.append(priority)
        data_Array[ix][0]=n

        times = pd.read_csv('time.csv', header=None)
        avg = pd.read_csv('avg.csv', header=None)
        priority = pd.read_csv('priority.csv', header=None)

        priority = priority.to_numpy().flatten()
        times = times.to_numpy().flatten()
        avg = avg.to_numpy().flatten()

        table = np.array([priority, times, avg]).T
        sorted = pd.DataFrame(table, columns=['priority', 'time', 'avg']).sort_values(by='priority')

        classes = np.digitize(sorted['priority'], [-np.inf,1,2,4,5,np.inf])
        sorted['class'] = classes

        mean = pd.DataFrame(sorted[sorted['class']==i].mean() for i in range(1,7))
        


        if priority_type:
            for k in range(5):
                data_Array[ix][k+2] = mean.iloc[k]['time']
                data_Array[ix][k+8] = mean.iloc[k]['avg']
        else:
            data_Array[ix][1] = mean.iloc[2]['time']
            data_Array[ix][7] = mean.iloc[2]['avg']
        print(data_Array)
        reset()

data = pd.DataFrame(data_Array, columns = ['n', 'time w/o p', 'time 0','time 1','time 2','time 3','time 4', 'space w/o p', 'space 0','space 1','space 2','space 3','space 4'])

data.to_csv('stat_data.csv')




                
            
        






    

