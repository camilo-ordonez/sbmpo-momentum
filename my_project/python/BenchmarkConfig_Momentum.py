from collections import namedtuple
import csv


max_iterations = 50000
max_generations = 100
horizon_time = 0.1
num_states = 2
num_controls = 1
grid_resolution = [0.005, 0.0005]
start_state = [0, 0]
goal_state = [3, 0]
branchout_factor = 7
samples = [-1, 1]

#my_list = [max_iterations,max_generations,sample_time,grid_resolution,start_state,goal_state,samples]

my_list = [[50000],[100],[0.1],[2],[1],[0.005, 0.0005],[-0.01, 0],[3, 0],[7],[-1,-0.6667,-0.3333, 0, 0.333, 0.6667, 1.0]]
print(my_list)
#my_list = [[1, 2, 3], [4, 5], [6, 7, 8, 9], [10]]

# create a new CSV file in write mode
with open('/home/camilo/Desktop/sbmpo-momentum/my_project_ws/my_project/csv/config.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    
    # write a single row with all the elements in my_list
    writer.writerow([num for lst in my_list for num in lst])


#list = [[1, 2],'3']

#params = namedtuple('params',['max_iterations','max_generations','sample_time', 'grid_resolution', 'start_state', 'goal_state','samples'])
# Momentum Patch 

 
#myparams = params(max_iterations,max_generations,sample_time,grid_resolution,start_state,goal_state,samples)


#myparams = params(5,[1,2,3])
#print(rows)


#aList = list(myparams)

#print(myparams)
#print(aList)
#with open('GFG','w') as f:
##    writer = csv.writer(f)  
#    writer.writerows(list)