import numpy as np

t = np.linspace(0,14,num = 15)
x = [0,0,0,0,0,1,1,1,1,1,2,2,2,2,3]
y = [0,1,2,3,4,4,5,5,6,7,7,8,9,9,9]
turnpoints = []

#temporary end points appended as the replica of just last cbs 
#append the start and end points before using this loop
x.append(x[-1]) 
y.append(y[-1])

for i in range(1,t.size-1):
    #movement in y direction
    if x[i] == x[i+1] and y[i] != y[i+1]:
        if x[i] != x[i-1] and y[i] == y[i-1]:
            turnpoints.append([x[i],y[i]]) 
    #movement in x direction
    elif x[i] != x[i+1] and y[i] == y[i+1]:
        if x[i] == x[i-1] and y[i] != y[i-1]:
            turnpoints.append([x[i],y[i]]) 
    #Halt
    else:
        turnpoints.append([-100,-100])
        
print(turnpoints)