import numpy

class Node:

    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def vcheck(grid,v,x,y):                                   #Function to check if the node is visited or not

    if(x>=len(grid) or y>=len(grid[0]) or x<0 or y<0):    #checks if coordinates are within boundary of the grid, returns False if not
        return False

    if(v[x][y]==-1 and grid[x][y]==0):                    #checks if the node is visited and if it is an obstacle, 
        return True                                       #returns True if it has not been visited and if its not an obstacle

    return False                                          #returns False otherwise


def findpath(goal_node):                                  #Function that returns the path from the goal node

    j=goal_node
    l=[]
    
    while(j.parent!=None):                                #Loop to insert the coordinates of the node into the path list
        l.insert(0,[j.row,j.col])
        j=j.parent                                        #Updates the node to insert to its parent node
         
    l.insert(0,[j.row,j.col])                             #Inserts the start node to the path list

    return(l)
    


def findnext(grid,pos,goal):                              #Function that returns the neighbours of the node(pos) passed to it

    global v
    n=[]

    xchange=[0,1,0,-1]                                    #Nodes will be appended to the list in the order, right, down, left, up
    ychange=[1,0,-1,0]

    for i in range(0,4):
        
        newx=pos.row+xchange[i]
        newy=pos.col+ychange[i]

        if(vcheck(grid,v,newx,newy)==True):
            mdist=abs(newx-goal[0])+abs(newy-goal[1])     #Manhattan Distance for the Heuristic
            e=Node(newx,newy,0,mdist)                     #Node object with the new coordinates
            e.parent=pos                                  #Parent is set to the node that is passed to the function
            e.g=pos.g+1                                   #Cost to come set to the previous nodes cost to come plus 1 as movement cost to neighbours is 1
            e.cost=e.g+e.h                                #Total cost of the node is set to the cost to come plus the heuristic
            n.append(e)                                   #Node object is appended to the list

    return n



def rdfs(grid,pos,goal):                                  #Recursive DFS Function
    global found        
    global v
    global steps
    global goal_node

    if(found==False):                                     

        if(pos.row==goal[0] and pos.col==goal[1]):        #Checks if current node is the goal node, and stops anymore recursion
            found=True
            goal_node=pos                                 #Records the node at which the goal is at
        
        if(v[pos.row][pos.col]==-1):                      #Checks if Node position is visited and sets it to visited if not.   
            v[pos.row][pos.col]=0
            steps=steps+1                                 #Increments the steps

        else:                                             #Does nothing if already visited
            return

        n=findnext(grid,pos,goal)                         #Finds the next valid neighbours and adds them into the priority queue.
        for i in n:
            rdfs(grid,i,goal)                             #Recursion for every node in the queue.



def bfs(grid, start, goal):
    
    global v

    path = []
    steps = 1                                             #Steps starts from 1 to account for the goal node
    found = False
    

    r=len(grid)
    c=len(grid[0])
    v=numpy.ones([r,c])     
    v=v*-1                                                #v is a list which is of same dimensions as the grid and is initialized to have all values -1. This indicates no node is visited yet.

    n=[]                                                  #n is the priority queue
    e=Node(start[0],start[1],0,1)
    e.g=0
    n.append(e)                                           #Start node object is created and cost to come set to 0 and appended into the priority queue

    v[start[0]][start[1]]=0                               #Sets the start coordinates to visited.

    
    while(found==False):

        n=n+findnext(grid,n[0],goal)                      #Adds the Valid neighbours into the priority queue.
        n.pop(0)                                          #Removes the first element in the priority queue and increments the steps.
        steps=steps+1
        
        if(len(n)==0):                                    #Check if there is a valid path or not
            break

        for i in n:                                       #Sets all the nodes in the list as visited, so the findnext function does not repeat nodes again.
            v[i.row][i.col]=0

        if(n[0].row==goal[0] and n[0].col==goal[1]):      #Goal Check
            found=True
            goal_node=n[0]
            

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
        path=findpath(goal_node)                                    #Only if the goal is found will the function to find a path be called.
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    
    global goal_node
    global steps
    global found
    global v

    path = []
    steps = 0                                                       #Steps starts at 0 as the function counts both start and goal
    found = False
   
    r=len(grid)
    c=len(grid[0])
    v=numpy.ones([r,c])
    v=v*-1
    
    e=Node(start[0],start[1],False,1)
    e.g=0
    
    rdfs(grid,e,goal)
    
    

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
        path=findpath(goal_node)
    else:
        print("No path found")
    
    return path, steps


def dijkstra(grid, start, goal):
    
    global v

    path = []
    steps = 1                                                       #Steps starts at 1 to account for the goal as the function does not count the goal.
    found = False

    r=len(grid)
    c=len(grid[0])
    v=numpy.ones([r,c])
    v=v*-1
    
    n=[]
    e=Node(start[0],start[1],0,1)
    e.g=0
    n.append(e)

    v[start[0]][start[1]]=0

    
    while(found==False):
        
        m=findnext(grid,n[0],goal)                                  #list that stores the Valid neighbour nodes

        for i in n:                                                 #Function that evaluates if there are duplicate node positions in n and m
            for a in m:
               
                if(i.row==a.row and i.col==a.col):                  #Removes the node with greater cost to come or updates the greater node if it comes first in the queue.
                    if(i.g<=a.g):
                        
                        m.remove(a)
                        
                    else:                                           
                        temp=n.index(i)
                        n.remove(i)
                        n.insert(temp,a)
                        m.remove(a)
        
        n=n+m

        v[n[0].row][n[0].col]=0
        n.pop(0)
        steps=steps+1

        if(len(n)==0):                                              #Check if path exists
            break

        d=n[0].g
        j=0

        for i in range(1,len(n)):                                   #Loop to find the node with smallest cost and bring it to the start of the queue
            if(n[i].g<d):
                j=i
                d=n[i].g

        n.insert(0,n[j])
        n.pop(j+1)

        if(n[0].row==goal[0] and n[0].col==goal[1]):                #Goal check
            found=True
            goal_node=n[0]
            
    


    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
        path=findpath(goal_node)
    else:
        print("No path found")

    return path,steps


def astar(grid, start, goal):
    
    global v

    path = []
    steps = 1
    found = False
    
    r=len(grid)
    c=len(grid[0])
    v=numpy.ones([r,c])
    v=v*-1

    n=[]
    mdist=abs(start[0]-goal[0])+abs(start[1]-goal[1])               #Since A*, We require the Heuristic. Manhattan Distance from start node to goal is calculated
    e=Node(start[0],start[1],0,mdist)
    e.g=0
    n.append(e)

    v[start[0]][start[1]]=0

    
    while(found==False):
        
        m=findnext(grid,n[0],goal)

        for i in n:
            for a in m:
               
                if(i.row==a.row and i.col==a.col):                  #Similar to Dijkstra, but here evaluates total cost
                    if(i.cost<=a.cost):
                        m.remove(a)
                        
                    else:
                        temp=n.index(i)
                        n.remove(i)
                        n.insert(temp,a)
                        m.remove(a)

        n=n+m

        v[n[0].row][n[0].col]=0
        n.pop(0)
        steps=steps+1

        if(len(n)==0):
            break
        
        d=n[0].cost
        j=0
        for i in range(1,len(n)):
            if(n[i].cost<d):
                j=i
                d=n[i].g

        n.insert(0,n[j])
        n.pop(j+1)

        if(n[0].row==goal[0] and n[0].col==goal[1]):
            found=True
            goal_node=n[0]

    

    if found:
        print(f"It takes {steps} steps to find a path using A*")
        path=findpath(goal_node)
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
