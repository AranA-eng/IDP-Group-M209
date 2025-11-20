#reversing thing needed for about turns & leaving shelves
#needs to 

import math #needed for isclose

class Node:
    def __init__(self, data):
        self.data = data
        self.next = None
        self.nextdir = None
        self.prev = None
        self.prevdir = None #prevdir is the direction of turning to get to previous node, if approaching from behind
        self.minor = None
        self.minorfdir = None #direction of turning for minor path is from forward
        self.minorbdir = None

initial = Node(-1)
initial.next = Node(0)
initial.nextdir = 10
facing = "f" #needs initial orientation

nodedirections = { #using junc cases: 10 ignore, 1 left, -1 right
0: (10, 10, 1, -1, -1), #nextdir, prevdir, minorfdir, minorbdir, minor node number
1: (10, 10, 1, -1, 39),
2: (-1, 1, 1, 10, 40), 
3: (10, 10, -1, 1, None),
4: (10, 10, -1, 1, None),
5: (10, 10, -1, 1, None),
6: (10, 10, -1, 1, None),
7: (10, 10, -1, 1, None),
8: (10, 10, -1, 1, None),
9: (10, 10, 10, 10, None),
10: (-1, 1, 10, 10, None),
11: (10, 10, -1, 1, 30), #link to other main branch
12: (-1, 1, 10, 10, None),
13: (10, 10, 10, 10, None),
14: (10, 10, -1, 1, None),
15: (10, 10, -1, 1, None),
16: (10, 10, -1, 1, None),
17: (10, 10, -1, 1, None),
18: (10, 10, -1, 1, None),
19: (10, 10, -1, 1, None),
20: (-1, 1, 10, -1, 41),
21: (10, 10, 1, -1, 42),
22: (10, 2, 10, 10, None), #in backwards direction, needs to do an about turn, and need to say if in backwards state at node 30, about turn, set to forwards then continue
23: (10, 10, -1, 1, None),
24: (10, 10, -1, 1, None),
25: (10, 10, -1, 1, None),
26: (10, 10, -1, 1, None),
27: (10, 10, -1, 1, None),
28: (10, 10, -1, 1, None),
29: (1, -1, 10, 10, None),
30: (10, 10, 1, -1, 11), #link to other main branch
31: (1, -1, 10, 10, None),
32: (10, 10, -1, 1, None),
33: (10, 10, -1, 1, None),
34: (10, 10, -1, 1, None),
35: (10, 10, -1, 1, None),
36: (10, 10, -1, 1, None),
37: (10, 10, -1, 1, None),
38: (10, 2, 10, 10, None), #in backwards direction, needs to do an about turn, and need to say if in backwards state at node 30, about turn, set to forwards then continue
39: (10, 10, 10, 10, None), #minor from 1
40: (10, 10, 10, 10, None), #minor from 2
41: (10, 10, 10, 10, None), #minor from 20
42: (10, 10, 10, 10, None) #minor from 21
}

def build_circular_list(): #make 22 for main arc ----- rectify
    nodes_firstset = [Node(i) for i in range(22)]

    #link nodes in a circular doubly-linked list
    for i in range(22):
        nodes_firstset[i].next = nodes_firstset[(i + 1) % 22]
        nodes_firstset[i].prev = nodes_firstset[(i - 1) % 22]
        
    nodes_secondset = [Node(i) for i in range(22, 39)]
    
    #link nodes in a normal doubly-linked list
    for i in range(17):
        if i == 0: #at Node 22 which is the last element of the second set list
            nodes_secondset[i].next = nodes_secondset[i + 1]
            nodes_secondset[i].prev = None
        elif i == 16: #at Node 38 which is the last element of the second set list
            nodes_secondset[i].next = None
            nodes_secondset[i].prev = nodes_secondset[i - 1]
        else:
            nodes_secondset[i].next = nodes_secondset[i + 1]
            nodes_secondset[i].prev = nodes_secondset[i - 1]
    
    for obj in nodes_firstset + nodes_secondset:
        obj.nextdir, obj.prevdir, obj.minorfdir, obj.minorbdir = [nodedirections[int(obj.data)][i] for i in range(4)]
        
        if nodedirections[int(obj.data)][4] == None:
            continue
        else:
            obj.minor = Node(nodedirections[int(obj.data)][4])
            
    return nodes_firstset, nodes_secondset



nodes_firstset, nodes_secondset = build_circular_list()

turn_bin = []

def choose_dir_lower(start, end):
    forward  = (end - start) % 22
    backward = (start - end) % 22
    return "f" if forward <= backward else "b"

def choose_dir_upper(start, end):
    return "f" if end > start else "b"

def choose_dir(start, end):
    #upper to upper
    if start <= 21 and end <= 21:
        return choose_dir_lower(start, end)
    
    #lower to lower
    if start >= 22 and end >= 22:
        return choose_dir_lower(start, end)
    
    if start <= 21: #go to 11
        return choose_dir_upper(start, 11)
    else: #go to 30
        return choose_dir_lower(start, 30)

def cross_ramp(start, way):
    global turn_bin
    
    if way == "f":
        #print(start.data, start.minorfdir)
        turn_bin.append(start.minorfdir)
    else:
        #print(start.data, start.minorbdir)
        turn_bin.append(start.minorbdir)




def traverse(start_node, end_node, way="f", skip_first = False, first_call = True):
    global turn_bin
    if first_call == True:
        turn_bin = []
    global facing
    
    if start_node < 22:
        current = nodes_firstset[start_node]
    else:
        current = nodes_secondset[start_node - 22]

    first_main  = start_node <= 21
    second_main = end_node <= 21



    if (first_main and second_main) or (not first_main and not second_main):   #both upper or both lower

        if first_main:   #lower level
            way = choose_dir_lower(start_node, end_node)
        else:   #upper section
            way = choose_dir_upper(start_node, end_node)



    else:   #going to bridge
        if start_node < end_node:
            bridge = nodes_firstset[11]   #going to node 11 to ascend
        else:
            bridge = nodes_secondset[8]   #going to node 30 to descend

        #go from start to bridge
        way_to_bridge = choose_dir(start_node, bridge.data)
        reached = traverse(start_node, bridge.data, way_to_bridge, first_call = False)
        facing = way_to_bridge
        cross_ramp(reached, way_to_bridge)
        
        #jump to minor node
        current = bridge.minor

        #go from bridge minor to end
        way_from_bridge = choose_dir(bridge.minor.data, end_node)
        facing = way_from_bridge
        
        if way_from_bridge == "f": #needs an extra turn when crossing ramp
            if current.data == 11:
                adjustment = -1
            else:
                adjustment = 1
        else:
            if current.data == 11:
                adjustment = 1
            else:
                adjustment = -1
        #print(current.data, adjustment)
        turn_bin.append(adjustment)
        
        traverse(bridge.minor.data, end_node, way_from_bridge, skip_first = True, first_call = False)
        return
    
    

    if facing == way:
        pass
    else:
        #print(2)
        turn_bin.append(2)
        facing = way
        
    first_iter = True 
    while True:
        #print(current.data)

        if current.data == end_node:
            #print(current.data)
            #print(facing)
            #turn_bin.append(current.data)
            return current

        if skip_first and first_iter:
            first_iter = False
        else:
            if way == "f":
                #print(current.data, current.nextdir)
                turn_bin.append(current.nextdir)
            else:
                #print(current.data, current.prevdir)
                turn_bin.append(current.prevdir)


        if way == "f":
            current = current.next
        else:
            current = current.prev


traverse(2, 0)
print(turn_bin)
#traverse(20, 3)
#print(turn_bin)


colour_to_number = {
"green": 1,
"blue": 2,
"red": 20,
"yellow": 21
}


#def unload():
    #read colour sensor
    #
#    if current_node.straight == 
#    traverse(current_node, Node(colour_to_number[colour]), way)




#build_circular_list()
#obj = build_circular_list()[0][4]
#obj = build_circular_list()[0][11]
#print(obj.data, obj.nextdir, obj.prevdir, obj.minorfdir, obj.minorbdir, obj.minor.data, obj.next.data, obj.prev.data) #need an if prev is None ...
