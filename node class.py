#reversing thing needed for about turns & leaving shelves
#after final turn, stop after 0.2 secs
#reversing thing needed for about turns & leaving shelves

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



def build_circular_list(): #make 22 for main arc
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



def choose_dir_lower(start, end): #for the lower level (circular linked list)
    forward  = (end - start) % 22
    backward = (start - end) % 22
    return "f" if forward <= backward else "b"


def choose_dir_upper(start, end): #for the upper level (normal linked list)
    return "f" if end > start else "b"


def choose_dir(start, end):
    if start <= 21 and end <= 21:  #upper to upper
        return choose_dir_lower(start, end)
    
    if start >= 22 and end >= 22:  #lower to lower
        return choose_dir_lower(start, end)
    
    if start <= 21: #go to 11
        return choose_dir_upper(start, 11)
    else: #go to 30
        return choose_dir_lower(start, 30)


def cross_ramp(start, way): #appending the minor turn when reaching the bridge node
    global turn_bin
    
    if way == "f":
        #print(start.data, start.minorfdir)
        turn_bin.append(start.minorfdir)
    else:
        #print(start.data, start.minorbdir)
        turn_bin.append(start.minorbdir)



def traverse(start_node, end_node, way="f", skip_first = False, first_call = True, crossing_bridge = False):
    global turn_bin
    global facing

    if first_call == True:
        turn_bin = [] #empty the turn bin
    
    if start_node < 22:
        current = nodes_firstset[start_node] #access first linked list
    else:
        current = nodes_secondset[start_node - 22] #access second linked list

    first_main  = start_node <= 21   #checking if start is on lower level
    second_main = end_node <= 21   #checking if start is on upper level



    if (first_main and second_main) or (not first_main and not second_main):   #both upper or both lower

        if first_main:   #lower level
            way = choose_dir_lower(start_node, end_node)
        else:   #upper level
            way = choose_dir_upper(start_node, end_node)



    else:   #going to bridge
        bridge = nodes_firstset[11] if (start_node < end_node) else nodes_secondset[8]  #going to node 11/30

        #go from start to bridge
        way_to_bridge = choose_dir(start_node, bridge.data)
        current = traverse(start_node, bridge.data, way_to_bridge, first_call = False, crossing_bridge = True) #do not want to append the nextdir of the bridge node
        cross_ramp(current, way_to_bridge)
        
        #jump to minor node
        current = bridge.minor

        #go from bridge minor to end
        way_from_bridge = choose_dir(bridge.minor.data, end_node)
        facing = way_from_bridge
        
        if way_from_bridge == "f": #needs an extra turn when crossing ramp
            if current.data == 11:
                adjustment = -1 #turn right if end of bridge you finish on is 11
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

    
    pop_check = False

    
    if facing == way: #need an extra 180 turn if not facing the right way
        pass
    else:
        #print(2)
        turn_bin.append(2)
        facing = way
        pop_check = True #need to pop element 1 if you had to do a 180 turn
        
    first_iter = True
    
    while True:
        #print(current.data)

        if current.data == end_node:
            if pop_check:
                turn_bin.pop(1)
            return current

        if skip_first and first_iter:
            first_iter = False #basically don't append first value, this was a previous issue
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


current = 0
facing = "f" #needs initial orientation
turn_bin = []
nodes_firstset, nodes_secondset = build_circular_list()


traverse(-1,10)
print(turn_bin)
print(facing)
traverse(10,31)
print(turn_bin)
print(facing)


colour_to_number = {
"green": 1,
"blue": 2,
"red": 20,
"yellow": 21
}



#def finish():
#    traverse(


#def unload():
    #read colour sensor
    #
#    if current_node.straight == 
#    traverse(current_node, Node(colour_to_number[colour]), way)

#def finish():
#    traverse(
