from navigation import robot_routing as Routing

# --- original node directions mapping (unchanged) ---
nodedirections = {
    0: (10, 10, 1, -1, -1),
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
    22: (10, 2, 10, 10, None),
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
    38: (10, 2, 10, 10, None),
    39: (10, 10, 10, 10, None), #minor from 1
    40: (10, 10, 10, 10, None), #minor from 2
    41: (10, 10, 10, 10, None), #minor from 20
    42: (10, 10, 10, 10, None), #minor from 21
}

graph = Routing.RouteGraph(nodedirections)
router = Routing.Router(graph)

r1 = router.route(0,22)

print(r1.turn_sequence)
print(r1.path_nodes[0])