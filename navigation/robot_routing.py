# robot_routing.py
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

# --- original node directions mapping (unchanged) ---
nodedirections = {
    -1: (10, 10, 1, -1, 0), #note that for the minor nodes, minordirs means the direction to turn at their major links to go f or b
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
    39: (10, 10, 1, -1, 1), #minor from 1
    40: (10, 10, 10, -1, 2), #minor from 2
    41: (10, 10, 10, 1, 20), #minor from 20
    42: (10, 10, 1, -1, 21), #minor from 21
}


# ----------------------
# Data classes & Node
# ----------------------
@dataclass
class RouteResult:
    turn_sequence: List[int]
    final_facing: str
    path_nodes: List[int]


class Node:
    def __init__(self, node_id: int, dirs: Tuple[int, int, int, int, Optional[int]]):
        self.id = node_id
        # unpack directions: nextdir, prevdir, minorfdir, minorbdir, minor_id
        self.nextdir, self.prevdir, self.minorfdir, self.minorbdir, self.minor_id = dirs
        # linked-list links (set by graph)
        self.next: Optional["Node"] = None
        self.prev: Optional["Node"] = None
        # reference to minor node (set by graph)
        self.minor: Optional["Node"] = None

    def __repr__(self):
        return f"Node({self.id})"


# ----------------------
# Graph builder
# ----------------------
class RouteGraph:
    """
    Builds two-level structure:
      - lower (0..21) : circular doubly-linked list
      - upper (22..38): linear doubly-linked list
    Also links minor nodes according to nodedirections.
    """

    def __init__(self, config: Dict[int, Tuple[int, int, int, int, Optional[int]]]):
        self.config = config
        self.nodes: Dict[int, Node] = {}
        self._build_nodes()
        self._link_primary()
        self._link_minors()

    def _build_nodes(self):
        for node_id, dirs in self.config.items():
            self.nodes[node_id] = Node(node_id, dirs)

    def _link_primary(self):
        # lower circular 0..21
        lower_ids = [i for i in range(22)]
        for i, nid in enumerate(lower_ids):
            node = self.nodes[nid]
            node.next = self.nodes[lower_ids[(i + 1) % len(lower_ids)]]
            node.prev = self.nodes[lower_ids[(i - 1) % len(lower_ids)]]

        # upper linear 22..38
        upper_ids = [i for i in range(22, 39)]
        for i, nid in enumerate(upper_ids):
            node = self.nodes[nid]
            if i > 0:
                node.prev = self.nodes[upper_ids[i - 1]]
            if i < len(upper_ids) - 1:
                node.next = self.nodes[upper_ids[i + 1]]

    def _link_minors(self):
        # minor_id in config is an integer or None. Link Node.minor to the node object if present.
        for nid, node in self.nodes.items():
            mid = node.minor_id
            if mid is None or mid == -1:
                node.minor = None
            else:
                # the minor id should exist in the graph; link it
                node.minor = self.nodes.get(mid)

    def get_node(self, node_id: int) -> Node:
        return self.nodes[node_id]


# ----------------------
# Router (traversal engine)
# ----------------------
class Router:
    def __init__(self, graph: RouteGraph):
        self.graph = graph
        # per-route state
        self.turn_bin: List[int] = []
        self.facing: str = "f"  # "f" or "b"
        self.path_nodes: List[int] = []

    # ---------- direction helpers ----------
    @staticmethod
    def choose_dir_lower(start: int, end: int) -> str:
        # works modulo 22 for circular lower level
        forward = (end - start) % 22
        backward = (start - end) % 22
        return "f" if forward <= backward else "b"

    @staticmethod
    def choose_dir_upper(start: int, end: int) -> str:
        return "f" if end > start else "b"

    def choose_dir(self, start: int, end: int) -> str:
        # both lower (0..21)
        if start <= 21 and end <= 21:
            return self.choose_dir_lower(start, end)
        # both upper (22..38)
        if start >= 22 and end >= 22:
            return self.choose_dir_upper(start, end)
        # crossing bridge: we'll signal with None to let _traverse handle bridge logic
        return None

    def _cross_ramp_append(self, node: Node, way: str):
        # append the minor turn when reaching the bridge node
        if way == "f":
            self.turn_bin.append(node.minorfdir)
        else:
            self.turn_bin.append(node.minorbdir)

    def is_minor(self, node: int):
        return node in [-1, 39, 40, 41, 42]

    # ---------- main traversal ----------
    def route(self, start: int, end: int) -> RouteResult:
        """
        Public API: returns RouteResult(turn_sequence, final_facing, path_nodes)
        """
        # reset state
        self.turn_bin = []
        self.path_nodes = []

        # call internal traverse
        self._traverse(start_node=start, end_node=end, first_call=True)

        return RouteResult(
            turn_sequence=self.turn_bin.copy(),
            final_facing=self.facing,
            path_nodes=self.path_nodes.copy(),
        )

    def _traverse(
        self,
        start_node: int,
        end_node: int,
        way: Optional[str] = "f",
        skip_first: bool = False,
        first_call: bool = True,
        crossing_bridge: bool = False,
    ) -> Node:
        """
        Internal traversal that mirrors the logic of your original function.
        Returns the final Node object reached (so callers can inspect it).
        """
        #determine whether extra handling is needed at start/end
        true_start = start_node
        true_end = end_node
        
        true_start_node = self.graph.get_node(true_start)
        true_end_node = self.graph.get_node(true_end)
        
        checkstart = self.is_minor(start_node)
        checkend = self.is_minor(end_node)
        
        if checkstart:
            start_node = true_start_node.minor.id
        if checkend:
            end_node = true_end_node.minor.id
        
        # choose node object for start
        current = self.graph.get_node(start_node)
        # track where we came across for path debugging
        if first_call:
            # initialize
            pass

        # determine whether start/end are in lower (0..21)
        first_main = start_node <= 21
        second_main = end_node <= 21

        # case: both on same level (both lower or both upper)
        if (first_main and second_main) or (not first_main and not second_main):
            if first_main:
                way = self.choose_dir_lower(start_node, end_node)
            else:
                way = self.choose_dir_upper(start_node, end_node)

        else:
            # We're crossing between levels -> need to go to the bridge node first
            # bridge is Node(11) on lower and Node(30) on upper in your design
            bridge = self.graph.get_node(11) if (start_node < end_node) else self.graph.get_node(30)

            # go from start to bridge
            way_to_bridge = self.choose_dir(start_node, bridge.id)
            
            if checkstart:
                self.turn_bin.append(2)
                self.path_nodes.append(true_start)
                if way == "f":
                    self.turn_bin.append(true_start_node.minorfdir)
                else:
                    self.turn_bin.append(true_start_node.minorbdir)
            
            # recursively traverse to the bridge (do not append the bridge's nextdir in that call)
            bridge_node = self._traverse(start_node, bridge.id, way_to_bridge, first_call=False, crossing_bridge=True)
            # append the ramp turn for crossing the bridge
            self._cross_ramp_append(bridge_node, way_to_bridge)

            # jump to bridge.minor (the minor node that represents the crossing)
            if bridge_node.minor is None:
                raise RuntimeError(f"Bridge node {bridge_node.id} has no minor link configured.")
            current = bridge_node.minor

            # go from bridge minor to end
            way_from_bridge = self.choose_dir(bridge_node.minor.id, end_node)
            self.facing = way_from_bridge  # the robot will be facing the direction of travel after jump

            # When crossing the ramp we append an extra correction turn (the original code had this)
            if way_from_bridge == "f":
                if current.id == 11:
                    adjustment = -1
                else:
                    adjustment = 1
            else:
                if current.id == 11:
                    adjustment = 1
                else:
                    adjustment = -1
            self.turn_bin.append(adjustment)

            # continue traversal from the minor node to the final destination
            self._traverse(bridge_node.minor.id, end_node, way_from_bridge, skip_first=True, first_call=False)
        
            # checkstart and checkend logic are needed in this branch because this never enters the while loop with the checks true
            if checkend:
                if self.facing == "f":
                    self.turn_bin.append(self.graph.get_node(end_node).minorfdir)
                else:
                    self.turn_bin.append(self.graph.get_node(end_node).minorbdir)
                self.path_nodes.append(true_end)
                self.facing = "b"
           
            # need to pop the extra turn arising from the junction value of the start's major link
            if checkstart:
                self.turn_bin.pop(2)  
            
            # need to change the initial turn from 2 (since node -1 is minor) into a 10 (because the robot doesn't start facing backwards)
            if true_start == -1:
                self.turn_bin[0] = 10
            
            return current

        # At this point, 'way' is set to "f" or "b" for regular traversal on a single level
        pop_check = False

        # Handling the output in case you start at a minor node, just before entering the loop
        if checkstart:
            self.turn_bin.append(2)
            self.path_nodes.append(true_start)
            if way == "f":
                self.turn_bin.append(true_start_node.minorfdir)
            else:
                self.turn_bin.append(true_start_node.minorbdir)
        
        else:
            # If we are not facing the desired way, we need an extra 180-turn (encoded as 2)
            if self.facing == way:
                # already facing correct way; nothing to do
                pass
            else:
                # append 180-degree turn
                self.turn_bin.append(2)
                self.facing = way
                pop_check = True  # we'll need special handling if we later skip first appended turn

        first_iter = True

        # iterate along the linked list until we reach the end_node
        while True:
            # record visited node id for debugging / route metadata
            self.path_nodes.append(current.id)

            if current.id == end_node:
                # if we made a 180-turn before starting, original code removed the 1st appended turn at index 1
                if pop_check:
                    # safe-pop if present
                    if len(self.turn_bin) > 1:
                        # original code: turn_bin.pop(1)
                        self.turn_bin.pop(1)
                        
                if checkend:
                    if self.facing == "f":
                        self.turn_bin.append(self.graph.get_node(end_node).minorfdir)
                    else:
                        self.turn_bin.append(self.graph.get_node(end_node).minorbdir)
                    self.path_nodes.append(true_end)
                    self.facing = "b"
                
                # need to pop the extra turn arising from the junction value of the start's major link
                if checkstart:
                    self.turn_bin.pop(2)  
                
                # need to change the initial turn from 2 (since node -1 is minor) into a 10 (because the robot doesn't start facing backwards)
                if true_start == -1:
                    self.turn_bin[0] = 10
                return current

            # skip appending the first node's turn if requested (bridge handoff case)
            if skip_first and first_iter:
                first_iter = False
            else:
                if way == "f":
                    self.turn_bin.append(current.nextdir)
                else:
                    self.turn_bin.append(current.prevdir)

            # advance the 'current' pointer along next/prev depending on direction
            if way == "f":
                if current.next is None:
                    # reached the end of linear list unexpectedly
                    raise RuntimeError(f"Cannot move forward from node {current.id}: next is None.")
                current = current.next
            else:
                if current.prev is None:
                    raise RuntimeError(f"Cannot move backward from node {current.id}: prev is None.")
                current = current.prev

# ----------------------
# Example usage (same behaviour as your original quick test)
# ----------------------

#if __name__ == "__main__":
#    graph = RouteGraph(nodedirections)
#    router = Router(graph)

    # replicate original tests
#    r1 = router.route(0, 22)
#    r1 = router.route(22, -1)
#    print("route 39->41 turns:", r1.turn_sequence)
#    print("The node sequence is: ", r1.path_nodes)
#    print("final facing:", r1.final_facing)

#    r2 = router.route(30, 10)
#    print("route 10->31 turns:", r2.turn_sequence)
#    print("final facing:", r2.final_facing)
