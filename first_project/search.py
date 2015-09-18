# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    closed = set()
    fringe = util.Stack()
    fringe.push((problem.getStartState(),))
    closed.add(problem.getStartState()[0])
    pathTrace = {}

    while True:
        if fringe.isEmpty():
            util.raiseNotDefined()
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            curr = node
            directions = []
            while (len(curr) > 1):
                directions.insert(0,curr[1])
                curr = pathTrace[curr]
            return directions
    	for child_node in problem.getSuccessors(node[0]):
            if child_node[0] not in closed:
                closed.add(child_node[0])
                fringe.push(child_node)
                pathTrace[child_node] = node

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    closed = set()
    fringe = util.Queue()
    fringe.push((problem.getStartState(),))
    closed.add(problem.getStartState()[0])
    pathTrace = {}

    while True:
        if fringe.isEmpty():
            util.raiseNotDefined()
        node = fringe.pop()
        #print node
        if problem.isGoalState(node[0]):
            curr = node
            directions = []
            while (len(curr) > 1):
                directions.insert(0,curr[1])
                curr = pathTrace[curr]
            print directions
            return directions
    	for child_node in problem.getSuccessors(node[0]):
            if child_node[0] not in closed:
                closed.add(child_node[0])
                fringe.push(child_node)
                pathTrace[child_node] = node

import heapq

class PriorityQueue:
    """
      Implements a priority queue data structure, but with a USEFUL pop()
      and a third option for cost without heurisitic
    """
    def  __init__(self):
        self.heap = []

    def push(self, item, parent, cost, priority=None):
        if priority is None:
            priority = cost
        trip = (cost,item, parent)
        quad = (priority, trip)
        heapq.heappush(self.heap,quad)

    def pop(self):
        a = heapq.heappop(self.heap)
        #print self.heap
        priority,trip = a
        cost,item,parent = trip
        return cost, item, parent

    def isEmpty(self):
        return len(self.heap) == 0

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    closed = set()
    fringe = PriorityQueue()
    start = problem.getStartState()
    fringe.push((start,None,0), None, 0)
    pathTrace = {}
    pathTrace[start] = None
    while True:
        if fringe.isEmpty():
            util.raiseNotDefined()
        cost, node, parent = fringe.pop()
        while node[0] in closed:
            cost, node, parent = fringe.pop()
        closed.add(node[0])
        pathTrace[node] = parent

        if problem.isGoalState(node[0]):
            curr = node
            directions = []
            while (curr[1] is not None):
                directions.insert(0,curr[1])
                curr = pathTrace[curr]
            return directions

        for child_node in problem.getSuccessors(node[0]):
            fringe.push(child_node, node, cost+child_node[2])

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    closed = set()
    fringe = PriorityQueue()
    start = problem.getStartState()
    fringe.push((start,None,0), None, 0,heuristic(start, problem))
    pathTrace = {}
    pathTrace[start] = None
    while True:
        if fringe.isEmpty():
            util.raiseNotDefined()
        cost, node, parent = fringe.pop()
        while node[0] in closed:
            cost, node, parent = fringe.pop()
        closed.add(node[0])
        pathTrace[node] = parent

        if problem.isGoalState(node[0]):
            curr = node
            directions = []
            while (curr[1] is not None):
                directions.insert(0,curr[1])
                curr = pathTrace[curr]
            print directions
            return directions

        for child_node in problem.getSuccessors(node[0]):
            fringe.push(child_node, node, cost+child_node[2], cost+child_node[2] + heuristic(child_node[0], problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
