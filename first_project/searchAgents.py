# searchAgents.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
This file contains all of the agents that can be selected to
control Pacman.  To select an agent, use the '-p' option
when running pacman.py.  Arguments can be passed to your agent
using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a searchFunction=depthFirstSearch

Commands to invoke other search strategies can be found in the
project description.

Please only change the parts of the file you are asked to.
Look for the lines that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the
project description for details.

Good luck and happy searching!
"""
from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search algorithm for a
    supplied search problem, then returns actions to follow that path.

    As a default, this agent runs DFS on a PositionSearchProblem to find location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game board. Here, we
        choose a path to the goal.  In this phase, the agent should compute the path to the
        goal and store it in a local variable.  All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in registerInitialState).  Return
        Directions.STOP if there is no further action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test,
    successor function and cost function.  This search problem can be
    used to find paths to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print 'Warning: this does not look like a regular search maze'

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function
    """

    def __init__(self, startingGameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print 'Warning: no food in corner ' + str(corner)
        self._expanded = 0 # Number of search nodes expanded

        "*** YOUR CODE HERE ***"
        heuristicInfo = {}
        """
        heuristicInfo['bltl'] = self.aStarSearch( self.corners[0], self.corners[1])
        heuristicInfo['blbr'] = self.aStarSearch( self.corners[0], self.corners[2])
        heuristicInfo['bltr'] = self.aStarSearch( self.corners[0], self.corners[3])
        heuristicInfo['tlbr'] = self.aStarSearch( self.corners[1], self.corners[2])
        heuristicInfo['tltr'] = self.aStarSearch( self.corners[1], self.corners[3])
        heuristicInfo['brtr'] = self.aStarSearch( self.corners[2], self.corners[3])
        
        heuristicInfo['tlbl'] = self.aStarSearch( self.corners[1], self.corners[0])
        heuristicInfo['brbl'] = self.aStarSearch( self.corners[2], self.corners[0])
        heuristicInfo['trbl'] = self.aStarSearch( self.corners[3], self.corners[0])
        heuristicInfo['brtl'] = self.aStarSearch( self.corners[2], self.corners[1])
        heuristicInfo['trtl'] = self.aStarSearch( self.corners[3], self.corners[1])
        heuristicInfo['trbr'] = self.aStarSearch( self.corners[3], self.corners[2])
        """
        heuristicInfo['bltl'] = dist( self.corners[0], self.corners[1])
        heuristicInfo['blbr'] = dist( self.corners[0], self.corners[2])
        heuristicInfo['bltr'] = dist( self.corners[0], self.corners[3])
        heuristicInfo['tlbr'] = dist( self.corners[1], self.corners[2])
        heuristicInfo['tltr'] = dist( self.corners[1], self.corners[3])
        heuristicInfo['brtr'] = dist( self.corners[2], self.corners[3])
        heuristicInfo['tlbl'] = dist( self.corners[0], self.corners[1])
        heuristicInfo['brbl'] = dist( self.corners[0], self.corners[2])
        heuristicInfo['trbl'] = dist( self.corners[0], self.corners[3])
        heuristicInfo['brtl'] = dist( self.corners[1], self.corners[2])
        heuristicInfo['trtl'] = dist( self.corners[1], self.corners[3])
        heuristicInfo['trbr'] = dist( self.corners[2], self.corners[3])
               
        self.heuristicInfo = heuristicInfo

    def dynamicHeur(self, start, corners):
        if len(corners) == 0:
            return 0
        if len(corners) == 1:
            for corner in corners:
                return self.heuristicInfo[start + corner]

        best = float('inf')
        for corner in corners:
            cur = self.heuristicInfo[start + corner]
            rec = self.dynamicHeur(corner, corners-set([corner]))
            val = cur + rec
            if val < best:
                best = val
        return best
    """    
    def aStarSearch(self, start, goal):
        "Search the node that has the lowest combined cost and heuristic first."
        closed = set()
        fringe = search.PriorityQueue()
        fringe.push(((start, frozenset()),None,0), None, 0, dist(start, goal))

        while True:
            if fringe.isEmpty():
                util.raiseNotDefined()
            cost, node, parent = fringe.pop()
            while node[0] in closed:
                if fringe.isEmpty():
                    util.raiseNotDefined()
                cost, node, parent = fringe.pop()
            closed.add(node[0])
            if dist(node[0][0], goal) is 0:
                return cost
            for child_node in self.getSuccessors(node[0]):
                fringe.push(child_node, node, cost+child_node[2], cost+child_node[2] + dist(child_node[0][0], goal))
    """
    def getStartState(self):
        "Returns the start state (in your state space, not the full Pacman state space)"
        return self.startingPosition, frozenset(['bl','tl','br','tr'])

    def isGoalState(self, state):
        "Returns whether this search state is a goal state of the problem"
        return len(state[1]) == 0

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            # Add a successor state to the successor list if the action is legal
            # Here's a code snippet for figuring out whether a new position hits a wall:
            #   x,y = currentPosition
            #   dx, dy = Actions.directionToVector(action)
            #   nextx, nexty = int(x + dx), int(y + dy)
            #   hitsWall = self.walls[nextx][nexty]
            
            newState = state
            x,y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            newPosition = (nextx, nexty)
            nc = set()
            hitsWall = self.walls[nextx][nexty]
            if (not newPosition == self.corners[0]) and 'bl' in state[1]:
                nc.add('bl')
            if (not newPosition == self.corners[1]) and 'tl' in state[1]:
                nc.add('tl')
            if (not newPosition == self.corners[2]) and 'br' in state[1]:
                nc.add('br')
            if (not newPosition == self.corners[3]) and 'tr' in state[1]:
                nc.add('tr')  
            newState = (newPosition, frozenset(nc))
            if not hitsWall:
                successors.append((newState, action, 1))

        self._expanded += 1
        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions == None: return 999999
        x,y= self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound
    on the shortest path from the state to a goal of the problem; i.e.
    it should be admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    if len(state[1]) == 0:
        return 0

    best = float('inf')
    for corner in state[1]:
        if corner == 'bl':
            cur = dist(state[0], corners[0])
            rec = problem.dynamicHeur('bl', state[1] - set(['bl']))
            val = cur + rec
            if val < best:
                best = val
        if corner == 'tl':
            cur = dist(state[0], corners[1])
            rec = problem.dynamicHeur('tl', state[1] - set(['tl']))
            val = cur + rec
            if val < best:
                best = val
        if corner == 'br':
            cur = dist(state[0], corners[2])
            rec = problem.dynamicHeur('br', state[1] - set(['br']))
            val = cur + rec
            if val < best:
                best = val
        if corner == 'tr':
            cur = dist(state[0], corners[3])
            rec = problem.dynamicHeur('tr', state[1] - set(['tr']))
            val = cur + rec
            if val < best:
                best = val
    return best
    

def dist(position, goal):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])

class AStarCornersAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """
    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0
        self.heuristicInfo = {} # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem

def dynamicHeur(start, corners, heuristicInfo):
    #print 'enter'
    if len(corners) == 0:
        return 0
    if len(corners) == 1:
        for corner in corners:
            return dist(start,corner)

    best = float('inf')
    for corner in corners:
        cur = dist(start,corner)
        if str((corner, corners-set([corner]))) not in heuristicInfo:
            #print 'recursive'
            heuristicInfo[str((corner, corners-set([corner])))] = dynamicHeur(corner, corners-set([corner]), heuristicInfo)
        rec = heuristicInfo[str((corner, corners-set([corner])))]
        val = cur + rec
        if val < best:
            best = val
    return best

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come up
    with an admissible heuristic; almost all admissible heuristics will be consistent
    as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the other hand,
    inadmissible or inconsistent heuristics may find optimal solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a
    Grid (see game.py) of either True or False. You can call foodGrid.asList()
    to get a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the problem.
    For example, problem.walls gives you a Grid of where the walls are.

    If you want to *store* information to be reused in other calls to the heuristic,
    there is a dictionary called problem.heuristicInfo that you can use. For example,
    if you only want to count the walls once and store that value, try:
      problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    (answer1, answer2, answer3, answer4) = (0,0,0,0)
    """
    #distance of closest pair*number of pellets-1 + closest to pacman
    food = []
    pellets = 0
    distance = float('inf')
    for i,row in enumerate(foodGrid):
        for j,square in enumerate(row):
            if square:
                food.append((i,j))
                dist2 = dist(position, (i,j))
                if dist2 < distance:
                    distance = dist2
                pellets = pellets + 1
    if pellets == 0:
        return 0
    
    closest = float('inf')
    for a in food:
        for b in food:
            if a is not b:
                between = dist(a,b)
                if between < closest:
                    closest = between
    if closest == float('inf'):
        closest = 1

    answer1 = closest*(pellets-1) + distance
    

    #return manhattan distance to farthest pellet
    farthest = 0
    for a in food:
        toPacman = dist(a, position)
        if toPacman > farthest:
            farthest = toPacman
    answer2 = farthest
    

    #return max horizontal distance + max vertical distance between pellets
    left, bottom = position
    right, top = position
    for a in food:
        (x, y) = a
        if x<left:
            left = x
        if x>right:
            right = x
        if y<bottom:
            bottom = y
        if y>top:
            top = y

    x,y = position

    horiz = min(x-left, right-x)
    vert = min(y-bottom, top-y)

    answer3 = right-left+top-bottom + horiz + vert
    """

    #same as cornersheuristic but with limited subset of pellets
    food = set()
    for i,row in enumerate(foodGrid):
        for j,square in enumerate(row):
            if square:
                food.add((i,j))
    if 'remove' not in problem.heuristicInfo:
        foodCopy = set(food)
        remove = set()
        for i in range(len(food)-13):
            remove.add(foodCopy.pop())
        problem.heuristicInfo['remove'] = frozenset(remove)
    remove = problem.heuristicInfo['remove']
    answer4 = dynamicHeur(position, food-remove, problem.heuristicInfo)


    #this didn't improve anything so I commented it out
    return max(answer1, answer2, answer3, answer4)

    """
    #sum of manhattan distance to nearest pellet/pacman for each pellet
    #inconsistent
    distance = float('inf')
    pellet = None
    for i,row in enumerate(foodGrid):
        for j,square in enumerate(row):
            if square:
                food.append((i,j))
                dist2 = dist(position, (i,j))
                if dist2 < distance:
                    distance = dist2
                    pellet = (i,j)
    if pellet is None:
        return 0
    
    food2 = list(food)
    food2.remove(pellet)

    total = 0
    for a in food2:
        closest = float('inf')
        for b in food:
            if a is not b:
                between = dist(a,b)
                if between < closest:
                    closest = between
        total = total + closest
    return total + distance
    """

class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print 'Path found with cost %d.' % len(self.actions)

    def findPathToClosestDot(self, gameState):
        "Returns a path (a list of actions) to the closest dot, starting from gameState"
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        return search.uniformCostSearch(problem)

class AnyFoodSearchProblem(PositionSearchProblem):
    """
      A search problem for finding a path to any food.

      This search problem is just like the PositionSearchProblem, but
      has a different goal test, which you need to fill in below.  The
      state space and successor function do not need to be changed.

      The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
      inherits the methods of the PositionSearchProblem.

      You can use this search problem to help you fill in
      the findPathToClosestDot method.
    """

    def __init__(self, gameState):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test
        that will complete the problem definition.
        """
        x,y = state

        return self.food[x][y]

##################
# Mini-contest 1 #
##################

class ApproximateSearchAgent(Agent):
    "Implement your contest entry here.  Change anything but the class name."

    def registerInitialState(self, state):
        "This method is called before any moves are made."
        "*** YOUR CODE HERE ***"

    def getAction(self, state):
        """
        From game.py:
        The Agent will receive a GameState and must return an action from
        Directions.{North, South, East, West, Stop}
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built.  The gameState can be any game state -- Pacman's position
    in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + point1
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False)
    return len(search.bfs(prob))
