# multiAgents.py
# --------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from util import manhattanDistance
from game import Directions
import random, util, math

from game import Agent

class ReflexAgent(Agent):
  """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
  """


  def getAction(self, gameState):
    """
    You do not need to change this method, but you're welcome to.

    getAction chooses among the best options according to the evaluation function.

    Just like in the previous project, getAction takes a GameState and returns
    some Directions.X for some X in the set {North, South, West, East, Stop}
    """
    # Collect legal moves and successor states
    legalMoves = gameState.getLegalActions()

    # Choose one of the best actions
    scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
    bestScore = max(scores)
    bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
    chosenIndex = random.choice(bestIndices) # Pick randomly among the best

    "Add more of your code here if you want to"

    return legalMoves[chosenIndex]

  def evaluationFunction(self, currentGameState, action):
    """
    Design a better evaluation function here.

    The evaluation function takes in the current and proposed successor
    GameStates (pacman.py) and returns a number, where higher numbers are better.

    The code below extracts some useful information from the state, like the
    remaining food (newFood) and Pacman position after moving (newPos).
    newScaredTimes holds the number of moves that each ghost will remain
    scared because of Pacman having eaten a power pellet.

    Print out these variables to see what you're getting, then combine them
    to create a masterful evaluation function.
    """
    # Useful information you can extract from a GameState (pacman.py)
    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    newFood = successorGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    food = []

    for i,row in enumerate(newFood):
        for j,square in enumerate(row):
            if square:
                food.append((i,j))
    if len(food) == 0:
      return float('inf')

    #return manhattan distance to farthest pellet
    closest = float('inf')
    for a in food:
        closest = min(closest,dist(a, newPos))


    ghosts = 0
    for ghost in newGhostStates:
      ghosts = ghosts + dist(newPos, ghost.getPosition())

    if ghosts <= 1:
      ghosts = -float('inf')
    else:
      ghosts = math.log(ghosts-1)

    return (0-closest-1000*len(food)) + ghosts

def dist(position, goal):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])

def scoreEvaluationFunction(currentGameState):
  """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
  """
  return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
  """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
  """

  def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
    self.index = 0 # Pacman is always agent index 0
    self.evaluationFunction = util.lookup(evalFn, globals())
    self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
  """
    Your minimax agent (question 2)
  """

  def getAction(self, gameState):
    """
      Returns the minimax action from the current gameState using self.depth
      and self.evaluationFunction.

      Here are some method calls that might be useful when implementing minimax.

      gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

      Directions.STOP:
        The stop direction, which is always legal

      gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

      gameState.getNumAgents():
        Returns the total number of agents in the game
    """
    if 0 is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf'), None
    for paction in pactions:
      result = self.minGetAction(gameState.generateSuccessor(0, paction), 0, 1)
      if result > best[0]:
        best = result, paction

    return best[1]


  def maxGetAction(self, gameState, plies):
    if plies is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf')
    for paction in pactions:
      best = max(best, self.minGetAction(gameState.generateSuccessor(0, paction), plies, 1))
  
    return best

  def minGetAction(self, gameState, plies, i):
    ghostions = gameState.getLegalActions(i)
    if len(ghostions) is 0:
      return self.evaluationFunction(gameState)

    worst = float('inf')
    if i+1 is gameState.getNumAgents():
      for ghostion in ghostions:
        worst = min(worst, self.maxGetAction(gameState.generateSuccessor(i, ghostion), plies+1))
    else:
      for ghostion in ghostions:
        worst = min(worst, self.minGetAction(gameState.generateSuccessor(i, ghostion), plies, i+1))
    return worst

class AlphaBetaAgent(MultiAgentSearchAgent):
  """
    Your minimax agent with alpha-beta pruning (question 3)
  """

  def getAction(self, gameState):
    """
      Returns the minimax action using self.depth and self.evaluationFunction
    """
    if 0 is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf'), None
    for paction in pactions:
      result = self.minGetAction(gameState.generateSuccessor(0, paction), 0, 1, best[0])
      if result > best[0]:
        best = result, paction

    return best[1]


  def maxGetAction(self, gameState, plies, worst):
    if plies is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf')
    for paction in pactions:
      best = max(best, self.minGetAction(gameState.generateSuccessor(0, paction), plies, 1, best))
      if best > worst:
        return best
  
    return best

  def minGetAction(self, gameState, plies, i, best):
    ghostions = gameState.getLegalActions(i)
    if len(ghostions) is 0:
      return self.evaluationFunction(gameState)

    worst = float('inf')
    if i+1 is gameState.getNumAgents():
      for ghostion in ghostions:
        worst = min(worst, self.maxGetAction(gameState.generateSuccessor(i, ghostion), plies+1, worst))
        if worst < best:
          return worst
    else:
      for ghostion in ghostions:
        worst = min(worst, self.minGetAction(gameState.generateSuccessor(i, ghostion), plies, i+1, best))
        if worst < best:
          return worst
    return worst

class ExpectimaxAgent(MultiAgentSearchAgent):
  """
    Your expectimax agent (question 4)
  """

  def getAction(self, gameState):
    """
      Returns the expectimax action using self.depth and self.evaluationFunction

      All ghosts should be modeled as choosing uniformly at random from their
      legal moves.
    """
    if 0 is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf'), None
    for paction in pactions:
      result = self.expGetAction(gameState.generateSuccessor(0, paction), 0, 1)
      if result > best[0]:
        best = result, paction
    #print best
    return best[1]


  def maxGetAction(self, gameState, plies):
    if plies is self.depth:
      return self.evaluationFunction(gameState)

    pactions = gameState.getLegalActions(0)
    if len(pactions) is 0:
      return self.evaluationFunction(gameState)

    best = -float('inf')
    for paction in pactions:
      best = max(best, self.expGetAction(gameState.generateSuccessor(0, paction), plies, 1))
  
    return best

  def expGetAction(self, gameState, plies, i):
    ghostions = gameState.getLegalActions(i)
    if len(ghostions) is 0:
      return self.evaluationFunction(gameState)

    worst = 0
    numGhosts = gameState.getNumAgents()-1
    if i is numGhosts:
      for ghostion in ghostions:
        worst = worst + self.maxGetAction(gameState.generateSuccessor(i, ghostion), plies+1)/numGhosts
    else:
      for ghostion in ghostions:
        worst = worst +  self.expGetAction(gameState.generateSuccessor(i, ghostion), plies, i+1)/numGhosts
    return worst

def betterEvaluationFunction(currentGameState):
  """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
  """
  "*** YOUR CODE HERE ***"

  newPos = currentGameState.getPacmanPosition()
  newFood = currentGameState.getFood()
  newGhostStates = currentGameState.getGhostStates()
  newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

  food = []

  for i,row in enumerate(newFood):
      for j,square in enumerate(row):
          if square:
              food.append((i,j))
  if len(food) == 0:
    return 999999999 + currentGameState.getScore()

  #return manhattan distance to farthest pellet
  closest = float('inf')
  for a in food:
      closest = min(closest,dist(a, newPos))


  ghosts = 0
  for ghost in newGhostStates:
    distance = dist(newPos, ghost.getPosition())
    val = 0-math.exp(0-(distance-4))
    ghosts = ghosts + val

  return 0 - closest - 20*len(food) + ghosts + currentGameState.getScore()

  #util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction

class ContestAgent(MultiAgentSearchAgent):
  """
    Your agent for the mini-contest
  """

  def getAction(self, gameState):
    """
      Returns an action.  You can use any method you want and search to any depth you want.
      Just remember that the mini-contest is timed, so you have to trade off speed and computation.

      Ghosts don't behave randomly anymore, but they aren't perfect either -- they'll usually
      just make a beeline straight towards Pacman (or away from him if they're scared!)
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
