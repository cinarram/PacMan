# search.py
# ---------

"""
Generic search algorithms which are called by Pacman agents (in searchAgents.py).
"""

import util
from Queue import Queue
from collections import deque

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
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
  """
  def depthFirstSearchAlg(v, explored=set(),path=[]):
    explored.add(v)
    if problem.isGoalState(v):
      return path
    succ = problem.getSuccessors(v)
    for entry in succ:
      node = entry[0]
      direction = entry[1]
      if node not in explored:
        path.append(direction)
        x = depthFirstSearchAlg(node, explored, path)
        if x:
          return x
    path.pop()

  return depthFirstSearchAlg(problem.getStartState())
    

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  """
  
  def breadthFirstSearchAlg(start):
    # a FIFO open_set
    open_set = deque()
    # an empty set to maintain visited nodes
    closed_set = set()
    # a dictionary to maintain meta information (used for path formation)
    meta = dict()  # key -> (parent state, action to reach child)

    # initialize
    assert start is not None
    meta[start] = (None, None)
    open_set.append(start)

    while not not open_set:

      parent_state = open_set.popleft()

      if problem.isGoalState(parent_state):
        return construct_path(parent_state, meta)

      for (child_state, action, cost) in problem.getSuccessors(parent_state):

        if child_state in closed_set:
          continue

        if child_state not in open_set:
          meta[child_state] = (parent_state, action)
          open_set.append(child_state)

      closed_set.add(parent_state)


  def construct_path(state, meta):
    action_list = list()
    
    while True:
        row = meta[state]
        if row[1] is not None:
          state = row[0]
          action = row[1]
          action_list.append(action)
          print action_list
        else:
          break
  
    action_list.reverse()
    return action_list

  T = breadthFirstSearchAlg(problem.getStartState())
  return T
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  Q = set()
  dist = dict()
  prev = dict()

  def Init(parent_state):
    for (child_state, action, cost) in problem.getSuccessors(parent_state):  
      if child_state not in Q:
        dist[child_state] = 999999
        prev[child_state] = 0
        Q.add(child_state)
      else:
        continue
      Init(child_state)

  start = problem.getStartState()
  Init(start)
  dist[start] = 0
  
  while not not Q:
    parent_state = min(dist, key=dist.get)
    Q.remove(parent_state)
    if problem.isGoalState(parent_state):
      S = []
      u = parent_state
      while prev[u] is not None:
        S.insert(u)
        u = prev[u]
      S.insert(u)
      return S
    for (child_state, action, cost) in problem.getSuccessors(parent_state):
      alt = problem.getCostOfActions()
      if alt < dist[child_state]:
        dist[child_state] = alt
        prev[child_state] = parent_state
  return dist, prev

  print S
  return S
    
    

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
