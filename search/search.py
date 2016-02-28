# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # print "Start: ", problem.getStartState()    
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())

    # if problem.isGoalState(problem.getStartState()):
    #     return []

    stack = util.Stack()
    """ Stack consider State, actions, state visited """
    stack.push((problem.getStartState(), [], []))
    while not stack.isEmpty():
        state, actions, visited = stack.pop()

        if problem.isGoalState(state):
            return actions
        visited += [state]

        for nextState, action, cost in problem.getSuccessors(state):
            # print nextState, action
            if not nextState in visited:
                stack.push((nextState, actions + [action], visited))

    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # if problem.isGoalState(problem.getStartState()):
    #     return []

    queue = util.Queue()
    queue.push((problem.getStartState(), []))    
    visited = [problem.getStartState()]

    while not queue.isEmpty():
        state, actions = queue.pop()
        if problem.isGoalState(state):
            # print len(actions)+1
            return actions

        for nextState, action, cost in problem.getSuccessors(state):
            if not nextState in visited:                
                visited.append(nextState)
                queue.push((nextState, actions + [action]))

    return []
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    heap = util.PriorityQueue()
    start = problem.getStartState()
    heap.push(start, 0)

    came_from = {}
    actions_cost = {}
    came_from[start] = None
    actions_cost[start] = 0
    
    while not heap.isEmpty():
        state = heap.pop()

        if problem.isGoalState(state):
            return recontruct_actions(came_from, start, state)

        for nextState, action, cost in problem.getSuccessors(state):
            newcost = actions_cost[state] + cost
            if nextState not in actions_cost or actions_cost[nextState] > newcost:
                actions_cost[nextState] = newcost
                priority = newcost
                heap.push(nextState, priority)
                came_from[nextState] = (state, action)

    return []
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def recontruct_actions(came_from, start, goal):
    current = goal
    actions = []
    while current != start:
        action = came_from[current][1]   # action to move from before current state to current state
        current = came_from[current][0]  # before current state
        actions.append(action)
    actions.reverse()
    return actions

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    heap = util.PriorityQueue()
    start = problem.getStartState()
    heap.push(start, 0)

    came_from = {}
    actions_cost = {}
    came_from[start] = None
    actions_cost[start] = 0
        
    while not heap.isEmpty():
        state = heap.pop()

        if problem.isGoalState(state):
            return recontruct_actions(came_from, start, state)

        for nextState, action, cost in problem.getSuccessors(state):
            newcost = actions_cost[state] + cost
            if nextState not in actions_cost or actions_cost[nextState] > newcost:
                actions_cost[nextState] = newcost
                priority = newcost + heuristic(nextState, problem)
                heap.push(nextState, priority)
                came_from[nextState] = (state, action)

    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
