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
    return [s, s, w, s, w, w, s, w]

def generalGraphSearch(problem, structure):
    """
    Defines a general algorithm to search a graph.
    Parameters are structure, which can be any data structure with .push() and .pop() methods, and problem, which is the
    search problem.
    """

    # Push the root node/start into the data structure in this format: [(state, action taken, cost)]
    # The list pushed into the structure for the second node will look something like this:
    # [(root_state, "Stop", 0), (new_state, "North", 1)]


def depthFirstSearch(problem):
    dfsStack = util.Stack()
    path = []
    currentState = problem.getStartState()
    if problem.isGoalState(currentState):
        return []
    dfsStack.push([(currentState, path)])
    visited = set()
    while not dfsStack.isEmpty():
        path = dfsStack.pop() #list of tuples, firts element of tuple is the current state and second element is the action)
        currentState = path[-1][0]  #the first element in the last tuple is our current state
        if problem.isGoalState(currentState):
            actions = [action[1] for action in path][1:] #return the actions (second elements of path's tuples) except the first one
            return actions
        visited.add(currentState)
        for s in problem.getSuccessors(currentState):
            if s[0] not in visited: #if successor's state is not visited
                sPath = path[:] + [s]  #successor's path is the original path plus the successor node
                dfsStack.push(sPath)

    return False  #in case goal state doesn't get found


# util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE *+**"
    bfsQueue = util.Queue()
    visited = set()
    path = []
    currentState = problem.getStartState()
    if problem.isGoalState(currentState):
        return []
    bfsQueue.push((problem.getStartState(), []))
    while not bfsQueue.isEmpty():
        pop = bfsQueue.pop()
        currentState = pop[0]
        path = pop[1]  #add action to the path
        visited.add(currentState)
        if problem.isGoalState(currentState):
            return path
        for s in problem.getSuccessors(currentState):
            if s[0] not in visited and s[0] not in (state[0] for state in bfsQueue.list):
                sPath = path + [s[1]]
                bfsQueue.push((s[0], sPath))
    return False

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    ucsPQ = util.PriorityQueue()
    currentState = problem.getStartState()
    ucsPQ.push((currentState, []), 0)
    if problem.isGoalState(currentState):
        return []
    visited = set()
    while not ucsPQ.isEmpty():
        pop = ucsPQ.pop()
        state = pop[0]
        path = pop[1]
        if problem.isGoalState(state):
            return path
        visited.add(state)
        for s in problem.getSuccessors(state):
            sPath = path + [s[1]]
            if s[0] not in visited and s[0] not in (state[2][0] for state in ucsPQ.heap):
                cost = problem.getCostOfActions(sPath)
                ucsPQ.push((s[0], sPath), cost)
            elif s[0] not in visited:   #not in visited but in the heap
                for state in ucsPQ.heap:
                    if state[2][0] == s[0]:
                        oldCost = problem.getCostOfActions(state[2][1])
                        currentCost = problem.getCostOfActions(sPath)
                        if currentCost < oldCost:
                            ucsPQ.update((s[0], sPath), currentCost)

    return False

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    astarPQ = util.PriorityQueue()
    astarPQ.push((problem.getStartState(), []), 0)
    visited =set()
    while not astarPQ.isEmpty():
        pop = astarPQ.pop()
        currentState = pop[0]
        path = pop[1]
        if problem.isGoalState(currentState):
            return path
        visited.add(currentState)
        for s in problem.getSuccessors(currentState):
            sPath=path+[s[1]]
            if s[0] not in visited and s[0] not in (state[2][0] for state in astarPQ.heap):
                cost = problem.getCostOfActions(sPath)
                astarPQ.push((s[0], sPath), cost + heuristic(s[0], problem))
            elif s[0] not in visited:
                for state in astarPQ.heap:
                    if state[2][0] == s[0]:
                        oldCost = problem.getCostOfActions(state[2][1])
                        currentCost = problem.getCostOfActions(sPath)
                        sHeuristic = heuristic(s[0], problem)
                        if currentCost + heuristic(s[0], problem) < oldCost + sHeuristic:
                            astarPQ.update((s[0], sPath), currentCost + sHeuristic)
    return False

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
