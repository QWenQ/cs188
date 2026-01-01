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

# @profile
def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    """
    a node in the frontier:
        node.state: current state
        node.parent: parent node that generates this node
        node.action: action taken to reach this node from parent
    how to avoid repeated states: P87
    """
    start = (problem.getStartState(), None, None)  # state, parent, action 
    frontier = util.Stack()
    frontier.push(start)
    reached = set()

    def Expand(problem: SearchProblem, node: tuple):
        state = node[0]
        for successor, action, _ in problem.getSuccessors(state):
            yield (successor, node, action)

    while not frontier.isEmpty():
        node = frontier.pop()
        reached.add(node[0])
        if problem.isGoalState(node[0]):
            actions = []
            while node[1] is not None:
                actions.append(node[2])
                node = node[1]
            actions.reverse()
            return actions
        for child in Expand(problem, node):
            if child[0] not in reached:
                frontier.push(child)
    return []
    


# @profile
def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    """
    a node in the frontier:
        node.state: current state
        node.parent: parent node that generates this node
        node.action: action taken to reach this node from parent
    how to avoid repeated states: P87
    """
    start = (problem.getStartState(), None, None)  # state, parent, action, path_cost
    frontier = util.Queue()
    frontier.push(start)
    reached = {start[0], }

    # @profile
    def Expand(problem: SearchProblem, node: tuple):
        state = node[0]
        for successor, action, _ in problem.getSuccessors(state):
            yield (successor, node, action)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            actions = []
            while node[1] is not None:
                actions.append(node[2])
                node = node[1]
            actions.reverse()
            return actions
        for child in Expand(problem, node):
            state = child[0]
            if state not in reached:
                reached.add(state)
                frontier.push(child)
    return []


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    """
    a node in the frontier:
        node.state: current state
        node.parent: parent node that generates this node
        node.action: action taken to reach this node from parent
        node.path_cost: total cost from start state to this node
    how to avoid repeated states: P87
    """
    start = (problem.getStartState(), None, None, 0)  # state, parent, action, path_cost
    frontier = util.PriorityQueue()
    frontier.push(start[0], start[3])
    reached = { start[0]: start }

    def Expand(problem: SearchProblem, node: tuple):
        state = node[0]
        for successor, action, step_cost in problem.getSuccessors(state):
            path_cost = node[3] + step_cost
            yield (successor, node, action, path_cost)

    while not frontier.isEmpty():
        state = frontier.pop()
        node = reached[state]
        if problem.isGoalState(node[0]):
            actions = []
            while node[1] is not None:
                actions.append(node[2])
                node = node[1]
            actions.reverse()
            return actions
        for child in Expand(problem, node):
            if child[0] not in reached or child[3] < reached[child[0]][3]:
                reached[child[0]] = child
                frontier.update(child[0], child[3])
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    """
    a node in the frontier:
        node.state: current state
        node.parent: parent node that generates this node
        node.action: action taken to reach this node from parent
        node.path_cost: total cost from start state to this node
        node.heuristic: heuristic value of this node
    how to avoid repeated states: P87
    """
    start = (problem.getStartState(), None, None, 0, heuristic(problem.getStartState(), problem))
    frontier = util.PriorityQueue()
    frontier.push(start[0], start[3] + start[4])
    reached = { start[0]: start }

    def Expand(problem: SearchProblem, node: tuple):
        state = node[0]
        for successor, action, step_cost in problem.getSuccessors(state):
            path_cost = node[3] + step_cost
            yield (successor, node, action, path_cost, heuristic(successor, problem))

    while not frontier.isEmpty():
        state = frontier.pop()
        node = reached[state]
        if problem.isGoalState(node[0]):
            actions = []
            while node[1] is not None:
                actions.append(node[2])
                node = node[1]
            actions.reverse()
            return actions
        for child in Expand(problem, node):
            if child[0] not in reached or (child[3] + child[4]) < (reached[child[0]][3]+ reached[child[0]][4]):
                reached[child[0]] = child
                frontier.update(child[0], child[3] + child[4])
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
