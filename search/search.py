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
    #Search the deepest nodes in the search tree first.

    start = problem.getStartState()  # where pacman starts the search
    visited = []  # empty list for adding visited nodes
    frontier = util.Stack()  # the stack is the data structure used in dfs algorithm
    frontier.push((start, []))  # push start state and action in queue

    while not frontier.isEmpty(): # while stack isn't empty do:
        state, actions = frontier.pop()  # i bring out last node that was added (state<-start, actions<-[] (first iteration))
        if state not in visited: # if this node is not visited do:
            visited.append(state) #I marked it as visited by putting its state in visited list

            if problem.isGoalState(state):  # exit if we reach the goal(if current state is our goal)
                return actions # return the moves computed, so the pacman should follow them to reach the food

            successor = problem.getSuccessors(state)  # succ[0]=parentXY succ[1]=action

            for s in successor: # for each parent(neighbour) do:
                frontier.push((s[0], actions + [s[1]]))
                # s[0] are parent coordinates and s[1] is direction  and actions are all the directions till that point

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    start = problem.getStartState()  # start state from where pacman starts the search
    visited = []  # emply list for adding visited nodes
    frontier = util.Queue()  # the queue is the data structure used in bfs algorithm
    frontier.push((start, []))  # push start state and action in queue

    while not frontier.isEmpty(): # while queue isn't empty do:
        state, actions = frontier.pop()  # i bring out last node that was added (state<-start, actions<-[] (first iteration))
        if state not in visited: # if this node is not visited do:
            visited.append(state) #I marked it as visited by putting its state in visited list
            if problem.isGoalState(state):  # exit if we reach the goal(if current state is our goal)
                return actions # return the moves computed, so the pacman should follow them to reach the food
            successor = problem.getSuccessors(state)  # succ[0]=parentXY succ[1]=action

            for s in successor: # for each parent(neighbour) do:
                frontier.push((s[0], actions + [s[1]]))
                # s[0] are parent coordinates and s[1] is direction  and actions are all the directions till that point

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    start = problem.getStartState() # start state from where pacman starts the search
    visited = [] # emply list for adding visited nodes
    frontier = util.PriorityQueue() # priority queue chose the node with the lowest cost # has 3 parameters: current state, actions, cost
    frontier.push((start, []), 0) #push start state, actions till this point=[], cost=0

    while not frontier.isEmpty(): # while priority queue isn't empty do:
        state, actions,  = frontier.pop() # I bring out last node that was added (state<-start, actions<-[] (first iteration), cost<-_)
        #i don't need to use third parameter(cost) because this will pe computed with function getCostOfActions
        if problem.isGoalState(state):  # exit if we reach the goal(if current state is our goal)
            return actions # return the moves computed, so the pacman should follow them to reach the food
        if state not in visited: # if this node isn't visited do:
            visited.append(state) #I marked it as visited by putting its state in visited list
            successors = problem.getSuccessors(state) # succ[0]=parentXY succ[1]=action

            for s in successors: # for each parent(neighbour) do:
                     frontier.push((s[0], actions + [s[1]]), problem.getCostOfActions(actions + [s[1]]))
                     # push to priority queue this node with its proprieties:
                     # s[0] = parent coordinates, actions + s[1] = all actions + directions for reaching parent, cost

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    start = problem.getStartState() # start state from where pacman starts the search
    visited = [] # emply list for adding visited nodes
    frontier = util.PriorityQueue() # priority queue chose the node with the lowest cost # has 3 parameters: current state, actions, nullHeuristic
    frontier.push((start, []), nullHeuristic(start, problem)) #push start state, actions till this point=[], heuristic from start to start...

    while not frontier.isEmpty(): # while priority queue isn't empty do:
        state, actions, = frontier.pop() # I bring out last node that was added (state<-start, actions<-[] (first iteration), nullHeuristic<-_)

        if problem.isGoalState(state): # exit if we reach the goal(if current state is our goal)
            return actions # return the moves computed, so the pacman should follow them to reach the food

        if state not in visited: # if this node isn't visited do:
            visited.append(state) #I marked it as visited by putting its state in visited list
            successors = problem.getSuccessors(state) # succ[0]=parentXY succ[1]=action

            for s in successors: # for each parent(neighbour) do:
                    frontier.push((s[0], actions + [s[1]]), problem.getCostOfActions(actions + [s[1]]) + heuristic(s[0], problem))
                    # push to priority queue this node with its proprieties:
                    #first parameter: parent, all actions till now + direction to parent, cost+nullHeuristic

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
