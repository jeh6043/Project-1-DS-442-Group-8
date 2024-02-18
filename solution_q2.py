from queue import LifoQueue
from collections import deque
import heapq

class Node:
    def __init__(self, state, parent = None, action = None, cost = 0, heuristic = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    
def readInput(filePath):
    input = []
    with open(filePath, 'r') as file:
        read = file.read()
        for node in read.split(','):
            if node == '_':
                input.append(None)
            else:
                input.append(int(node))
    return input

def goalState(state):
    if state[0] and state[1] and state[2]:
        if state[0] + state[1] + state[2] == 11:
            return True
    return False

def expand(state):
    successors = []
    actions = [(0, -1, 'D'), (0, 1, 'U'), (-1, 0, 'L'), (1, 0, 'R')]
    noneIndex = state.index(None)
    for action in actions:
        newIndex = noneIndex + action[0] + 3 * action[1]
        if 0 <= newIndex < 9:
            newState = state[:]
            newState[noneIndex], newState[newIndex] = newState[newIndex], newState[noneIndex]
            if action[2] == 'L':
                successors.append((newState, newState[noneIndex], 'R'))
            elif action[2] == 'R':
                successors.append((newState, newState[noneIndex], 'L'))
            else:
                successors.append((newState, newState[noneIndex], action[2]))
    return successors

def manhattanDistance(state):
    distance = 0
    for node in range(1, 9):
        nodeIndex = state.index(node)
        goalIndex = node - 1
        distance += abs(nodeIndex // 3 - goalIndex // 3) + abs(nodeIndex % 3 - goalIndex % 3)
    return distance

def straightLineDistance(state):
    distance = 0
    for node in range(1, 9):
        nodeIndex = state.index(node)
        goalIndex = node - 1
        distance += ((nodeIndex // 3 - goalIndex // 3) ** 2 + (nodeIndex % 3 - goalIndex % 3) ** 2) ** 0.5
    return distance

def dfs(initialState):
    closed = set()
    fringe = LifoQueue()
    fringe.put(Node(initialState))
    while not fringe.empty():
        node = fringe.get()
        nodeState = node.state
        if goalState(nodeState):
            reversePath = []
            while node.parent:
                reversePath.append(node.action)
                node = node.parent
            path = reversePath[::-1]
            print(','.join(path[1:]), end = ' ')
            return
        if tuple(nodeState) not in closed:
            closed.add(tuple(nodeState))
            successors = expand(nodeState)
            for successor in successors:
                fringe.put(Node(successor[0], node, f'{successor[1]}{successor[2]}'))
    return None

def bfs(initialState):
    closed = set()
    fringe = deque()
    fringe.append(Node(initialState))
    while fringe:
        node = fringe.popleft()
        nodeState = node.state
        if goalState(nodeState):
            reversePath = []
            while node.parent:
                reversePath.append(node.action)
                node = node.parent
            path = reversePath[::-1]
            print(','.join(path), end = ' ')
            return
        if tuple(nodeState) not in closed:
            closed.add(tuple(nodeState))
            successors = expand(nodeState)
            for successor in successors:
                fringe.append(Node(successor[0], node, f'{successor[1]}{successor[2]}'))
    return None

def ucs(initialState):
    closed = set()
    fringe = []
    heapq.heappush(fringe, Node(initialState, cost = 0))
    while fringe:
        node = heapq.heappop(fringe)
        nodeState = node.state
        if goalState(nodeState):
            reversePath = []
            while node.parent:
                reversePath.append(node.action)
                node = node.parent
            path = reversePath[::-1]
            print(','.join(path), end = ' ')
            return
        if tuple(nodeState) not in closed:
            closed.add(tuple(nodeState))
            successors = expand(nodeState)
            for successor in successors:
                cost = node.cost + 1
                heapq.heappush(fringe, Node(successor[0], node, f'{successor[1]}{successor[2]}', cost))
    return None

def aStar(initialState):
    closed = set()
    fringe = []
    heapq.heappush(fringe, Node(initialState, cost = 0, heuristic = 0))
    while fringe:
        node = heapq.heappop(fringe)
        nodeState = node.state
        if goalState(nodeState):
            reversePath = []
            while node.parent:
                reversePath.append(node.action)
                node = node.parent
            path = reversePath[::-1]
            print(','.join(path), end = ' ')
            return
        if tuple(nodeState) not in closed:
            closed.add(tuple(nodeState))
            successors = expand(nodeState)
            for successor in successors:
                cost = node.cost + 1
                heuristic = manhattanDistance(nodeState)
                heapq.heappush(fringe, Node(successor[0], node, f'{successor[1]}{successor[2]}', cost, heuristic))
    return None

def aStarStraightLine(initialState):
    closed = set()
    fringe = []
    heapq.heappush(fringe, Node(initialState, cost = 0, heuristic = 0))
    while fringe:
        node = heapq.heappop(fringe)
        nodeState = node.state
        if goalState(nodeState):
            reversePath = []
            while node.parent:
                reversePath.append(node.action)
                node = node.parent
            path = reversePath[::-1]
            print(','.join(path), end = ' ')
            return
        if tuple(nodeState) not in closed:
            closed.add(tuple(nodeState))
            successors = expand(nodeState)
            for successor in successors:
                cost = node.cost + 1
                heuristic = straightLineDistance(nodeState)
                heapq.heappush(fringe, Node(successor[0], node, f'{successor[1]}{successor[2]}', cost, heuristic))
    return None

def main():
    initialState = readInput(r"C:\Users\Yunus\Downloads\input.txt")
    print('The solution of Q1.1(DFS) is:')
    dfsPath = dfs(initialState)
    if dfsPath:
        for action in dfsPath:
            print(action)
    print('\n')
    print('The solution of Q1.2(BFS) is:')
    bfsPath = bfs(initialState)
    if bfsPath:
        for action in bfsPath:
            print(action)
    print('\n')
    print('The solution of Q1.3(UCS) is:')
    ucsPath = ucs(initialState)
    if ucsPath:
        for action in ucsPath:
            print(action)
    print('\n')
    print('The solution of Q1.4(A* search) is:')
    aStarPath = aStar(initialState)
    if aStarPath:
        for action in aStarPath:
            print(action)
main()