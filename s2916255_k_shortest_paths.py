import sys
import time
###################
# Heap Functions. #
###################
# Returns the index of the parent for the given node.
# Input: queue -> A list representing a priority queue (heap).
#        index -> An integer, the index of a particular node.
def GetParentIndex(queue, index):
    return (index - 1) // 2

# Returns the index of the left child for the given node. 
# If the child index is larger than the queue length (i.e., the child does not exist),
# return None.
# Input: queue -> A list representing a priority queue (heap).
#        index -> An integer, the index of a particular node.
def GetLeftChildIndex(queue, index):
    leftChildIndex = (2 * index) + 1
    return leftChildIndex if leftChildIndex <= len(queue) - 1 else None

# Returns the index of the right child for the given node. 
# If the child index is larger than the queue length (i.e., the child does not exist),
# return None.
# Input: queue -> A list representing a priority queue (heap).
#        index -> An integer, the index of a particular node.
def GetRightChildIndex(queue, index):
    rightChildIndex = (2 * index) + 2
    return rightChildIndex if rightChildIndex <= len(queue) - 1 else None

# Returns the index of a given node by it's character name.
# Takes linear time, but there was no better solution.
# Input: queue -> A list representing a priority queue (heap).
#        name -> A character that represents the name of the node to be found in the heap.
def GetElementIndex(queue, name):
    for node in range(0, len(queue)):
        if queue[node][0] == name:
            return node
    return None
# Inserts a node into the queue using the downheap operation.
# Input: queue -> A list representing a priority queue (heap).
#        node -> A list with a character and tuple in the form of [vertexName, (currentPathLength, penultimateVertex)].
def InsertNode(queue, node):
    # Add the node to the end of the heap, get it's current information.
    queue.append(node)
    # Check if this is the first node in the heap.
    # If so, just return.
    if (len(queue) == 1):
        return queue
    # Get the current node's information.
    currentNodeIndex = len(queue) - 1
    currentNodePriority = node[1]
    # Get the parent's information.
    parentIndex = GetParentIndex(queue, currentNodeIndex)
    parentPriority = queue[parentIndex][1]
    # While the node is not the root node and it has a lesser priority than it's current parent,
    # move it up through the heap.
    while currentNodeIndex > 0 and parentPriority > currentNodePriority:
        # Swap the nodes.
        queue[currentNodeIndex], queue[parentIndex] = queue[parentIndex], queue[currentNodeIndex]
        # Get the new information.
        currentNodeIndex = GetParentIndex(queue, currentNodeIndex)
        currentNodePriority = queue[currentNodeIndex][1]
        parentIndex = GetParentIndex(queue, currentNodeIndex)
        parentPriority = queue[parentIndex][1]
    # After the node has been inserted at the right place, return the queue.
    return queue

# Performs a downheapify on a node at any given index, placing the node in it's correct position.
# Input: queue -> A list representing a priority queue (heap).
#        index -> An integer, the index of a particular node.
def Heapify(queue, inputIndex):
    leftChildIndex = GetLeftChildIndex(queue, inputIndex)
    rightChildIndex = GetRightChildIndex(queue, inputIndex)
    largestPriority = queue[inputIndex][1]
    smallestChildIndex = inputIndex
    if leftChildIndex != None:
        leftChildPriority = queue[leftChildIndex][1]
        if leftChildPriority < largestPriority:
            smallestChildIndex = leftChildIndex
            largestPriority = leftChildPriority
    if rightChildIndex != None:
        rightChildPriority = queue[rightChildIndex][1]
        if rightChildPriority < largestPriority:
            smallestChildIndex = rightChildIndex
            largestPriority = rightChildPriority
    if smallestChildIndex != inputIndex:
        queue[inputIndex], queue[smallestChildIndex] = queue[smallestChildIndex], queue[inputIndex]
        return Heapify(queue, smallestChildIndex)
    else:
        return queue

# Removes the minimum node in the heap (i.e. the root node) and returns it.
# Input: queue -> A list representing a priority queue (heap).
def DeleteMinimum(queue):
    # If there is only one node in the heap, just return that and exit.
    if len(queue) == 1:
        returnNode = queue.pop(0)
        return returnNode, queue
    # If there is more than one node, have to do a longer process.
    # Remove the root node, replace it with the last element.
    returnNode = queue[0]
    queue[0] = queue[-1]
    queue.pop()
    # Do a downheapify on the root node.
    queue = Heapify(queue, 0)
    # Return the popped node.
    return returnNode, queue

def DecreaseNode(queue, vertexIndex):
    parentIndex = GetParentIndex(queue, vertexIndex)
    if vertexIndex == 0 or queue[vertexIndex][1] > queue[parentIndex][1]:
        return queue
    else:
        queue[vertexIndex], queue[parentIndex] = queue[parentIndex], queue[vertexIndex]
        return DecreaseNode(queue, parentIndex)

############################
# Miscellaneous Functions. #
############################
# Function to create the graph and return it.
# Input: A text file.
def CreateGraph(inputFile):
    tmpGraph = {}
    tmpOrigin = ''
    tmpDestination = ''
    tmpPathCount = 0
    lineCounter = 0
    edgeCounter = 0
    # Use lineCounter to make sure we're getting the right lines out of the file.
    for line in inputFile:
        line = line.split(" ")
        # Get the number of edges from the input so we know where to stop for the final line.
        if lineCounter == 0:
            edgeCounter = int(line[1]) + 1
        # If we are passed the intial line and not at the last.
        elif lineCounter > 0 and lineCounter < edgeCounter:
            # If either of the vertices in the edge are not recorded, then recorded them in the graph.
            if line[0] not in tmpGraph.keys():
                tmpGraph[line[0]] = {}
            if line[1] not in tmpGraph.keys():
                tmpGraph[line[1]] = {}
            # Then also record the edge into the graph.
            tmpGraph[line[0]][line[1]] = float(line[2])
        # Get the origin and destination vertices as well as the number of paths to find.
        else:
            tmpOrigin = line[0]
            tmpDestination = line[1]
            tmpPathCount = int(line[2])
        lineCounter += 1
    return tmpGraph, tmpOrigin, tmpDestination, tmpPathCount

def CreateShortestPath(destinationVertex, path):
    totalCost = 0
    nextNode = destinationVertex
    tmpPath = [path[nextNode][0]]
    while nextNode != None:
        tmpPath.append(nextNode)
        # print(path[nextNode])
        nextNode = path[nextNode][1]
    return list(reversed(tmpPath))
# A version of Dijkstra's algorithm for finding the shortest path from one vertex to another in a graph.
# Input: inputGraph -> A hash table of hash tables to represent the problems graph as an adjacency list.
#        sourceVertex -> A character, the name of the vertex to start the path at.
def Dijkstra(inputGraph, sourceVertex, destinationVertex):
    priorityQueue = []
    path = {}

    for vertex in inputGraph:
        if vertex != sourceVertex:
            priorityQueue = InsertNode(priorityQueue, [vertex, sys.maxsize, None])
    priorityQueue = InsertNode(priorityQueue, [sourceVertex, 0, None])

    for vertex in range(0, len(priorityQueue) - 1):
        nextNode, priorityQueue = DeleteMinimum(priorityQueue)
        path[nextNode[0]] = nextNode[1:]
        adjacentVerticies = inputGraph[nextNode[0]]

        for vertex in adjacentVerticies.keys():
            
            vertexIndex = GetElementIndex(priorityQueue, vertex)
            if vertexIndex != None:
                pathCost = nextNode[1] + inputGraph[nextNode[0]][vertex]

                if pathCost < priorityQueue[vertexIndex][1]:
                    priorityQueue[vertexIndex][1] = pathCost
                    priorityQueue[vertexIndex][2] = nextNode[0]

                    if len(priorityQueue) > 1:
                        priorityQueue = DecreaseNode(priorityQueue, vertexIndex)
    path = CreateShortestPath(destinationVertex, path)
    return path

def GetRootPathCost(graph, path):
    cost = 0
    if len(path) == 1:
        return 0
    else:
        for vertex in range(0, len(path) - 1):
            cost += graph[path[vertex]][path[vertex + 1]]
    return cost

def KShortestPaths(graph, sourceVertex, destinationVertex, k):
    shortestPaths = [Dijkstra(graph, sourceVertex, destinationVertex)]
    potentialShortestPaths = []

    for pathCount in range(1, k):
        for node in range(0, len(shortestPaths[pathCount - 1]) - 2):
            spurNode = shortestPaths[-1][node]
            rootPath = shortestPaths[-1][:node + 1]
            rootPathCost = GetRootPathCost(graph, rootPath)
            removedEdges = []

            for path in shortestPaths:
                if rootPath == path[:node + 1]:
                    if str(path[node + 1]) in graph[str(path[node])].keys():
                        removedEdges.append([str(path[node]), str(path[node + 1]), graph[str(path[node])][str(path[node + 1])]])
                        del graph[str(path[node])][str(path[node + 1])]
                    else:
                        continue
            spurPath = Dijkstra(graph, spurNode, destinationVertex)
            spurPath[-1] += rootPathCost
            totalPath = rootPath[:-1] + spurPath
            potentialShortestPaths = InsertNode(potentialShortestPaths, [totalPath, totalPath[-1]])
            for edge in removedEdges:
                graph[edge[0]][edge[1]] = float(edge[2])
        minPath, potentialShortestPaths = DeleteMinimum(potentialShortestPaths)
        shortestPaths.append(minPath[0])
        
    return shortestPaths

###################
# Driver Function #
###################
# Open the input file, use it to create the graph and extract data, then close it.
inputFile = open(sys.argv[1])
print("Choose your output option as either 0 or 1:")
print("0). Display only the path costs.")
print("1). Display the paths taken as well as costs.")
outputType = int(input())
graph, originVertex, destinationVertex, numberOfPaths = CreateGraph(inputFile)
inputFile.close()
startTime = time.time()
endTime = time.time()
paths = KShortestPaths(graph, originVertex, destinationVertex, numberOfPaths)
print("Results:")
if outputType == 0:
    for path in paths:
        print("Path cost:", end=" ")
        print(path[-1], end=", ")
        print("in", endTime - startTime, "seconds.")
else:
    for path in paths:
        print("Path:", end=" ")
        print(path[:-1], end=". ")
        print("Cost:", end=" ")
        print(path[-1], end=", ")
        print("in", endTime - startTime, "seconds.")
