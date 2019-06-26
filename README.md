# Pathfinding-with-Yen-and-Dijkstra

An interesting problem that my lecture came up with.

**Background:**

Melanie is a second-year computer science student currently studying at Griffith School of ICT. One day, Melanie’s father came to her and told her that he wants to travel from Southport to Brisbane CBD and asks Melanie to recommend exactly K-shortest loopless paths for some reason. Luckily, Melanie is undertaking the “2801ICT: Computing Algorithms” course convened by Dr. Saiful Islam. 

Though, Melanie knows both Dijkstra’s algorithm and Bellman Ford’s algorithm very well as these two algorithms have been taught in Week 10, she is not exactly sure how to solve this K-shortest loopless paths problem for the Queensland road network graph. She then discussed this problem with Dr. Saiful. Dr. Saiful thinks that this is an interesting problem and it should be solved by each student in his class including Melanie as part of Assignment 3. 

Melanie’s father is happy if and only if the first recommended path is the shortest path between Southport to Brisbane CBD and the rest of the K-1 paths are approximated shortest paths as these paths are his backup paths.  

**Problem Description:**

The shortest path problem is about finding a path between 2 vertices in a graph such that the total sum of the edges weights is minimum. This problem could be solved easily using (BFS) if all edge weights were 1, but here weights can take any value. 

There are TWO popular algorithms in the literature for computing the shortest paths from the source vertex to all other vertices in a graph: (i) Dijkstra's Algorithm; and (ii) Bellman Ford’s Algorithm. The Bellman Ford’s algorithm exploits the idea that a shortest path contains at most K − 1 edges, and a shortest path cannot have a cycle.

The Bellman Ford’s algorithm works for graphs with negative weight cycle(s). The time complexity of Dijkstra's Algorithm with min-priority queue is O(V + ElogV). On the other hand, the time complexity of Bellman Ford algorithm is relatively high O(VE), in case E = (V^2), O(V^3). Both Dijkstra's and Bellman Ford’s algorithms unable to solve the this K-shortest loopless paths (KSP) problem. 

There exist many works in the literature to solve KSP problem. Some useful references are given at the end of this document. In this problem, you are given 2 integers (N, M), N is the number of vertices, M is the number of edges. 
