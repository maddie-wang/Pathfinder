// Assignment 7: Trailblazer
// Maddie Wang, 12/5/2017.
// Trailblazer searches for paths between two nodes in a graph represented by HashMaps, mazes, terrains, and worlds.
// To do so, it implements four path-searching algorithms: Depth First Search, Breadth First Search, Dijkstra's Algorithm,
// and A*. Trailblazer also generates random mazes though a minimum spanning tree algorithm, Kruskal.

#include "trailblazer.h"
#include "stack.h"
#include "pqueue.h"
#include "hashmap.h"
#include "hashset.h"
#include "queue.h"

using namespace std;
bool edgesConnect(Edge* current, Edge* edge);
bool isCycle(Edge* current, HashMap<Vertex*, int> &cluster, HashMap<int, Set<Vertex*>> &clusterHashMap);
void updateClusters(Edge* current, HashMap<Vertex*, int> &cluster, HashMap<int, Set<Vertex*>> &clusterHashMap, int hashCounter);
bool depthHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*> &path, HashSet<Vertex*> &visited);
Vector<Vertex*> generatePath(Vertex* end);

// depthFirstSearch will find a path in the graph from the starting position to the ending position in a world using
// depth first search. It will return the Vextex* path it finds from start to end; if there is no valid path, it will return an empty path.
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    HashSet<Vertex*> visited;
    if (depthHelper(graph, start, end, path, visited)) { // if a path from start to end exists, return such path
        return path;
    }
    else { // otherwise return empty path
        return path;
    }
}
// depthHelper uses DFS recursive backtracking to determine if a path exists from start to end. It will modify path
// and return true if such a path exists.
// Color Key: Green = Nodes currently inspecting. Grey = Nodes that didn't lead to the end vertex.
bool depthHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*> &path, HashSet<Vertex*> &visited) {
    if (visited.contains(start)) { // base case failure: we have already visited the current position
        return false;
    }
    if (start == end) { // base case success: we landed on our ending vertex in our depth search
        start->setColor(GREEN);
        path.add(start);
        return true;
    }
    path.add(start);
    start->setColor(GREEN);
    visited.add(start);
    for (Vertex* v : graph.getNeighbors(start)) { // recursively looks @ neighbors
        if (depthHelper(graph, v, end, path, visited)) // if found path to ending vertex, return true
            return true;
    }
    start->setColor(GRAY); // backtracking: did not find path from vertex,
    visited.remove(start); // removes our current node from the path and from visited HashSet.
    path.remove(path.size()-1);
    return false;
}

// breadthFirstSearch will find the shortest path in the graph from the starting position to the ending position in a world using
// breadth first search. It will return the Vextex* path it finds from start to end; if there is no valid path, it will return an empty path.
// Color Key: Yellow = Enqueued nodes. Green = Dequeued Nodes.
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Queue<Vertex*> toExplore;
    toExplore.add(start);
    start->setColor(YELLOW);
    while (!toExplore.isEmpty()) { // eventually iterates through every vertex reachable by start
        Vertex* current = toExplore.dequeue();
        current->setColor(GREEN);
        if (current == end) { // if current vertex is the ending vertex
            return generatePath(end); // generates path of vertexes by following pointers from end to start
        }
        for (Edge* edge : current->edges) { // Enqueues all of current's neighbors to explore
            if (edge->finish->getColor()!=GREEN && edge->finish->getColor()!=YELLOW) { // checks if not visited
                edge->finish->setColor(YELLOW);
                toExplore.enqueue(edge->finish);
                edge->finish->previous = current; // Each of current's neighbors points back to current
            }
        }
    }
    Vector<Vertex*> path;
    return path; // else, the ending position was impossible to reach; returns empty path
}

// generatePath generates a Vertex* path by following the pointers from the ending vertex to the starting vertex,
// adding each of those vertexes it encounters in a stack, and outputing them in referse order. The Vertex* path
// it returns is organized in a vector, Vertices listed from start to finish.
Vector<Vertex*> generatePath(Vertex* end) {
    Stack<Vertex*> vertexStack;
    Vector<Vertex*> path;
    while (end!=NULL) { // follows pointers from end to starting vertex and adds them to a stack
        vertexStack.add(end);
        end = end->previous;
    }
    while (!vertexStack.isEmpty()) { // path adds the vertices from start to end
        path.add(vertexStack.pop());
    }
    return path;

}

// dijkstrasAlgorithm finds the shortest path from a starting node to a target node in a weighted graph
// It will return the Vextex* path it finds from start to end; if there is no valid path, it will return an empty path.
// Color Key: Yellow = Enqueued nodes. Green = Dequeued Nodes.
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    PriorityQueue<Vertex*> pq; // using a priority queue so that the vertices with lowest cost will always be first to dequeue
    start->cost = 0;
    start->setColor(YELLOW);
    pq.enqueue(start, 0); // enqueues starting vertex with starting cost 0
    while (!pq.isEmpty()) { // eventually iterates through every vertex reachable by start
        Vertex* current = pq.dequeue();
        current->setColor(GREEN);
        if (current == end) { // if current vertex is the ending vertex
            return generatePath(end); // generates path of vertexes by following pointers from end to start
        }
        for (Edge* edge : current->edges) { // for current's neighbors
            if (edge->finish->getColor()==UNCOLORED) { // case 1: if neighbor = uncolored, set its cost to the cost to get from current
                // to neighbor and enqueue it in priority queue with that cost
                edge->finish->setColor(YELLOW);
                edge->finish->cost = current->cost + edge->cost;
                edge->finish->previous = current; // neighbor points to current
                pq.enqueue(edge->finish, edge->finish->cost);
            }
            else if (edge->finish->getColor()==YELLOW && edge->finish->cost > current->cost + edge->cost) { // case 2: if neighbor = enqueued and
                // the cost to get there from current is cheaper than the present cost to get there, set its cost to the cost to get from
                // current to neighbor and enqueue it in priority queue with that cost
                edge->finish->cost = current->cost + edge->cost;
                edge->finish->previous = current;
                pq.changePriority(edge->finish, edge->finish->cost);

            }
        }
    }
    Vector<Vertex*> path;
    return path; // else, the ending position was impossible to reach; returns empty path
}

// aStar uses heuristics to help find the shortest path from starting vertex to ending vertex in a weighted graph. It
// will return the Vextex* path it finds from start to end; if there is no valid path, it will return an empty path.
// Color Key: Yellow = Enqueued nodes. Green = Dequeued Nodes.
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    PriorityQueue<Vertex*> pq; // using a priority queue so that the vertices with lowest cost will always be first to dequeue
    start->cost = 0;
    start->setColor(YELLOW);
    pq.enqueue(start, heuristicFunction(start, end)); // enqueues starting vertex with starting cost of its heuristic from start
    while (!pq.isEmpty()) { // eventually iterates through every vertex reachable by start
        Vertex* current = pq.dequeue();
        current->setColor(GREEN);
        if (current == end) { // if current vertex is the ending vertex
            return generatePath(end); // generates path of vertexes by following pointers from end to start
        }
        for (Edge* edge : current->edges) { // for current's neighbors
            if (edge->finish->getColor()==UNCOLORED) { // case 1: if neighbor = uncolored, set its cost to the cost to get from current
                // to neighbor and enqueue it in priority queue with that cost + neighbors's heuristic cost
                edge->finish->setColor(YELLOW);
                edge->finish->cost = current->cost + edge->cost;
                edge->finish->previous = current; // neighbor points to current
                pq.enqueue(edge->finish, edge->finish->cost + heuristicFunction(edge->finish, end));
            }
            else if (edge->finish->getColor()==YELLOW && edge->finish->cost > current->cost + edge->cost) { // case 2: if neighbor = enqueued and
                // the cost to get there from current is cheaper than the present cost to get there, set its cost to the cost to get from
                // current to neighbor and enqueue it in priority queue with that cost + neighbors's heuristic cost
                edge->finish->cost = current->cost + edge->cost;
                edge->finish->previous = current; // neighbor points to current
                pq.changePriority(edge->finish, edge->finish->cost + heuristicFunction(edge->finish, end));

            }
        }
    }
    Vector<Vertex*> path;
    return path; // else, the ending position was impossible to reach; returns empty path
}

// kruskal finds the minimum spanning tree in a given weighted graph. It returns the edges of the minimum spanning tree (edgeSet)
// Note: huge is generated around 12 seconds
Set<Edge*> kruskal(BasicGraph& graph) {
    graph.resetData();
    PriorityQueue<Edge*> pq; // using a priority queue so that the edges with lowest cost will always be first to dequeue
    Set<Edge*> edgeSet;
    int hashCounter = 1;
    HashMap<Vertex*, int> cluster;
    HashMap<int, Set<Vertex*>> clusterHashMap;
    for (Edge* edge : graph.getEdgeSet()) { // enqueues all edges
        pq.add(edge, edge->cost);
    }
    while (!pq.isEmpty()) { // iterates through / checks every edge
        Edge* current = pq.dequeue();
        if (!isCycle(current, cluster, clusterHashMap)) { //If does not form a cycle, add the edge to the edge set and update clusters
            edgeSet.add(current);
            updateClusters(current, cluster, clusterHashMap, hashCounter);
            hashCounter++; // increment hashCounter so that every generated cluster has their own unique counter
        }
    }
    return edgeSet;
}

// isCycle determines whether or not an Edge forms a cycle given two HashMaps
bool isCycle(Edge* current, HashMap<Vertex*, int> &cluster, HashMap<int, Set<Vertex*>> &clusterHashMap) {
    // checks if current->start exists and whether current->start's integer that it maps to in cluster
    // maps to a Set of Vertex* in clusterHashMap that contains current->finish.
    if (cluster.containsKey(current->start) && clusterHashMap[cluster[current->start]].contains(current->finish)) {
        return true;
    }
    return false;
}

// updateClusters updates the cluster and clusterHashMap maps with current depending on whether or not current is part of a cluster
// or combines two clusters. For reference, cluster maps Vertex* to int, and clusterHashMap maps int to a Set<Vertex*>.
// In this implementation, a Vertex* in cluster corresponds to an int, and that int maps to a Set of Vertex* it connects to.
// Meaning, multiple Vertices that map to the same int have the same Set of Vertex* - Thus they are all connected.
void updateClusters (Edge* current, HashMap<Vertex*, int> &cluster, HashMap<int, Set<Vertex*>> &clusterHashMap, int hashCounter) {
    bool start = cluster.containsKey(current->start); // checks if start and finish are in clusters (true if they are)
    bool finish = cluster.containsKey(current->finish);
    if (start && finish) { // if they are both in clusters, combine them since our current edge joins two clusters
        clusterHashMap[hashCounter] = clusterHashMap[cluster[current->finish]] + clusterHashMap[cluster[current->start]];
    }
    else if (start) { // else if only start is in a cluster, combine start's set of vertices it connects to with the finish vertex
        clusterHashMap[hashCounter] = clusterHashMap[cluster[current->start]] + current->finish;
    }
    else if (finish) { // else if only finish is in a cluster, combine finish's set of vertices it connects to with the start vertex
        clusterHashMap[hashCounter] = clusterHashMap[cluster[current->finish]] + current->start;
    }
    else { // else the edge isn't connected to any cluster, so make a new Set with the edge's vertices
        Set<Vertex*> tempSet;
        tempSet.add(current->start);
        tempSet.add(current->finish);
        clusterHashMap[hashCounter] = tempSet;
    }
    for (Vertex* v : clusterHashMap[hashCounter]) { // map every vertex* in the combined set to the hashCounter.
        // now, the hashCounter maps to the connected set of vertices and the vertices in that cluster all map to the hashCounter
        cluster[v] = hashCounter;
    }
}
