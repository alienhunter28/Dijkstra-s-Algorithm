#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

struct Edge {
    int to;
    int weight;
};

int dijkstraShortestPathCost(const vector<vector<Edge>>& graph, int start, int end) {
    int numNodes = graph.size();
    vector<int> distance(numNodes, numeric_limits<int>::max());
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    distance[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        int currentDistance = pq.top().first;
        pq.pop();

        if (currentDistance > distance[current]) {
            continue;
        }

        for (const Edge& edge : graph[current]) {
            int neighbor = edge.to;
            int newDistance = distance[current] + edge.weight;

            if (newDistance < distance[neighbor]) {
                distance[neighbor] = newDistance;
                pq.push({newDistance, neighbor});
            }
        }
    }

    return distance[end];
}

vector<int> dijkstraShortestPath(const vector<vector<Edge>>& graph, int start, int end) {
    int numNodes = graph.size();
    vector<int> distance(numNodes, numeric_limits<int>::max());
    vector<int> parent(numNodes, -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    distance[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        int currentDistance = pq.top().first;
        pq.pop();

        if (currentDistance > distance[current]) {
            continue;
        }

        for (const Edge& edge : graph[current]) {
            int neighbor = edge.to;
            int newDistance = distance[current] + edge.weight;

            if (newDistance < distance[neighbor]) {
                distance[neighbor] = newDistance;
                parent[neighbor] = current;
                pq.push({newDistance, neighbor});
            }
        }
    }

    vector<int> path;
    for (int at = end; at != -1; at = parent[at]) {
        path.push_back(at);
    }

    reverse(path.begin(), path.end());

    return path;
}

int main() {
    vector<vector<Edge>> graph = {
        {{1, 10}, {4,3 }},
        {{2, 2},{4,4}},
        {{3, 9}},
        {{2,7}},
        {{1,1},{2,8},{3,2}}
    };

    int cost = dijkstraShortestPathCost(graph, 0, 2);
    cout << "Cost of the shortest path: " << cost << endl;

    vector<int> path = dijkstraShortestPath(graph, 0, 2);
    cout << "Path of the lowest cost: ";
    for (int node : path) {
        cout << node << " ";
    }
    cout << endl;

    return 0;
}
