#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

// Structure to represent a node and its distance from the source
struct Node {
    int id;
    int distance;

    Node(int _id, int _distance) : id(_id), distance(_distance) {}
};

// Comparator for the priority queue
struct NodeComparator {
    bool operator()(const Node& a, const Node& b) {
        return a.distance > b.distance;
    }
};

int networkDelayTime(vector<vector<int>>& times, int n, int k) {
    // Created an adjacency list to represent the directed graph
    vector<vector<pair<int, int>>> graph(n + 1);
    for (const auto& time : times) {
        int u = time[0], v = time[1], w = time[2];
        graph[u].emplace_back(v, w);
    }

    // Initialized distances to infinity
    vector<int> distances(n + 1, INT_MAX);
    distances[k] = 0;

    // Priority queue to get the node with the minimum distance
    priority_queue<Node, vector<Node>, NodeComparator> pq;
    pq.emplace(k, 0);

    while (!pq.empty()) {
        int currentNode = pq.top().id;
        int currentDistance = pq.top().distance;
        pq.pop();

        // Skip if the distance to the current node is greater than the recorded distance
        if (currentDistance > distances[currentNode])
            continue;

        // Traversed the neighbors of the current node
        for (const auto& neighbor : graph[currentNode]) {
            int neighborNode = neighbor.first;
            int neighborDistance = neighbor.second;

            // Updated the distance if a shorter path is found
            if (currentDistance + neighborDistance < distances[neighborNode]) {
                distances[neighborNode] = currentDistance + neighborDistance;
                pq.emplace(neighborNode, distances[neighborNode]);
            }
        }
    }

    // Finding the maximum distance among all nodes
    int maxDistance = 0;
    for (int i = 1; i <= n; ++i) {
        if (distances[i] == INT_MAX)
            return -1;
        maxDistance = max(maxDistance, distances[i]);
    }

    return maxDistance;
}

int main() {

    vector<vector<int>> times = {{2, 1, 1}, {2, 3, 1}, {3, 4, 1}};

    int n = 4;
    int k = 2;

    int result = networkDelayTime(times, n, k);
    cout << "Minimum time for all nodes to receive the signal : " << result << endl;

    return 0;
}
