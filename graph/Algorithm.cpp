#include "Algorithm.hpp"
#include "Utility.hpp"
#include <chrono>
Algorithm::Algorithm(Graph& graph) : graph_(graph), pathLength_(0), executionTime_(0) {}

bool Algorithm::dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode) {
    path_.clear();
    auto startTime = std::chrono::high_resolution_clock::now();

    std::unordered_map<std::shared_ptr<Node>, float> distances;
    std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<Node>> previous;

    for (const auto& node : graph_.getNodes()) {
        distances[node] = std::numeric_limits<float>::infinity();
    }
    distances[startNode] = 0;

    auto cmp = [&distances](const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) {
        return distances[lhs] > distances[rhs];
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(cmp)> queue(cmp);

    queue.push(startNode);

    while (!queue.empty()) {
        auto current = queue.top();
        queue.pop();

        if (current == endNode) {
            // Calculate path length
            pathLength_ = 0;
            while (current != startNode) {
                path_.push_back(current);
                pathLength_ += euclideanDistance(current->getPosition(), previous[current]->getPosition());
                current = previous[current];
            }
            path_.push_back(startNode);
            std::reverse(path_.begin(), path_.end());
            return true;
        }


        for (const auto& edge : current->getEdges()) {
            auto neighbor = (edge->getStartNode() == current) ? edge->getEndNode() : edge->getStartNode();
            float newDistance = distances[current] + edge->getWeight();

            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
                previous[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    executionTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    return false;
}

bool Algorithm::aStar(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode) {
    path_.clear();

    std::unordered_map<std::shared_ptr<Node>, float> distances;
    std::unordered_map<std::shared_ptr<Node>, float> costs;
    std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<Node>> previous;

    for (const auto& node : graph_.getNodes()) {
        distances[node] = std::numeric_limits<float>::infinity();
    }
    distances[startNode] = 0;
    costs[startNode] = euclideanDistance(startNode->getPosition(), endNode->getPosition());

    auto cmp = [&costs](const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) {
        return costs[lhs] > costs[rhs];
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(cmp)> queue(cmp);

    queue.push(startNode);

    while (!queue.empty()) {
        auto current = queue.top();
        queue.pop();

        if (current == endNode) {
            while (current != startNode) {
                path_.push_back(current);
                current = previous[current];
            }
            path_.push_back(startNode);
            std::reverse(path_.begin(), path_.end());
            return true;
        }

        for (const auto& edge : current->getEdges()) {
            auto neighbor = (edge->getStartNode() == current) ? edge->getEndNode() : edge->getStartNode();
            float newDistance = distances[current] + edge->getWeight();

            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
                costs[neighbor] = newDistance + euclideanDistance(neighbor->getPosition(), endNode->getPosition());
                previous[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }

    return false;
}

bool Algorithm::bellmanFord(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode) {
    path_.clear();
    auto startTime = std::chrono::high_resolution_clock::now();

    std::unordered_map<std::shared_ptr<Node>, float> distances;
    std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<Node>> previous;

    for (const auto& node : graph_.getNodes()) {
        distances[node] = std::numeric_limits<float>::infinity();
    }
    distances[startNode] = 0;

    for (size_t i = 0; i < graph_.getNodes().size() - 1; ++i) {
        for (const auto& edge : graph_.getEdges()) {
            auto nodeA = edge->getStartNode();
            auto nodeB = edge->getEndNode();
            float weight = edge->getWeight();

            if (distances[nodeA] != std::numeric_limits<float>::infinity() && distances[nodeA] + weight < distances[nodeB]) {
                distances[nodeB] = distances[nodeA] + weight;
                previous[nodeB] = nodeA;
            }

            if (distances[nodeB] != std::numeric_limits<float>::infinity() && distances[nodeB] + weight < distances[nodeA]) {
                distances[nodeA] = distances[nodeB] + weight;
                previous[nodeA] = nodeB;
            }
        }
    }

    // Check for negative weight cycles
    for (const auto& edge : graph_.getEdges()) {
        auto nodeA = edge->getStartNode();
        auto nodeB = edge->getEndNode();
        float weight = edge->getWeight();

        if (distances[nodeA] + weight < distances[nodeB] || distances[nodeB] + weight < distances[nodeA]) {
            return false;
        }
    }

    if (previous.find(endNode) == previous.end()) {
        return false;
    }

    // Calculate path length
    pathLength_ = 0;
    std::shared_ptr<Node> current = endNode;
    while (current != startNode) {
        path_.push_back(current);
        pathLength_ += euclideanDistance(current->getPosition(), previous[current]->getPosition());
        current = previous[current];
    }
    path_.push_back(startNode);
    std::reverse(path_.begin(), path_.end());

    auto endTime = std::chrono::high_resolution_clock::now();
    executionTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    return true;
}


const std::vector<std::shared_ptr<Node>>& Algorithm::getPath() const {
    return path_;
}

float Algorithm::getPathLength() const {
    return pathLength_;
}

float Algorithm::getExecutionTime() const {
    return executionTime_;
}