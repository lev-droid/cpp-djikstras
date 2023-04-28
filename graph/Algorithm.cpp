#include "Algorithm.hpp"
#include "Utility.hpp"

Algorithm::Algorithm(Graph& graph) : graph_(graph) {}

bool Algorithm::dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode) {
    path_.clear();

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
                previous[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }

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

const std::vector<std::shared_ptr<Node>>& Algorithm::getPath() const {
    return path_;
}
