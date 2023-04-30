// Algorithm.hpp
#pragma once
#include <memory>
#include <unordered_map>
#include <queue>
#include "Graph.hpp"
#include "Node.hpp"

class Algorithm {
public:
    explicit Algorithm(Graph& graph);
    bool dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);
    bool aStar(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);
    bool bellmanFord(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);

    const std::vector<std::shared_ptr<Node>>& getPath() const;
    float getPathLength() const;
    float getExecutionTime() const;

private:
    Graph& graph_;
    std::vector<std::shared_ptr<Node>> path_;
    float pathLength_;
    float executionTime_;
};
