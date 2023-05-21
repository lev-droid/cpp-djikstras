// Algorithm.hpp
#pragma once
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "Graph.hpp"
#include "Node.hpp"

#include <functional>

// Update the callback type definition to include visited nodes and neighbors
using DijkstraCallback = std::function<void(std::shared_ptr<Node>, const std::vector<std::shared_ptr<Node>>&, const std::vector<std::shared_ptr<Node>>&, const std::vector<std::shared_ptr<Node>>&)>;
class Algorithm {
public:

    explicit Algorithm(Graph& graph);
    bool dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, DijkstraCallback callback = nullptr);
    bool aStar(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);
    bool bellmanFord(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);
    void renderPath(sf::RenderWindow& window) const;
    void setCurrentPath(const std::vector<std::shared_ptr<Node>>& path);
    void resetPath();
    void renderStep(sf::RenderWindow& window);
    void resetStep();

    const std::vector<std::shared_ptr<Node>>& getPath() const;
    float getPathLength() const;
    float getExecutionTime() const;

    size_t getStepIndex() const;
    void setStepIndex(size_t stepIndex);



private:
    Graph& graph_;
    std::vector<std::shared_ptr<Node>> path_;
    std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>> steps;
    std::unordered_set<std::shared_ptr<Node>> processedNodes;
    std::vector<std::shared_ptr<Node>> currentPath;
    std::vector<std::vector<std::shared_ptr<Node>>> currentPaths;


    float pathLength_;
    float executionTime_;
    size_t stepIndex_;

};