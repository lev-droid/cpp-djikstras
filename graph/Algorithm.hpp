/*
Algorithm.hpp
This header file defines the Algorithm class, which provides implementations for various graph algorithms. It includes necessary dependencies such as Graph.hpp and Node.hpp.

Class: Algorithm
Constructors
Algorithm(Graph& graph): Constructs an Algorithm object with a reference to a Graph object.
Algorithms
bool dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, DijkstraCallback callback = nullptr): Performs Dijkstra's algorithm on the graph starting from the startNode and ending at the endNode. Optionally, a callback function can be provided to receive updates during the algorithm execution.
bool aStar(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode): Performs the A* algorithm on the graph starting from the startNode and ending at the endNode.
bool bellmanFord(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode): Performs the Bellman-Ford algorithm on the graph starting from the startNode and ending at the endNode.
Renderers
void renderPath(sf::RenderWindow& window) const: Renders the path found by the algorithm on the specified sf::RenderWindow.
void renderStep(sf::RenderWindow& window, sf::Color processedNodeColor, sf::Color currentNodeColor): Renders the current step of the algorithm on the specified sf::RenderWindow with the given colors for processed nodes and the current node.
Setters, Getters, and Resetters
void setCurrentPath(const std::vector<std::shared_ptr<Node>>& path): Sets the current path to the specified vector of nodes.

void addCurrentPath(const std::vector<std::shared_ptr<Node>>& path): Adds the specified path to the list of current paths.

void resetPath(): Resets the current path.

void addProcessedNode(const std::shared_ptr<Node>& node): Adds the specified node to the set of processed nodes.

const std::unordered_set<std::shared_ptr<Node>>& getProcessedNodes() const: Returns the set of processed nodes.

void resetProcessedNodes(): Resets the set of processed nodes.

void addStep(const std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>& step): Adds the specified step (pair of nodes) to the list of steps.

const std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>>& getSteps() const: Returns the list of steps.

void clearSteps(): Clears the list of steps.

const std::vector<std::shared_ptr<Node>>& getPath() const: Returns the path found by the algorithm.

float getPathLength() const: Returns the length of the path found by the algorithm.

float getExecutionTime() const: Returns the execution time of the algorithm.

size_t getStepIndex() const: Returns the current step index.

void setStepIndex(size_t stepIndex): Sets the current step index.


*/


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
    //Algorithms
    bool dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, DijkstraCallback callback = nullptr);

    //Renderers
    void renderPath(sf::RenderWindow& window) const;
    void renderStep(sf::RenderWindow& window, sf::Color processedNodeColor, sf::Color currentNodeColor);

    //setters getters and resetterse
    void setCurrentPath(const std::vector<std::shared_ptr<Node>>& path);
    void addCurrentPath(const std::vector<std::shared_ptr<Node>>& path);
    void resetPath();

    void addProcessedNode(const std::shared_ptr<Node>& node);
    const std::unordered_set<std::shared_ptr<Node>>& getProcessedNodes() const;
    void resetProcessedNodes();

    void addStep(const std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>& step);
    const std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>>& getSteps() const;
    void clearSteps();


    const std::vector<std::shared_ptr<Node>>& getPath() const;
    float getPathLength() const;
    float getExecutionTime() const;

    size_t getStepIndex() const;
    void setStepIndex(size_t stepIndex);
    void resetPathLine();



private:
    Graph& graph_;
    std::vector<std::shared_ptr<Node>> path_;
    std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>> steps;
    std::unordered_set<std::shared_ptr<Node>> processedNodes;
    std::vector<std::shared_ptr<Node>> currentPath;
    std::vector<std::vector<std::shared_ptr<Node>>> currentPaths;
    std::shared_ptr<sf::Text> pathName;
    sf::VertexArray pathLine;
    sf::Font font_;

    float pathLength_;
    float executionTime_;
    size_t stepIndex_;

};

