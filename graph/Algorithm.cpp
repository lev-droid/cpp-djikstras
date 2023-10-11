/*
Algorithm.cpp
This source file contains the implementations of the member functions of the Algorithm class.

Member Functions
Algorithm::Algorithm(Graph& graph): Constructs an Algorithm object with a reference to a Graph object.
bool Algorithm::dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, DijkstraCallback callback): Performs Dijkstra's algorithm on the graph starting from the startNode and ending at the endNode. Optionally, a callback function can be provided to receive updates during the algorithm execution.
bool Algorithm::aStar(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode): Performs the A* algorithm on the graph starting from the startNode and ending at the endNode.
bool Algorithm::bellmanFord(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode): Performs the Bellman-Ford algorithm on the graph starting from the startNode and ending at the endNode.
void Algorithm::renderPath(sf::RenderWindow& window) const: Renders the path found by the algorithm on the specified sf::RenderWindow.
void Algorithm::renderStep(sf::RenderWindow& window, sf::Color processedNodeColor, sf::Color currentNodeColor): Renders the current step of the algorithm on the specified sf::RenderWindow with the given colors for processed nodes and the current node.
void Algorithm::setCurrentPath(const std::vector<std::shared_ptr<Node>>& path): Sets the current path to the specified vector of nodes.
void Algorithm::addCurrentPath(const std::vector<std::shared_ptr<Node>>& path): Adds the specified path to the list of current paths.
void Algorithm::resetPath(): Resets the current path.
void Algorithm::addProcessedNode(const std::shared_ptr<Node>& node): Adds the specified node to the set of processed nodes.
const std::unordered_set<std::shared_ptr<Node>>& Algorithm::getProcessedNodes() const: Returns the set of processed nodes.
void Algorithm::resetProcessedNodes(): Resets the set of processed nodes.
void Algorithm::addStep(const std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>& step): Adds the specified step (pair of nodes) to the list of steps.
const std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>>& Algorithm::getSteps() const: Returns the list of steps.
void Algorithm::clearSteps(): Clears the list of steps.
const std::vector<std::shared_ptr<Node>>& Algorithm::getPath() const: Returns the path found by the algorithm.
float Algorithm::getPathLength() const: Returns the length of the path found by the algorithm.
float Algorithm::getExecutionTime() const: Returns the execution time of the algorithm.
size_t Algorithm::getStepIndex() const: Returns the current step index.
void Algorithm::setStepIndex(size_t stepIndex): Sets the current step index.
*/



#include "Algorithm.hpp"
#include "Utility.hpp"
#include <chrono>
#include <iostream>
#include <random>


Algorithm::Algorithm(Graph& graph) : graph_(graph), pathLength_(0), executionTime_(0), stepIndex_(0) {
}





bool Algorithm::dijkstra(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, DijkstraCallback callback) {
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

            // Send the final path to the callback
            if (callback != nullptr) {
                callback(current, {}, {}, path_);
            }

            return true;
        }

        for (const auto& edge : current->getEdges()) {
            auto neighbor = (edge->getStartNode() == current) ? edge->getEndNode() : edge->getStartNode();
            float newDistance = distances[current] + edge->getWeight();

            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
                previous[neighbor] = current;
                queue.push(neighbor);

                if (callback != nullptr) {
                    std::vector<std::shared_ptr<Node>> partialPath;
                    auto currentNode = neighbor;
                    while (currentNode != startNode && previous.find(currentNode) != previous.end()) {
                        partialPath.push_back(currentNode);
                        currentNode = previous[currentNode];
                    }
                    partialPath.push_back(startNode);
                    std::reverse(partialPath.begin(), partialPath.end());
                    callback(current, { current }, { neighbor }, partialPath);
                    addCurrentPath(partialPath);  // Add this line
                }
            }
        }
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    executionTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    return false;
}



void Algorithm::renderPath(sf::RenderWindow& window) const {
    sf::VertexArray pathLines(sf::LinesStrip, currentPath.size());
    for (std::size_t i = 0; i < currentPath.size(); ++i) {
        pathLines[i].position = currentPath[i]->getPosition();
        pathLines[i].color = sf::Color::Red;
    }
    window.draw(pathLines);
}




void Algorithm::renderStep(sf::RenderWindow& window, sf::Color processedNodeColor, sf::Color currentNodeColor) {
    // Draw the current step
    if (!steps.empty() && stepIndex_ < steps.size()) {
        const auto& step = steps[stepIndex_];

        if (step.first && step.second) {
            sf::VertexArray stepLine(sf::Lines, 2);
            stepLine[0].position = step.first->getPosition();
            stepLine[0].color = sf::Color::Red;
            stepLine[1].position = step.second->getPosition();
            stepLine[1].color = sf::Color::Red;
            window.draw(stepLine);

            if (!font_.loadFromFile("arial.ttf")) {
                std::cerr << "Failed to load font file." << std::endl;
            }


            sf::Text stepText;
            stepText.setFont(font_);  // Replace 'yourFont' with the appropriate font
            stepText.setString("Step " + std::to_string(stepIndex_)); // Display step index
            stepText.setCharacterSize(12); // Set text size
            stepText.setFillColor(sf::Color::White); // Set text color
            stepText.setPosition(step.second->getPosition().x, step.second->getPosition().y - 20); // Adjust position as needed
            window.draw(stepText);

        }
     
        // Color processed nodes


        // Color current node
        if (step.first) {
            step.first->setColor(currentNodeColor);
        }
    }

    if (!currentPaths.empty() && stepIndex_ < currentPaths.size()) {
        const auto& currentPath = currentPaths[stepIndex_];
        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
            if (stepIndex_ < steps.size()) {
                resetPathLine();
                pathLine[0].position = currentPath[i]->getPosition();
                pathLine[0].color = sf::Color::Red;
                pathLine[1].position = currentPath[i + 1]->getPosition();
                pathLine[1].color = sf::Color::Red;
                window.draw(pathLine);

                sf::Text identifierText;
                identifierText.setFont(font_); // Replace 'yourFont' with the desired font
                identifierText.setCharacterSize(12); // Adjust the font size as needed
                identifierText.setFillColor(sf::Color::White); // Text color
                identifierText.setString("Path " + std::to_string(stepIndex_)); // Display step index
                identifierText.setPosition(pathLine[0].position.x, pathLine[0].position.y - 20); // Adjust position as needed
                window.draw(identifierText);

            }

        }

    }
    if (stepIndex_ >= steps.size() && !currentPath.empty()) {
        sf::VertexArray pathLines(sf::LinesStrip, currentPath.size());
        for (std::size_t i = 0; i < currentPath.size(); ++i) {
            pathLines[i].position = currentPath[i]->getPosition();
            pathLines[i].color = sf::Color::Red;
        }
        window.draw(pathLines);
    }
    for (const auto& node : processedNodes) {
        if (node) {
            node->setColor(processedNodeColor);
        }
    }

}


//Steps
void Algorithm::addStep(const std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>& step) {
    steps.push_back(step);
}

const std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>>& Algorithm::getSteps() const {
    return steps;
}


void Algorithm::clearSteps() {
    steps.clear();
}

//Nodes
void Algorithm::addProcessedNode(const std::shared_ptr<Node>& node) {
    processedNodes.insert(node);
}
const std::unordered_set<std::shared_ptr<Node>>& Algorithm::getProcessedNodes() const {
    return processedNodes;
}


void Algorithm::resetProcessedNodes() {
    processedNodes.clear();
}


//Paths
void Algorithm::addCurrentPath(const std::vector<std::shared_ptr<Node>>& path) {
    currentPaths.push_back(path);
}


void Algorithm::setCurrentPath(const std::vector<std::shared_ptr<Node>>& path) {
    currentPath = path;
}

void Algorithm::resetPath() {
    currentPath.clear();

}

void Algorithm::resetPathLine() {
    pathLine.clear();
    pathLine.resize(2);
}


size_t Algorithm::getStepIndex() const {
    return stepIndex_;
}

void Algorithm::setStepIndex(size_t stepIndex) {
    stepIndex_ = stepIndex;
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