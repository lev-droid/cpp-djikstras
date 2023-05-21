#include "Algorithm.hpp"
#include "Utility.hpp"
#include <chrono>


Algorithm::Algorithm(Graph& graph) : graph_(graph), pathLength_(0), executionTime_(0), stepIndex_(0) {
}

size_t Algorithm::getStepIndex() const {
    return stepIndex_;
}

void Algorithm::setStepIndex(size_t stepIndex) {
    stepIndex_ = stepIndex;
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
                }
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

    if (previous.find(endNode) == previous.end())    {
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

void Algorithm::renderPath(sf::RenderWindow& window) const {
    sf::VertexArray pathLines(sf::LinesStrip, currentPath.size());
    for (std::size_t i = 0; i < currentPath.size(); ++i) {
        pathLines[i].position = currentPath[i]->getPosition();
        pathLines[i].color = sf::Color::Red;
    }
    window.draw(pathLines);
}

void Algorithm::resetPath() {
    currentPath.clear();
}

void Algorithm::setCurrentPath(const std::vector<std::shared_ptr<Node>>& path) {
    currentPath = path;
}


void Algorithm::renderStep(sf::RenderWindow& window) {
    sf::Color currentNodeColor(255, 140, 0); // Orange

    sf::Color processedNodeColor(135, 206, 250); // Light blue
    if (!steps.empty() && getStepIndex() < steps.size()) {
        const auto& step = steps[getStepIndex()];
        if (step.first && step.second) {
            sf::VertexArray stepLine(sf::Lines, 2);
            stepLine[0].position = step.first->getPosition();
            stepLine[0].color = sf::Color::Red;
            stepLine[1].position = step.second->getPosition();
            stepLine[1].color = sf::Color::Red;
            window.draw(stepLine);
        }

        // Color processed nodes
        for (const auto& node : processedNodes) {
            if (node) {
                node->setColor(processedNodeColor);
            }
        }

        // Color current node
        if (step.first) {
            step.first->setColor(currentNodeColor);
        }
    }


    if (!currentPaths.empty() && getStepIndex() < currentPaths.size()) {
        const auto& currentPath = currentPaths[getStepIndex()];
        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
            if (getStepIndex() < steps.size()) {
                sf::VertexArray pathLine(sf::Lines, 2);
                pathLine[0].position = currentPath[i]->getPosition();
                pathLine[0].color = sf::Color::Red;
                pathLine[1].position = currentPath[i + 1]->getPosition();
                pathLine[1].color = sf::Color::Red;
                window.draw(pathLine);
            }
        }
    }
}

void Algorithm::resetStep() {
    steps.clear();
    currentPaths.clear();
    processedNodes.clear();
}
