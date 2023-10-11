#include "Graph.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
void Graph::clear() {
    edges_.clear();
    nodes_.clear();
}


void Graph::addNode(const sf::Vector2f& position) {
    auto newNode = std::make_shared<Node>(position);
    nodes_.push_back(newNode);
}

void Graph::removeNode(std::shared_ptr<Node> node) {
    for (const auto& edge : node->getEdges()) {
        removeEdge(edge);
    }
    nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
}

void Graph::addEdge(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, float weight) {
    auto newEdge = std::make_shared<Edge>(startNode, endNode, weight);
    startNode->addEdge(newEdge);
    endNode->addEdge(newEdge);
    edges_.push_back(newEdge);
}

void Graph::removeEdge(std::shared_ptr<Edge> edge) {
    edge->getStartNode()->removeEdge(edge);
    edge->getEndNode()->removeEdge(edge);
    edges_.erase(std::remove(edges_.begin(), edges_.end(), edge), edges_.end());
}

const std::vector<std::shared_ptr<Node>>& Graph::getNodes() const {
    return nodes_;
}

const std::vector<std::shared_ptr<Edge>>& Graph::getEdges() const {
    return edges_;
}

void Graph::draw(sf::RenderWindow& window, const std::shared_ptr<Node>& startNode, const std::shared_ptr<Node>& goalNode) const {
    for (const auto& edge : edges_) {
        edge->draw(window);
    }

    for (const auto& node : nodes_) {
        node->draw(window);
    }

    if (startNode) {
        sf::CircleShape startIndicator(Node::NODE_RADIUS);
        startIndicator.setFillColor(sf::Color::Green);
        startIndicator.setOrigin(Node::NODE_RADIUS, Node::NODE_RADIUS);
        startIndicator.setPosition(startNode->getPosition());
        window.draw(startIndicator);
    }

    if (goalNode) {
        sf::CircleShape goalIndicator(Node::NODE_RADIUS);
        goalIndicator.setFillColor(sf::Color::Red);
        goalIndicator.setOrigin(Node::NODE_RADIUS, Node::NODE_RADIUS);
        goalIndicator.setPosition(goalNode->getPosition());
        window.draw(goalIndicator);
    }
}

void Graph::loadFromCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file: " << filename << std::endl;
        return;
    }

    clear(); // Clear the current graph

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string type, data1, data2, data3;

        std::getline(ss, type, ',');
        std::getline(ss, data1, ',');
        std::getline(ss, data2, ',');
        std::getline(ss, data3, ',');

        if (type == "Node") {
            float x = std::stof(data1);
            float y = std::stof(data2);
            addNode(sf::Vector2f(x, y));
        }
        else if (type == "Edge") {
            int startIdx = std::stoi(data1);
            int endIdx = std::stoi(data2);
            float weight = std::stof(data3);
            if (startIdx < nodes_.size() && endIdx < nodes_.size()) {
                addEdge(nodes_[startIdx], nodes_[endIdx], weight);
            }
        }
    }

    file.close();
}

