#include "Graph.hpp"

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
    for (const auto& node : nodes_) {
        node->draw(window);
    }

    for (const auto& edge : edges_) {
        edge->draw(window);
    }

    if (startNode) {
        sf::CircleShape startIndicator(Node::NODE_RADIUS * 1.5f);
        startIndicator.setFillColor(sf::Color::Green);
        startIndicator.setOrigin(Node::NODE_RADIUS * 1.5f, Node::NODE_RADIUS * 1.5f);
        startIndicator.setPosition(startNode->getPosition());
        window.draw(startIndicator);
    }

    if (goalNode) {
        sf::CircleShape goalIndicator(Node::NODE_RADIUS * 1.5f);
        goalIndicator.setFillColor(sf::Color::Red);
        goalIndicator.setOrigin(Node::NODE_RADIUS * 1.5f, Node::NODE_RADIUS * 1.5f);
        goalIndicator.setPosition(goalNode->getPosition());
        window.draw(goalIndicator);
    }
}