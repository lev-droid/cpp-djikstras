#include "Edge.hpp"
#include "Node.hpp"

Edge::Edge(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, float weight)
    : startNode_(startNode), endNode_(endNode), weight_(weight) {}

std::shared_ptr<Node> Edge::getStartNode() const {
    return startNode_;
}

std::shared_ptr<Node> Edge::getEndNode() const {
    return endNode_;
}

float Edge::getWeight() const {
    return weight_;
}

void Edge::setWeight(float weight) {
    weight_ = weight;
}

void Edge::draw(sf::RenderWindow& window) const {
    sf::Vector2f startPoint = startNode_->getPosition();
    sf::Vector2f endPoint = endNode_->getPosition();

    sf::VertexArray line(sf::Lines, 2);
    line[0].position = startPoint;
    line[0].color = sf::Color::White;
    line[1].position = endPoint;
    line[1].color = sf::Color::White;

    window.draw(line);
}
