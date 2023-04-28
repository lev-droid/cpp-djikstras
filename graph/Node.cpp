#include "Node.hpp"
#include "Edge.hpp"

Node::Node(const sf::Vector2f& position) : position_(position) {}

const sf::Vector2f& Node::getPosition() const {
    return position_;
}

void Node::setPosition(const sf::Vector2f& position) {
    position_ = position;
}

void Node::addEdge(std::shared_ptr<Edge> edge) {
    edges_.push_back(edge);
}

void Node::removeEdge(std::shared_ptr<Edge> edge) {
    edges_.erase(std::remove(edges_.begin(), edges_.end(), edge), edges_.end());
}

const std::vector<std::shared_ptr<Edge>>& Node::getEdges() const {
    return edges_;
}

void Node::draw(sf::RenderWindow& window) const {
    constexpr float radius = 15.0f;
    const sf::Color nodeColor = sf::Color::Red;

    sf::CircleShape circle(radius);
    circle.setFillColor(nodeColor);
    circle.setPosition(position_.x - radius, position_.y - radius);

    window.draw(circle);
}

