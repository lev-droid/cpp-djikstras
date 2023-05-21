#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

class Edge;

class Node {
public:
    static constexpr float NODE_RADIUS = 10.0f;
    Node(const sf::Vector2f& position);

    const sf::Vector2f& getPosition() const;
    void setPosition(const sf::Vector2f& position);

    void addEdge(std::shared_ptr<Edge> edge);
    void removeEdge(std::shared_ptr<Edge> edge);

    const std::vector<std::shared_ptr<Edge>>& getEdges() const;
    void setColor(const sf::Color& color);
    void draw(sf::RenderWindow& window) const;

private:
    sf::Vector2f position_;
    std::vector<std::shared_ptr<Edge>> edges_;
    sf::Color nodeColor_ = sf::Color::White;
};
