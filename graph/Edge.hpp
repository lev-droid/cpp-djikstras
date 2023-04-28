#pragma once
#include <SFML/Graphics.hpp>
#include <memory>

class Node;

class Edge {
public:
    Edge(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, float weight);

    std::shared_ptr<Node> getStartNode() const;
    std::shared_ptr<Node> getEndNode() const;

    float getWeight() const;
    void setWeight(float weight);

    void draw(sf::RenderWindow& window) const;

private:
    std::shared_ptr<Node> startNode_;
    std::shared_ptr<Node> endNode_;
    float weight_;
};
