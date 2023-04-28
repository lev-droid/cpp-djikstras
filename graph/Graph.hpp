#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include "Node.hpp"
#include "Edge.hpp"

class Graph {
public:
    void addNode(const sf::Vector2f& position);
    void removeNode(std::shared_ptr<Node> node);

    void addEdge(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode, float weight);
    void removeEdge(std::shared_ptr<Edge> edge);

    const std::vector<std::shared_ptr<Node>>& getNodes() const;
    const std::vector<std::shared_ptr<Edge>>& getEdges() const;

    void draw(sf::RenderWindow& window, const std::shared_ptr<Node>& startNode = nullptr, const std::shared_ptr<Node>& goalNode = nullptr) const;
private:
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    sf::Font font_;
};
