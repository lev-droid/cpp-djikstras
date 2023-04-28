#include <SFML/Graphics.hpp>
#include <iostream>
#include "Graph.hpp"
#include "Node.hpp"
#include "Edge.hpp"
#include "Algorithm.hpp"
#include "Utility.hpp"

std::shared_ptr<Node> getNodeAtPosition(const Graph& graph, const sf::Vector2f& position) {
    for (const auto& node : graph.getNodes()) {
        if (euclideanDistance(node->getPosition(), position) <= Node::NODE_RADIUS) {
            return node;
        }
    }
    return nullptr;
}

void updateEdgeWeights(Graph& graph) {
    for (const auto& edge : graph.getEdges()) {
        float newWeight = euclideanDistance(edge->getStartNode()->getPosition(), edge->getEndNode()->getPosition());
        edge->setWeight(newWeight);
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Graph Shortest Path Visualizer");
    window.setFramerateLimit(60);

    Graph graph;
    graph.addNode(sf::Vector2f(100, 100)); // Add a single node to the graph

    Algorithm algorithm(graph);

    bool draggingNode = false;
    std::shared_ptr<Node> selectedNode = nullptr;
    bool draggingEdge = false;
    std::shared_ptr<Node> startNode = nullptr;
    std::shared_ptr<Node> endNode = nullptr;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f mousePosition(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));
                    auto node = getNodeAtPosition(graph, mousePosition);

                    if (node) {
                        draggingNode = true;
                        selectedNode = node;
                    }
                    else {
                        graph.addNode(mousePosition);
                    }
                }
                else if (event.mouseButton.button == sf::Mouse::Right) {
                    sf::Vector2f mousePosition(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));
                    auto node = getNodeAtPosition(graph, mousePosition);

                    if (node) {
                        if (!draggingEdge) {
                            draggingEdge = true;
                            startNode = node;
                        }
                        else {
                            if (startNode != node) {
                                endNode = node;
                                graph.addEdge(startNode, endNode, euclideanDistance(startNode->getPosition(), endNode->getPosition()));
                                draggingEdge = false;
                                startNode = nullptr;
                                endNode = nullptr;
                            }
                        }
                    }
                    else {
                        draggingEdge = false;
                        startNode = nullptr;
                        endNode = nullptr;
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    draggingNode = false;
                    selectedNode = nullptr;
                }
            }

            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::R) {
                    if (selectedNode) {
                        graph.removeNode(selectedNode);
                        selectedNode = nullptr;
                    }
                }
            }
        }

        if (draggingNode && selectedNode) {
            sf::Vector2f mousePosition = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            selectedNode->setPosition(mousePosition);
            updateEdgeWeights(graph);
        }


        window.clear();

        std::shared_ptr<Node> start = graph.getNodes()[0];
        std::shared_ptr<Node> goal = graph.getNodes()[graph.getNodes().size() - 1];
        graph.draw(window, start, goal);

        // Visualize the Dijkstra algorithm
        if (algorithm.dijkstra(graph.getNodes()[0], graph.getNodes()[graph.getNodes().size() - 1])) {
            // Draw the shortest path
            const auto& path = algorithm.getPath();
            sf::VertexArray pathLines(sf::LinesStrip, path.size());

        for (std::size_t i = 0; i < path.size(); ++i) {
            pathLines[i].position = path[i]->getPosition();
            pathLines[i].color = sf::Color::Red;
        }

        window.draw(pathLines);
    }

    window.display();
}

return 0;

}