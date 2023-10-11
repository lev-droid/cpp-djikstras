#include <SFML/graphics.hpp>
#include <iostream>
#include "Graph.hpp"
#include "Node.hpp"
#include "Edge.hpp"
#include "UI.hpp"
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
}    sf::Color currentNodeColor(255, 140, 0); // Orange
    sf::Color processedNodeColor(191, 64, 191); // Light blue
    /// Remove unused variables and comments
    int main() {
        sf::RenderWindow window(sf::VideoMode(800, 600), "Graph Shortest Path Visualizer");
        window.setFramerateLimit(60);


        Graph graph;
        Algorithm algorithm(graph);
        UI ui(graph, algorithm, window);


        bool draggingNode = false, draggingEdge = false, stepByStepEnabled = false;
        UI::Mode prevMode = UI::Mode::NORMAL;


        std::shared_ptr<Node> selectedNode = nullptr, startNode = nullptr, endNode = nullptr;
        std::vector<std::vector<std::shared_ptr<Node>>> currentPaths;
        std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>> steps;
        std::unordered_set<std::shared_ptr<Node>> processedNodes;



        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                ui.handleEvent(event, window);




                if (event.type == sf::Event::Closed) {
                    window.close();
                }
                if (!ui.getSelectedButton()) {

                    if (event.type == sf::Event::MouseButtonPressed) {
                        if (event.mouseButton.button == sf::Mouse::Left) {
                            sf::Vector2f mousePosition = window.mapPixelToCoords(sf::Mouse::getPosition(window));
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
                            sf::Vector2f mousePosition = window.mapPixelToCoords(sf::Mouse::getPosition(window));
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
                        if (ui.getMode() == UI::Mode::STEP_BY_STEP) {
                            if (event.key.code == sf::Keyboard::N) {
                                if (algorithm.getStepIndex() < steps.size() - 1) {
                                    algorithm.setStepIndex(algorithm.getStepIndex() + 1);
                                }
                            }
                            else if (event.key.code == sf::Keyboard::P) {
                                if (algorithm.getStepIndex() > 0) {
                                    algorithm.setStepIndex(algorithm.getStepIndex() - 1);
                                }
                            }

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
            std::shared_ptr<Node> start = nullptr;
            std::shared_ptr<Node> goal = nullptr;

            if (!graph.getNodes().empty()) {
                start = graph.getNodes()[0];
                goal = graph.getNodes()[graph.getNodes().size() - 1];
            }

            graph.draw(window, start, goal);
            // Visualize the Dijkstra algorithm
            if (graph.getNodes().size() > 1) {
                if (ui.getMode() == UI::Mode::NORMAL) {

                    if (algorithm.dijkstra(graph.getNodes()[0], graph.getNodes()[graph.getNodes().size() - 1])) {
                        // Set the found path as current path
                        algorithm.setCurrentPath(algorithm.getPath());
                        // Draw the shortest path
                        algorithm.renderPath(window); 
                    }
                }
                else if (ui.getMode() == UI::Mode::STEP_BY_STEP) {
                    if (graph.getNodes().size() > 1) {
                        algorithm.clearSteps();
                        algorithm.clearSteps();
                        algorithm.resetPath();
                        algorithm.setCurrentPath(std::vector<std::shared_ptr<Node>>());

                        DijkstraCallback callback = [&](std::shared_ptr<Node> currentNode, const std::vector<std::shared_ptr<Node>>& visitedNodes, const std::vector<std::shared_ptr<Node>>& neighbors, const std::vector<std::shared_ptr<Node>>& currentPath) {
                            if (!visitedNodes.empty()) {  // check if visitedNodes is not empty to avoid rendering an edge from a path already visited on previous iterations of the algorithm
                                algorithm.addStep(std::make_pair(currentNode, neighbors.front()));
                            }
                            else {
                                algorithm.addStep(std::make_pair(currentNode, nullptr));
                            }
                            algorithm.setCurrentPath(currentPath);
                        };

                        if (algorithm.dijkstra(graph.getNodes()[0], graph.getNodes()[graph.getNodes().size() - 1], callback)) {
                            algorithm.setCurrentPath(algorithm.getPath());
                        }
                        else {
                            algorithm.setCurrentPath(std::vector<std::shared_ptr<Node>>());

                        }

                        // Rendering is now handled by Algorithm::renderStep
                        algorithm.renderStep(window, processedNodeColor, currentNodeColor);
                    }
                }

                if (ui.getMode() == UI::Mode::AUTO_STEP_BY_STEP) {
                    std::cout << "In AUTO_STEP_BY_STEP mode" << std::endl; // Debug print
                    if (ui.getAutoStepClock().getElapsedTime().asSeconds() >= ui.getAutoStepInterval()) {
                        std::cout << "Advancing step" << std::endl; // Debug print
                        if (algorithm.getStepIndex() < steps.size() - 1) {
                            algorithm.setStepIndex(algorithm.getStepIndex() + 1);
                        }
                        ui.resetAutoStepClock();
                    }
                    algorithm.renderStep(window, processedNodeColor, currentNodeColor);

                }

            }

            ui.draw(window);
            window.display();
        }

        return 0;

    }
