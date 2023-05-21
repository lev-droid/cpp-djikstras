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
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Graph Shortest Path Visualizer");
    window.setFramerateLimit(60);


    Graph graph;
    Algorithm algorithm(graph);
    UI ui(graph, algorithm, window);

    sf::Color currentNodeColor(255, 140, 0); // Orange
    sf::Color processedNodeColor(135, 206, 250); // Light blue

    bool draggingNode = false;
    bool draggingEdge = false;
    bool stepByStepEnabled = false;
    UI::Mode prevMode = UI::Mode::NORMAL;


    std::shared_ptr<Node> selectedNode = nullptr;
    std::shared_ptr<Node> startNode = nullptr;
    std::vector<std::shared_ptr<Node>> currentPath;
    std::shared_ptr<Node> endNode = nullptr;
    std::vector<std::vector<std::shared_ptr<Node>>> currentPaths;
    std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>> steps;
    std::unordered_set<std::shared_ptr<Node>> processedNodes;



    while (window.isOpen()) {
        std::cout << "Current mode: " << UI::modeToString(ui.getMode()) << std::endl;
        sf::Event event;
        while (window.pollEvent(event)) {
            ui.handleEvent(event, window);

            if (prevMode != ui.getMode()) {
                prevMode = ui.getMode();
                if (ui.getMode() == UI::Mode::STEP_BY_STEP) {
                    steps.clear();
                    currentPaths.clear();
                    algorithm.setStepIndex(0);
                    processedNodes.clear(); // Add this line

                }
            }



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
                    steps.clear();
                    currentPath.clear();
                    processedNodes.clear();

                    DijkstraCallback callback = [&](std::shared_ptr<Node> currentNode, const std::vector<std::shared_ptr<Node>>& visitedNodes, const std::vector<std::shared_ptr<Node>>& neighbors, const std::vector<std::shared_ptr<Node>>& currentPath) {
                        if (!neighbors.empty()) {
                            steps.emplace_back(currentNode, neighbors.front());
                        }
                        else {
                            steps.emplace_back(currentNode, nullptr);
                        }
                        processedNodes.insert(visitedNodes.begin(), visitedNodes.end());


                        // Store the current path
                        currentPaths.push_back(currentPath);
                    };


                    if (algorithm.dijkstra(graph.getNodes()[0], graph.getNodes()[graph.getNodes().size() - 1], callback)) {
                        currentPath = algorithm.getPath();
                    }
                    else {
                        currentPath.clear();
                    }

                    // Draw the current step
                    if (!steps.empty() && algorithm.getStepIndex() < steps.size()) {
                        const auto& step = steps[algorithm.getStepIndex()];
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


                    if (!currentPaths.empty() && algorithm.getStepIndex() < currentPaths.size()) {
                        const auto& currentPath = currentPaths[algorithm.getStepIndex()];
                        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
                            if (algorithm.getStepIndex() < steps.size()) {
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
            }
        }

        ui.draw(window);
        window.display();
    }

    return 0;

}

