#include <SFML/graphics.hpp>
#include <iostream>
#include "Graph.hpp"
#include "Node.hpp"
#include "Edge.hpp"
#include "UI.hpp"
#include "Algorithm.hpp"
#include "Utility.hpp"
// Function to get a node at a given position in the graph
std::shared_ptr<Node> getNodeAtPosition(const Graph& graph, const sf::Vector2f& position) {
    for (const auto& node : graph.getNodes()) {
        // Check if the distance between the node and the given position is less than or equal to the node's radius
        if (euclideanDistance(node->getPosition(), position) <= Node::NODE_RADIUS) {
            return node;
        }
    }
    return nullptr;
}

// Function to update the weights of all edges in the graph based on the Euclidean distance between their start and end nodes
void updateEdgeWeights(Graph& graph) {
    for (const auto& edge : graph.getEdges()) {
        float newWeight = euclideanDistance(edge->getStartNode()->getPosition(), edge->getEndNode()->getPosition());
        edge->setWeight(newWeight);
    }
}    
// Define colors for visualization
sf::Color currentNodeColor(255, 140, 0); // Orange
sf::Color processedNodeColor(191, 64, 191); // Light blue
    /// Remove unused variables and comments
    int main() {
        // Create a window for the application
        sf::RenderWindow window(sf::VideoMode(800, 600), "Graph Shortest Path Visualizer");
        window.setFramerateLimit(60);

        // Initialize graph, algorithm, and UI objects
        Graph graph;
        Algorithm algorithm(graph);
        UI ui(graph, algorithm, window);

        // Variables to manage user interactions
        bool draggingNode = false, draggingEdge = false, stepByStepEnabled = false;
        UI::Mode prevMode = UI::Mode::NORMAL;
        std::shared_ptr<Node> selectedNode = nullptr, startNode = nullptr, endNode = nullptr;
        std::vector<std::vector<std::shared_ptr<Node>>> currentPaths;
        std::vector<std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>>> steps;
        std::unordered_set<std::shared_ptr<Node>> processedNodes;


        // Main loop to keep the window open
        while (window.isOpen()) {
            sf::Event event;
            // Poll and handle events
            while (window.pollEvent(event)) {
                ui.handleEvent(event, window);
                // Close the window if the close event is triggered
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
                if (!ui.getSelectedButton()) {
                    // Handle mouse and keyboard interactions if no UI button is selected
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
            // Handle node dragging
            if (draggingNode && selectedNode) {
                sf::Vector2f mousePosition = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                selectedNode->setPosition(mousePosition);
                updateEdgeWeights(graph);
            }
            // Clear the window for drawing
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

                //draw the step by step path
                else if (ui.getMode() == UI::Mode::STEP_BY_STEP) {
                    if (graph.getNodes().size() > 1) {
                        // reset the algorithm to rreset contstructed path
                        algorithm.clearSteps();
                        algorithm.resetPath();
                        algorithm.setCurrentPath(std::vector<std::shared_ptr<Node>>());
                        //construct the path
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
                //automated stepping
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
            // Draw the UI elements
            ui.draw(window);
            // Display the rendered frame
            window.display();
        }

        return 0;

    }
