#include "UI.hpp"
#include <iostream>
#include <random>

UI::UI(Graph& graph, Algorithm& algorithm, sf::RenderWindow& window)
    : graph_(graph), algorithm_(algorithm), selectedButton_(nullptr), mainView_(window.getView()), uiView_(window.getDefaultView()) {
    if (!font_.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font file." << std::endl;
    }

    std::shared_ptr<sf::Text> button;

    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Clear Graph");
    button->setPosition(10, 10);
    buttons_.push_back(button);

    // Add more buttons as needed...
    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Generate Random Graph");
    button->setPosition(10, 40);
    buttons_.push_back(button);



    pathLengthText_ = std::make_shared<sf::Text>();
    pathLengthText_->setFont(font_);
    pathLengthText_->setPosition(10, 70);
    pathLengthText_->setCharacterSize(14);

    executionTimeText_ = std::make_shared<sf::Text>();
    executionTimeText_->setFont(font_);
    executionTimeText_->setPosition(10, 90);
    executionTimeText_->setCharacterSize(14);

    numNodesEdgesText_ = std::make_shared<sf::Text>();
    numNodesEdgesText_->setFont(font_);
    numNodesEdgesText_->setPosition(10, 110);
    numNodesEdgesText_->setCharacterSize(14);




}

void UI::draw(sf::RenderWindow& window) {
    // Set the UI view before drawing the buttons
    window.setView(uiView_);
    for (const auto& button : buttons_) {
        window.draw(*button);
    }
    window.draw(*pathLengthText_);
    window.draw(*executionTimeText_);
    window.draw(*numNodesEdgesText_);
    window.setView(mainView_);
}


std::shared_ptr<sf::Text> UI::getSelectedButton() const {
    return selectedButton_;
}


void UI::handleEvent(const sf::Event& event, sf::RenderWindow& window) {
    if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
            sf::Vector2f mousePosition(static_cast<float>(event.mouseButton.x),
                static_cast<float>(event.mouseButton.y));
            for (const auto& button : buttons_) {
                if (button->getGlobalBounds().contains(mousePosition)) {
                    selectedButton_ = button;
                    break;
                }
            }
        }

        if (event.mouseButton.button == sf::Mouse::Middle) {
            panning_ = true;
        }
    }

    if (event.type == sf::Event::MouseButtonReleased) {
        if (event.mouseButton.button == sf::Mouse::Left) {
            selectedButton_ = nullptr;
        }

        if (event.mouseButton.button == sf::Mouse::Middle) {
            panning_ = false;
        }
    }

    if (event.type == sf::Event::MouseWheelScrolled) {
        if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel) {
            float zoomAmount = 1.0f + event.mouseWheelScroll.delta * 0.1f;
            mainView_.zoom(zoomAmount);
            window.setView(mainView_);
            zoomFactor_ *= zoomAmount;
        }
    }



    if (event.type == sf::Event::MouseMoved) {
        if (panning_) {
            sf::Vector2i currentMousePosition(event.mouseMove.x, event.mouseMove.y);
            sf::Vector2i deltaMousePosition = currentMousePosition - lastMousePosition_;

            sf::Vector2f oldMousePosition = window.mapPixelToCoords(lastMousePosition_);
            sf::Vector2f newMousePosition = window.mapPixelToCoords(currentMousePosition);

            sf::Vector2f deltaPosition = oldMousePosition - newMousePosition;
            mainView_.move(deltaPosition);
            window.setView(mainView_);
        }

        lastMousePosition_ = sf::Vector2i(event.mouseMove.x, event.mouseMove.y);
    }

    updateUI();
}


void UI::updateUI() {
    if (selectedButton_) {
        if (selectedButton_->getString() == "Clear Graph") {

            if (!graph_.getNodes().empty()) {
                graph_.clear();
            }
        }
        else if (selectedButton_->getString() == "Generate Random Graph") {
            generateRandomGraph(10, 20, 800, 600);
        }
        // Handle other button actions...
    }

        float pathLength = algorithm_.getPathLength();
    pathLengthText_->setString("Path Length: " + std::to_string(pathLength));

    // Update execution time
    float executionTime = algorithm_.getExecutionTime();
    executionTimeText_->setString("Execution Time: " + std::to_string(executionTime) + " ms");

    // Update number of nodes and edges
    size_t numNodes = graph_.getNodes().size();
    size_t numEdges = graph_.getEdges().size();
    numNodesEdgesText_->setString("Nodes: " + std::to_string(numNodes) + ", Edges: " + std::to_string(numEdges));
}


void UI::generateRandomGraph(size_t numNodes, size_t numEdges, float maxX, float maxY) {
    graph_.clear();

    // Generate random nodes
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> xDist(0, maxX);
    std::uniform_real_distribution<float> yDist(0, maxY);

    for (size_t i = 0; i < numNodes; ++i) {
        graph_.addNode(sf::Vector2f(xDist(gen), yDist(gen)));
    }

    // Generate random edges
    std::uniform_int_distribution<size_t> nodeDist(0, numNodes - 1);
    for (size_t i = 0; i < numEdges; ++i) {
        auto startNode = graph_.getNodes()[nodeDist(gen)];
        auto endNode = graph_.getNodes()[nodeDist(gen)];

        if (startNode != endNode) {
            float weight = euclideanDistance(startNode->getPosition(), endNode->getPosition());
            graph_.addEdge(startNode, endNode, weight);
        }
    }
}