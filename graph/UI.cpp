#include "UI.hpp"
#include <iostream>
#include <random>

UI::UI(Graph& graph, Algorithm& algorithm, sf::RenderWindow& window)
    : graph_(graph), algorithm_(algorithm), selectedButton_(nullptr), mainView_(window.getView()), uiView_(window.getDefaultView()), mode(Mode::NORMAL) {
    if (!font_.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font file." << std::endl;
    }

    std::shared_ptr<sf::Text> button;


    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Load Graph from CSV");
    button->setPosition(10, 10); // Adjust the position as needed
    buttons_.push_back(button);

    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Clear Graph");
    button->setPosition(10, 40);
    buttons_.push_back(button);

    // Add more buttons as needed...
    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Generate Random Graph");
    button->setPosition(10, 70);
    buttons_.push_back(button);



    button = std::make_shared<sf::Text>();
    button->setFont(font_);
    button->setString("Change Mode");
    button->setPosition(10, 100);
    buttons_.push_back(button);

    pathLengthText_ = std::make_shared<sf::Text>();
    pathLengthText_->setFont(font_);
    pathLengthText_->setPosition(10, 140);
    pathLengthText_->setCharacterSize(14);

    executionTimeText_ = std::make_shared<sf::Text>();
    executionTimeText_->setFont(font_);
    executionTimeText_->setPosition(10, 160);
    executionTimeText_->setCharacterSize(14);

    numNodesEdgesText_ = std::make_shared<sf::Text>();
    numNodesEdgesText_->setFont(font_);
    numNodesEdgesText_->setPosition(10, 180);
    numNodesEdgesText_->setCharacterSize(14);




}

UI::Mode UI::getMode() const {
    return mode;
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
    window.draw(stepByStepButton);
    window.setView(mainView_);
}

void UI::setMode(Mode mode) {
    this->mode = mode;
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

    if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::A) { // Press 'A' to toggle automatic mode
            if (mode == Mode::STEP_BY_STEP) {
                setMode(Mode::AUTO_STEP_BY_STEP);
                autoStepClock_.restart(); // Reset the timer when switching to auto mode
            }
            else if (mode == Mode::AUTO_STEP_BY_STEP) {
                setMode(Mode::STEP_BY_STEP);
            }
        }
    }


    updateUI();
}

std::string UI::modeToString(Mode mode) {
    switch (mode) {
    case Mode::NORMAL:
        return "NORMAL";
    case Mode::STEP_BY_STEP:
        return "STEP_BY_STEP";
    default:
        return "UNKNOWN";
    }
}

void UI::updateUI() {
    if (selectedButton_) {
        if (selectedButton_->getString() == "Clear Graph") {

            if (!graph_.getNodes().empty()) {
                resetAlgorithm();
            }
        }
        else if (selectedButton_->getString() == "Generate Random Graph") {
            resetAlgorithm();
            generateRandomGraph(20, 30, 800, 600);

        }
        else if (selectedButton_->getString() == "Change Mode") {
            setMode(mode == Mode::NORMAL ? Mode::STEP_BY_STEP : Mode::NORMAL);

        }
        else if (selectedButton_->getString() == "Load Graph from CSV") {
            std::string filename = "path_to_your_file.csv"; // Adjust this to your file path
            graph_.loadFromCSV(filename);
            resetAlgorithm();
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
    resetAlgorithm();

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

void UI::resetAlgorithm() {
    graph_.clear();
    algorithm_.setStepIndex(0);
    algorithm_.resetProcessedNodes();
    algorithm_.clearSteps();
    algorithm_.resetPath();
    algorithm_.resetPathLine();


}

const sf::Clock& UI::getAutoStepClock() const {
    return autoStepClock_;
}

float UI::getAutoStepInterval() const {
    return autoStepInterval_;
}

void UI::resetAutoStepClock() {
    autoStepClock_.restart();
}
