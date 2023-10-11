#pragma once
#pragma once

#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>
#include "Graph.hpp"
#include "Utility.hpp"
#include "Algorithm.hpp"



class UI {
public:
    enum class Mode {
        NORMAL,
        STEP_BY_STEP,
        AUTO_STEP_BY_STEP
    };



    UI(Graph& graph, Algorithm& algorithm, sf::RenderWindow& window);
    void draw(sf::RenderWindow& window);
    void handleEvent(const sf::Event& event, sf::RenderWindow& window);
    void setMode(Mode mode);

    void resetAlgorithm();
    const sf::Clock& getAutoStepClock() const;
    float getAutoStepInterval() const;
    void resetAutoStepClock();

    Mode getMode() const;
    static std::string modeToString(Mode mode);
    std::shared_ptr<sf::Text> getSelectedButton() const;
private:

    Graph& graph_;
    Algorithm& algorithm_;

    // UI components
    sf::Font font_;
   std::vector<std::shared_ptr<sf::Text>> buttons_;
    std::shared_ptr<sf::Text> selectedButton_;
    std::shared_ptr<sf::Text> pathLengthText_;

    std::shared_ptr<sf::Text> executionTimeText_;
    std::shared_ptr<sf::Text> numNodesEdgesText_;
    sf::RectangleShape stepByStepButton;
    sf::Clock autoStepClock_;
    float autoStepInterval_ = 0.5f; // 1 second interval for automatic steps

    float zoomFactor_;
    bool panning_;
    Mode mode;

    sf::View mainView_;
    sf::View uiView_;
    sf::Vector2i lastMousePosition_;
    void updateUI();
    void generateRandomGraph(size_t numNodes, size_t numEdges, float maxX, float maxY);


};
