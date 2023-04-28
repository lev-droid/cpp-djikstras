#include "Utility.hpp"

float euclideanDistance(const sf::Vector2f& point1, const sf::Vector2f& point2) {
    float deltaX = point1.x - point2.x;
    float deltaY = point1.y - point2.y;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}
