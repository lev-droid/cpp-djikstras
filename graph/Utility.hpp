#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>

float euclideanDistance(const sf::Vector2f& point1, const sf::Vector2f& point2);
void drawGlow(sf::RenderTarget& target, const sf::CircleShape& shape, const sf::Color& color, float intensity, int blurRadius);