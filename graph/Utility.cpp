#include "Utility.hpp"

float euclideanDistance(const sf::Vector2f& point1, const sf::Vector2f& point2) {
    float deltaX = point1.x - point2.x;
    float deltaY = point1.y - point2.y;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}


void drawGlow(sf::RenderTarget& target, const sf::CircleShape& shape, const sf::Color& color, float intensity, int blurRadius) {
    sf::CircleShape glow(shape);
    glow.setFillColor(color);
    glow.setRadius(shape.getRadius() + blurRadius);
    glow.setOrigin(shape.getOrigin().x + blurRadius, shape.getOrigin().y + blurRadius);
    glow.setPosition(shape.getPosition());

    for (int i = blurRadius; i > 0; --i) {
        float alpha = intensity * (static_cast<float>(i) / static_cast<float>(blurRadius));
        glow.setFillColor(sf::Color(color.r, color.g, color.b, static_cast<sf::Uint8>(alpha)));
        glow.setRadius(shape.getRadius() + i);
        glow.setOrigin(shape.getOrigin().x + i, shape.getOrigin().y + i);
        target.draw(glow);
    }

    target.draw(shape);
}