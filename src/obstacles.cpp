#include "../include/obstacles.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

Obstacle::Obstacle(float posX, float posY, float posZ, float halfSizeX, float halfSizeY, float halfSizeZ)
    : posX(posX), posY(posY), posZ(posZ), halfSizeX(halfSizeX), halfSizeY(halfSizeY), halfSizeZ(halfSizeZ) {}

float Obstacle::isCollision(const Eigen::Vector3f& point) const {
    return (point.x() >= getMinX() && point.x() <= getMaxX() &&
        point.y() >= getMinY() && point.y() <= getMaxY() &&
        point.z() >= getMinZ() && point.z() <= getMaxZ());
}
float Obstacle::getMinX() const { return posX - halfSizeX; }
float Obstacle::getMaxX() const { return posX + halfSizeX; }
float Obstacle::getMinY() const { return posY - halfSizeY; }
float Obstacle::getMaxY() const { return posY + halfSizeY; }
float Obstacle::getMinZ() const { return posZ - halfSizeZ; }
float Obstacle::getMaxZ() const { return posZ + halfSizeZ; }

ObstacleParser::ObstacleParser(const std::string& filename) {
    parseCSV(filename);
    computeBounds();
}

void ObstacleParser::parseCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<float> values;

        while (std::getline(ss, token, ',')) {
            values.push_back(std::stof(token));
        }
        obstacles.emplace_back(values[0], values[1], values[2], values[3], values[4], values[5]);

    }
}

void ObstacleParser::computeBounds() {
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    for (const auto& obstacle : obstacles) {
        minX = std::min(minX, obstacle.getMinX());
        maxX = std::max(maxX, obstacle.getMaxX());
        minY = std::min(minY, obstacle.getMinY());
        maxY = std::max(maxY, obstacle.getMaxY());
        minZ = std::min(minZ, obstacle.getMinZ());
        maxZ = std::max(maxZ, obstacle.getMaxZ());
    }

    bounds = { minX, maxX, minY, maxY, minZ, maxZ };
}

const std::vector<float>& ObstacleParser::getBounds() const {
    return bounds;
}

std::vector<Obstacle> ObstacleParser::getObstacles() const {
    return obstacles;
}