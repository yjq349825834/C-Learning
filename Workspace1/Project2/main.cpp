#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>


// Structure to represent a step
struct Step {
    long long timestamp; // in nanoseconds
    double displacement; // in meters
    double heading;      // in radians
    double x_position;   // in meters, calculated
    double y_position;   // in meters, calculated
};

// Function to calculate Euclidean distance between two steps
double euclidean_distance(const Step& step1, const Step& step2) {
    double dx = step1.x_position - step2.x_position;
    double dy = step1.y_position - step2.y_position;
    return sqrt(dx * dx + dy * dy);
}

double calculate_erraticism(const Step& currentStep, const std::vector<Step>& steps, int numNeighbors, double thresholdDistance) {
    if (steps.size() < static_cast<size_t>(numNeighbors)) {
        return 0.0; // If there are not enough previous steps, set erraticism to 0
    }

    double maxDistance = 0.0;
    
    for (size_t i = 0; i < numNeighbors; ++i) {
        size_t index = steps.size() - 2 - i; // Index of the step to compare
        // # Calculate Euclidean distance to all previous steps
        if (index < steps.size()) {
            double distance = euclidean_distance(currentStep, steps[index]);
            // Calculate the Euclidean distance to the farthest neighbor
            if (distance > maxDistance) {
                maxDistance = distance;
            }
        }
    }


    // Calculate erraticism and normalize it in the interval [0, 1]
    double erraticism = 1.0 - std::min(maxDistance / thresholdDistance, 1.0);
    return erraticism;
}


int main() {

    // Input the file path or use the default file path
    std::string file_path;
    std::cout << "Enter the steplog file path (or press Enter to use the default): ";
    std::getline(std::cin, file_path);

    // Use the default file path if the file path not provided
    if (file_path.empty()) {
        file_path = "2017-01-20Z14-30-05.steplog";
        std::cout << "Using the default steplog file: " << file_path << std::endl;
    }

    std::ifstream inputFile(file_path);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Unable to open the steplog file." << std::endl;
        return 1;
    }

    // Read steps from the file
    std::vector<Step> steps;
    std::string line;
    double cumulativeX = 0.0, cumulativeY = 0.0;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        Step step;
        char comma;
        double ignore;
        if (iss >> step.timestamp >> comma >> step.displacement >> comma >> step.heading >> comma >> ignore >> comma >> ignore) {
            // Calculate cumulative x and y positions
            cumulativeX += step.displacement * cos(step.heading);
            cumulativeY += step.displacement * sin(step.heading);
            step.x_position = cumulativeX;
            step.y_position = cumulativeY;
            steps.push_back(step);
        }
    }

    inputFile.close();

    // Output file
    std::ofstream outputFile("nearest_neighbour_output.csv");
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to create the output file." << std::endl;
        return 1;
    }

    // Write header to the output file
    outputFile << "timestamp,erraticism" << std::endl;

    // Nearest neighbour parameters
    int numNeighbors = 25; // Adjust as needed
    double thresholdDistance = 14; // Adjust as needed

    // Calculate and write erraticism for each step
    for (size_t i = 0; i < steps.size(); ++i) {
        double erraticism = 0.0;
        if (i >= numNeighbors) {
            erraticism = calculate_erraticism(steps[i], std::vector<Step>(steps.begin() + std::max(0, static_cast<int>(i) - numNeighbors), steps.begin() + i), numNeighbors, thresholdDistance);
        }
        outputFile << steps[i].timestamp << "," << erraticism << std::endl;
    }

    outputFile.close();

    std::cout << "Output written to nearest_neighbour_output.csv" << std::endl;

    return 0;
}