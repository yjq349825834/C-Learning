#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include "./eigen-3.4.0/Eigen/Dense"

// Structure to represent a step
struct Step {
    long long timestamp; // in nanoseconds
    double displacement; // in meters
    double heading;      // in radians
    double x_position;   // in meters, calculated
    double y_position;   // in meters, calculated
    double erraticism;   // erraticism value
};

// Function to perform Kalman filter update step
void kalman_filter_update(const Step& measurement, Eigen::VectorXd& state_mean, Eigen::MatrixXd& state_covariance,
    const Eigen::MatrixXd& observation_matrices, const Eigen::MatrixXd& observation_covariance,
    const Eigen::MatrixXd& transition_matrices, const Eigen::MatrixXd& transition_covariance) {

    // Kalman prediction step
    Eigen::VectorXd predicted_state_mean = transition_matrices * state_mean;
    Eigen::MatrixXd predicted_state_covariance = transition_matrices * state_covariance * transition_matrices.transpose() +
        transition_covariance;

    // Kalman update step
    Eigen::MatrixXd kalman_gain = predicted_state_covariance * observation_matrices.transpose() *
        (observation_matrices * predicted_state_covariance * observation_matrices.transpose() +
            observation_covariance).inverse();

    // Measurement vector
    Eigen::VectorXd measurement_vector(2);
    measurement_vector << measurement.x_position, measurement.y_position;

    // Update state mean and covariance
    state_mean = predicted_state_mean + kalman_gain * (measurement_vector - observation_matrices * predicted_state_mean);
    state_covariance = (Eigen::MatrixXd::Identity(2, 2) - kalman_gain * observation_matrices) * predicted_state_covariance;
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
    std::ofstream outputFile("kalman_output.csv");
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to create the output file." << std::endl;
        return 1;
    }

    // Write header to the output file
    outputFile << "timestamp,erraticism" << std::endl;

    // Kalman filter parameters
    Eigen::VectorXd state_mean(2);
    state_mean.setZero();
    Eigen::MatrixXd state_covariance = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd observation_matrices = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd observation_covariance = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd transition_matrices = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd transition_covariance = 0.01 * Eigen::MatrixXd::Identity(2, 2);

    // Initialize min and max erraticism values
    double erraticismMin = std::numeric_limits<double>::max();
    double erraticismMax = -std::numeric_limits<double>::max();

    // Process steps using Kalman filter
    for (size_t i = 0; i < steps.size(); ++i) {
        // Perform Kalman filter update for all steps
        kalman_filter_update(steps[i], state_mean, state_covariance, observation_matrices, observation_covariance,
            transition_matrices, transition_covariance);

        // Calculate erraticism based on Kalman filter estimate
        Eigen::VectorXd filtered_state_mean = observation_matrices * state_mean;
        double erraticism = (filtered_state_mean - (Eigen::VectorXd(2) << steps[i].x_position, steps[i].y_position).finished()).norm();

        // Update min and max erraticism values
        erraticismMin = std::min(erraticismMin, erraticism);
        erraticismMax = std::max(erraticismMax, erraticism);

        // Store erraticism in Step struct
        steps[i].erraticism = erraticism;

    }

    for (size_t i = 0; i < steps.size(); ++i) {
		 // Normalize erraticism values to the range [0, 1]
        double normalizedErraticism = 1.0 - (steps[i].erraticism - erraticismMin) / (erraticismMax - erraticismMin);

        // Update the output file with normalized erraticism
        outputFile << steps[i].timestamp << "," << normalizedErraticism << std::endl;
    }


    outputFile.close();

    std::cout << "Output written to kalman_output.csv" << std::endl;

    return 0;
}




