/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

const double EPS = 0.00001;

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    if (is_initialized)
        return;

    num_particles = 200;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0;
        weights.push_back(1.0);
        particles.push_back(particle);
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    // Creating normal distributions
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // New state
    for (int i = 0; i < num_particles; i++) {

        double theta = particles[i].theta;

        if (fabs(yaw_rate) < EPS) {
            particles[i].x += velocity * delta_t * cos(theta);
            particles[i].y += velocity * delta_t * sin(theta);

        } else {
            particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
            particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        // Adding noise.
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html

    weights.clear();
    for (int i = 0; i < num_particles; i++) {
        std::vector<LandmarkObs> predicted;

        double particle_px = particles[i].x;
        double particle_py = particles[i].y;
        double particle_theta = particles[i].theta;

        //Convert car observations to map coordinates
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();

        double weight = 1;
        for (const auto &observation : observations) {

            double map_obs_x = observation.x * cos(particle_theta) - observation.y * sin(particle_theta) + particle_px;
            double map_obs_y = observation.x * sin(particle_theta) + observation.y * cos(particle_theta) + particle_py;

            if (sqrt(pow(map_obs_x - particle_px, 2) + pow(map_obs_y - particle_py, 2)) > sensor_range) continue;

            particles[i].sense_x.push_back(map_obs_x);
            particles[i].sense_y.push_back(map_obs_y);

            double min_range = 1000000000;

            int min_k = -1;
            for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

                double diff_x = map_landmarks.landmark_list[k].x_f - map_obs_x;
                double diff_y = map_landmarks.landmark_list[k].y_f - map_obs_y;

                double range = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
                if (range < min_range) {
                    min_range = range;
                    min_k = k;
                }
            }

            double landmark_x = map_landmarks.landmark_list[min_k].x_f;
            double landmark_y = map_landmarks.landmark_list[min_k].y_f;

            particles[i].associations.push_back(map_landmarks.landmark_list[min_k].id_i);

            // calculate weight
            weight = weight
                     * exp(-0.5 * (pow((landmark_x - map_obs_x) / std_landmark[0], 2)
                                   + pow((landmark_y - map_obs_y) / std_landmark[1], 2)))
                     / (2 * M_PI * std_landmark[0] * std_landmark[1]);
        }

        particles[i].weight = weight;
        weights.push_back(weight);

    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    discrete_distribution<int> distribution(weights.begin(), weights.end());
    std::vector<Particle> sample_particles;

    weights.clear();

    for (int i = 0; i < num_particles; i++) {
        int chosen = distribution(gen);
        sample_particles.push_back(particles[chosen]);
        weights.push_back(particles[chosen].weight);
    }

    particles = sample_particles;

}

Particle
ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x,
                                std::vector<double> sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
