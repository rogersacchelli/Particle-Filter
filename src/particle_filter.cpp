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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 3;
	weights.assign (num_particles, 1.0/num_particles);
	
	std::default_random_engine gen_part_pos;

	std::normal_distribution<double> N_part_x(x, std[0]);
	std::normal_distribution<double> N_part_y(y, std[1]);
	std::normal_distribution<double> N_part_theta(theta, std[2]);

	for (int i=0; i < num_particles; i++){

		float part_x = N_part_x(gen_part_pos);
		float part_y = N_part_y(gen_part_pos);
		float part_theta = N_part_theta(gen_part_pos);

		particles.push_back({i, part_x, part_y, part_theta, 1});

	}
	
	for(int i=0; i < 1; i++){
		std::cout << i << '-' << particles[i].id << ' ' << particles[i].x << ' ' << particles[i].y << std::endl ;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::default_random_engine gen_pred;

	std::normal_distribution<double> N_pred_vel(velocity, std_pos[0]);
	std::normal_distribution<double> N_pred_yaw_r(yaw_rate, std_pos[1]);

	float vel_pred = N_pred_vel(gen_pred);
	float yaw_r_pred = N_pred_yaw_r(gen_pred);

	for (int i=0; i<particles.size();i++){
		particles[i].x = particles[i].x + (vel_pred/yaw_r_pred) * (sin(particles[i].theta + yaw_r_pred *delta_t ) - sin(yaw_r_pred));
		particles[i].y = particles[i].y + (vel_pred/yaw_r_pred) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_r_pred * delta_t));
		particles[i].theta = particles[i].theta + yaw_r_pred * delta_t;

	}

	

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	// T = [cos(theta)	sin(theta)	xt
	//			sin(theta)	cos(theta)	yt
	//			0						0						1	]

	// It is important to remember that T represents a rotation followed by a translation (not the other way around). 
	// Each primitive can be transformed using the inverse of $ T$, resulting in a transformed solid model of the robot. 
	// The transformed robot is denoted by $ {\cal A}(x_t,y_t,\theta)$, and in this case there are three degrees of freedom. 
	// The homogeneous transformation matrix is a convenient representation of the combined transformations; 
	// therefore, it is frequently

	// Through all particles
	const double den_weight = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	double w = 0.0;
	// translated observation
	double x_t;
	double y_t;
	
	for(int i = 0; i<particles.size();i++){
		// Translate observation into map coordinates associatated to particle

		// Define nearest neighbor and calculate weight for particle.
		// validate if min_dist < sensor_range
		float sensor_distance = 0.0;
		int min_dist_idx = 0;

		// Throgh all the measurements for this particle, check if measurement is lower than sensor range
		for(int j = 0; j<observations.size();j++){

			// translated observation
			x_t = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
			y_t = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;

			sensor_distance = dist(x_t, y_t, particles[i].x, particles[i].y);
			if (sensor_distance <= sensor_range ){
				// Calculate weights
			
				// const double dist_x = pow(x_t - map_landmarks.landmark_list[k].x_f, 2) / (2 * std_landmark[0] * std_landmark[0]);
				// const double dist_y = pow(y_t - map_landmarks.landmark_list[k].y_f, 2) / (2 * std_landmark[1] * std_landmark[1]);

				// TODO: replace particles[i]. to closest to x_t map landmark and the same for y_t
				 
				const double diff_x = pow(x_t - particles[i].x, 2) / (2 * std_landmark[0] * std_landmark[0]);
				const double diff_y = pow(y_t - particles[i].y, 2) / (2 * std_landmark[1] * std_landmark[1]);	
				particles[i].weight*= exp(-(diff_x + diff_y)) * den_weight;

				std::cout << "predicted - X: " << particles[i].x << " y: " << particles[i].y << " theta: " << particles[i].theta << std::endl;
				std::cout << "observed - X: " << observations[j].x << " y: " << observations[j].y << std::endl;
				std::cout << "particle: " << i << " observation: " << j << " x_t: " << x_t << " y_t: " << y_t << " dist_x: " << diff_x << " 	dist_y: " 
				<< diff_y << " weight_particle: " << particles[i].weight << " sensor_distance: " << sensor_distance  << std::endl;

			}

			// For each observation there's an error associated to the measurement, 
			// let's replace the observation to the real map landmark value.

			/* for(k=0; k < map_landmarks.landmark_list.size(); k++){

				min_dist = dist(x_t, y_t, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f)

				if (min_dist <= sensor_range){

				}
				if(k == 0){
					min_dist = dist(x_t, y_t, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
					min_dist_idx = k;
				}
				if (dist(x_t, y_t, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f) < min_dist){
					min_dist = dist(x_t, y_t, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
					min_dist_idx = k;
				}
			}*/

		}
		w+=particles[i].weight;

	}
	std::cout << "W: " << w << std::endl;
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
