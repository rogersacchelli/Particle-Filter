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

/* *******************************************
	PROJECT SUBMISSION FOR CAR ND
   	STUDENT: ROGER SACCHELLI
   ******************************************* */


using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 1000;	
	
	std::default_random_engine gen_part_pos;

	std::normal_distribution<double> N_part_x(x, std[0]);
	std::normal_distribution<double> N_part_y(y, std[1]);
	std::normal_distribution<double> N_part_theta(theta, std[2]);

	particles.resize(num_particles);
	weights.resize(num_particles);

	for (int i=0; i < num_particles; i++){
		particles[i].id = i;
		particles[i].x = N_part_x(gen_part_pos);
		particles[i].y = N_part_y(gen_part_pos);
		particles[i].theta = N_part_theta(gen_part_pos);
		particles[i].weight = 1.0;

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

	const double vel_pred = N_pred_vel(gen_pred);
	const double yaw_r_pred = N_pred_yaw_r(gen_pred);

	const double vel_yaw = vel_pred/yaw_r_pred;
	const double yaw_r_dt = yaw_r_pred * delta_t;
	
    normal_distribution<double> N_pred_x(0.0, std_pos[0]);
	normal_distribution<double> N_pred_y(0.0, std_pos[1]);
	normal_distribution<double> N_pred_theta(0.0, std_pos[2]);

	for (int i=0; i<particles.size();i++){
		// Check if car is turning or not
		if (yaw_rate > 0.001){
			particles[i].x = particles[i].x + (vel_yaw) * (sin(particles[i].theta + yaw_r_dt ) - sin(particles[i].theta));
			particles[i].y = particles[i].y + (vel_yaw) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_r_dt));
			particles[i].theta = particles[i].theta + yaw_r_dt;
		}else{
			particles[i].x = particles[i].x + vel_pred * delta_t * cos(particles[i].theta);
            particles[i].y = particles[i].y + vel_pred * delta_t * sin(particles[i].theta);
		}
		// Add random Gaussian noise
        particles[i].x += N_pred_x(gen_pred);
        particles[i].y += N_pred_y(gen_pred);
        particles[i].theta += N_pred_theta(gen_pred);
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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
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
		double sensor_distance = 0.0;

		// Throgh all the measurements for this particle, check if measurement is lower than sensor range
		for(int j = 0; j<observations.size();j++){

			// minimum distante to calculate particle weight
			
			// translated observation
			x_t = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
			y_t = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;

			sensor_distance = dist(x_t, y_t, particles[i].x, particles[i].y);
			if (sensor_distance <= sensor_range ){
				// Calculate weights
			
				// TODO: replace particles[i]. to closest to x_t map landmark and the same for y_t.

				double min_dist = 9999.0;
				double eval_dist;
				int min_dist_idx = 0;

				for(int k=0; k < map_landmarks.landmark_list.size(); k++){

					eval_dist = dist(x_t, y_t, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
					if (eval_dist < min_dist and eval_dist < sensor_range){
						min_dist = eval_dist;
						min_dist_idx = k;
					}

				}
				 
				const double diff_x = pow(x_t - map_landmarks.landmark_list[min_dist_idx].x_f, 2) / (2 * std_landmark[0] * std_landmark[0]);
				const double diff_y = pow(y_t - map_landmarks.landmark_list[min_dist_idx].y_f, 2) / (2 * std_landmark[1] * std_landmark[1]);	
				particles[i].weight*= exp(-(diff_x + diff_y)) * den_weight;

				//std::cout << "==========================================================================================================" << std::endl;
				//std::cout << "Particle: " << i << " Observation: " << j << std::endl;
				//std::cout << "predicted - X:" << particles[i].x << " y:" << particles[i].y << " theta:" << particles[i].theta << std::endl;
				//std::cout << "observed - X:" << observations[j].x << " y:" << observations[j].y << std::endl;
				//std::cout << "closest observed landmark idx:" << min_dist_idx << " x:" << map_landmarks.landmark_list[min_dist_idx].x_f << " y:" << map_landmarks.landmark_list[min_dist_idx].y_f  << std::endl;
				//std::cout << " x_t: " << x_t << " y_t: " << y_t << " dist_x: " << diff_x << " 	dist_y: " << diff_y << " weight_particle: " << particles[i].weight << " sensor_distance: " << sensor_distance  << std::endl;
			
			}

		}
		w+=particles[i].weight;
	}

	// Weights norm
	for (int i = 0; i < num_particles; i++){
		particles[i].weight /= w * (2 * M_PI * std_landmark[0] * std_landmark[1]);
		weights[i] = particles[i].weight;
	}	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::default_random_engine gen_resample;
    discrete_distribution<> dist_particles(weights.begin(), weights.end());
    vector<Particle> resampled_particles;
    resampled_particles.resize(num_particles);
    for (int i = 0; i < num_particles; i++) {
        resampled_particles[i] = particles[dist_particles(gen_resample)];
    }
    particles = resampled_particles;
}


Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<double>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<double>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

