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
#define EPS 0.00001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // if filter is already initialized ,skip the rest
  if (is_initialized) {
    return;
  }

  // Initializing the number of particles
  num_particles = 100;

  // Extracting standard deviations for initial GPS and yaw measurement noise
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Creating normal distributions for parameters, mean and standard deviations
  normal_distribution<double> distribution_x(x, std_x);
  normal_distribution<double> distribution_y(y, std_y);
  normal_distribution<double> distribution_theta(theta, std_theta);

  // Generate particles with normal distribution with mean on GPS and yaw values.
  for (int i = 0; i < num_particles; i++) {

    Particle particle;
    particle.id = i;
    particle.x = distribution_x(gen);
    particle.y = distribution_y(gen);
    particle.theta = distribution_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
	}

  // The filter is now initialized.
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Extracting standard deviations for motion gaussians
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Creating normal distributions
  normal_distribution<double> distribution_x(0, std_x);
  normal_distribution<double> distribution_y(0, std_y);
  normal_distribution<double> distribution_theta(0, std_theta);

  // Calculate new state.
  for (int i = 0; i < num_particles; i++) {

  	double theta = particles[i].theta;

    if ( fabs(yaw_rate) < EPS ) {
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );

    } else {
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    }

    // Adding noise.
    particles[i].x += distribution_x(gen);
    particles[i].y += distribution_y(gen);
    particles[i].theta += distribution_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	for (unsigned int i = 0; i < observations.size(); i++) {

	    // grab current observation
	    LandmarkObs o = observations[i];

	    // init minimum distance to maximum possible
	    double min_dist = numeric_limits<double>::max();

	    // init id of landmark from map placeholder to be associated with the observation
	    int map_id = -1;

	    for (unsigned int j = 0; j < predicted.size(); j++) {
	      // grab current prediction
	      LandmarkObs p = predicted[j];

	      // get distance between current/predicted landmarks
	      double cur_dist = dist(o.x, o.y, p.x, p.y);

	      // find the predicted landmark nearest the current observed landmark
	      if (cur_dist < min_dist) {
	        min_dist = cur_dist;
	        map_id = p.id;
	      }
	    }

	    // set the observation's id to the nearest predicted landmark's id
	    observations[i].id = map_id;
	  }
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
  double stdLndmrkRange = std_landmark[0];
  double stdLndmrkBearing = std_landmark[1];

  for (int i = 0; i < num_particles; i++) {

    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    // Find landmarks in particle's range.
    double sns_rng_sqr = sensor_range * sensor_range;
    vector<LandmarkObs> inRangeProbableLndmrks;
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      float lndmrkX = map_landmarks.landmark_list[j].x_f;
      float lndmrkY = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      double dX = x - lndmrkX;
      double dY = y - lndmrkY;
      if ( dX*dX + dY*dY <= sns_rng_sqr ) {
    	  inRangeProbableLndmrks.push_back(LandmarkObs{ id, lndmrkX, lndmrkY });
      }
    }

    // Transform observation coordinates.
    vector<LandmarkObs> mappedObservations;
    for(unsigned int j = 0; j < observations.size(); j++) {
      double xx = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double yy = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mappedObservations.push_back(LandmarkObs{ observations[j].id, xx, yy });
    }

    // Observation association to landmark.
    dataAssociation(inRangeProbableLndmrks, mappedObservations);

    // Resetting weight of particle.
    particles[i].weight = 1.0;
    // Calculate weights.
    for(unsigned int j = 0; j < mappedObservations.size(); j++) {
      double obsX = mappedObservations[j].x;
      double obsY = mappedObservations[j].y;

      int landmarkId = mappedObservations[j].id;

      double landmarkX, landmarkY;
      unsigned int k = 0;
      unsigned int noOfLndmrks = inRangeProbableLndmrks.size();
      bool found = false;
      while( !found && k < noOfLndmrks ) {
        if ( inRangeProbableLndmrks[k].id == landmarkId) {
          found = true;
          landmarkX = inRangeProbableLndmrks[k].x;
          landmarkY = inRangeProbableLndmrks[k].y;
        }
        k++;
      }

      // Calculating weight using multivariate gaussian probability function
      double dX = obsX - landmarkX;
      double dY = obsY - landmarkY;

      double weight = ( 1/(2*M_PI*stdLndmrkRange*stdLndmrkBearing)) * exp( -( dX*dX/(2*stdLndmrkRange*stdLndmrkRange) + (dY*dY/(2*stdLndmrkBearing*stdLndmrkBearing)) ) );
      if (weight == 0) {
        particles[i].weight *= EPS;
      } else {
        particles[i].weight *= weight;
      }
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Get weights and max weight.
 vector<double> weights;
  double maxWeight = numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if ( particles[i].weight > maxWeight ) {
      maxWeight = particles[i].weight;
    }
  }

  // Creating distributions.
  uniform_real_distribution<double> distDouble(0.0, maxWeight);
  uniform_int_distribution<int> distInt(0, num_particles - 1);

  // Generating index.
  int index = distInt(gen);

  double beta = 0.0;

  // the wheel
  vector<Particle> newParticles;
  for(int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    newParticles.push_back(particles[index]);
  }

  particles = newParticles;
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
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
