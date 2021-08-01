/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
   num_particles = 256;  // TODO: Set the number of particles

   std::normal_distribution<double> x_distribution(x, std[0]);
   std::normal_distribution<double> y_distribution(y, std[1]);
   std::normal_distribution<double> theta_distribution(theta, std[2]);
   std::default_random_engine random;

   for (int i = 0; i < num_particles; i++)
   {
       Particle p;
       p.id = i;
       p.x = x_distribution(random);
       p.y = y_distribution(random);
       p.theta = theta_distribution(random);
       p.weight = 1.0;

       particles.push_back(p);
   }

   is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

   std::default_random_engine random;
   for (auto& p : particles)
   {
       // C1 and C2 are simply intermediate computations for ease
       const double c1 = velocity / yaw_rate;
       const double c2 = yaw_rate * delta_t;

       // For (x,y,theta) transform the particle position by the velocity
       // and yaw rate, and sample from the normal distribution defined
       // by the standard deviations in std_pos[]
       const double x = p.x + c1 * (sin(p.theta + c2) - sin(p.theta));
       std::normal_distribution<double> x_dist(x, std_pos[0]);
       p.x = x_dist(random);

       const double y = p.y + c1 * (cos(p.theta) - cos(p.theta + c2));
       std::normal_distribution<double> y_dist(y, std_pos[1]);
       p.y = y_dist(random);

       const double theta = p.theta + c2;
       std::normal_distribution<double> theta_dist(theta, std_pos[2]);
       p.theta = theta_dist(random);
   }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

   for (auto& o : observations) {
       double min_dist = std::numeric_limits<double>::max();
       int min_landmark_id = -1;

       for (const auto& p : predicted)
       {
           // If the linear distance in either the x or y axis is greater
           // than min_distance, don't calculate the quadratic distance
           if (abs(o.x - p.x) > min_dist ||
               abs(o.y - p.y) > min_dist)
           {
               continue;
           }

           // Distance between observation (x,y) and predicted (x,y)
           const double distance = dist(o.x, o.y, p.x, p.y);

           // Find the ID of the closest particle p to the landmark o
           if (distance < min_dist) {
               min_dist = distance;
               min_landmark_id = p.id;
           }
       }

       // Assign the predicted landmark ID to the observation ID
       o.id = min_landmark_id;
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

   for (auto& p : particles)
   {
       // Find the set of landmarks within the sensor range
       // and add to a new vector "predicted"
       vector<LandmarkObs> predicted;
       for (const auto& lm : map_landmarks.landmark_list)
       {
           // If the linear distance in either axis is greater than
           // the sensor range, do not calculate the quadratic distance
           if (abs(p.x - lm.x) > sensor_range ||
               abs(p.y - lm.y) > sensor_range)
           {
               continue;
           }

           if (dist(p.x, p.y, lm.x, lm.y) < sensor_range)
           {
               predicted.push_back(lm);
           }
       }

       // Transform each of the observations from the vehicle coordinate
       // system to the map coordinate system to be compared to particles
       vector<LandmarkObs> transformed_observations;
       for (const auto& o : observations)
       {
           const double x = p.x + cos(p.theta) * o.x - sin(p.theta) * o.y;
           const double y = p.y + sin(p.theta) * o.x + cos(p.theta) * o.y;
           transformed_observations.push_back(LandmarkObs{o.id, x, y});
       }

       // Find the nearest neighbors between the in-range landmarks (predicted)
       // and the sensor observations in the global coordinate system
       dataAssociation(predicted, transformed_observations);

       // Create dummy vectors of ID associations and measurements
       // which will be used for visualizations in the simulation
       vector<int> associations;
       vector<double> sense_x;
       vector<double> sense_y;

       double weight = 1.0;
       const double std_x = std_landmark[0];
       const double std_y = std_landmark[1];
       for (const auto& o : transformed_observations)
       {
           for (const auto& lm : predicted)
           {
               if (o.id == lm.id)
               {
                   // Find the association weight using the mult-variate Gaussian function
                   // using the function defined in the lesson.
                   const double x = (lm.x - o.x) * (lm.x - o.x) / (2.f * std_x * std_x);
                   const double y = (lm.y - o.y) * (lm.y - o.y) / (2.f * std_y * std_y);
                   weight *= exp(-(x+y)) / (2.f * M_PI * std_x * std_y);

                   // Add the associated landmark ID and (x,y) measurement to the
                   // visualization / debug vectors
                   associations.push_back((o.id));
                   sense_x.push_back((o.x));
                   sense_y.push_back((o.y));
               }
           }
       }

       // Update the particle's weight with the product of all the
       // weights of each association to the observable landmarks
       p.weight = weight;

       // For debugging / visualization purposes add the associated
       // observation vectors, and sense vectors
       SetAssociations(p, associations, sense_x, sense_y);
   }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

   // Create a stand-alone vector of weights from all the particles
   std::vector<double> weights;
   for (const auto& p : particles)
   {
       weights.push_back(p.weight);
   }

   // Create a random device to sample from the weight distribution
   std::random_device rd;
   std::mt19937 gen(rd());

    // Use a discrete_distribution per cppreference.com to replace
   // the particles vector with a new vector of sampled particles
   // based on the particle weights
   std::discrete_distribution<> d(weights.begin(), weights.end());
   std::vector<Particle> samples;

   for (int i = 0; i < num_particles; i++)
   {
       samples.push_back(particles[d(gen)]);
   }

   particles = samples;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}