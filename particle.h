#ifndef PARTICLE_H
#define PARTICLE_H

/**
 * @copyright
 *
 * Copyright 2012 Kevin Schluff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, 
 * as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 */

#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

typedef unsigned int uint;

// E.g., Generate 50 particles per second, they live for 20s for a steady state
// of 1000 active particles
const float PARTICLE_RATE = 750.0; ///  particles generated / second
const float PARTICLE_TTL_MAX = 1.0;  /// initial time to live
const uint MAX_PARTICLES = 750;    /// PARTICLE_RATE*PARTICLE_TTL_MAX;
const float PARTICLE_GRAVITY_ACCEL = 30.; /// pixels/s^2
const float PARTICLE_AIR_RESISTANCE = .75; /// 1 - Resistive force, proportional to velocity
class Particle
{
public:

   enum States
   {
      STATE_X,
      STATE_Y,
      STATE_X_VEL,
      STATE_Y_VEL,
      STATE_X_ACCEL,
      STATE_Y_ACCEL,
      NUM_STATES
   };

   // Default placeholder particle
   Particle()
      :m_state(NUM_STATES, 1), m_alpha(0), m_ttl(0.), m_color(0,0,0)
   {
   }

   Particle( const Particle& other)
      :m_alpha(other.m_alpha), m_ttl(other.m_ttl), m_color(other.m_color)
   {     
      // Force deep copy
      other.m_state.copyTo(m_state);
   }

   Particle& operator=(const Particle& other)
   {
      if( &other != this )
      {
	 // Force deep copy
	 other.m_state.copyTo(m_state);
	 m_alpha = other.m_alpha;
	 m_ttl = other.m_ttl;
	 m_color = other.m_color;
      }
      return *this;
   }

   /**
    * Initialize a new particle from the allocated pool.
    */
   static Particle& create(const cv::Mat_<float>& state);
   
   /**
    * Advance the position, velocity and acceleration of the
    * particles according to the kinematic transition matrix.
    */
   static void time_update(float dt);
   
   /**
    * Alpha-blend and display the particles.
    */
   static void draw_particles(cv::Mat& image);

   /**
    * Generate new random particles between the previous point and this point
    */
   static void generate_particles(cv::Mat_<float>& state, float dt);

   // State accessors
   float x() const { return m_state(STATE_X); }
   float y() const { return m_state(STATE_Y); }
   float x_vel() const { return m_state(STATE_X_VEL); }
   float y_vel() const { return m_state(STATE_Y_VEL); }
   float x_accel() const { return m_state(STATE_X_ACCEL); }
   float y_accel() const { return m_state(STATE_Y_ACCEL); }

private:

   /// The pool of particles
   static Particle s_particles[MAX_PARTICLES];
   /// Index of the next particle to createa
   static uint s_next;
   /// The random number generator state
   static cv::RNG s_rng;
   /// The kinematic transition matrix based on equations
   /// of motion for pixie dust.
   static cv::Mat_<float> s_transition_matrix;

   /// The current position, velocity, acceleration state for a particle
   cv::Mat_<float> m_state;
   /// The current alpha transparency level: 1.0 opaque, 0.0 transparent
   float m_alpha;
   /// Time to live in seconds
   float m_ttl; 
   /// Randomized particle colour
   cv::Vec3b m_color;
};


#endif
