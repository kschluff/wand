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

#include "particle.h"

using namespace cv;
using namespace std;

uint Particle::s_next = 0;
Particle Particle::s_particles[MAX_PARTICLES];
RNG Particle::s_rng;
Mat_<float> Particle::s_transition_matrix(NUM_STATES, NUM_STATES);

Particle& Particle::create(const Mat_<float>& state)
{

   Particle& p = s_particles[s_next];

   state.copyTo(p.m_state);
   
   // Add a bit of random 'magic'
   p.m_state.at<float>(STATE_X) += s_rng.uniform(-5.f, 5.f);
   p.m_state.at<float>(STATE_Y) += s_rng.uniform(-5.f, 5.f);
   p.m_state.at<float>(STATE_X_VEL) += s_rng.uniform(-10.f, 10.f);
   p.m_state.at<float>(STATE_Y_VEL) += s_rng.uniform(-5.f, 5.f);

   // Force constant accel for Y 
   p.m_state.at<float>(STATE_Y_ACCEL) = PARTICLE_GRAVITY_ACCEL;

   // Start at full opacity
   p.m_alpha = 1.0;
   p.m_ttl = PARTICLE_TTL_MAX;
   
//   p.m_color = Vec3b(s_rng.uniform(0, 255), s_rng.uniform(200, 255), s_rng.uniform(200, 255));
   p.m_color = Vec3b(s_rng.uniform(200, 255), s_rng.uniform(20, 255), s_rng.uniform(20, 255));

   
   s_next = (s_next + 1) % MAX_PARTICLES;

   return p;
}
   
void Particle::time_update(float dt)
{
   float dt2 = .5 * dt * dt;
   float k = PARTICLE_AIR_RESISTANCE;
   s_transition_matrix = *(Mat_<float>(NUM_STATES, NUM_STATES) << 
			 1, 0, dt,  0, dt2,   0,  // x(t) = x(t-1) + x_vel*dt + 1/2 * x_accel * dt^2
			 0, 1,  0, dt,   0, dt2,  // y(t) = y(t-1) + y_vel*dt + 1/2 * y_accel * dt^2
			 0, 0,  k,  0,  dt,   0,  // x_vel(t) = k*x_vel(t-1) + x_accel * dt
			 0, 0,  0,  1,   0,  dt,  // y_vel(t) = y_vel(t-1) + y_accel * dt
			 0, 0,  0,  0,   0,   0,  // x_accel(t) = 0 (constant velocity)
			 0, 0,  0,  0,   0,   1); // y_accel(t) = y_accel(t-1) (constant accel due to gravity)

   for( uint i = 0; i < MAX_PARTICLES; i++)
   {
      Particle& p = s_particles[i];

      // Update every visible particle
      if(p.m_alpha > 0.0)
      {
	 p.m_state = s_transition_matrix * p.m_state;
	 p.m_ttl = MAX(0.0, p.m_ttl - dt);
	 p.m_alpha =  p.m_ttl/PARTICLE_TTL_MAX;
      }
   } 
}

void Particle::draw_particles(Mat& image)
{

   Rect bounds(0,0, image.cols, image.rows-1);

   for(uint i = 0; i < MAX_PARTICLES; i++)
   {
      Particle& p = s_particles[i];
 
      // Draw every visible particle
      if( p.m_alpha > 0.0 )
      {
	 int x = p.x();
	 int y = p.y();
	 if(bounds.contains(Point(x,y)))
	 {
	    image.at<Vec3b>(y, x) = image.at<Vec3b>(y, x) * (1.f - p.m_alpha) + (p.m_color * p.m_alpha);
	    image.at<Vec3b>(y+1, x) = image.at<Vec3b>(y+1, x) * (1.f - p.m_alpha) + (p.m_color * p.m_alpha);
	 }
      }
   }
}

void Particle::generate_particles(Mat_<float>& state, float dt)
{
   static Mat prev_state;

   if( !prev_state.empty())
   {   
      Mat temp1, temp2, interp_state;

      uint num_particles = PARTICLE_RATE * dt;
      temp1 = (state - prev_state);
      for(uint i = 0; i < num_particles; i++)
      {
	 float r = s_rng.uniform(0.f, 1.f);
	 temp2 = r * temp1;
	 interp_state = prev_state + temp2;
	 create(interp_state);
      };
   }

   state.copyTo(prev_state);
}
