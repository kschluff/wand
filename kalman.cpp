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

#include "kalman.h"
#include <iostream>

using namespace cv;
using namespace std;

// dt = 1 for velocity in pixels/frame
// dt  = 1/FPS for velocity in pixels/second
const float dt = 1;
const float dt2 = 0.5 * dt * dt; //  1/2 dt^2 for acceleration

Kalman::Kalman()
   :m_positionFilter(),
    m_initialized(false)
{}

Kalman::~Kalman()
{}


void Kalman::init(const Point& initial, float positionProcessNoise, float positionMeasurementNoise)
{
   
   //
   // Initialize the position filter
   //

   m_positionFilter.init(
      6,    // State vector dimension
      2,    // Measurement vector dimension
      0 );  // Control vector dimension

   m_positionFilter.transitionMatrix = *(Mat_<float>(6, 6) << 
			       1, 0, dt,  0, dt2,   0, 
			       0, 1,  0, dt,   0, dt2, 
			       0, 0,  1,  0,  dt,   0, 
			       0, 0,  0,  1,   0,  dt, 
			       0, 0,  0,  0,   1,   0, 
			       0, 0,  0,  0,   0,   1);

   m_positionFilter.measurementMatrix = *(Mat_<float>(2, 6) << 
				1, 0, 0, 0, 0, 0, 
				0, 1, 0, 0, 0, 0); 

   
   setIdentity(m_positionFilter.processNoiseCov, Scalar::all(positionProcessNoise));
   // Constant acceleration is not a great model, so add some noise
   m_positionFilter.processNoiseCov.at<float>(STATE_X_ACCEL, STATE_X_ACCEL) *= 100;
   m_positionFilter.processNoiseCov.at<float>(STATE_Y_ACCEL, STATE_Y_ACCEL) *= 100;


//   cout << "Process noise covariance: " << endl;
//   cout << m_positionFilter.processNoiseCov << endl;

   setIdentity(m_positionFilter.measurementNoiseCov, Scalar::all(positionMeasurementNoise));

   // Needed for first iteration only
   setIdentity(m_positionFilter.errorCovPost, Scalar::all(0.1));
   
   // Set initial position, velocity and acceleration
   m_positionFilter.statePost.at<float>(STATE_X) = initial.x;
   m_positionFilter.statePost.at<float>(STATE_Y) = initial.y;

   m_positionFilter.statePost.at<float>(STATE_X_VEL) = 0;
   m_positionFilter.statePost.at<float>(STATE_Y_VEL) = 0;

   m_positionFilter.statePost.at<float>(STATE_X_ACCEL) = 0;
   m_positionFilter.statePost.at<float>(STATE_Y_ACCEL) = 0;

   m_initialized = true;

}

Point Kalman::predict()
{

   const Mat& predictedPosition = m_positionFilter.predict();

   Point result;
   result.x = predictedPosition.at<float>(STATE_X);
   result.y = predictedPosition.at<float>(STATE_Y);

   return result;
}

Point Kalman::correct(const Point& observed)
{
   Mat measuredPosition = Mat::zeros(2, 1, CV_32F);
   measuredPosition.at<float>(0) = observed.x;
   measuredPosition.at<float>(1) = observed.y;

   const Mat& correctedPosition = m_positionFilter.correct(measuredPosition);

   Point result;
   result.x = correctedPosition.at<float>(STATE_X);
   result.y = correctedPosition.at<float>(STATE_Y);
  
   return result;
}

void Kalman::propagate()
{
   m_positionFilter.statePost = m_positionFilter.statePre;
}
