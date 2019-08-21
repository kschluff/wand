#ifndef KALMAN_HPP
#define KALMAN_HPP
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

#include <opencv2/opencv.hpp>
#include <cmath>

/**
 * The position filter uses the following dynamic model
 * to estimate position, velocity and acceleration, 
 * with position observables from Kinect dept image..
 * 
 * [ x'     ]       [ 1  0  dt 0  dt2 0  ] [ x     ]        // x position
 * [ y'     ]    =  [ 0  1  0  dt 0  dt2 ] [ y     ] + R    // y position
 * [ x_vel' ]       [ 0  0  1  0  dt 0   ] [ x_vel ]        // x velocity
 * [ y_vel' ]       [ 0  0  0  1  0  dt  ] [ y_vel ]        // y velocity
 * [ x_acc' ]       [ 0  0  0  0  1  0   ] [ x_acc ]        // x acceleration
 * [ y_acc' ]       [ 0  0  0  0  0  1   ] [ y_acc ]        // y acceleration

 * where, dt2 = 1/2 dt^2
 * 
 * [ x_obs ]   = [ 1 0 0 0 0 0 ] [ x     ]  + Q
 * [ y_obs ]     [ 0 1 0 0 0 0 ] [ y     ]
 *                               [ x_vel ]
 *                               [ y_vel ]
 *                               [ x_acc ]
 *                               [ y_acc ]
 *

 */
class Kalman 
{
public:

   Kalman();
   virtual ~Kalman();

   /**
    * Set the filter initial conditions from an 
    * initial measured point.
    */
   void init(const cv::Point& initial, float positionProcessNoise = 1e-5, 
	     float positionMeasurmentNoise = 1e-3);

   /** 
    * Project forward the system state and 
    * return the predicted search window.
    */
   cv::Point predict();

   /**
    * Apply observed position, return
    * the corrected search window.
    */
   cv::Point correct(const cv::Point& observed);

   /**
    * Propagate the system state with no new 
    * measurements.
    */
   void propagate();

   // Accessors

   enum PosStates
   {
      STATE_X,
      STATE_Y,
      STATE_X_VEL,
      STATE_Y_VEL,
      STATE_X_ACCEL,
      STATE_Y_ACCEL,
      NUM_POS_STATES
   };

   bool initialized() const
   { return m_initialized; };

   cv::Point2f predicted_position() const
   { return cv::Point2f(m_positionFilter.statePre.at<float>(STATE_X), m_positionFilter.statePre.at<float>(STATE_Y)); }

   cv::Point2f corrected_position() const
   { return cv::Point2f(m_positionFilter.statePost.at<float>(STATE_X), m_positionFilter.statePost.at<float>(STATE_Y)); }

   cv::Point2f predicted_velocity() const
   { return cv::Point2f(m_positionFilter.statePre.at<float>(STATE_X_VEL), m_positionFilter.statePre.at<float>(STATE_Y_VEL)); }

   cv::Point2f corrected_velocity() const
   { return cv::Point2f(m_positionFilter.statePost.at<float>(STATE_X_VEL), m_positionFilter.statePost.at<float>(STATE_Y_VEL)); }

   float predicted_velocity_angle() const
   { return std::atan2(m_positionFilter.statePre.at<float>(STATE_X_VEL), m_positionFilter.statePre.at<float>(STATE_Y_VEL)); }

   cv::Point2f predicted_acceleration() const
   { return cv::Point2f(m_positionFilter.statePre.at<float>(STATE_X_ACCEL), m_positionFilter.statePre.at<float>(STATE_Y_ACCEL)); }

   cv::Point2f corrected_acceleration() const
   { return cv::Point2f(m_positionFilter.statePost.at<float>(STATE_X_ACCEL), m_positionFilter.statePost.at<float>(STATE_Y_ACCEL)); }

   cv::Mat& corrected_state()
   { return m_positionFilter.statePost; }

private:
   
   cv::KalmanFilter m_positionFilter;
   bool m_initialized;
   
};


#endif
