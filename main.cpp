/**
 * Magic pixie-dust wand effect using OpenCV and Kinect
 */

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
#include <vector>
#include <iostream>
#include <cmath>
#include <sys/time.h>
#include "kalman.h"
#include "particle.h"

using namespace cv;
using namespace std;

const char* WINDOW = "Fairy Wand";

Point track_nearest_point(const Mat& depth_map, const Mat& depth_image, Mat_<float>& state)
{   
   double min_val = 0, max_val = 0;
   Point min_loc, max_loc;
   Point position;
   static const float process_noise = 1e-5;
   static const float measurements_noise = 1e-1;
   static const Rect bounds(0, 0, depth_image.cols, depth_image.rows);

   static Kalman filter;
   
   // Find the nearest point. The depth image is used to ignore 0 values, which
   // are points that are outside of the Kinect's depth sensor range
   minMaxLoc(depth_map, &min_val, &max_val, &min_loc, &max_loc, depth_image);

   if( !filter.initialized() )
   {
      // Initialize the Kalman filter based on the initial position
      // Noise values are tuned empirically (i.e., looked OK visually)
      position = min_loc;
      filter.init(position, process_noise, measurements_noise);
      filter.predict();
   }

   position = filter.correct(min_loc);
  
   if( !bounds.contains(position) )
   {
      // The filter sometimes becomes unstable; quick fix is to re-init
      filter.init(min_loc, process_noise, measurements_noise);
      filter.predict();
   }
   else
   {
      state = filter.corrected_state();
      filter.predict();
   }

   return position;
}

int main(int argc, char** argv)
{
   Mat depth_map;    // 10-bit depth map from Kinect depth sensor
   Mat depth_image;  // 8-bit depth image for use as a mask
   Mat bgr_image;    // colour image from Kinect camera
   timeval start_time, end_time; // Track elapsed time per frame
   Point position(0,0);  // Position of the "wand" (or nearest object to the Kinect)
   Mat_<float>state(6, 1); // Position, velocity, acceleration state of wand
   namedWindow(WINDOW, CV_WINDOW_FREERATIO | CV_GUI_NORMAL);

   VideoCapture cap(CV_CAP_OPENNI);
   if( !cap.isOpened() )
   {
      cerr << "Failed to open Kinect device" << endl;
      exit(1);
   }

   VideoWriter recorder;

   if( argc > 1 )
   {
      int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
      int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
//      int fps = cap.get(CV_CAP_PROP_FPS);
      recorder.open(argv[1], CV_FOURCC('j','p','e','g'), 14, Size(width, height));
      if( !recorder.isOpened() )
      {
	 cerr << "Failed to open video writer" << endl;
	 exit(1);
      }

   }

   for(;;)
   { 
      // Start timing
      gettimeofday(&start_time, 0);

      // Get the depth and colour images from the Kinect
      cap.grab();
      cap.retrieve(bgr_image, CV_CAP_OPENNI_BGR_IMAGE);
      cap.retrieve(depth_map, CV_CAP_OPENNI_DEPTH_MAP);

      // Flip to a mirror image, for facing the camera
      flip(bgr_image, bgr_image, 1);
      flip(depth_map, depth_map, 1);
      
      // Convert 10-bit depth image to 8-bit
      // TODO - there's probably a more efficient way to do this
      depth_map.convertTo(depth_image, CV_8UC1, .25);
   
      // Find the nearest point and estimate PVA state
      position = track_nearest_point(depth_map, depth_image, state);
//      circle(bgr_image, position, 5, Scalar(255, 0, 0), 2);

      // Delay, esc to quit.  This value works OK on my 2007 Macbook
      char ch = waitKey(50);
      if(ch == 27)
	 exit(0);
      
      // Elapsed time in seconds
      gettimeofday(&end_time, 0);
      float dt = (float)(end_time.tv_sec - start_time.tv_sec) + ((float)(end_time.tv_usec - start_time.tv_usec)) * 1E-6; 

//      cout << "Frame rate:" << 1.f/dt << " dt: " << dt << endl;

      // Update the positions of each particle, generate new ones between the previous
      // and current position and draw all of them
      Particle::time_update(dt);
      Particle::generate_particles(state, dt);
      Particle::draw_particles(bgr_image);

      imshow(WINDOW, bgr_image);
      if( recorder.isOpened() )
      {
	 recorder.write(bgr_image);
      }
   }

   return 0;
}
