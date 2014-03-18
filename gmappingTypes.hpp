#ifndef gmapping_TYPES_HPP
#define gmapping_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/time.h>
#include <base/pose.h>
#include <vector>

namespace gmapping {

	struct Config {
		bool inverted_laser; // (bool, default: false)
			// 
		int throttle_scans; // (int, default: 1)
			// Process 1 out of every this many scans (set it to a higher number to skip more scans)
		std::string laser_frame; // (string, default: "laser")
			// The frame attached to the laser.
		std::string base_frame; // (string, default: "base_link")
			// The frame attached to the mobile base.
		std::string map_frame; // (string, default: "map")
			// The frame attached to the map.
		std::string odom_frame; // (string, default: "odom")
			// The frame attached to the odometry system.
		float map_update_interval; // (float, default: 5.0)
			// How long (in seconds) between updates to the map. Lowering this number updates the occupancy grid more often, at the expense of greater computational load.
		float maxUrange; // (float, default: 80.0)
			// The maximum usable range of the laser. A beam is cropped to this value.
		float sigma; // (float, default: 0.05)
			// The sigma used by the greedy endpoint matching
		int kernelSize; // (int, default: 1)
			// The kernel in which to look for a correspondence
		float lstep; // (float, default: 0.05)
			// The optimization step in translation
		float astep;  // (float, default: 0.05)
			// The optimization step in rotation
		int iterations;  // (int, default: 5)
			// The number of iterations of the scanmatcher
		float lsigma; // (float, default: 0.075)
			// The sigma of a beam used for likelihood computation
		float ogain; // (float, default: 3.0)
			// Gain to be used while evaluating the likelihood, for smoothing the resampling effects
		int lskip; // (int, default: 0)
			// Number of beams to skip in each scan.
		float srr; // (float, default: 0.1)
			// Odometry error in translation as a function of translation (rho/rho)
		float srt; // (float, default: 0.2)
			// Odometry error in translation as a function of rotation (rho/theta)
		float str; // (float, default: 0.1)
			// Odometry error in rotation as a function of translation (theta/rho)
		float stt; // (float, default: 0.2)
			// Odometry error in rotation as a function of rotation (theta/theta)
		float linearUpdate; // (float, default: 1.0)
			// Process a scan each time the robot translates this far
		float angularUpdate; // (float, default: 0.5)
			// Process a scan each time the robot rotates this far
		float temporalUpdate; // (float, default: -1.0)
			// Process a scan if the last scan proccessed is older than the update time in seconds. A value less than zero will turn time based updates off.
		float resampleThreshold; // (float, default: 0.5)
			// The Neff based resampling threshold
		int particles; // (int, default: 30)
			// Number of particles in the filter
		float xmin; // (float, default: -100.0)
			// Initial map size
		float ymin; // (float, default: -100.0)
			// Initial map size
		float xmax; // (float, default: 100.0)
			// Initial map size
		float ymax; // (float, default: 100.0)
			// Initial map size
		float delta; // (float, default: 0.05)
			// Processing parameters (resolution of the map)
		float llsamplerange; // (float, default: 0.01)
			// Translational sampling range for the likelihood
		float llsamplestep; // (float, default: 0.01)
			// Translational sampling step for the likelihood
		float lasamplerange; // (float, default: 0.005)
			// Angular sampling range for the likelihood
		float lasamplestep; // (float, default: 0.005)
			// Angular sampling step for the likelihood
		float transform_publish_period; // (float, default: 0.05)
			// How long (in seconds) between transform publications.
		float occ_thresh; // (float, default: 0.25)
			// Threshold on gmapping's occupancy values. Cells with greater occupancy are considered occupied (i.e., set to 100 in the resulting sensor_msgs/LaserScan). New in 1.1.0.
		float maxRange; //(float, default: NULL)
			// The maximum range of the sensor. If regions with no obstacles within the range of the sensor should appear as free space in the map, set maxUrange < maximum range of the real sensor <= maxRange.
		Config() : inverted_laser(false)
				 , throttle_scans(1)
				 , laser_frame("laser")
				 , base_frame("base_link")
				 , map_frame("map")
				 , odom_frame("odom")
				 , map_update_interval(1.0)
				 , maxUrange(60.0)
				 , sigma(0.05)
				 , kernelSize(1)
				 , lstep(0.05)
				 , astep(0.05)
				 , iterations(5)
				 , lsigma(0.075)
				 , ogain(3.0)
				 , lskip(0)
				 , srr(0.1)
				 , srt(0.2)
				 , str(0.1)
				 , stt(0.2)
				 , linearUpdate(0.5)
				 , angularUpdate(0.2) 
				 , temporalUpdate(-1.0)
				 , resampleThreshold(0.5)
				 , particles(30) 
				 , xmin(-25.0) // -100.0
				 , ymin(-25.0) 
				 , xmax(25.0)
				 , ymax(25.0)
				 , delta(0.05)
				 , llsamplerange(0.01)
				 , llsamplestep(0.01)
				 , lasamplerange(0.005)
				 , lasamplestep(0.005)
				 , transform_publish_period(0.05)
				 , occ_thresh(0.25) 
				 , maxRange(60.0) {}
	};


	struct OccupancyGrid 
	{	
		base::Time stamp;
		std::string frame_id;
		base::Time map_load_time;
		float resolution;
		uint32_t height;
		uint32_t width;
		base::Pose origin;
		std::vector<uint8_t> data;
	};


	/* wrapper for base::samples::LaserScan as it doesnot support floating range values */
	struct Laser 
	{
		base::Time time;
		double start_angle;
		double angular_resolution;
		std::vector<double> ranges;
		double minRange;
		double maxRange;
	};


}

#endif

