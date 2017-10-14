/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <unistd.h>
#include <base/Logging.hpp>
#include <exception>
#include <cstdlib>


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))



using namespace gmapping;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

::gmapping::OccupancyGrid Task::getDynamicMap()
{
    boost::mutex::scoped_lock(map_mutex_);
    if(got_map_ && map_.width && map_.height)
        return map_;
    else
       return ::gmapping::OccupancyGrid();
}




/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    map_to_odom_.position = Eigen::Vector3d(0, 0, 0);
    map_to_odom_.orientation = Eigen::Quaterniond().setIdentity();

    base_to_laser_.position = Eigen::Vector3d(0.25, 0, 0);
    base_to_laser_.orientation = Eigen::Quaterniond().setIdentity();

    laser_count_ = 0;
    transform_thread_ = NULL;
    gsp_ = new GMapping::GridSlamProcessor();


    gsp_laser_ = NULL;
    gsp_laser_angle_increment_ = 0.0;
    gsp_odom_ = NULL;

    got_first_scan_ = false;
    got_map_ = false;


    config_ = _config.get();
    /* just comment this line out to remove threading stuff */
    transform_thread_ = new boost::thread(boost::bind(&Task::publishLoop, this, config_.transform_publish_period));
    LOG_INFO_S << "Configuration complete .." ;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateMap(const Laser& scan) {
  boost::mutex::scoped_lock(map_mutex_);

  GMapping::ScanMatcher matcher;
  double* laser_angles = new double[scan.ranges.size()];
  double theta = scan.start_angle;

  for(unsigned int i=0; i<scan.ranges.size(); i++)
  {
    laser_angles[i] = theta;
    theta += gsp_laser_angle_increment_;
  }

  matcher.setLaserParameters(scan.ranges.size(), laser_angles,
                             gsp_laser_->getPose());

  delete[] laser_angles;
  matcher.setlaserMaxRange(config_.maxRange);
  matcher.setusableRange(config_.maxUrange);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];

  double entropy = computePoseEntropy();
  if(entropy> 0.0)
    _entropy.write(entropy);

  if(!got_map_) {
    map_.resolution = config_.delta;
    map_.origin.position = Eigen::Vector3d(0, 0, 0);
    map_.origin.orientation = Eigen::Quaterniond().setIdentity(); 
  } 

  GMapping::Point center;
  center.x=(config_.xmin + config_.xmax) / 2.0;
  center.y=(config_.ymin + config_.ymax) / 2.0;

  GMapping::ScanMatcherMap smap(center, config_.xmin, config_.ymin, config_.xmax, config_.ymax, 
                                config_.delta);

  LOG_INFO_S << "Trajectory tree:" ;
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    
    if(!n->reading)
    {
      LOG_INFO_S << "Reading is NULL" ;
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }


  // the map may have expanded, so resize message as well
  if(map_.width != (unsigned int) smap.getMapSizeX() || map_.height != (unsigned int) smap.getMapSizeY()) {
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    config_.xmin = wmin.x; config_.ymin = wmin.y;
    config_.xmax = wmax.x; config_.ymax = wmax.y;
    
  
    map_.width = smap.getMapSizeX();
    map_.height = smap.getMapSizeY();
    map_.origin.position = Eigen::Vector3d(config_.xmin, config_.ymin, 0);
    map_.data.resize(map_.width * map_.height);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      int index = MAP_IDX(map_.width, x, y);
      if(occ < 0) {
        // 200, 0 and 250 are grey-scale values for differerent occupancy regions
         map_.data[index] = 200; // unknown
      }
      else if(occ > config_.occ_thresh)
      { 
        map_.data[index] = 0; // obstacle
      }
      else {
         map_.data[index] = 255; // free
      }
       
    }
  }
  got_map_ = true;
  map_.stamp = base::Time::now();
  map_.frame_id = config_.map_frame;
  createImage(map_);
  _occupancy_grid.write(map_);
}

bool Task::createImage(OccupancyGrid map) {
  base::samples::frame::Frame image(map.width, map.height);
  image.time = map.stamp;
  image.pixel_size = 1;
  image.row_size = map.width;
  std::vector<uint8_t> data(map.data.begin(), map.data.end());
  image.setImage(data);
  _occupancy_grid_image.write(image);
}

bool Task::initMapper(const Laser& scan, GMapping::OrientedPoint& initialPose) {
  if (config_.maxRange == NULL) {
    config_.maxRange = scan.maxRange - 0.01;
  }

  gsp_laser_angle_increment_ = scan.angular_resolution;
  gsp_laser_beam_count_ = scan.ranges.size();
  GMapping::OrientedPoint gmap_pose(0, 0, 0);
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(gsp_laser_angle_increment_),
                                         gmap_pose,
                                         0.0,
                                         config_.maxRange);  

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  //Create GMapping odometry module
  gsp_odom_ = new GMapping::OdometrySensor(config_.odom_frame);

  gsp_->setMatchingParameters(config_.maxUrange, config_.maxRange, config_.sigma, config_.kernelSize, config_.lstep, config_.astep, config_.iterations, config_.lsigma, config_.ogain, config_.lskip);
  gsp_->setMotionModelParameters(config_.srr, config_.srt, config_.str, config_.stt);
  gsp_->setUpdateDistances(config_.linearUpdate, config_.angularUpdate, config_.resampleThreshold);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(config_.particles, config_.xmin, config_.ymin, config_.xmax, config_.ymax, config_.delta, initialPose);
  gsp_->setllsamplerange(config_.llsamplerange);
  gsp_->setllsamplestep(config_.llsamplestep);
  gsp_->setlasamplerange(config_.lasamplerange);
  gsp_->setlasamplestep(config_.lasamplestep);
  
  GMapping::sampleGaussian(1,time(NULL));
  LOG_INFO_S << "Initialization complete .. ";
  return true;

}

bool Task::addScan(const Laser& scan, GMapping::OrientedPoint& gmap_pose) {
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;
  double* ranges_double = new double[scan.ranges.size()];

  // scan data preprocessing
  for(unsigned int i=0; i < scan.ranges.size(); i++)
  {
    if(scan.ranges[i] < scan.minRange)
      ranges_double[i] = (double)scan.minRange;
    else if (scan.ranges[i] > scan.maxRange)
      ranges_double[i] = (double)scan.maxRange;
    else
      ranges_double[i] = (double)scan.ranges[i];
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.time.toSeconds());
  delete[] ranges_double;
  reading.setPose(gmap_pose);

  return gsp_->processScan(reading);
}



void Task::scanTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan)
{
  Eigen::Affine3d tf;
  if (!_odometry2body.get(ts, tf, false)) 
    return;
  laser_count_++;
  if ((laser_count_ % config_.throttle_scans) != 0)
  return;

  /* Since ROCK LaserScan supports only int ranges, using a new custom message type 'Laser' (see gmappingTypes.hpp) to emulate ROS senson_msgs::LaserScan */
  Laser scan_sample;
  scan_sample.time = scan.time;
  scan_sample.start_angle = scan.start_angle;
  scan_sample.angular_resolution = scan.angular_resolution;
  scan_sample.minRange = (double) scan.minRange / 1000.0;
  scan_sample.maxRange = (double) scan.maxRange / 1000.0;
  scan_sample.ranges.resize(scan.ranges.size());
  for (int i = 0; i < scan.ranges.size(); i++)
    scan_sample.ranges[i] = (double) scan.ranges[i] / 1000.0; 


  base::samples::RigidBodyState odom_to_base; 
  odom_to_base.setTransform(tf);
  base::samples::RigidBodyState odom_to_laser;
  odom_to_laser.setTransform(odom_to_base.getTransform().inverse() * base_to_laser_.getTransform());


  GMapping::OrientedPoint odom_pose = GMapping::OrientedPoint(odom_to_laser.position[0], odom_to_laser.position[1], odom_to_laser.getYaw() * (M_PI / 180.0));
  static base::Time last_map_update = base::Time();

  if(!got_first_scan_)
  {
    if(!initMapper(scan_sample, odom_pose))
      return;
    got_first_scan_ = true;
  }
  
  if(addScan(scan_sample, odom_pose))
  {
    LOG_INFO_S << "################## Scan processed #######################" ;

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

    LOG_INFO_S << "new best pose:" << mpose.x << " " << mpose.y << " " << mpose.theta ;
    LOG_INFO_S << "odom pose:" << odom_pose.x << " " << odom_pose.y << " " << odom_pose.theta ;
    LOG_INFO_S << "correction: " << mpose.x - odom_pose.x << " " << mpose.y - odom_pose.y << " " << mpose.theta - odom_pose.theta ;
    
    base::samples::RigidBodyState map_to_laser;
    map_to_laser.position = Eigen::Vector3d(mpose.x, mpose.y, 0.0);
    map_to_laser.orientation = Eigen::AngleAxisd(mpose.theta, Eigen::Vector3d::UnitZ());
    map_to_laser.orientation.normalize();  

    map_to_odom_mutex_.lock();
    map_to_odom_.setTransform(map_to_laser.getTransform().inverse() * odom_to_laser.getTransform().inverse()); 
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan_sample.time - last_map_update) > base::Time::fromSeconds(config_.map_update_interval))
    {
      updateMap(scan_sample);
      last_map_update = scan_sample.time;
      LOG_INFO_S << "Updated the map" ;
    }
  } 
}


void Task::updateHook()
{
    TaskBase::updateHook();
}


void Task::errorHook()
{
    TaskBase::errorHook();
     if(transform_thread_){
        transform_thread_->join();
        delete transform_thread_;
    }
}
void Task::stopHook()
{
    TaskBase::stopHook();
   
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete gsp_;
    if(gsp_laser_)
        delete gsp_laser_;
    if(gsp_odom_)
        delete gsp_odom_;

}


double Task::computePoseEntropy()
{
  double weight_total = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}



void Task::publishLoop(double transform_publish_period){
    if(transform_publish_period == 0)
        return;
    while (1) {
        publishTransform();
        sleep(transform_publish_period);
    }
}

void Task::publishTransform() {
    map_to_odom_mutex_.lock();
    base::samples::RigidBodyState tf = map_to_odom_;
    tf.time = base::Time::now() + base::Time::fromSeconds(config_.transform_publish_period);
    tf.sourceFrame = config_.map_frame;
    tf.targetFrame = config_.odom_frame;
    _map_to_odom.write(tf);
    map_to_odom_mutex_.unlock();
}
