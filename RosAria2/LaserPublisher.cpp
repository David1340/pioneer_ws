#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "LaserPublisher.h"
#include "ArTimeToROSTime.h"

#include <math.h>

#include <boost/algorithm/string.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/transform_datatypes.h>

LaserPublisher::LaserPublisher(ArLaser *_l, rclcpp::Node* _n, bool _broadcast_tf, const std::string& _tf_frame, const std::string& _parent_tf_frame, const std::string& _global_tf_frame) :
  laserReadingsCB(this, &LaserPublisher::readingsCB),
  node(_n),
  laser(_l),
  tfname(_tf_frame),
  parenttfname(_parent_tf_frame),
  globaltfname(_global_tf_frame),
  broadcast_tf(_broadcast_tf)
{
  assert(_l);
  laser->lockDevice();
  laser->addReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  std::string laserscan_name(laser->getName());
  boost::erase_all(laserscan_name,".");
  laserscan_name += "_laserscan";
  std::string pointcloud_name(laser->getName());
  boost::erase_all(pointcloud_name,".");
  pointcloud_name += "_pointcloud";
  laserscan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_name, 20);
  pointcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud>(pointcloud_name, 50);

  transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  tf2::Quaternion q;
  if(laser->hasSensorPosition())
  {
    lasertf.setOrigin(tf2::Vector3(laser->getSensorPositionX()/1000.0, laser->getSensorPositionY()/1000.0, laser->getSensorPositionZ()/1000.0));
    q.setRPY(0, 0, ArMath::degToRad(laser->getSensorPositionTh()));
  }
  else
  {
    lasertf.setOrigin(tf2::Vector3(0, 0, 0));
    q.setRPY(0, 0, 0);
  }
  lasertf.setRotation(q);


  laserscan.header.frame_id = "laser_frame";
  laserscan.angle_min = ArMath::degToRad(laser->getStartDegrees());
  laserscan.angle_max = ArMath::degToRad(laser->getEndDegrees());
  //laserscan.time_increment = ?
  laserscan.range_min = 0; //laser->getMinRange() / 1000.0;
  laserscan.range_max = laser->getMaxRange() / 1000.0;
  pointcloud.header.frame_id = globaltfname;

  // Get angle_increment of the laser
  laserscan.angle_increment = 0;
  if(laser->canSetIncrement()) {
    laserscan.angle_increment = laser->getIncrement();
  }
  else if(laser->getIncrementChoice() != NULL) {
    laserscan.angle_increment = laser->getIncrementChoiceDouble();
  }
  assert(laserscan.angle_increment > 0);
  laserscan.angle_increment *= M_PI/180.0;

  //readingsCallbackTime = new ArTime;
}

LaserPublisher::~LaserPublisher()
{
  laser->lockDevice();
  laser->remReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  //delete readingsCallbackTime;
}

void LaserPublisher::readingsCB()
{
  //printf("readingsCB(): %lu ms since last readingsCB() call.\n", readingsCallbackTime->mSecSince());
  assert(laser);
  laser->lockDevice();
  publishLaserScan();
  publishPointCloud();
  laser->unlockDevice();
  if(broadcast_tf){
    auto time_stamp = convertArTimeToROS(laser->getLastReadingTime(), node);
    tf2::TimePoint time_stamped = tf2::TimePoint(std::chrono::nanoseconds(int(time_stamp)));
    tf2::Stamped<tf2::Transform> stamped(lasertf, time_stamped, parenttfname);
    geometry_msgs::msg::TransformStamped tf_stamped = tf2::toMsg(stamped);
    tf_stamped.child_frame_id = tfname;
    transform_broadcaster->sendTransform(tf_stamped);
    }
  //readingsCallbackTime->setToNow();
}

void LaserPublisher::publishLaserScan()
{
  auto n_secs = convertArTimeToROS(laser->getLastReadingTime(), node);
  laserscan.header.stamp = rclcpp::Time(n_secs);
  const std::list<ArSensorReading*> *readings = laser->getRawReadings();
  assert(readings);
  //printf("laserscan: %lu readings\n", readings->size());
  laserscan.ranges.resize(readings->size());
  size_t n = 0;
  if (laser->getFlipped()) {
    // Reverse the data
    for(std::list<ArSensorReading*>::const_reverse_iterator r = readings->rbegin(); r != readings->rend(); ++r)
    {
      assert(*r);

      if ((*r)->getIgnoreThisReading()) {
	laserscan.ranges[n] = -1;
      }
      else {
	laserscan.ranges[n] = (*r)->getRange() / 1000.0;
      }

      ++n;
    }
  }
  else {
    for(std::list<ArSensorReading*>::const_iterator r = readings->begin(); r != readings->end(); ++r)
    {
      assert(*r);

      if ((*r)->getIgnoreThisReading()) {
	laserscan.ranges[n] = -1;
      }
      else {
	laserscan.ranges[n] = (*r)->getRange() / 1000.0;
      }

      ++n;
    }
  }

  laserscan_pub->publish(laserscan);
}

void LaserPublisher::publishPointCloud()
{
  assert(laser);

  auto n_secs = convertArTimeToROS(laser->getLastReadingTime(), node);
  pointcloud.header.stamp = rclcpp::Time(n_secs);

  const ArRangeBuffer& range_buffer = laser->getCurrentRangeBuffer();
  const std::list<ArPoseWithTime>& poses = range_buffer.getBuffer();
  if (poses.empty()) {
    RCLCPP_WARN(node->get_logger(), "Laser point cloud buffer is empty");
    return;
  }

  pointcloud.points.resize(poses.size());
  size_t n = 0;

  for (const auto& pose : poses) {
    pointcloud.points[n].x = pose.getX() / 1000.0;
    pointcloud.points[n].y = pose.getY() / 1000.0;
    pointcloud.points[n].z = laser->hasSensorPosition()
      ? laser->getSensorPositionZ() / 1000.0
      : 0.0;
    ++n;
  }


  pointcloud_pub->publish(pointcloud);
}


