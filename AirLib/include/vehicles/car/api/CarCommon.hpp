#ifndef air_DroneCarCommon_hpp
#define air_DroneCarCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr {  namespace airlib {

// enum class DrivetrainType {
//     MaxDegreeOfFreedom = 0,
//     ForwardOnly
// };

struct CarCommonState {
  float speed;
  int gear;
  float rpm;
  float maxrpm;
  bool handbrake;
  Kinematics::State kinematics_estimated;
  CollisionInfo collision;
  GeoPoint gps_location;
  uint64_t timestamp;

  //        MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, timestamp);

  CarCommonState()
  {}


  CarCommonState(float speed_val,
	   int gear_val,
	   float rpm_val,
	   float maxrpm_val,
	   bool handbrake_val, 
	   const Kinematics::State& kinematics_estimated_val,
	   uint64_t timestamp_val)
    : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), 
      kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
  {
  }
  
  CarCommonState(float speed_val,
  	   int gear_val,
  	   float rpm_val,
  	   float maxrpm_val,
  	   bool handbrake_val, 
  	   const Kinematics::State& kinematics_estimated_val,
	   const CollisionInfo& collision_val,
	   const GeoPoint& gps_location_val,
  	   uint64_t timestamp_val)
    : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), 
      kinematics_estimated(kinematics_estimated_val), collision(collision_val), gps_location(gps_location_val), timestamp(timestamp_val)
  {
  }

  //shortcuts
  const Vector3r& getPosition() const
  {
    return kinematics_estimated.pose.position;
  }
  const Quaternionr& getOrientation() const
  {
    return kinematics_estimated.pose.orientation;
  }

};


    // CarCommonState to() const
    // {
    //     return CarCommonState(
    //         speed, gear, rpm, maxrpm, handbrake, kinematics_estimated.to(), timestamp);
    // }


    //Check to make sure that this is needed for the car
    //properties of vehicle
// struct CarApiParams {
//     CarApiParams() {};
//     //what is the breaking distance for given velocity?
//     //Below is just proportionality constant to convert from velocity to breaking distance
//     float vel_to_breaking_dist = 0.5f;   //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
//     float min_breaking_dist = 1; //min breaking distance
//     float max_breaking_dist = 3; //min breaking distance
//     float breaking_vel = 1.0f;
//     float min_vel_for_breaking = 3;

//     //what is the differential positional accuracy of cur_loc?
//     //this is not same as GPS accuracy because translational errors
//     //usually cancel out. Typically this would be 0.2m or less
//     float distance_accuracy = 0.1f;

//     //what is the minimum clearance from obstacles?
//     float obs_clearance = 2;

//     //what is the +/-window we should check on obstacle map?
//     //for example 2 means check from ticks -2 to 2
//     int obs_window = 0;
// };

}} //namespace
#endif
