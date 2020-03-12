// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibAdapators_hpp
#define air_CarRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"
#include "vehicles/car/api/CarCommon.hpp"

namespace msr { namespace airlib_rpclib {

class CarRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        float brake = 0;
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate);

        CarControls()
        {}

        CarControls(const msr::airlib::CarApiBase::CarControls& s)
        {
            throttle = s.throttle;
            steering = s.steering;
            brake = s.brake;
            handbrake = s.handbrake;
            is_manual_gear = s.is_manual_gear;
            manual_gear = s.manual_gear;
            gear_immediate = s.gear_immediate;
        }
        msr::airlib::CarApiBase::CarControls to() const
        {
            return msr::airlib::CarApiBase::CarControls(throttle, steering, brake, handbrake,
                is_manual_gear, manual_gear, gear_immediate);
        }
    };

    struct CarState {
      float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        KinematicsState kinematics_estimated;
      CollisionInfo collision;
        GeoPoint gps_location;
        uint64_t timestamp;

      MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, collision, gps_location, timestamp);
      
        CarState()
        {}

        CarState(const msr::airlib::CarState& s)
        {
	  speed = s.speed;
	  gear = s.gear;
	  rpm = s.rpm;
	  maxrpm = s.maxrpm;
	  handbrake = s.handbrake;
	  kinematics_estimated = s.kinematics_estimated;
	  collision = s.collision;
	  gps_location = s.gps_location;
	  timestamp = s.timestamp;
        }
        msr::airlib::CarState to() const
        {
            return msr::airlib::CarState(
					 speed, gear, rpm, maxrpm, handbrake, kinematics_estimated.to(), collision.to(), gps_location.to(), timestamp);
        }
    };
};

}} //namespace


#endif
