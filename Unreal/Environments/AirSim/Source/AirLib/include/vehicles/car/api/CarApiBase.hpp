// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "CarCommon.hpp"

typedef msr::airlib::CarState CarState;

namespace msr { namespace airlib {

class CarApiBase : public VehicleApiBase  {

  
protected:
  /************************* State APIs *********************************/
  virtual Kinematics::State getKinematicsEstimated() const = 0;
  virtual GeoPoint getGpsLocation() const = 0;
  //  virtual const MultirotorApiParams& getMultirotorApiParams() const = 0;
  
public:
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        void set_throttle(float throttle_val, bool forward)
        {
            if (forward) {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            else {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = - std::abs(throttle_val);
            }
        }
    };


public:

    // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
	//CarApiBase() {}

    CarApiBase(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        initialize(vehicle_setting, sensor_factory, state, environment);
    }

    virtual void update() override
    {
        VehicleApiBase::update();

        getSensors().update();
    }

    void reportState(StateReporter& reporter) override
    {
        getSensors().reportState(reporter);
    }

    // sensor helpers
    virtual const SensorCollection& getSensors() const override
    {
        return sensors_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

      /************* other short hands ************/
    virtual Vector3r getPosition() const
    {
        return getKinematicsEstimated().pose.position;
    }
    virtual Vector3r getVelocity() const
    {
        return getKinematicsEstimated().twist.linear;
    }
    virtual Quaternionr getOrientation() const
    {
      return getKinematicsEstimated().pose.orientation;
    }

    void initialize(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();
        
        addSensorsFromSettings(vehicle_setting);

        getSensors().initialize(&state, &environment);
    }

    void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
    {
        // use sensors from vehicle settings; if empty list, use default sensors.
        // note that the vehicle settings completely override the default sensor "list";
        // there is no piecemeal add/remove/update per sensor.
        const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
            = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;

        sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
    }

    virtual void setCarControls(const CarControls& controls) = 0;
  //  virtual CarState getCarState() const = 0;
      /************************* high level status APIs *********************************/
  CarState getCarState() const
    {
        CarState state;
        state.kinematics_estimated = getKinematicsEstimated();
        //TODO: add GPS health, accuracy in API
        state.gps_location = getGpsLocation();
        state.timestamp = clock()->nowNanos();
	//        state.landed_state = getLandedState();
	// state.rc_data = getRCData();
        //state.ready = isReady(state.ready_message);
        //state.can_arm = canArm();
        return state;
    }

    virtual const CarApiBase::CarControls& getCarControls() const = 0;

    virtual ~CarApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors

protected:
    virtual void resetImplementation() override
    {
        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }
};


}} //namespace
#endif
