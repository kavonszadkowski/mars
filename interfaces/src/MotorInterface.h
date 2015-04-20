/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 *  SensorInterface.h
 *
 *  Created by Kai von Szadkowski
 *
 */

#ifndef MOTORINTERFACE_H
#define MOTORINTERFACE_H

#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <configmaps/ConfigData.h>

namespace mars {
    namespace interfaces {

        class MotorInterface: public data_broker::ProducerInterface ,
                              public data_broker::ReceiverInterface {
        public:
          MotorInterface(ControlCenter *center, ConfigMap config):
                  control(center) {
          }
          virtual ~MotorInterface() {}

          void init(const std::string& name = "", MotorType type = MOTOR_TYPE_UNDEFINED);

          // function methods

          /**
          * refreshAngle updates the motor angles
          */
          void refreshAngle();
          void update(sReal time_ms);
          void deactivate(void);
          void attachJoint(SimJoint *joint);
		  void attachPlayJoint(SimJoint *joint);


          // getters
		  int getAxis() const;
		  sReal getAxisPosition(void) const;
		  void getCoreExchange(core_objects_exchange* obj) const;
		  sReal getCurrent(void) const;
		  sReal getDesiredPosition(void) const;
		  sReal getDesiredSpeed(void) const;
		  unsigned long getIndex(void) const;
		  bool isServo() const;
		  SimJoint* getJoint() const;
		  unsigned long getJointIndex(void) const;
		  const std::string getName() const;
		  sReal getMaxEffort() const;
		  sReal getMaxSpeed() const;
		  SimJoint* getPlayJoint() const;
		  sReal getPosition() const;
		  const MotorData getSMotor(void) const;
		  sReal getSpeed() const;
		  sReal getTorque(void) const;
		  sReal getValue(void) const;


		  // setters
		  void setAxis(int tempAxis);
		  void setPosition(sReal angle);
		  void setDesiredPosition(sReal angle);
		  void setDesiredSpeed(sReal vel);
		  void setGain(sReal gain);
		  void setMaxEffort(sReal effort);
		  void setMaxSpeed(sReal value);
		  void setName(const std::string &newname);
		  void setSMotor(const MotorData &sMotor);
		  void setSpeed(sReal v);
		  void setType(int type);
		  void setValue(sReal value);

	      // methods inherited from data broker interfaces
	      void getDataBrokerNames(std::string *groupName, std::string *dataName) const;

	      virtual void produceData(const data_broker::DataInfo &info,
	              data_broker::DataPackage *package,  int callbackParam);

	      virtual void receiveData(const data_broker::DataInfo &info,
	              const data_broker::DataPackage &package, int callbackParam);


		  // these methods will be deprecated in future MARS versions

	      sReal getMotorMaxForce() const __attribute__ ((deprecated("use getMaxEffort")));
		  sReal getMaximumVelocity() const __attribute__ ((deprecated("use getMaxSpeed")));
		  void setActualAngle(sReal angle) __attribute__ ((deprecated("use setCurrentPosition")));
		  void setDesiredMotorAngle(sReal angle) __attribute__ ((deprecated("use setDesiredPosition")));
		  void setDesiredMotorVelocity(sReal vel) __attribute__ ((deprecated("use setDesiredSpeed")));
		  void setMotorGain(sReal gain) __attribute__ ((deprecated("use setGain")));
		  void setMotorMaxForce(sReal force) __attribute__ ((deprecated("use setMaxEffort")));
		  void setVelocity(sReal v) __attribute__ ((deprecated("use setSpeed")));
          sReal getVelocity() const __attribute__ ((deprecated("use getSpeed")));
          sReal getActualAngle() const __attribute__ ((deprecated("use getPosition")));
          sReal getDesiredMotorPosition() const __attribute__ ((deprecated("use getDesiredPosition")));
		  void setMaximumVelocity(sReal value) __attribute__ ((deprecated("use getMaxSpeed")));

        protected:
            MotorControllerInterface controller;
            std::string type; // type of the motor
            std::string name; // name of the motor
            SimJoint* parentjoint, childjoint;
            bool active; // whether the motor is active or not
            ControlCenter *control;
            ConfigMap config;
            sreal position, desiredPosition, position1, position2;
            sReal motorEffort, maxEffort;
            sReal current, effort, i_current, last_current, last_velocity, joint_velocity;
            sReal currentSpeed, desiredSpeed;
            int axis, type;
            sReal maxSpeed; //maximum speed the motor can reach

            // for dataBroker communication
            data_broker::DataPackage dbPackage;
            unsigned long dbPushId;
            long dbIdIndex, dbValueIndex, dbPositionIndex, dbCurrentIndex, dbTorqueIndex;
        };

    } // end of namespace interfaces
} // end of namespace mars

#endif // MOTORINTERFACE_H
