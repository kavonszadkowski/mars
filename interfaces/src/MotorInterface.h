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

        protected:
            MotorControllerInterface controller;
            std::string type; // type of the motor
            std::string name; // name of the motor
            SimJoint* parentjoint, childjoint;
            bool active; // whether the motor is active or not
            ControlCenter *control;
            ConfigMap config;
            sReal motorEffort, maxEffort, currentPosition1, currentPosition2;
            sReal current, effort, i_current, last_current, last_velocity, joint_velocity;
            sReal currentSpeed, currentPosition, desiredSpeed, desiredPosition;
            int axis, type;
            sReal maxSpeed; //maximum speed the motor can reach

            // only used by MotorManager
            void attachJoint(SimJoint *joint);
            void attachPlayJoint(SimJoint *joint);

            //backwards compatibility
            interfaces::sReal getActualAngle() const; // --> getActualPosition, should be getCurrentPosition



            // for dataBroker communication
            data_broker::DataPackage dbPackage;
            unsigned long dbPushId;
            long dbIdIndex, dbValueIndex, dbPositionIndex, dbCurrentIndex, dbTorqueIndex;
        };

    } // end of namespace interfaces
} // end of namespace mars

#endif // MOTORINTERFACE_H
