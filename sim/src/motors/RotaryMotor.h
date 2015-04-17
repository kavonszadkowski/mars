//
// Created by kavonszadkowski on 10.04.15.
//

#ifndef _MARS_ROTARYMOTOR_H_
#define _MARS_ROTARYMOTOR_H_

#ifdef _PRINT_HEADER_
#warning "SimMotor.h"
#endif

#include "SimJoint.h"

#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/MotorInterface.h>
#include <mars/interfaces/MotorData.h>

#include <iostream>

namespace mars {

    namespace interfaces {
        class ControlCenter;
    }

    namespace sim {

        /**
         * Each SimMotor object publishes its state on the dataBroker.
         * The name under which the data is published can be obtained from the
         * motorId via MotorManager::getDataBrokerNames.
         * The data_broker::DataPackage will contain the following items:
         *  - "id" (long)
         *  - "value" (double)
         *  - "position" (double)
         *  - "current" (double)
         *  - "torque" (double)
         */
        class RotaryMotor : public SimMotor {

        public:
            SimMotor(interfaces::ControlCenter *control,
            ConfigMap config);
            ~SimMotor(void);

            // motor methods
            void deactivate(void);

            // methods inherited from controller interface

            /**
             * Get current position of the motor in radians.
             *
             * @return actual joint angle
             */
            interfaces::sReal getCurrentPosition() const;


            int getAxis() const;

            /**
             * returns desired angle of the motor
             *
             * @return desired motor angle
             */
            interfaces::sReal getDesiredMotorAngle() const;

            /**
             * return motor joint
             *
             * @return motor joint
             */
            SimJoint* getJoint() const;
            SimJoint* getPlayJoint() const;

            /**
             * return the name of the motor
             *
             * @return name of the motor
             */
            const std::string getName() const;


            /**
             * get the gain this motor can achieve
             *
             * @return gain
             */
            interfaces::sReal getMotorGain() const;

            /**
             * get the maximum force this motor can produce
             *
             * @return maximal force this motor can produce
             */
            interfaces::sReal getMaxEffort() const;
            interfaces::sReal getMotorMaxForce() const;

            /**
             * returns if joint is a servomotor
             *
             * @return true is joint is a servo motor
             */
            bool isServo() const;

            /**
             * set actual joint angle
             *
             * @param angle actual joint angle
             */
            void setActualAngle(interfaces::sReal angle);

            /**
             * sets the axis of the motor
             *
             * @param tempAxis axis of the motor
             */
            void setAxis(int tempAxis);

            /**
             * sets an angle the joint shall achieve
             *
             * @param angle angle to achieve
             */
            void setDesiredMotorAngle(interfaces::sReal angle);

            /**
             * set the desired angular velocity
             * @param vel velocity in rad/s
             */
            void setDesiredMotorVelocity(interfaces::sReal vel);

            /**
             * set the gain this motor can achieve
             *
             * @param gain
             */
            void setMotorGain(interfaces::sReal gain);

            /**
             * set the maximum force this motor can produce
             *
             * @param force maximal force this motor can produce
             */
            void setMaxEffort(interfaces::sReal force);
            void setMotorMaxForce(interfaces::sReal force); //deprecated

            /**
             * sets a name for the motor
             *
             * @param name of the motor
             */
            void setName(const std::string &newname);


            /**
             * set if joint is a servo motor
             *
             * @param true if joint shall be a servo motor
             */
            void setType(int type);

            /**
             * sets the p part of the pid control
             *
             * @param p value
             */
            void setP(interfaces::sReal p);

            /**
             * sets the i part of the pid control
             *
             * @param i value
             */
            void setI(interfaces::sReal i);

            /**
             * sets the d part of the pid control
             *
             * @param d value
             */
            void setD(interfaces::sReal d);

            /**
             * gets the p part of the pid control
             *
             * return p value
             */
            interfaces::sReal getP() const;

            /**
             * gets the i part of the pid control
             *
             * return i value
             */
            interfaces::sReal getI() const;

            /**
             * gets the d part of the pid control
             *
             * return d value
             */
            interfaces::sReal getD() const;

            /**
             * refreshAngle updates the motor angles
             */
            void refreshAngle();

            /**
             * sets the maximum velocity allowed for the motor
             *
             * @param value maximum velocity of the motor
             */
            void setMaxSpeed(interfaces::sReal value);

            /**
             * gets the maximum velocity allowed for this motor
             *
             * @return maximum velocity allowed for this motor
             */
            interfaces::sReal getMaxSpeed() const;

            /**
             * set the actual velocity of the motor
             *
             * @param y actual velocity of the motor
             */
            void setSpeed(interfaces::sReal v);

            /**
             * get the actual velocity of the motor
             *
             * @return the actual velocity of the motor
             */
            interfaces::sReal getSpeed() const;
            /**
             * set the PID value
             */
            void setPID(interfaces::sReal mP, interfaces::sReal mI, interfaces::sReal mD);

            //  void setSMotor(const MotorData &sMotor);
            const interfaces::MotorData getSMotor(void) const;
            void update(interfaces::sReal time_ms);
            unsigned long getIndex(void) const;
            unsigned long getJointIndex(void) const;
            void getCoreExchange(interfaces::core_objects_exchange* obj) const;
            void setValue(interfaces::sReal value);
            void setValueDesiredVelocity(interfaces::sReal value)
            {
                desired_velocity = value;
            }
            interfaces::sReal getValue(void) const;
            interfaces::sReal getActualPosition(void) const;
            interfaces::sReal getCurrent(void) const;
            interfaces::sReal getTorque(void) const;


            void setSMotor(const interfaces::MotorData &sMotor);
            void getDataBrokerNames(std::string *groupName, std::string *dataName) const;

            virtual void produceData(const data_broker::DataInfo &info,
                    data_broker::DataPackage *package,
                    int callbackParam);
            virtual void receiveData(const data_broker::DataInfo &info,
                    const data_broker::DataPackage &package,
                    int callbackParam);

        private:
            interfaces::ControlCenter *control;
            interfaces::MotorData sMotor;
            interfaces::sReal motorForce, maxEffort, actualAngle1, actualAngle2;
            interfaces::sReal p, i, d;
            int axis, type;
            interfaces::sReal time;
            interfaces::sReal actual_velocity, actual_position, desired_position, desired_velocity;
            interfaces::sReal last_error;
            interfaces::sReal integ_error;
            interfaces::sReal pwm;
            interfaces::sReal current, torque, i_current, last_current, last_velocity, joint_velocity;
            SimJoint* myJoint, *myPlayJoint;
            std::string name;
            bool activated;

            // for current fit function
            // currently only done for SpaceClimber motor
            interfaces::sReal kXY, kX, kY, k;

            // for dataBroker communication
            data_broker::DataPackage dbPackage;
            unsigned long dbPushId;
            long dbIdIndex, dbValueIndex, dbPositionIndex, dbCurrentIndex, dbTorqueIndex;

        };

    } // end of namespace sim
} // end of namespace mars

#endif //_MARS_ROTARYMOTOR_H_
