//
// Created by kavonszadkowski on 10.04.15.
//

#ifndef _MARS_MOTORCONTROLLERINTERFACE_H_
#define _MARS_MOTORCONTROLLERINTERFACE_H_

#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>

namespace mars {
    namespace interfaces {

        class MotorControllerInterface {
        public:
            MotorControllerInterface(ControlCenter *center, ConfigMap config):
                    control(center) {
            }
            virtual ~MotorControllerInterface() {}

        protected:
            std::string type; // type of the motorcontroller

            ControlCenter *control;

            // for current fit function
            // currently only done for SpaceClimber motor
            interfaces::sReal kXY, kX, kY, k;

        };

    } // end of namespace interfaces
} // end of namespace mars

#endif //_MARS_MOTORCONTROLLERINTERFACE_H_
