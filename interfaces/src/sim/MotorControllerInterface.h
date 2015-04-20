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

  		  void setP(interfaces::sReal p);
  		  void setI(interfaces::sReal i);
  		  void setD(interfaces::sReal d);
		  interfaces::sReal getP() const;
		  interfaces::sReal getI() const;
		  interfaces::sReal getD() const;
  		  void setPID(interfaces::sReal mP, interfaces::sReal mI, interfaces::sReal mD);

        protected:
            std::string type; // type of the motorcontroller

            ControlCenter *control;

            interfaces::sReal p, i, d;
            interfaces::sReal last_error;
            interfaces::sReal integ_error;
            interfaces::sReal pwm;
            interfaces::sReal time;

            // for current fit function
            // currently only done for SpaceClimber motor
            interfaces::sReal kXY, kX, kY, k;

        };

    } // end of namespace interfaces
} // end of namespace mars

#endif //_MARS_MOTORCONTROLLERINTERFACE_H_
