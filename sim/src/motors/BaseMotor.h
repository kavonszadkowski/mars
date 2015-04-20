/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
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

#ifndef BASEMOTOR_H
#define BASEMOTOR_H

#ifdef _PRINT_HEADER_
#warning "BaseMotor.h"
#endif

#include "SimJoint.h"

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
     * Each BaseMotor object publishes its state on the dataBroker.
     * The name under which the data is published can be obtained from the 
     * motorId via MotorManager::getDataBrokerNames.
     * The data_broker::DataPackage will contain the following items:
     *  - "id" (long)
     *  - "value" (double)
     *  - "position" (double)
     *  - "current" (double)
     *  - "torque" (double)
     */
    class BaseMotor : public interfaces::MotorInterface,
                     public interfaces::MotorControllerInterface{

    public:
      BaseMotor(interfaces::ControlCenter *control, const ConfigMap config);
      ~BaseMotor(void);

      // public methods
      void setValueDesiredVelocity(interfaces::sReal value)
      {
        desiredSpeed = value;
      }

    };

  } // end of namespace sim
} // end of namespace mars

#endif
