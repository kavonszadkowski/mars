/*
 * BaseMotor.h
 *
 *  Created on: Apr 20, 2015
 *      Author: kai
 */

#ifndef BASEMOTOR_H_
#define BASEMOTOR_H_

#include "MotorInterface.h"

namespace mars {

class BaseMotor: public mars::interfaces::MotorInterface {
public:
	BaseMotor();
	virtual ~BaseMotor();
};

} /* namespace mars */
#endif /* BASEMOTOR_H_ */
