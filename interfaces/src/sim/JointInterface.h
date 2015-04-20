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

#ifndef JOINT_INTERFACE_H
#define JOINT_INTERFACE_H

#ifdef _PRINT_HEADER_
  #warning "JointInterface.h"
#endif

#include "NodeInterface.h"
#include "../JointData.h" // her is declared the jointStruct

namespace mars {
  namespace interfaces {

    class JointInterface {
    public:
      virtual ~JointInterface() {}

      unsigned char axis_index=0;

      // function members
	  void rotateAxis(const utils::Quaternion &rotatem, unsigned char axis_index=0);
	  void update(interfaces::sReal calc_ms);
	  void reattachJoint(void);
	  void attachMotor(unsigned char axis_index);
	  void detachMotor(unsigned char axis_index);
	  void updateStepSize(void);

		// getters
		const utils::Vector getAnchor(void) const;
		SimNode* getAttachedNode(unsigned char axis_index=0) const;
		const utils::Vector getAxis(unsigned char axis_index=0) const;
		void getCoreExchange(interfaces::core_objects_exchange *obj) const;
		interfaces::sReal getCurrentPosition(unsigned char axis_index=0) const;
		const utils::Vector getForceVector(unsigned char axis_index=0) const;
		unsigned long getIndex(void) const;
		interfaces::JointType getJointType(void) const;
		const utils::Vector getJointLoad(void) const;
      	interfaces::sReal getMotorTorque(void) const;
      	interfaces::NodeId getNodeId(unsigned char node_index=0) const;
      	const interfaces::JointData getSJoint(void) const;
      	interfaces::sReal getSpeed(unsigned char axis_index=0) const;
      	interface::sReal getTorque(interfaces::sReal torque, unsigned char axis_index=0) const;
      	const utils::Vector getTorqueVector(unsigned char axis_index=0) const;

		// setters
		void setAnchor(const utils::Vector &pos);
		void setAttachedNodes(SimNode *node, SimNode *node2);
		void setAxis(unsigned char axis_index=0);
		void setEffortLimit(interfaces::sReal force, unsigned char axis_index=0);
		void setId(unsigned long i);
		void setJointType(interfaces::JointType type);
		void setOfflineValue(interfaces::sReal value);
		void setPhysicalJoint(interfaces::JointInterface *physical_joint);
		void setSDParams(interfaces::JointData *sJoint);
		void setSJoint(const interfaces::JointData &sJoint);
		void setSpeed(interfaces::sReal speed, unsigned char axis_index=0);
		void setTorque(interfaces::sReal torque, unsigned char axis_index=0);

      // create the joint using the joint structure given as argument
      virtual bool createJoint(JointData *joint,
                               const NodeInterface *node1, 
                               const NodeInterface *i_node2) = 0; // physic interfaces for the node
      virtual void setWorldObject(PhysicsInterface *world) = 0;
      virtual void getAxisTorque(utils::Vector *t, unsigned char axis_index=0) const = 0;
      virtual void getAxis2Torque(utils::Vector *t) const = 0;
    };

  } // end of namespace interfaces
} // end of namespace mars

#endif // JOINT_INTERFACE_H
