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

#include "SimJoint.h"
#include "SimNode.h"

#include <mars/data_broker/DataBrokerInterface.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/utils/mathUtils.h>

#include <iostream>
#include <cstdio>

namespace mars {
  namespace sim {

    using namespace std;
    using namespace utils;
    using namespace interfaces;

    SimJoint::SimJoint(ControlCenter *c, const JointData &sJoint_)
      : control(c) {

      physical_joint = 0;
      setSJoint(sJoint_);

      setupDataPackageMapping();
      data_broker::DataPackage dbPackage;
      dbPackageMapping.writePackage(&dbPackage);
      std::string groupName, dataName;
      getDataBrokerNames(&groupName, &dataName);
      if(control->dataBroker) {
        control->dataBroker->pushData(groupName, dataName,
                                      dbPackage, NULL,
                                      data_broker::DATA_PACKAGE_READ_FLAG);
        control->dataBroker->registerTimedProducer(this, groupName, dataName,
                                                   "mars_sim/simTimer", 0);
      }
    }

    SimJoint::~SimJoint() {
      std::string groupName, dataName;
      getDataBrokerNames(&groupName, &dataName);
      if(control->dataBroker) {
        control->dataBroker->unregisterTimedProducer(this, groupName, dataName,
                                                     "mars_sim/simTimer");
      }
      if(physical_joint) delete physical_joint;
    }

    void SimJoint::setupDataPackageMapping() {
      dbPackageMapping.clear();
      dbPackageMapping.add("id", &id);

      dbPackageMapping.add("axis1/x", &axis1.x());
      dbPackageMapping.add("axis1/y", &axis1.y());
      dbPackageMapping.add("axis1/z", &axis1.z());
      dbPackageMapping.add("axis1/angle", &actualAngle1);
      dbPackageMapping.add("axis1/speed", &speed1);
      dbPackageMapping.add("axis1/torque/x", &axis1_torque.x());
      dbPackageMapping.add("axis1/torque/y", &axis1_torque.y());
      dbPackageMapping.add("axis1/torque/z", &axis1_torque.z());

      dbPackageMapping.add("axis2/x", &axis2.x());
      dbPackageMapping.add("axis2/y", &axis2.y());
      dbPackageMapping.add("axis2/z", &axis2.z());
      dbPackageMapping.add("axis2/angle", &actualAngle2);
      dbPackageMapping.add("axis2/speed", &speed2);
      dbPackageMapping.add("axis2/torque/x", &axis2_torque.x());
      dbPackageMapping.add("axis2/torque/y", &axis2_torque.y());
      dbPackageMapping.add("axis2/torque/z", &axis2_torque.z());

      dbPackageMapping.add("force1/x", &f1.x());
      dbPackageMapping.add("force1/y", &f1.y());
      dbPackageMapping.add("force1/z", &f1.z());
      dbPackageMapping.add("torque1/x", &t1.x());
      dbPackageMapping.add("torque1/y", &t1.y());
      dbPackageMapping.add("torque1/z", &t1.z());

      dbPackageMapping.add("force2/x", &f2.x());
      dbPackageMapping.add("force2/y", &f2.y());
      dbPackageMapping.add("force2/z", &f2.z());
      dbPackageMapping.add("torque2/x", &t2.x());
      dbPackageMapping.add("torque2/y", &t2.y());
      dbPackageMapping.add("torque2/z", &t2.z());

      dbPackageMapping.add("anchor/x", &anchor.x());
      dbPackageMapping.add("anchor/y", &anchor.y());
      dbPackageMapping.add("anchor/z", &anchor.z());
      dbPackageMapping.add("jointLoad/x", &joint_load.x());
      dbPackageMapping.add("jointLoad/y", &joint_load.y());
      dbPackageMapping.add("jointLoad/z", &joint_load.z());
      dbPackageMapping.add("motorTorque", &motor_torque);
    }

    void SimJoint::produceData(const data_broker::DataInfo &info,
                               data_broker::DataPackage *dbPackage,
                               int callbackParam) {
      dbPackageMapping.writePackage(dbPackage);
    }

    void SimJoint::setAttachedNodes(SimNode* node1, SimNode* node2){
      snode1 = node1;
      snode2 = node2;
      Quaternion inverseQNode1 = node1->getRotation().inverse();
      node1ToAnchor = sJoint.anchor - node1->getPosition();
      node1ToAnchor = inverseQNode1*node1ToAnchor;
      axis1InNode1 = inverseQNode1*sJoint.axis1;
    }

    SimNode* SimJoint::getAttachedNode1() const {
      return snode1;
    }

    SimNode* SimJoint::getAttachedNode2() const {
      return snode2;
    }

    void SimJoint::setJointType(JointType type){
      sJoint.type = type;
    }

    JointType SimJoint::getJointType() const {
      return sJoint.type;
    }

    void SimJoint::setAnchor(const Vector &pos){
      // here we set the axis angles to zero in the simulation
      // so we have to change the angle offsets
      sJoint.angle1_offset = actualAngle1;
      sJoint.angle2_offset = actualAngle2;
      sJoint.anchor = pos;
      if(physical_joint) physical_joint->setAnchor(pos);
    }

    const Vector SimJoint::getAnchor() const {
      return anchor;
    }

    void SimJoint::setAxis1(const Vector &axis) {
      sJoint.axis1 = axis1 = axis;
      axis1InNode1 = snode1->getRotation().inverse()*sJoint.axis1;
      // i don't know if the joint angle will be set to zero here to
      if(physical_joint) physical_joint->setAxis(axis);
    }

    void SimJoint::rotateAxis1(const Quaternion &rotate) {
      //sJoint.axis1 = QVRotate(rotate, sJoint.axis1);
      sJoint.axis1 = (rotate * sJoint.axis1);
      cout << " rotate" << endl;
      if (physical_joint) physical_joint->setAxis(sJoint.axis1);
    }


    const Vector SimJoint::getAxis1() const {
      return axis1;
    }

    void SimJoint::setAxis2(const Vector &axis) {
      sJoint.axis2 = axis;
      if (physical_joint) physical_joint->setAxis2(axis);
    }

    const Vector SimJoint::getAxis2() const {
      return axis2;
    }

    void SimJoint::setId(unsigned long i) {
      sJoint.index = i;
    }

    unsigned long SimJoint::getIndex() const {
      return sJoint.index;
    }

    sReal SimJoint::getActualAngle1() const {
      return actualAngle1;
    }

    sReal SimJoint::getActualAngle2() const {
      return actualAngle2;
    }

    void SimJoint::update(sReal calc_ms){
      CPP_UNUSED(calc_ms);
      if (physical_joint) {
        // update the position and rotation of the node
        actualAngle1 = (sJoint.angle1_offset + invert*physical_joint->getPosition());
        actualAngle2 = (sJoint.angle2_offset + invert*physical_joint->getPosition2());

        physical_joint->getAnchor(&anchor);
        physical_joint->getAxis(&axis1);
        physical_joint->getAxis2(&axis2);
        physical_joint->getForce1(&f1);
        physical_joint->getForce2(&f2);
        physical_joint->getTorque1(&t1);
        physical_joint->getTorque2(&t2);
        physical_joint->update();
        physical_joint->getAxisTorque(&axis1_torque);
        physical_joint->getAxis2Torque(&axis2_torque);
        physical_joint->getJointLoad(&joint_load);
        axis1_torque *= invert;
        axis2_torque *= invert;
        joint_load *= invert;
        speed1 = invert*physical_joint->getVelocity();
        speed2 = invert*physical_joint->getVelocity2();
        motor_torque = invert*physical_joint->getMotorTorque();
      }
    }

    void SimJoint::setSJoint(const JointData &sJoint) {
      this->sJoint = sJoint;
      id = sJoint.index;
      anchor = sJoint.anchor;
      axis1 = sJoint.axis1;
      axis2 = sJoint.axis2;
      actualAngle1 = sJoint.angle1_offset;
      actualAngle2 = sJoint.angle2_offset;
      f1.x() = f1.y() = f1.z() = 0;
      f2.x() = f2.y() = f2.z() = 0;
      t1.x() = t1.y() = t1.z() = 0;
      t2.x() = t2.y() = t2.z() = 0;
      axis1_torque.x() = axis1_torque.y() = axis1_torque.z() = 0;
      axis2_torque.x() = axis2_torque.y() = axis2_torque.z() = 0;
      joint_load.x() = joint_load.y() = joint_load.z() = 0;
      speed1 = speed2 = 0;
      motor_torque = 0;
      lowStop1 = sJoint.lowStopAxis1;
      highStop1 = sJoint.highStopAxis1;
      lowStop2 = sJoint.lowStopAxis2;
      highStop2 = sJoint.highStopAxis2;
      if(sJoint.invertAxis) {
        invert = -1;
      }
      else {
        invert = 1;
      }
    }

    const JointData SimJoint::getSJoint(void) const {
      JointData tmp = sJoint;

      tmp.axis1 = axis1;
      tmp.axis2 = axis2;
      tmp.anchor = anchor;
      tmp.angle1_offset = actualAngle1;
      tmp.angle2_offset = actualAngle2;
      tmp.lowStopAxis1 = lowStop1;
      tmp.highStopAxis1 = highStop1;
      tmp.lowStopAxis2 = lowStop2;
      tmp.highStopAxis2 = highStop2;
      return tmp;
    }

    void SimJoint::setInterface(JointInterface* physical_joint) {
      this->physical_joint = physical_joint;
    }

    void SimJoint::setForceLimit(sReal force) {
      physical_joint->setForceLimit(force);
    }

    void SimJoint::setForceLimit2(sReal force) {
      physical_joint->setForceLimit2(force);
    }

    void SimJoint::setVelocity(sReal velocity) {
      physical_joint->setVelocity(velocity*invert);
    }

    void SimJoint::setVelocity2(sReal velocity) {
      physical_joint->setVelocity2(velocity*invert);
    }

    sReal SimJoint::getVelocity(void) const {
      return speed1;
    }

    sReal SimJoint::getVelocity2(void) const {
      return speed2;
    }

    void SimJoint::setTorque(sReal torque) {
      physical_joint->setTorque(torque*invert);
    }

    void SimJoint::setTorque2(sReal torque) {
      physical_joint->setTorque2(torque*invert);
    }

    void SimJoint::setJointAsMotor(int axis) {
      physical_joint->setJointAsMotor(axis);
    }

    void SimJoint::unsetJointAsMotor(int axis) {
      physical_joint->unsetJointAsMotor(axis);
    }

    void SimJoint::getCoreExchange(core_objects_exchange *obj) const {
      obj->index = sJoint.index;
      obj->name = sJoint.name;
      obj->groupID = 0;
      obj->pos = anchor;
      obj->rot = angleAxisToQuaternion(actualAngle1*invert, axis1);
    }

    const Vector SimJoint::getForce1() const {
      return f1;
    }

    const Vector SimJoint::getForce2() const {
      return f2;
    }

    const Vector SimJoint::getTorque1() const {
      return t1;
    }

    const Vector SimJoint::getTorque2() const {
      return t2;
    }

    void SimJoint::reattacheJoint(void) {
      Vector pos;

      sJoint.angle1_offset = actualAngle1;
      sJoint.angle2_offset = actualAngle2;
      if(sJoint.anchorPos == ANCHOR_NODE1) {
        pos = snode1->getPosition();
        setAnchor(pos);
      }
      else if(sJoint.anchorPos == ANCHOR_NODE2) {
        pos = snode2->getPosition();
        setAnchor(pos);
      }
      else if(sJoint.anchorPos == ANCHOR_CENTER) {
        pos = (snode1->getPosition() + snode2->getPosition()) / 2.;
        setAnchor(pos);
      }
      else if(physical_joint) {
        physical_joint->reattacheJoint();
        physical_joint->getAnchor(&sJoint.anchor);
      }

      sJoint.axis1 = snode1->getRotation()*axis1InNode1;
      if(physical_joint) physical_joint->setAxis(sJoint.axis1);

      node1ToAnchor = sJoint.anchor - snode1->getPosition();
      node1ToAnchor = snode1->getRotation().inverse()*node1ToAnchor;
    }

    const Vector SimJoint::getAxis1Torque(void) const {
      return axis1_torque;
    }

    const Vector SimJoint::getAxis2Torque(void) const {
      return axis2_torque;
    }

    const Vector SimJoint::getJointLoad(void) const {
      return joint_load;
    }

    unsigned long SimJoint::getNodeIndex1() const {
      return sJoint.nodeIndex1;
    }

    unsigned long SimJoint::getNodeIndex2() const {
      return sJoint.nodeIndex2;
    }

    void SimJoint::changeStepSize(void) {
      physical_joint->changeStepSize(sJoint);
    }

    void SimJoint::setSDParams(JointData *sJoint) {
      this->sJoint.spring_constant = sJoint->spring_constant;
      this->sJoint.damping_constant = sJoint->damping_constant;
      this->sJoint.lowStopAxis1 = sJoint->lowStopAxis1;
      this->sJoint.highStopAxis1 = sJoint->highStopAxis1;
      this->sJoint.damping_const_constraint_axis1 = sJoint->damping_const_constraint_axis1;
      this->sJoint.spring_const_constraint_axis1 = sJoint->spring_const_constraint_axis1;
      this->sJoint.lowStopAxis2 = sJoint->lowStopAxis2;
      this->sJoint.highStopAxis2 = sJoint->highStopAxis2;
      this->sJoint.damping_const_constraint_axis2 = sJoint->damping_const_constraint_axis2;
      this->sJoint.spring_const_constraint_axis2 = sJoint->spring_const_constraint_axis2;
      changeStepSize();
    }

    sReal SimJoint::getMotorTorque(void) const {
      return motor_torque;
    }

    void SimJoint::getDataBrokerNames(std::string *groupName,
                                      std::string *dataName) const {
      char format[] = "Joints/%05lu_%s";
      int size = snprintf(0, 0, format, sJoint.index, sJoint.name.c_str());
      char buffer[size];
      sprintf(buffer, format, sJoint.index, sJoint.name.c_str());
      *groupName = "mars_sim";
      *dataName = buffer;
    }

    void SimJoint::setOfflineValue(sReal value) {
      if(snode2) {
        sReal tmp = value;
        value -= actualAngle1;
        actualAngle1 = tmp;

        Vector pivot = snode1->getPosition()+snode1->getRotation()*node1ToAnchor;
        Vector axis = snode1->getRotation()*axis1InNode1;

        if(sJoint.type == JOINT_TYPE_HINGE) {
          Quaternion q = angleAxisToQuaternion(-value*invert, axis);
          control->nodes->rotateNode(snode2->getID(), pivot, q, sJoint.index);
        }
        else if(sJoint.type == JOINT_TYPE_SLIDER) {
          Vector pos2 = snode2->getPosition();
          axis *= invert*value / axis.norm();
          pos2 += axis;
          control->nodes->positionNode(snode2->getID(), pos2, sJoint.index);
        }
      }
    }

    sReal SimJoint::getLowStop() const
    { return lowStop1; }
      
    sReal SimJoint::getHighStop() const
    { return highStop1; }

    sReal SimJoint::getLowStop2() const
    { return lowStop2; }

    sReal SimJoint::getHighStop2() const
    { return highStop2; }

    void SimJoint::setLowStop(sReal lowStop) {
      this->lowStop1 = lowStop;
      physical_joint->setLowStop(-lowStop*invert);
    }
    void SimJoint::setHighStop(sReal highStop) {
      this->highStop1 = highStop;
      physical_joint->setHighStop(-highStop*invert);
    }

    void SimJoint::setLowStop2(sReal lowStop2) {
      this->lowStop2 = lowStop2;
      physical_joint->setLowStop2(lowStop2*invert);
    }

    void SimJoint::setHighStop2(sReal highStop2) {
      this->highStop2 = highStop2;
      physical_joint->setHighStop2(highStop2*invert);
    }


  } // end of namespace sim
} // end of namespace mars
