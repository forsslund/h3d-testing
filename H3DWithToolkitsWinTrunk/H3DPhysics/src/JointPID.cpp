//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file JointPID.cpp
/// \brief Source file for JointPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/JointPID.h>
#include <H3D/H3DPhysics/SingleAxisHingeJoint.h>
#include <H3D/H3DPhysics/DoubleAxisHingeJoint.h>
#include <H3D/H3DPhysics/SliderJoint.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase JointPID::database( 
  "JointPID", 
  &newInstance<JointPID>, 
  typeid( JointPID ), 
  &H3DPIDNode::database );

namespace JointPIDInternals {
  FIELDDB_ELEMENT( JointPID, linearControl, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, angularControl1, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, angularControl2, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, joint, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, errorSleepThreshold, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, useJointMotor, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, switchForcesToBody2, INPUT_OUTPUT );
  FIELDDB_ELEMENT( JointPID, applyTorqueAsForce, INPUT_OUTPUT );  
}

/// Constructor
JointPID::JointPID( 
  Inst< SFNode      > _metadata, 
	Inst< SFPIDController >  _linearControl,
	Inst< SFPIDController >  _angularControl1,
  Inst< SFPIDController >  _angularControl2,
  Inst< SFJointNode > _joint,
  Inst< SFFloat > _errorSleepThreshold,
  Inst< SFBool > _useJointMotor,
  Inst< SFBool > _switchForcesToBody2,
  Inst< SFBool > _applyTorqueAsForce ):
  H3DPIDNode ( _metadata ),
  linearControl( _linearControl ),
  angularControl1( _angularControl1 ),
  angularControl2( _angularControl2 ),
  joint ( _joint ),
  errorSleepThreshold( _errorSleepThreshold ),
  useJointMotor ( _useJointMotor ),
  rt_use_joint_motor ( false ),
  bodyId1 ( 0 ),
  bodyId2 ( 0 ),
  jointId ( 0 ),
  linearPID ( NULL ),
  angularPID1 ( NULL ),
  angularPID2 ( NULL ),
  joint_type ( JointType::Unsupported ),
  fixed ( false ),
  switchForcesToBody2 ( _switchForcesToBody2 ),
  applyTorqueAsForce( _applyTorqueAsForce ),
  rt_switch_force_to_body2 ( false ){
	
  type_name = "JointPID";
  database.initFields( this );

  errorSleepThreshold->setValue( 0 );
  useJointMotor->setValue( false );
  applyTorqueAsForce->setValue( false );
}

void JointPID::traverseSG( TraverseInfo &ti ) {
  H3DPIDNode::traverseSG(ti);

  PhysicsEngineThread *pt;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if ( pt && pt == engine_thread ) {
    if ( linearPID ) 
      linearPID->traverseSG ( ti );
    if ( angularPID1 ) 
      angularPID1->traverseSG ( ti );
    if ( angularPID2 ) 
      angularPID2->traverseSG ( ti );
  }

  rt_error_sleep_threshold = errorSleepThreshold->getValue();
  rt_use_joint_motor = useJointMotor->getValue();
  rt_switch_force_to_body2 = switchForcesToBody2->getValue();
  rt_apply_torque_as_force = applyTorqueAsForce->getValue();
}

void JointPID::updateActuation () {
  if ( engine_thread && joint_type != JointType::Unsupported ) {
    // Get current body and joint state
    RigidBodyParameters* bodyParams1= new RigidBodyParameters;



    engine_thread->getRigidBodyParameters( bodyId1 , *bodyParams1 );    

    // update error... HERE

    if( !bodyParams1->getFixed() &&
      (bodyParams1->getEnabled() ||
      (linearPID && H3DAbs( linearPID->getCurrentError() ) > rt_error_sleep_threshold) ||
        (angularPID1 && H3DAbs( angularPID1->getCurrentError() ) > rt_error_sleep_threshold) ||
        (angularPID2 && H3DAbs( angularPID2->getCurrentError() ) > rt_error_sleep_threshold)) ) {

      // Get the params for body and joint from physics engine
      RigidBodyParameters* bodyParams2 = new RigidBodyParameters;
      JointParameters* jointParams = createJointParameters();
      engine_thread->getConstraintParameters( jointId, *jointParams );
      if( bodyId2 ) {
        engine_thread->getRigidBodyParameters( bodyId2, *bodyParams2 );
      }

      // Accumulate control force and torque
      Vec3f force, torque;
      bool use_joint_motor = rt_use_joint_motor;
      bool switch_forces_to_body2 = rt_switch_force_to_body2;
      bool apply_torque_as_force = rt_apply_torque_as_force;

      // Linear PIDs
      if( linearPID ) {
        force += doPIDControl( *linearPID, *bodyParams1, *bodyParams2, *jointParams, ControlType::Linear, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2 );
      }

      // Angular PIDs
      if( angularPID1 ) {
        if( apply_torque_as_force ){
          force += doPIDControl( *angularPID1, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2, true );
        } else {
          torque += doPIDControl( *angularPID1, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2 );
        }

      }
      if( angularPID2 ) {
        if( apply_torque_as_force ){
          force += doPIDControl( *angularPID2, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis2, use_joint_motor, !switch_forces_to_body2, true );
        } else {
          torque += doPIDControl( *angularPID2, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis2, use_joint_motor, !switch_forces_to_body2 );
        }
      }

      if( !use_joint_motor ) {
        if( !fixed && switch_forces_to_body2 ) {
          // body2
          bodyParams2->setForce( bodyParams2->getForce() - force );
          bodyParams2->setTorque( bodyParams2->getTorque() - torque );
          engine_thread->setRigidBodyParameters( bodyId2, *bodyParams2 );
          delete bodyParams1;
        } else {
          // body1
          bodyParams1->setForce( bodyParams1->getForce() + force );
          bodyParams1->setTorque( bodyParams1->getTorque() + torque );
          engine_thread->setRigidBodyParameters( bodyId1, *bodyParams1 );
          delete bodyParams2;
        }
        delete jointParams;
      } else {
        jointParams->setMotorTarget( force.x + torque.x );
        engine_thread->setConstraintParameters( jointId, *jointParams );
        delete bodyParams1;
        delete bodyParams2;
      }
  
    } else {
      // reset PIDControllers
      if ( linearPID ) linearPID->resetPID();
      if ( angularPID1 ) angularPID1->resetPID();
      if ( angularPID2 ) angularPID2->resetPID();
      delete bodyParams1;
    }
  }
}

void JointPID::initialize ( PhysicsEngineThread& pt ) {
  if ( !engine_thread ) {
    H3DRigidBodyJointNode* j= joint->getValue();
    if ( j ) {
      RigidBody* b= dynamic_cast<RigidBody*>(j->body1->getValue());
      if ( b ) {
        if ( j->isInitialized() && b->isInitialized() ) {
	        jointId= j->getConstraintId();
          bodyId1= b->getBodyId();
          initialOrientation1= b->orientation->getValue();
          RigidBody* b2= dynamic_cast<RigidBody*>(j->body2->getValue());
          if ( b2 && b2->isInitialized() ) {
            bodyId2= b2->getBodyId();
            initialOrientation2= b2->orientation->getValue();
          }
          
          linearPID= linearControl->getValue();
          angularPID1= angularControl1->getValue();
          angularPID2= angularControl2->getValue();

          fixed= !j->body2->getValue();

          // Get joint type
          if ( dynamic_cast<SingleAxisHingeJoint*>(j) ) {
            joint_type= JointType::SingleAxisHinge;
          } else if ( dynamic_cast<DoubleAxisHingeJoint*>(j) ) {
            joint_type= JointType::DoubleAxisHinge;
          } else if ( dynamic_cast<SliderJoint*>(j) ) {
            joint_type= JointType::Slider;
          } else {
            joint_type= JointType::Unsupported;
            Console(4) << "Warning: Joint type " << j->getTypeName() << " is not supported by JointPID!" << endl;
          }

          engine_thread= &pt;
        }
      }
    }
  }
}

JointParameters* JointPID::createJointParameters () {
  switch ( joint_type ) {
  case JointType::SingleAxisHinge:
    return new SingleAxisHingeJointParameters;
  case JointType::DoubleAxisHinge:
    return new DoubleAxisHingeJointParameters;
  case JointType::Slider:
    return new SliderJointParameters;
  }

  return NULL;
}

H3DFloat JointPID::getValue ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType ) {
  H3DFloat value= 0.0f;

  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    value= (controlType==ControlType::Angular) ? params->getAngle() : 0;
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    switch ( axisType ) {
    case AxisType::Axis1:
      value= (controlType==ControlType::Angular) ? params->getHinge1Angle() : 0;
      break;
    case AxisType::Axis2:
      value= (controlType==ControlType::Angular) ? params->getHinge2Angle() : 0;
      break;
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    value= (controlType==ControlType::Linear) ? params->getSeparation() : 0;
    break; }
  }

  return value;
}
  
H3DFloat JointPID::getVelocity ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType ) {
  H3DFloat velocity= 0.0f;

  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    velocity= (controlType==ControlType::Angular) ? params->getAngleRate() : 0;
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    switch ( axisType ) {
    case AxisType::Axis1:
      velocity= (controlType==ControlType::Angular) ? params->getHinge1AngleRate() : 0;
      break;
    case AxisType::Axis2:
      velocity= (controlType==ControlType::Angular) ? params->getHinge2AngleRate() : 0;
      break;
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    velocity= (controlType==ControlType::Linear) ? params->getSeparationRate() : 0;
    break; }
  }

  return velocity;
}

H3DFloat JointPID::getMinTarget ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType ) {
  H3DFloat target= 0.0f;

  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    target= (controlType==ControlType::Angular) ? params->getMinAngle() : 1;
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    switch ( axisType ) {
    case AxisType::Axis1:
      target= (controlType==ControlType::Angular) ? params->getMinAngle1() : 1;
      break;
    case AxisType::Axis2:
      target= 1;
      break;
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    target= (controlType==ControlType::Linear) ? params->getMinSeparation() : 1;
    break; }
  }

  return target;
}

H3DFloat JointPID::getMaxTarget ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType ) {
  H3DFloat target= 0.0f;

  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    target= (controlType==ControlType::Angular) ? params->getMaxAngle() : -1;
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    switch ( axisType ) {
    case AxisType::Axis1:
      target= (controlType==ControlType::Angular) ? params->getMaxAngle1() : -1;
      break;
    case AxisType::Axis2:
      target= -1;
      break;
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    target= (controlType==ControlType::Linear) ? params->getMaxSeparation() : -1;
    break; }
  }

  return target;
}

Vec3f JointPID::getAnchorPoint ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType, bool applyToBody1 ) {
  Vec3f anchor_point;

  // getBody1AnchorPoint and getBody2AnchorPoint both return the anchor in world coordinates,
  // if the joint is not "violated" they should return the same value
  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    anchor_point= (controlType==ControlType::Angular) ? ( applyToBody1 ? params->getBody1AnchorPoint() : params->getBody2AnchorPoint() ) : Vec3f();
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    if( ( axisType == AxisType::Axis1 || axisType == AxisType::Axis2 ) && controlType==ControlType::Angular ){
      if( applyToBody1 ){
        anchor_point= params->getBody1AnchorPoint();
      } else {
        anchor_point= params->getBody2AnchorPoint();
      }
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    anchor_point = Vec3f();
    break; }
  }

  return anchor_point;
}

Vec3f JointPID::getAxis ( JointParameters& _joint, ControlType::e controlType, AxisType::e axisType ) {
  Vec3f axis;

  switch ( joint_type ) {
  case JointType::SingleAxisHinge: {
    SingleAxisHingeJointParameters* params= static_cast<SingleAxisHingeJointParameters*>(&_joint);
    axis= (controlType==ControlType::Angular) ? params->getAxis() : Vec3f();
    break; }
  case JointType::DoubleAxisHinge: {
    DoubleAxisHingeJointParameters* params= static_cast<DoubleAxisHingeJointParameters*>(&_joint);
    switch ( axisType ) {
    case AxisType::Axis1:
      axis= (controlType==ControlType::Angular) ? params->getAxis1() : Vec3f();
      break;
    case AxisType::Axis2:
      axis= (controlType==ControlType::Angular) ? params->getAxis2() : Vec3f();
      break;
    }
    break; }
  case JointType::Slider: {
    SliderJointParameters* params= static_cast<SliderJointParameters*>(&_joint);
    axis= (controlType==ControlType::Linear) ? params->getAxis() : Vec3f();
    break; }
  }

  return axis;
}

Vec3f JointPID::doPIDControl (
      PIDController& pid, 
      RigidBodyParameters& rigidBody1, 
      RigidBodyParameters& rigidBody2, 
      JointParameters& _joint, 
      ControlType::e controlType, 
      AxisType::e axisType,
      bool useJointMotor,
      bool applyToBody1,
      bool convertTorqueToForce ) {
  // Get current value and velocity
  H3DFloat value= getValue ( _joint, controlType, axisType );
  H3DFloat velocity= getVelocity ( _joint, controlType, axisType );
  H3DFloat minTarget= getMinTarget ( _joint, controlType, axisType );
  H3DFloat maxTarget= getMaxTarget ( _joint, controlType, axisType );

  // Do pid control
  H3DFloat output= pid.doControl ( 
    value, velocity,
    controlType==ControlType::Angular, // Check for angle wrap?
    minTarget, maxTarget );

  if( useJointMotor ) {
    return Vec3f( output, 0, 0 );
  }

  // Transform actuation force
  Vec3f correctedAxis= pid.getAxis();
  if ( correctedAxis.length() < Constants::f_epsilon ) {
    // Use joint axis instead
    correctedAxis= getAxis ( _joint, controlType, axisType );
  }
  
  if ( !fixed ) {
    if ( applyToBody1 ) {
      correctedAxis= rigidBody1.getOrientation()*(-initialOrientation1)*correctedAxis;
    } else {
      correctedAxis= rigidBody2.getOrientation()*(-initialOrientation2)*correctedAxis;
    }
  }

  if( convertTorqueToForce ){

    Vec3f ap = getAnchorPoint( _joint, controlType, axisType, applyToBody1 );
    Vec3f d;
    
    if ( applyToBody1 ) {
      d = ap - rigidBody1.getPosition();
    } else {
      d = ap - rigidBody2.getPosition();
    }
    
    if( d.length() > Constants::f_epsilon ){
      correctedAxis = d.crossProduct( correctedAxis );
      output /= d.length();
    }
  }

  return output*correctedAxis;
}