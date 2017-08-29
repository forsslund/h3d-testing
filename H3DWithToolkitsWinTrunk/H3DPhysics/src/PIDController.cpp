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
/// \file PIDController.cpp
/// \brief Source file for PIDController, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/PIDController.h>

#include <fstream>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PIDController::database( 
  "PIDController", 
  &newInstance<PIDController>, 
  typeid( PIDController ), 
  &X3DNode::database);

namespace PIDControllerInternals {
	FIELDDB_ELEMENT( PIDController, pidParams, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, axis, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, target, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, targetVelocity, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, maxRateOfChange, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, maxActuation, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, maxAccumulatedError, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, enabled, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, useFeedForwardVelocity, INPUT_OUTPUT );
	FIELDDB_ELEMENT( PIDController, currentError, OUTPUT_ONLY );
  FIELDDB_ELEMENT( PIDController, currentActuation, OUTPUT_ONLY );
  FIELDDB_ELEMENT( PIDController, scale, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, offset, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, continuousJoint, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, fixedTimeStep, INPUT_OUTPUT );
  FIELDDB_ELEMENT( PIDController, writeToLog, INPUT_OUTPUT );
}

/// Constructor
PIDController::PIDController( 
  Inst< SFNode      > _metadata, 
  Inst< SFBool      > _enabled,
  Inst< SFBool      > _useFeedForwardVelocity,
	Inst< SFVec4f     > _pidParams,
	Inst< SFVec3f     > _axis,
	Inst< TSSFFloat   > _target,
  Inst< TSSFFloat   > _targetVelocity,
	Inst< SFFloat     > _maxRateOfChange,
	Inst< SFFloat     > _maxActuation,
	Inst< SFFloat     > _maxAccumulatedError,
	Inst< SFFloat     > _currentError,
  Inst< SFFloat     > _currentActuation,
  Inst< SFFloat     > _scale,
  Inst< SFFloat     > _offset,
  Inst< SFBool      > _continuousJoint,
  Inst< SFFloat     > _fixedTimeStep,
  Inst< SFString    > _writeToLog ):
  X3DNode( _metadata ),
  enabled( _enabled ),
  useFeedForwardVelocity( _useFeedForwardVelocity ),
  pidParams( _pidParams ),
  axis( _axis ),
  target( _target ),
  targetVelocity( _targetVelocity ),
  maxRateOfChange( _maxRateOfChange ),
  maxActuation( _maxActuation ),
  maxAccumulatedError( _maxAccumulatedError ),
  currentError( _currentError ),
  currentActuation ( _currentActuation ),
  scale ( _scale ),
  offset ( _offset ),
  continuousJoint ( _continuousJoint ),
  fixedTimeStep ( _fixedTimeStep ),
  writeToLog ( _writeToLog ),
  valueUpdater ( new ValueUpdater ) {
	
  type_name = "PIDController";
  database.initFields( this );
	
  pid.targetField= target.get();
  pid.targetVelocityField= targetVelocity.get();

  valueUpdater->setName ( "valueUpdater" );
  valueUpdater->setOwner ( this );

  // set default values
  enabled->setValue( true ); 
  pidParams->setValue( Vec4f(0,0,0,0) );
  axis->setValue( Vec3f(0,0,0) );
  target->setValue( (H3DFloat)0 );
  targetVelocity->setValue( (H3DFloat)0 );
  maxRateOfChange->setValue( (H3DFloat)100 );
  maxActuation->setValue( (H3DFloat)1000 );
  maxAccumulatedError->setValue( (H3DFloat)5 );
  currentError->setValue( (H3DFloat)0, id );
  currentActuation->setValue( (H3DFloat)0, id );
  scale->setValue((H3DFloat)1);
  offset->setValue((H3DFloat)0);
  useFeedForwardVelocity->setValue( true );
  continuousJoint->setValue( false );
  fixedTimeStep->setValue( (H3DFloat)0 ),

  
  enabled->route ( valueUpdater );  
  pidParams->route ( valueUpdater );
  axis->route ( valueUpdater );
  target->route ( valueUpdater );
  maxRateOfChange->route ( valueUpdater );
  maxActuation->route ( valueUpdater );
  maxAccumulatedError->route ( valueUpdater );
  currentError->route ( valueUpdater );
  currentActuation->route ( valueUpdater );
  useFeedForwardVelocity->route ( valueUpdater );
  targetVelocity->route ( valueUpdater );
  scale->route ( valueUpdater );
  offset->route ( valueUpdater );
  continuousJoint->route ( valueUpdater );
  fixedTimeStep->route( valueUpdater );
}

void PIDController::traverseSG( TraverseInfo &ti ) {
  X3DNode::traverseSG(ti);

  string tmp_write_to_log= writeToLog->getValue();

  pid_lock.lock();
  PIDControl tmp= pid;
  pid.writeToLog= tmp_write_to_log;
  pid_lock.unlock();

  currentError->setValue ( tmp.currentError, id );
  currentActuation->setValue ( tmp.currentActuation, id );
}

H3DFloat PIDController::doControl ( H3DFloat currentValue, H3DFloat currentVelocity, bool wrapAngles, H3DFloat minTarget, H3DFloat maxTarget ) {
  pid_lock.lock();
  H3DFloat actuation= pid.doControl ( currentValue, currentVelocity, wrapAngles, minTarget, maxTarget  );
  pid_lock.unlock();

  return actuation;
}

void PIDController::resetPID( ) {
  pid_lock.lock();
  pid.reset();
  pid_lock.unlock();

}

Vec3f PIDController::getAxis () {
  pid_lock.lock();
  Vec3f v= pid.axis;
  pid_lock.unlock();

  return v;
}

H3DFloat PIDController::getCurrentError () {
  pid_lock.lock();
  H3DFloat error= pid.currentError;
  pid_lock.unlock();

  return error;
}

H3DFloat PIDController::getCurrentActuation () {
  pid_lock.lock();
  H3DFloat actuation= pid.currentActuation;
  pid_lock.unlock();

  return actuation;
}

void PIDController::ValueUpdater::update () {
  PIDController* node= static_cast<PIDController*>(getOwner());

  node->pid_lock.lock();

  if ( hasCausedEvent ( node->pidParams ) ) {
    node->pid.pidParams= node->pidParams->getValue();
  }

  if ( hasCausedEvent ( node->maxRateOfChange ) ) {
    node->pid.maxRateOfChange= node->maxRateOfChange->getValue();
  }

  if ( hasCausedEvent ( node->maxActuation ) ) {
    node->pid.maxActuation= node->maxActuation->getValue();
  }

  if ( hasCausedEvent ( node->maxAccumulatedError ) ) {
    node->pid.maxAccumulatedError= node->maxAccumulatedError->getValue();
  }

  if ( hasCausedEvent ( node->axis ) ) {
    node->pid.axis= node->axis->getValue();
    node->pid.axis.normalizeSafe();
  }

  if ( hasCausedEvent ( node->enabled ) ) {
    node->pid.enabled= node->enabled->getValue();
  }

  if ( hasCausedEvent ( node->useFeedForwardVelocity ) ) {
    node->pid.useFeedForwardVelocity= node->useFeedForwardVelocity->getValue();
  }

  if ( hasCausedEvent ( node->scale ) ) {
    node->pid.scale= node->scale->getValue();
  }

  if ( hasCausedEvent ( node->offset ) ) {
    node->pid.offset= node->offset->getValue();
  }

  if ( hasCausedEvent ( node->continuousJoint ) ) {
    node->pid.continuousJoint= node->continuousJoint->getValue();
  }

  if( hasCausedEvent( node->fixedTimeStep ) ) {
    node->pid.fixedTimeStep = node->fixedTimeStep->getValue();
  }
  node->pid_lock.unlock();

  EventCollectingField < PeriodicUpdate < Field > >::update();
}

PIDController::PIDControl::PIDControl () :
target ( 0 ),
targetVelocity ( 0 ),
scale ( 1 ),
offset ( 0 ),
maxRateOfChange ( 0 ),
maxActuation ( 0 ),
maxAccumulatedError ( 0 ),
currentError ( 0 ),
currentActuation ( 0 ),
accumulatedError ( 0 ),
prevTime ( 0 ),
lastValue ( 0 ),
wrappedValue ( 0 ),
enabled ( true ),
useFeedForwardVelocity ( true ),
continuousJoint ( true ),
fixedTimeStep ( 0 ),
targetField ( NULL ),
targetVelocityField ( NULL ),
start_log_time ( -1 )
{
  error[0] = 0;
  error[1] = 0; 
}

void PIDController::PIDControl::reset() {
  prevTime = 0;
  accumulatedError = 0;
  error[0] = 0;
  error[1] = 0;
}

H3DFloat PIDController::PIDControl::doControl ( H3DFloat currentValue, H3DFloat currentVelocity, bool wrapAngles, H3DFloat minTarget, H3DFloat maxTarget ) {
  if ( enabled ) {
    // time stamp the entry
    H3DTime curTime = TimeStamp();

    if ( writeToLog != "" ) {
      ofstream f ( writeToLog.c_str(), std::ofstream::app );
      if ( start_log_time < 0 ) {
        start_log_time= curTime;
      }
      f << (curTime-start_log_time) << "," << target << "," << currentValue << endl;
    }

    // Get new target values if they are available
    if ( targetField && targetVelocityField ) {
      target= scale * targetField->getValueRT() + offset;
      targetVelocity= scale * targetVelocityField->getValueRT();
    }

    H3DFloat current_value= currentValue;
    if ( wrapAngles ) {

      H3DFloat deltaValue= currentValue-lastValue;

      if ( deltaValue > Constants::pi ) {
  	    // if value is going from -3.14 to 3.14, subtract from wrappedValue
        wrappedValue-= (H3DFloat)Constants::pi+lastValue + (H3DFloat)Constants::pi-currentValue;
      } else if ( deltaValue < -Constants::pi ) {
  	    // if value is going from 3.14 to -3.14, add to wrappedValue
        wrappedValue+= (H3DFloat)Constants::pi+currentValue + (H3DFloat)Constants::pi-lastValue;
      } else {
        wrappedValue+= deltaValue;
      }
      lastValue= currentValue;
      current_value= wrappedValue;
    }

    if(!continuousJoint)
    {
      if ( minTarget < maxTarget ) {
        // check for joint limits
		    if(target > maxTarget)
			    target = maxTarget;
		    if(target < minTarget)
			    target = minTarget;
      }
    }
			  
	  // update error values
	  error[0] = error[1];
	  error[1] = target-current_value;
    currentError= error[1];
	  // calculate error rate
	  float errorRate = 0;
	  float dt = 0;
    if (prevTime != 0) {
      if( fixedTimeStep <= 0 ) {
        // use measured time step
        dt = (float)(curTime - prevTime);
        if( dt < Constants::f_epsilon ) {
          dt = Constants::f_epsilon;
        }
      } else {
        dt = fixedTimeStep;
      }
	    errorRate = (float)(1.0*(error[1]-error[0])/(1.0*dt));
    }
    prevTime = curTime;

	  // calculate error accumulation
	  float accError = accumulatedError;
	  accError += (float)(0.5*(error[1]+error[0])*dt);

	  // cap the error accumulation
	  if (accError>maxAccumulatedError)
		  accError = maxAccumulatedError;
	  if (accError<-maxAccumulatedError)
		  accError = -maxAccumulatedError;
	  accumulatedError = accError;
		    
	  // calculate the pid output
	  //                P Term---------------| I Term---------------| D Term---------------|
	  float magnitude = pidParams.x*error[1] + pidParams.y*accError + pidParams.z*errorRate; 
			
	  // cap the currentVelocity
	  if (currentVelocity >maxRateOfChange && magnitude>0)
		  magnitude = 0;
	  if (currentVelocity < -maxRateOfChange && magnitude<0)
		  magnitude = 0;
			  

	  // cap the torque limit 
	  if (magnitude > maxActuation)
	  magnitude = maxActuation;
	  if (magnitude < -maxActuation)
	  magnitude = -maxActuation;

    currentActuation= magnitude;

    // use feedforward velocity if enabled
    if(useFeedForwardVelocity)
    {
      target = target + pidParams.w * targetVelocity*dt;
    }

    return currentActuation;
  } else {
    return 0;
  }
}
