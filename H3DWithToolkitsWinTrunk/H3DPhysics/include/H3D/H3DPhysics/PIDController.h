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
/// \file PIDController.h
/// \brief Header file for PIDController, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>
#include <H3D/H3DPhysics/ThreadedField.h>

namespace H3D{
	
  class H3DPHYS_API PIDController : public X3DNode {
  public:
    typedef TypedSFNode < H3DRigidBodyJointNode > SFJointNode;

    /// Allow PID target to be set and accessed from separate threads
    typedef ThreadedField < SFFloat > TSSFFloat;

    // Constructor.
	  PIDController( 
          Inst< SFNode      >  _metadata = 0,
          Inst< SFBool      >  _enabled = 0,
          Inst< SFBool      >  _useFeedForwardVelocity = 0,
			    Inst< SFVec4f     >  _pidParams = 0,
			    Inst< SFVec3f     >  _axis = 0,
          Inst< TSSFFloat   >  _target = 0,
          Inst< TSSFFloat   >  _targetVelocity = 0,
			    Inst< SFFloat     >  _maxRateOfChange = 0,
			    Inst< SFFloat     >  _maxActuation = 0,
			    Inst< SFFloat     >  _maxAccumulatedError = 0,
			    Inst< SFFloat     >  _currentError = 0,
          Inst< SFFloat     >  _currentActuation = 0,
          Inst< SFFloat     >  _scale = 0,
          Inst< SFFloat     >  _offset = 0,
          Inst< SFBool      >  _continuousJoint = 0,
          Inst< SFFloat     >  _fixedTimeStep = 0,
          Inst< SFString    >  _writeToLog = 0 );

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

	  /// Calculates the new PID output to update the actuation: Force/Torque
	  virtual H3DFloat doControl ( H3DFloat currentValue, H3DFloat currentVelocity, bool wrapAngles= true, H3DFloat minTarget= 1, H3DFloat maxTarget= -1 );

    /// Thread safe access to the axis currently being controlled
    Vec3f getAxis ();

    /// Thread safe access to the current error, updated at physics thread rate
    H3DFloat getCurrentError ();

    /// Thread safe access to the current actuation force, updated at physics thread rate
    H3DFloat getCurrentActuation ();

    /// Reset PIDController values such as errors and other values that
    /// could affect the next control loop.
    void resetPID();

    /// Enable PID control
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFBool >  enabled;


    /// Enable feedforward velocity control
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFBool >  useFeedForwardVelocity;

	  /// PID parameters Kp kd Ki and kf (feedforward term)
	  ///
    /// <b>Access type:</b> input and output
    auto_ptr< SFVec4f >  pidParams;

	  /// axis of the joint, if no value is provided it uses body1's orientation as the axis
	  ///
    /// <b>Access type:</b> input and output
    auto_ptr< SFVec3f >  axis;

	  /// Target angle or seperation
	  ///
    /// <b>Access type:</b> input and output
    auto_ptr< TSSFFloat >  target;

    /// Target angle or seperation velocity
	  ///
    /// <b>Access type:</b> input and output
    auto_ptr< TSSFFloat >  targetVelocity;

	  /// Limit on joint velocity
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFFloat >  maxRateOfChange;

	  /// Limit on joint actuation
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFFloat >  maxActuation;

	  /// Cap for accumulation error
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFFloat >  maxAccumulatedError;

	  /// Current error
	  ///
    /// <b>Access type:</b> outputOnly
	  auto_ptr< SFFloat >  currentError;

    /// Current actuation
	  ///
    /// <b>Access type:</b> outputOnly
	  auto_ptr< SFFloat >  currentActuation;

    /// scale the target
	  ///
    /// <b>Access type:</b> outputOnly
	  auto_ptr< SFFloat >  scale;

    /// offset the scaled target
	  ///
    /// <b>Access type:</b> outputOnly
	  auto_ptr< SFFloat >  offset;

    /// enable continuous joint control
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< SFBool >  continuousJoint;

    /// The time step to use for integration
    /// 
    /// If <= 0 then the real/measured time elapsed is used
    ///
    /// <b>Access type:</b> input and output
    /// <b>Default value: </b> 0
    auto_ptr< SFFloat > fixedTimeStep;

    auto_ptr < SFString > writeToLog;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
	
  protected:

    class ValueUpdater : public EventCollectingField < PeriodicUpdate < Field > > {
      virtual void update ();
    };

    class PIDControl {
    public:
      PIDControl ();

      H3DFloat doControl ( H3DFloat currentValue, H3DFloat currentVelocity, bool wrapAngles, H3DFloat minTarget, H3DFloat maxTarget );
      void reset();
      // Inputs
      Vec4f pidParams;
      // processed target value scale* rawTarget + offset + accumulated feedforward velocity
      H3DFloat target;
      // scaled target velocity
      H3DFloat targetVelocity;

      TSSFFloat* targetField;
      TSSFFloat* targetVelocityField;

      H3DFloat scale;
      H3DFloat offset;
      H3DFloat maxRateOfChange;
      H3DFloat maxActuation;
      H3DFloat maxAccumulatedError;
      Vec3f axis;
      bool enabled;
      bool useFeedForwardVelocity;
      bool continuousJoint;
      H3DFloat fixedTimeStep;

      // Outputs
      H3DFloat currentError;
      H3DFloat currentActuation;

      string writeToLog;

    protected:
      /// Holds accumulated error between target and actual value
	    float accumulatedError;

      
	    /// Holds the latest two error values, used for calculating rate of error 
	    float error[2];

	    /// Time stamp of previous pid loop call. 
	    H3DTime prevTime;

      H3DFloat lastValue;
      H3DFloat wrappedValue;

      H3DTime start_log_time;
    };

    auto_ptr < ValueUpdater > valueUpdater;

    PIDControl pid;
    MutexLock pid_lock;
  };
}

#endif
