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
/// \file BodyPID.h
/// \brief Header file for BodyPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BODYPID__
#define __BODYPID__

#include <H3D/H3DPhysics/H3DPIDNode.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/PIDController.h>

namespace H3D{
	
  /// \class BodyPID
  class H3DPHYS_API BodyPID : public H3DPIDNode {
  public:
    typedef TypedSFNode < RigidBody > SFBody;
    typedef TypedSFNode < PIDController > SFPIDController;

    class OnTargetPosition : public OnValueChangeSField < AutoUpdate < SFVec3f > > {
      virtual void onValueChange( const Vec3f& _v ){
        
        BodyPID* o= static_cast < BodyPID* > ( getOwner() );
        
        if ( o->linearPID1 ) {
          o->linearPID1->target->setValue ( _v.x );
        }
        
        if ( o->linearPID2 ) {
          o->linearPID2->target->setValue ( _v.y );
        }
        
        if ( o->linearPID3 ) {
          o->linearPID3->target->setValue ( _v.z );
        }

      };

    };

    class OnTargetOrientation : public OnValueChangeSField < AutoUpdate < SFRotation > > {
      virtual void onValueChange( const Rotation& _v ){
        
        BodyPID* o= static_cast < BodyPID* > ( getOwner() );
        o->orientation_target= H3DUtil::Quaternion ( _v );

      };
    };

    // Constructor.
    BodyPID( 
      Inst< SFNode              > _metadata = 0,
      Inst< OnTargetPosition    > _targetPosition= 0,
      Inst< OnTargetOrientation > _targetOrientation= 0,

      Inst< SFPIDController > _linearControl1 = 0,
      Inst< SFPIDController > _linearControl2 = 0,
      Inst< SFPIDController > _linearControl3 = 0,
      
      Inst< SFPIDController > _angularControl1 = 0,
      Inst< SFPIDController > _angularControl2 = 0,
      Inst< SFPIDController > _angularControl3 = 0,
      
      Inst< SFBody          > _body = 0,
      Inst< SFString    >  _writeToLog = 0 );
    
    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );
    
	  /// Calculates the new PID output to update the actuation: Force/Torque
	  virtual void updateActuation ();

    auto_ptr < OnTargetPosition    > targetPosition;
    auto_ptr < OnTargetOrientation > targetOrientation;

    auto_ptr < SFPIDController > linearControl1;
    auto_ptr < SFPIDController > linearControl2;
    auto_ptr < SFPIDController > linearControl3;
      
    auto_ptr < SFPIDController > angularControl1;
    auto_ptr < SFPIDController > angularControl2;
    auto_ptr < SFPIDController > angularControl3;
      
    auto_ptr < SFBody > body;

    auto_ptr < SFString > writeToLog;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
	
  protected:

    /// Initializes for the given PhysicsEngineThread.
	  virtual void initialize ( PhysicsEngineThread& pt );
    
    H3DBodyId bodyId;
    
    PIDController* linearPID1;
    PIDController* linearPID2;
    PIDController* linearPID3;

    PIDController* angularPID1;
    PIDController* angularPID2;
    PIDController* angularPID3;

    H3DUtil::Quaternion orientation_target;

    H3DTime start_log_time;
    string write_to_Log;
    MutexLock pid_lock;

  };
}

#endif
