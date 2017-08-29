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
/// \file BodyPID.cpp
/// \brief Source file for BodyPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/BodyPID.h>

#include <fstream>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BodyPID::database( 
  "BodyPID", 
  &newInstance<BodyPID>, 
  typeid( BodyPID ), 
  &H3DPIDNode::database );

namespace BodyPIDInternals {
	FIELDDB_ELEMENT( BodyPID, targetPosition, INPUT_OUTPUT );
	FIELDDB_ELEMENT( BodyPID, targetOrientation, INPUT_OUTPUT );
	
  FIELDDB_ELEMENT( BodyPID, linearControl1, INPUT_OUTPUT );
	FIELDDB_ELEMENT( BodyPID, linearControl2, INPUT_OUTPUT );
  FIELDDB_ELEMENT( BodyPID, linearControl3, INPUT_OUTPUT );

  FIELDDB_ELEMENT( BodyPID, angularControl1, INPUT_OUTPUT );
	FIELDDB_ELEMENT( BodyPID, angularControl2, INPUT_OUTPUT );
  FIELDDB_ELEMENT( BodyPID, angularControl3, INPUT_OUTPUT );

  FIELDDB_ELEMENT( BodyPID, body, INPUT_OUTPUT );
  FIELDDB_ELEMENT( BodyPID, writeToLog, INPUT_OUTPUT );
}

/// Constructor
BodyPID::BodyPID( 
  Inst< SFNode              > _metadata,
  Inst< OnTargetPosition    > _targetPosition,
  Inst< OnTargetOrientation > _targetOrientation,

  Inst< SFPIDController > _linearControl1,
  Inst< SFPIDController > _linearControl2,
  Inst< SFPIDController > _linearControl3,
      
  Inst< SFPIDController > _angularControl1,
  Inst< SFPIDController > _angularControl2,
  Inst< SFPIDController > _angularControl3,
      
  Inst< SFBody          > _body,
  Inst< SFString    >  _writeToLog ) :
  H3DPIDNode( _metadata ),
  targetPosition( _targetPosition ),
  targetOrientation( _targetOrientation ),
  linearControl1( _linearControl1 ),
  linearControl2( _linearControl2 ),
  linearControl3( _linearControl3 ),
  angularControl1( _angularControl1 ),
  angularControl2( _angularControl2 ),
  angularControl3( _angularControl3 ),
  body ( _body ),
  orientation_target ( Rotation (1,0,0, 0) ),
  writeToLog ( _writeToLog ),
  start_log_time ( -1 )  {

  type_name = "BodyPID";
  database.initFields( this );
  linearPID1 = NULL;
  linearPID2 = NULL;
  linearPID3 = NULL;

  angularPID1 = NULL;
  angularPID2 = NULL;
  angularPID3 = NULL;
}

void BodyPID::traverseSG( TraverseInfo &ti ) {
  H3DPIDNode::traverseSG(ti);

  string tmp_write_to_log= writeToLog->getValue();
  pid_lock.lock();
  write_to_Log = tmp_write_to_log;
  pid_lock.unlock();

  PhysicsEngineThread *pt;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if ( pt && pt == engine_thread ) {
    if ( linearPID1 ) 
      linearPID1->traverseSG ( ti );
    if ( linearPID2 ) 
      linearPID2->traverseSG ( ti );
    if ( linearPID3 ) 
      linearPID3->traverseSG ( ti );

    if ( angularPID1 ) 
      angularPID1->traverseSG ( ti );
    if ( angularPID2 ) 
      angularPID2->traverseSG ( ti );
    if ( angularPID3 ) 
      angularPID3->traverseSG ( ti );
  }
}

void BodyPID::updateActuation () {
  if ( engine_thread ) {
    // Get current body state
    RigidBodyParameters* bodyParams= new RigidBodyParameters;

    // Get the params for body from physics engine
		engine_thread->getRigidBodyParameters( bodyId, *bodyParams );
    
    // Accumulate control force and torque
    Vec3f force, torque;

    // Linear PIDs
    if ( linearPID1 ) {
      force.x= linearPID1->doControl ( 
        bodyParams->getPosition().x, bodyParams->getLinearVelocity().x, false /*don't wrap angles*/ );
    }
    if ( linearPID2 ) {
      force.y= linearPID2->doControl ( 
        bodyParams->getPosition().y, bodyParams->getLinearVelocity().y, false /*don't wrap angles*/ );
    }
    if ( linearPID3 ) {
      force.z= linearPID3->doControl ( 
        bodyParams->getPosition().z, bodyParams->getLinearVelocity().z, false /*don't wrap angles*/ );
    }

    // Angular PIDs
    Vec3f vel= bodyParams->getAngularVelocity ();

    // Control using the error rather than the target to avoid issues
    // with euler angles continuity
    H3DUtil::Quaternion cur ( bodyParams->getOrientation() );
    H3DUtil::Quaternion error= (cur*orientation_target.inverse()).inverse();
    Vec3f error_angles= error.toEulerAngles();

    if ( angularPID1 ) {
      angularPID1->target->setValue ( error_angles.x );
      torque.x= angularPID1->doControl ( 0, vel.x, false /*don't wrap angles*/ );
    }
    if ( angularPID2 ) {
      angularPID2->target->setValue ( error_angles.y );
      torque.y= angularPID2->doControl ( 0, vel.y, false /*don't wrap angles*/ );
    }
    if ( angularPID3 ) {
      angularPID3->target->setValue ( error_angles.z );
      torque.z= angularPID3->doControl ( 0, vel.z, false /*don't wrap angles*/ );
    }

    string tmp_write_to_log;
    pid_lock.lock();
    tmp_write_to_log = write_to_Log;
    pid_lock.unlock();

    if ( tmp_write_to_log != "" ) {
      H3DTime curTime = TimeStamp();
      ofstream f ( tmp_write_to_log.c_str(), std::ofstream::app );
      if ( start_log_time < 0 ) {
        start_log_time= curTime;
      }
      
      Vec3f cur_angles= cur.toEulerAngles();
      Vec3f tar_angles= orientation_target.toEulerAngles();

      // Scale and offset in the target are not considered
      f << (curTime-start_log_time) << "," <<  linearPID1->target->getValueRT() << "," << linearPID2->target->getValueRT() << "," << linearPID3->target->getValueRT() << "," <<
        bodyParams->getPosition().x << "," << bodyParams->getPosition().y << "," <<  bodyParams->getPosition().z << "," <<
        tar_angles.x << "," << tar_angles.y << "," << tar_angles.z << "," << 
        cur_angles.x << "," << cur_angles.y << "," << cur_angles.z << "," << endl;
        
    }

    // Apply force and torque to body
    bodyParams->setForce ( bodyParams->getForce() + force );
    bodyParams->setTorque ( bodyParams->getTorque() + torque );
    engine_thread->setRigidBodyParameters( bodyId, *bodyParams);
  }
}

void BodyPID::initialize ( PhysicsEngineThread& pt ) {
  if ( !engine_thread ) {
    
    RigidBody* b= body->getValue();
    if ( b && b->isInitialized() ) {
	    bodyId= b->getBodyId();
      
      // handle the case where the targetPosition is set before the init.
      Vec3f tp = targetPosition->getValue();
      linearControl1->getValue()->target->setValue ( tp.x );
      linearControl2->getValue()->target->setValue ( tp.y );
      linearControl3->getValue()->target->setValue ( tp.z );

      linearPID1= linearControl1->getValue();
      linearPID2= linearControl2->getValue();
      linearPID3= linearControl3->getValue();

      angularPID1= angularControl1->getValue();
      angularPID2= angularControl2->getValue();
      angularPID3= angularControl3->getValue();

      engine_thread= &pt;
    }

  }
}

