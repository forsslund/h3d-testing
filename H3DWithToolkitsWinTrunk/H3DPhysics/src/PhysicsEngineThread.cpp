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
/// \file PhysicsEngineThread.cpp
/// \brief cpp file for PhysicsEngineThread.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Cylinder.h>
#include <H3D/Cone.h>
#include <H3D/X3DComposedGeometryNode.h>
#include <H3D/Coordinate.h>
#include <H3D/H3DHapticsDevice.h>
#include <HAPI/HAPIHapticsDevice.h>

#include <iostream>

using namespace H3D;

PhysicsEngineThread::PhysicsEngineThread( const string & _engine,
                                          Priority thread_priority,
                                          int thread_frequency,
                                          bool _useMainThread ):
  PeriodicThread( thread_priority, thread_frequency ),
  engine( _engine ),
  simulation_running( false ),
  update_rate( 0 ),
  time_in_last_loop( 0 ),
  time_of_last_update( 0 ),
  frame_count ( 0 ),
  step_size( (H3DFloat) (1.0f / thread_frequency) ),
  frame_rate_interval ( ((H3DFloat)(1.0f / thread_frequency))*10.0 ),
  world_params ( new WorldParameters ),
  useMainThread ( _useMainThread ),
  synchroniseTheThreads( false ) {
  
  // initialize the physics engine in use
  callbacks()->initEngine( this );  
}

PhysicsEngineThread::~PhysicsEngineThread() {
  clearAllCallbacks();

  // clean up
  callbacks()->deInitEngine( this );
}

void PhysicsEngineThread::setUseMainThread ( bool _useMainThread ) {
  if ( _useMainThread && !useMainThread ) {
    // Switch to using main thread
    // Transfer existing callbacks from thread to mainThreadCallbacks

    callback_lock.lock();
    callbacks_added_lock.lock();
    // For threads with a low frequency it could be that the callback is
    // in callbacks_added, therefore lock both locks and then go through
    // both lists.
    for( CallbackList::iterator i = callbacks_added.begin();
         i != callbacks_added.end(); ++i ) {
      mainThreadCallbacks.push_back ( *i );
    }
    callbacks_added.clear();

    for( CallbackList::iterator i = PeriodicThread::callbacks.begin();
         i != PeriodicThread::callbacks.end(); ++i ) {
      mainThreadCallbacks.push_back ( *i );
    }
    PeriodicThread::callbacks.clear();

    callbacks_added_lock.unlock();
    callback_lock.unlock();

  } else if ( !_useMainThread && useMainThread ) {
    // Switch to using separate thread
    // Transfer existing callbacks from mainThreadCallbacks to thread

    for( CallbackList::iterator i = mainThreadCallbacks.begin();
         i != mainThreadCallbacks.end(); ++i ) {
      if( frequency < 0 ) {
        callback_lock.lock();
        // signal the thread that a new callback is available if it is waiting for
        // one.
        if( PeriodicThread::callbacks.size() == 0 ) callback_lock.signal();
        // add the new callback
        PeriodicThread::callbacks.push_back( *i );
        callback_lock.unlock();
      } else {
        callbacks_added_lock.lock();
        // add the new callback
        callbacks_added.push_back( *i );
        callbacks_added_lock.unlock();
      }
    }
    mainThreadCallbacks.clear();
  }
  useMainThread= _useMainThread;
}

void PhysicsEngineThread::mainThreadStep () {
  if ( useMainThread ) {
    // Process async callbacks
    vector< PeriodicThread::CallbackList::iterator > to_remove;
    for( PeriodicThread::CallbackList::iterator i = mainThreadCallbacks.begin(); i != mainThreadCallbacks.end(); ++i ) {
      PeriodicThread::CallbackCode c = ( (*i).second ).first( ( (*i).second ).second );
      if( c == PeriodicThread::CALLBACK_DONE ) {
        to_remove.push_back( i );
      }
    }
    // remove all callbacks that returned CALLBACK_DONE.
    for( vector< PeriodicThread::CallbackList::iterator >::iterator i = to_remove.begin(); i != to_remove.end(); ++i ) {
      free_ids.push_back( (*(*i) ).first );
      mainThreadCallbacks.erase( *i );
    }
  }
}

bool PhysicsEngineThread::removeAsynchronousCallbackNoLock( int callback_handle )
{
  if ( useMainThread ) {
    return removeAsynchronousCallback ( callback_handle );
  } else {
    //callback_lock.lock();
    for( CallbackList::iterator i = PeriodicThread::callbacks.begin();
         i != PeriodicThread::callbacks.end(); ++i ) {
      if( (*i).first == callback_handle ) {
        free_ids.push_back( callback_handle );
        PeriodicThread::callbacks.erase( i );
        //callback_lock.unlock();
        return true;
      }
    }
    //callback_lock.unlock();
    return false;
  }
}

void PhysicsEngineThread::synchronousCallback( CallbackFunc func, void *data ) {
  if ( useMainThread ) {
    func ( data );
  } else {
    // If called from the physics thread then just execute the callback
    // immediately, otherwise deadlock would occur
    if ( pthread_equal ( getThreadId(), ThreadBase::getCurrentThreadId() ) ) {
      func ( data );
    } else {
      PeriodicThread::synchronousCallback ( func, data );
    }
  }
}

int PhysicsEngineThread::asynchronousCallback( CallbackFunc func, void *data ) {
  if ( useMainThread ) {
    // Add callback to our list
    int cb_id = genCallbackId();
    // add the new callback
    mainThreadCallbacks.push_back( make_pair( cb_id, make_pair( func, data ) ) );
    return cb_id;
  } else {
    return PeriodicThread::asynchronousCallback ( func, data );
  }
}

bool PhysicsEngineThread::removeAsynchronousCallback( int callback_handle ) {
  if ( useMainThread ) {
    // Remove callback from our list
    for( CallbackList::iterator i = mainThreadCallbacks.begin(); i != mainThreadCallbacks.end(); ++i ) {
      if( (*i).first == callback_handle ) {
        // Add callback_handle integer to the free_ids list in order
        // to reuse id later.
        free_ids.push_back( callback_handle );
        mainThreadCallbacks.erase( i );
        return true;
      }
    }
    return false;
  } else {
    return PeriodicThread::removeAsynchronousCallback ( callback_handle );
  }
}

void PhysicsEngineThread::setWorldParameters( WorldParameters& wp ) {
  wp.setEngine( *this );

  world_params_lock.lock();
  world_params->copyInputParameters ( wp );
  world_params_lock.unlock();

  // World parameters must be set before body parameters, so use synchronous callback
  // todo: Implement better solution if speed is a problem
  //asynchronousCallback( callbacks()->setWorldParameters, wp );
  synchronousCallback( callbacks()->setWorldParameters, &wp );
}

void PhysicsEngineThread::synchroniseWithSceneGraph( ) {
  
  synchroniseTheThreads_lock.lock();
  if ( synchroniseTheThreads ){
    synchronousCallback( callbacks()->synchroniseWithSceneGraph, NULL );
    synchroniseTheThreads = false;
  }
  synchroniseTheThreads_lock.unlock();
  
  nodesToDeleteInSync_lock.lock();
  nodesToDeleteInSync.clear();
  nodesToDeleteInSync_lock.unlock();
}

void PhysicsEngineThread::addNodeToDeleteInSynch( Node *n ) {
  
  nodesToDeleteInSync_lock.lock();
  nodesToDeleteInSync.push_back( n );
  nodesToDeleteInSync_lock.unlock();
}

void PhysicsEngineThread::synchroniseAtTheEndOfTheFrame() {
  
  synchroniseTheThreads_lock.lock();
  synchroniseTheThreads = true;
  synchroniseTheThreads_lock.unlock();

}

H3DCollidableId PhysicsEngineThread::addCollidable( CollidableParameters& p ) {

  p.setEngine( *this );
  preUpdateCollidable ( p );
  synchronousCallback( callbacks()->addCollidable, &p );
  if( p.getCollidableId() == 0 ) {
    delete &p;
    return 0;
  }
  collidable_lock.lock();
  collidables[ p.getCollidableId() ] = &p;
  collidable_lock.unlock();

  return p.getCollidableId();
}

bool PhysicsEngineThread::removeCollidable( H3DCollidableId id ) {
  // remove the collidable from the collidable map. 
  collidable_lock.lock();
  CollidableMap::iterator i = collidables.find( id );
  bool have_shape = i != collidables.end();
  collidable_lock.unlock();

  if( have_shape ) {
    collidable_lock.lock();
    CollidableParameters* params= (*i).second;
    collidables.erase( i );
    collidable_lock.unlock();
    synchronousCallback( callbacks()->removeCollidable, params );
    delete params;
    return true;
  } else {
    return false;
  }  
}

bool PhysicsEngineThread::setCollidableParameters( H3DCollidableId id,
                                                  CollidableParameters& p )
{
  p.setCollidableId( id );
  p.setEngine( *this );
  preUpdateCollidable ( p );
  collidable_lock.lock();
  collidables[id]->copyInputParameters( p );
  collidable_lock.unlock();

  asynchronousCallback( callbacks()->setCollidableParameters, &p );

  return true;
}

bool PhysicsEngineThread::getCollidableParameters( H3DCollidableId id,
                                                  CollidableParameters& params )

{
  collidable_lock.lock();
  params.copyOutputParameters( *collidables[id] );
  collidable_lock.unlock();  
  return true;
}

H3DSpaceId PhysicsEngineThread::addSpace( SpaceParameters& p ) {
  p.setEngine( *this );

  synchronousCallback( callbacks()->addSpace, &p );
  if( p.getSpaceId() == 0 ) {
    delete &p;
    return 0;
  }
  space_lock.lock();
  spaces[ p.getSpaceId() ] = &p;
  space_lock.unlock();

  return p.getSpaceId();  
}

bool PhysicsEngineThread::removeSpace( H3DSpaceId id ) {
  // remove the space from the space map. 
  space_lock.lock();
  SpaceMap::iterator i = spaces.find( id );
  bool have_space = i != spaces.end();
  space_lock.unlock();

  if( have_space ) {
    space_lock.lock();
    SpaceParameters* params= (*i).second;
    spaces.erase( i );
    space_lock.unlock();
    synchronousCallback( callbacks()->removeSpace, params );
    delete params;
    return true;
  } else {
    return false;
  }
}
H3DBodyId PhysicsEngineThread::addRigidBody( RigidBodyParameters& p ) {
  p.setEngine( *this );
  synchronousCallback( callbacks()->addRigidBody, &p );

  // set initial output values.
  p.setOrientation( p.getStartOrientation() );
  p.setPosition( p.getStartPosition() );
  if( p.getBodyId() == 0 ) {
    delete &p;
    return 0;
  }
  rigid_body_lock.lock();
  rigid_bodies[ p.getBodyId() ] = &p;
  rigid_body_lock.unlock();

  return p.getBodyId();
}

bool PhysicsEngineThread::removeRigidBody( H3DBodyId id ) {
  // remove the rigid body from the rigid bodies held in the
  // PhysicsEngineThread. 

  rigid_body_lock.lock();
  RigidBodyMap::iterator i = rigid_bodies.find( id );
  bool have_body = i != rigid_bodies.end();
  rigid_body_lock.unlock();

  if( have_body ) {
    rigid_body_lock.lock();
    RigidBodyParameters* params= (*i).second;
    rigid_bodies.erase( i );
    rigid_body_lock.unlock();
    synchronousCallback( callbacks()->removeRigidBody, params );
    delete params;
    return true;
  } else {
    // rigid body id does not exist, error
    return false;
  }
}

bool PhysicsEngineThread::setRigidBodyParameters( H3DBodyId body_id, 
                                                 RigidBodyParameters &p)
{

  bool have_body = false;
  p.setBodyId( body_id );
  p.setEngine( *this );
  
  rigid_body_lock.lock();
  RigidBodyMap::iterator i = rigid_bodies.find( body_id );
  have_body = i != rigid_bodies.end();
  if( have_body ) { 
    rigid_bodies[body_id]->copyInputParameters( p );
  }
  rigid_body_lock.unlock();

  if( have_body ) {
    asynchronousCallback( callbacks()->setRigidBodyParameters, &p );
  }
  else {
    RigidBodyParameters *ptr_p = static_cast< RigidBodyParameters *>( &p );
    delete ptr_p;
  }

  return have_body;
}

bool PhysicsEngineThread::getRigidBodyParameters( H3DBodyId body,
                                                 RigidBodyParameters& params )
{
  bool have_body = false;
  rigid_body_lock.lock();
  RigidBodyMap::iterator i = rigid_bodies.find( body );
  have_body = i != rigid_bodies.end();
  if( have_body ) { 
    params.copyOutputParameters( *rigid_bodies[body] );
    params.copyInputParameters( *rigid_bodies[body] );
  }
  rigid_body_lock.unlock();

  return have_body;
}

void PhysicsEngineThread::startSimulation() {
  if( !simulation_running ) {
    // add callback for calculating the rate of the thread and to update
    // output parameters in objects in the physics engine.
    simulation_cb_handle = asynchronousCallback( callbacks()->doSimulationSteps, this );
    save_update_cb_handle = asynchronousCallback( saveUpdateRate, this );
    simulation_running = true;
  }
}

void PhysicsEngineThread::stopSimulation() {
  if( simulation_running ) {
    removeAsynchronousCallback( simulation_cb_handle );
    removeAsynchronousCallback( save_update_cb_handle );
    simulation_running = false;
  }
}

int PhysicsEngineThread::setGlobalExternalForce( H3DBodyId body_id,
                                                const Vec3f &force )
{
  rigid_body_lock.lock();
  RigidBodyMap::iterator i = rigid_bodies.find( body_id );
  bool have_body = i != rigid_bodies.end();
  if( have_body ) ( *i ).second->setGraphicsFrameForce( force );
  rigid_body_lock.unlock();

  if( have_body ) return 0;
  else return -1;
}

int PhysicsEngineThread::setGlobalExternalTorque( H3DBodyId body_id,
                                                 const Vec3f &torque )
{
  rigid_body_lock.lock();
  RigidBodyMap::iterator i = rigid_bodies.find( body_id );
  bool have_body = i != rigid_bodies.end();
  if( have_body ) ( *i ).second->setGraphicsFrameTorque( torque );
  rigid_body_lock.unlock();

  if( have_body ) return 0;
  else return -1;
  return 0;
}
/// Create a new constraint with the given parameters.
H3DConstraintId PhysicsEngineThread::addConstraint( ConstraintParameters& p ) {

  p.setEngine( *this );
  synchronousCallback( callbacks()->addConstraint, &p );

  if( p.getConstraintId() == 0 ) {
    delete &p;
    return 0;
  }

  constraint_lock.lock();
  constraints[ p.getConstraintId() ] = &p;
  constraint_lock.unlock();

  return p.getConstraintId();
}
/// Remove an constraint from the physics engine. Returns true on success.
bool PhysicsEngineThread::removeConstraint( H3DConstraintId constraint ) {
  constraint_lock.lock();
  ConstraintMap::iterator i = constraints.find( constraint );
  bool have_body = i != constraints.end();
  constraint_lock.unlock();

  if( have_body ) {
    constraint_lock.lock();
    ConstraintParameters* params= (*i).second;
    constraints.erase( i );
    constraint_lock.unlock();
    synchronousCallback( callbacks()->removeConstraint, params );
    delete params;
    return true;
  } else {
    return false;
  }
}
/// Get the parameters for a given constraint. The H3DSoftBodyNodeParameters
/// structure will be updated with the values from the constraint
/// Returns true on success.
bool PhysicsEngineThread::getConstraintParameters( H3DConstraintId constraint, 
                                                  ConstraintParameters& params )
{
  constraint_lock.lock();
  ConstraintMap::iterator i= constraints.find( constraint );
  if ( i != constraints.end() ) {
    params.copyOutputParameters( *(*i).second );
    params.copyInputParameters( *(*i).second );
  }
  constraint_lock.unlock();
  return true;
}

/// Set the parameters for a given constraint.
/// Only the paramaters in the ConstraintParameters
/// structure that has actually been set will be changed.
bool PhysicsEngineThread::setConstraintParameters( H3DConstraintId constraint, 
                                                  ConstraintParameters& params )
{
  params.setConstraintId ( constraint );
  params.setEngine ( *this );
  constraint_lock.lock();
  constraints[constraint]->copyInputParameters( params );
  constraint_lock.unlock();

  asynchronousCallback( callbacks()->setConstraintParameters, &params );
  return true;
}

int PhysicsEngineThread::getCurrentContacts( list< ContactParameters > &contacts ) {

#ifndef USE_CONTACTS_LOCK
  pair< list< PhysicsEngineParameters::ContactParameters > *,
    PhysicsEngineThread * > data = make_pair( &contacts, this );
#endif

  PROFILE_BEGIN ( threadSync );

#ifndef USE_CONTACTS_LOCK
  synchronousCallback( callbacks()->getCurrentContacts, &data );
#else
  currentContacts_lock.lock();
  contacts= currentContacts;
  currentContacts_lock.unlock();
#endif

  PROFILE_END ();

  return 0;
}

void PhysicsEngineThread::updateObjectParameters() {
  rigid_body_lock.lock();
  for( RigidBodyMap::iterator i = rigid_bodies.begin();
    i != rigid_bodies.end(); ++i ) {
      callbacks()->getRigidBodyParameters( (*i).second );
  }
  rigid_body_lock.unlock();

  // Update constraint parameters
  constraint_lock.lock();
  for ( ConstraintMap::iterator i = constraints.begin(); i != constraints.end(); ++i ) {
    callbacks()->getConstraintParameters( (*i).second );
  }
  constraint_lock.unlock();
}

void PhysicsEngineThread::preUpdateCollidable ( CollidableParameters& p ) {
  if ( ShapeParameters* s= dynamic_cast<ShapeParameters*>(&p) ) {
    if ( s->haveOriginalShape() && s->getOriginalShape() ) {
      s->setShape ( static_cast<X3DGeometryNode*>(s->getOriginalShape()->clone()) );
      if ( p.haveScale() ) {
        if ( (p.getScale()-Vec3f ( 1, 1, 1 )).lengthSqr() > H3DUtil::Constants::f_epsilon ) {
          scaleGeometry ( *s->getShape(), p.getScale() );
        }
      }

      s->getShape()->setName( "clone_" + s->getOriginalShape()->getName() );
      if( s->getUpdateShapeBounds() ) {
        s->getShape()->boundTree->upToDate();
      }
    }
  }
}

void PhysicsEngineThread::scaleGeometry ( X3DGeometryNode& geometry, const Vec3f& scale ) {
  bool success= false;
  if ( X3DComposedGeometryNode* g= dynamic_cast<X3DComposedGeometryNode*> ( &geometry ) ) {
    if ( Coordinate* c= dynamic_cast<Coordinate*> ( g->coord->getValue () ) ) {
      MFVec3f::vector_type points= c->point->getValue();
      for ( size_t i= 0; i < points.size(); ++i ) {
        points[i]= Matrix4f ( Vec3f(), H3DUtil::ArithmeticTypes::Rotation(), scale ) * points[i];
      }
      c->point->setValue ( points );
      success= true;
    }
  } else if ( Sphere* g= dynamic_cast<Sphere*> ( &geometry ) ) {
    if ( H3DAbs(scale.x-scale.y) > H3DUtil::Constants::f_epsilon || 
         H3DAbs(scale.y-scale.z) > H3DUtil::Constants::f_epsilon ) {
       Console(4) << "Warning: Non-uniform scaling for Sphere is not supported!" << endl;
    }
    g->radius->setValue ( g->radius->getValue() * scale.x );
    success= true;
  } else if ( Cylinder* g= dynamic_cast<Cylinder*> ( &geometry ) ) {
    if ( H3DAbs(scale.x-scale.z) > H3DUtil::Constants::f_epsilon ) {
       Console(4) << "Warning: Non-uniform xz scaling for Cylinder is not supported!" << endl;
    }
    g->radius->setValue ( g->radius->getValue() * scale.x );
    g->height->setValue ( g->height->getValue() * scale.y );
    success= true;
  } else if ( Cone* g= dynamic_cast<Cone*> ( &geometry ) ) {
    if ( H3DAbs(scale.x-scale.z) > H3DUtil::Constants::f_epsilon ) {
       Console(4) << "Warning: Non-uniform xz scaling for Cylinder is not supported!" << endl;
    }
    g->bottomRadius->setValue ( g->bottomRadius->getValue() * scale.x );
    g->height->setValue ( g->height->getValue() * scale.y );
    success= true;
  } else if ( Box* g= dynamic_cast<Box*> ( &geometry ) ) {
    const Vec3f& s= g->size->getValue();
    g->size->setValue ( Vec3f ( s.x*scale.x, s.y*scale.y, s.z*scale.z ) );
    success= true;
  }
  
  if ( !success ) {
    Console(4) << "Warning: Scaling is not supported for this geometry type: " << geometry.getTypeName() << endl;
  }
}

/// \todo This function should be renamed.
H3DUtil::PeriodicThread::CallbackCode PhysicsEngineThread::saveUpdateRate(void *data) {

  PROFILE_BEGIN ( saveUpdateRate );

  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  H3DTime current_time = TimeStamp();
  if( pt->time_of_last_update != 0 ) {
    H3DTime dt = current_time  - pt->time_of_last_update;
    if ( dt > pt->frame_rate_interval ) {
      pt->update_rate = (int) (pt->frame_count / dt);
      pt->time_of_last_update = current_time;
      pt->frame_count = 0;
    }
  } else {
    pt->time_of_last_update = current_time;
  }
  ++(pt->frame_count);
  pt->updateObjectParameters();

  //
  PhysicsEngineParameters::WorldParameters world_params = 
    pt->getWorldParameters();

  // only step if RigidBodyCollection is enabled.
  if( world_params.getEnabled() ) {

#ifdef USE_CONTACTS_LOCK
    pair< ContactList*,
      PhysicsEngineThread* > contact_data = make_pair( &pt->currentContacts, pt );
    
    pt->currentContacts_lock.lock();
    pt->currentContacts.clear();
    pt->callbacks()->getCurrentContacts ( &contact_data );
    pt->currentContacts_lock.unlock();
#endif

    // Get time step to apply damping forces
    H3DFloat timeStep= pt->getStepSize();
    if ( !world_params.getUseStaticTimeStep() && pt->getUpdateRate() > 0 ) {
      timeStep= 1.0f/pt->getUpdateRate();
    }

    // For backward compatibility
    // (timeStep not previously considered when adding damping
    // i.e., timestep=1 when running at exactly desired loop rate) 
    timeStep/= pt->getStepSize();

    // add damping to each rigid body that has damping enabled
    list< H3DBodyId > bodies;
    pt->rigid_body_lock.lock();
    pt->getCurrentRigidBodies( bodies );
    pt->rigid_body_lock.unlock();

    for( list< H3DBodyId >::iterator i = bodies.begin(); 
      i != bodies.end(); ++i ) {
        
        PhysicsEngineParameters::RigidBodyParameters p;
        // Since the moment getCurrentRigidBodies() copies rigid_bodies to bodies till now,
        // in case a body was deleted from the rigid_bodies, a rare random crash was
        // happening. Return value of getRigidBodyParameters handles that scenario.
        if  ( !pt->getRigidBodyParameters( *i, p ) )
          break;
        PhysicsEngineParameters::ExternalForceTorqueParameters *f_params =
          &pt->force_torque_params;
        f_params->engine_thread = pt;
        f_params->body_id = *i;
        f_params->position = p.getPosition();
        if( p.getAutoDamp() ) {  
          f_params->force = p.getLinearVelocity() * -p.getLinearDampingFactor() * timeStep;
          f_params->torque = p.getAngularVelocity() * -p.getAngularDampingFactor() * timeStep;

          f_params->force += p.getForce() + p.getGraphicsFrameForce();
          f_params->torque += p.getTorque() + p.getGraphicsFrameTorque();
        } else {
          f_params->force = p.getForce() + p.getGraphicsFrameForce();
          f_params->torque = p.getTorque() + p.getGraphicsFrameTorque();
        }

        // Add manipulation forces from haptics devices
        pt->addHapticInteractionForces ( p, *f_params );

        pt->callbacks()->addGlobalExternalForceAndTorque( f_params );

        // reset H3D objects forces and torques
        bool have_body = false;
        pt->rigid_body_lock.lock();
        // Since the moment getCurrentRigidBodies() copies rigid_bodies to bodies till now,
        // in case a body was deleted from the rigid_bodies, a rare random crash was
        // happening. Check, therefore, for the existance of the body again.
        RigidBodyMap::iterator bf = pt->rigid_bodies.find( *i );
        have_body = bf != pt->rigid_bodies.end();
        if( have_body ){  
          pt->rigid_bodies[*i]->setForce( Vec3f(0,0,0) );
          pt->rigid_bodies[*i]->setTorque( Vec3f(0,0,0) );
        }
        pt->rigid_body_lock.unlock();

#ifdef DEBUG_RB_LAG
      if ( p.getDebug() ) {

        FILE* lagDebugFile= pt->lagDebugFiles[*i];

        if ( !lagDebugFile ) {
          stringstream ss;
          ss << "LagDebugPhysics" << *i;
          lagDebugFile= fopen ( ss.str().c_str(), "w" );
          pt->lagDebugFiles[*i]= lagDebugFile;
        }
        LARGE_INTEGER time;
        QueryPerformanceCounter ( &time );
        fprintf ( lagDebugFile, "%lli, %f, %f, %f\n", time.QuadPart, p.getPosition().x, p.getPosition().y, p.getPosition().z );
      }
#endif
    }
  }

  PROFILE_END ();

  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

void PhysicsEngineThread::setHapticsDevices ( const NodeVector& _hapticsDevices ) {
  hapticsDevices_lock.lock();
  hapticsDevices= _hapticsDevices;
  hapticsDevices_lock.unlock();
}

void PhysicsEngineThread::addHapticInteractionForces ( 
  PhysicsEngineParameters::RigidBodyParameters& params, 
  PhysicsEngineParameters::ExternalForceTorqueParameters& forceParams ) {
  
  hapticsDevices_lock.lock();
  // For each haptics device
  for ( NodeVector::const_iterator i= hapticsDevices.begin(); i != hapticsDevices.end(); ++i ) {
    HAPI::HAPIHapticsDevice* hd= static_cast<H3DHapticsDevice*>(*i)->getHAPIDevice();
    if( hd ) {
      // For each layer
      for( unsigned int layer = 0; layer < hd->nrLayers(); ++layer ) {
        HAPI::HAPIHapticsRenderer* renderer = 
          hd->getHapticsRenderer( layer );
        if( renderer ) {
          HAPI::HAPIHapticsRenderer::Contacts contacts= renderer->getContacts();
          // For each contact
          for( HAPI::HAPIHapticsRenderer::Contacts::iterator j = contacts.begin();
                j != contacts.end(); ++j ) {
            // The geometry in contact
            X3DGeometryNode *geom =
              static_cast< X3DGeometryNode * >( (*j).first->getUserData() );
            HAPI::HAPISurfaceObject::ContactInfo contact= (*j).second;

            Matrix4d global_to_local= (*j).first->getInverse();
            Matrix3d global_vec_to_local = global_to_local.getRotationPart();

            Vec3f cp( global_to_local * contact.globalSurfaceContactPoint() );
            Vec3f f( global_vec_to_local * contact.globalForce() );

            // Is the contacted geometry attached to this rigid body?
            vector< H3DCollidableId > geometries= params.getGeometry();
            for ( vector< H3DCollidableId >::iterator k= geometries.begin(); k != geometries.end(); ++k ) {
              collidable_lock.lock();
              CollidableParameters* c= collidables[ *k ];
              /// \todo Recurse inside OffsetParameters
              if ( ShapeParameters* s= dynamic_cast<ShapeParameters*>(c) ) {
                if ( s->getOriginalShape() == geom ) {
                  // The haptics device is in contact with this rigid body
                  HAPI::Vec3 global_point= params.getOrientation()*cp + params.getPosition();
                  HAPI::Vec3 global_force= params.getOrientation()*(-f);

                  HAPI::Vec3 lever_arm= global_point - params.getPosition();
                  forceParams.force+= Vec3f(global_force);
                  forceParams.torque+= Vec3f(lever_arm.crossProduct( global_force ));
                }
              }
              collidable_lock.unlock();
            }
          }
        }
      }
    }
  }
  hapticsDevices_lock.unlock();
}

bool PhysicsEngineThread::supportsPhysicsEngine( const string &engine ) {
  return registered_physics_engines().find( engine ) != registered_physics_engines().end();
}

vector< string > PhysicsEngineThread::getSupportedPhysicsEngineNames() {
  return registered_physics_engines().getAllKeys();
}
