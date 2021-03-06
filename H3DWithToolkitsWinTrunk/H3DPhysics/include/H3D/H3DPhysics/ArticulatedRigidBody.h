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
/// \file ArticulatedRigidBody.h
/// \brief Header file for ArticulatedRigidBody, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ARTICULATEDRIGIDBODY__
#define __ARTICULATEDRIGIDBODY__

//#include <H3D/X3DNode.h>
//#include <H3D/SFFloat.h>
//#include <H3D/Box.h>
//#include <H3D/Sphere.h>
//#include <H3D/Cylinder.h>
//#include <H3D/SFVec3f.h>
//#include <H3D/SFBool.h>
//#include <H3D/MFNode.h>
//#include <H3D/MFVec3f.h>
//#include <H3D/Shape.h>
//#include <H3D/SFMatrix3f.h>
//#include <H3D/SFRotation.h>
//#include <H3D/PeriodicUpdate.h>
//#include <H3D/H3DPhysics/CollidableShape.h>
//#include <H3D/H3DPhysics/H3DBodyNode.h>
//#include <H3D/H3DPhysics/PhysicsEngineThread.h>
//#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/MFRotation.h>
#include <H3D/H3DPhysics/RigidBody.h>

namespace H3D{  
  
  /// \ingroup X3DNodes
  /// \class ProxyRigidBody
  /// \brief The ProxyRigidBody is a rigid body used to represent the bodies of an
  /// ArticulatedRigidBody. 
  ///
  /// The ProxyRigidBody is not added to the simulation, therefore its fields can not
  /// be used as an input to the ohysics engine( It can be supported in the future, check
  /// the implementation for details ). Its main purpose is to represent the bodies of
  /// an ArticulatedRigidBody so that these bodies can be used in joints, constraints etc.
  ///
  /// Not all of the properties of rigid bodies are supported. Appying a force, via BodySpring
  /// for instance, may not work.
  ///
  // Calling get/setRigidBodyParameters with body id of a ProxyRigidBody will not work/might even crash?
  ///
  /// \par Internal routes:
  /// \dotfile ProxyRigidBodyBody.dot
  class H3DPHYS_API ProxyRigidBody : public RigidBody {
  public:

    /// Constructor.
    ProxyRigidBody();

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti){};

    /// Render the collidables of the body.
    virtual void renderCollidable( bool render_only_enabled_collidables ){};

    /// Initialize the rigid body for the given PhysicsEngineThread. I.e. 
    /// create a new rigid body in the physics engine with the parameters
    /// of the rigid body fields. Returns 0 on success.
    virtual bool initializeBody( H3D::PhysicsEngineThread& pt );

    /// Deletes this rigid body node from the given PhysicsEngineThread.
    virtual bool deleteBody();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    friend class ArticulatedRigidBody;

  protected:
    /// Returns a RigidBodyParameter to describe the rigid body. By default
    /// the function returns a RigidBodyParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::RigidBodyParameters *getRigidBodyParameters( bool all_params = false ){return NULL;};
  };


  /// \ingroup X3DNodes
  /// \class ArticulatedRigidBody
  /// \brief The ArticulatedRigidBody is single body comprising a set of links
  /// (each of which behaves like a rigid body) connected together with 6DOF joints.
  ///
  /// They support higher mass ratios, more accurate drive models, have better dynamic
  /// stability and a more robust recovery from joint separation than standard group
  /// of bodies connected with joints. However, they are considerably more expensive.
  ///
  /// Every articulation has a tree-like structure - so there can be no loops or breaks.
  ///
  /// Implemented only in PhysX3, in testing phase. Use it only you know what it is doing.
  ///
  /// \par Internal routes:
  /// \dotfile ArticulatedRigidBody.dot
  class H3DPHYS_API ArticulatedRigidBody : public RigidBody {
  public:

    // Soecialized collidable node to update the collidable of the fake rigid bodies
    // when the collidable geometry is changed.
    class MFArticulatedCollidables : public MFCollidableNode {

      virtual void onAdd( Node *n ) {
        MFCollidableNode::onAdd( n );
        if( n ) {
          ArticulatedRigidBody *arb = static_cast< ArticulatedRigidBody* >( getOwner() );
          ProxyRigidBody *frb = new ProxyRigidBody();
          frb->geometry->push_back( n );
          arb->proxyBodies->push_back( frb, arb->id );
          }
        }

      virtual void onRemove( Node *n ) {
        if( n ) {
          ArticulatedRigidBody *arb = static_cast< ArticulatedRigidBody* >( getOwner() );
          ProxyRigidBody* deleteData = NULL;
          const NodeVector& fakeBodies= arb->proxyBodies->getValue();            
          for ( size_t i=0; i < fakeBodies.size(); ++i ){
            ProxyRigidBody* frb = static_cast<ProxyRigidBody*>(fakeBodies[i]);
            if( frb->geometry->getValue()[0] == n ){
              deleteData = frb;
              break;
            }
          }
          if ( deleteData ){
            arb->proxyBodies->erase( deleteData, arb->id );
          }
        }
        MFCollidableNode::onRemove( n );        
      }
    };

    typedef TypedMFNode < ProxyRigidBody > MFProxyRigidBody;

    /// Field class to update position/orientation of each proxy body when the
    /// positions/orientations of ArticulatedRigidBody is updated.
    ///
    /// routes_in[0] - The positions of the articulated body.
    /// routes_in[1] - The orientations of the articulated body.
    class UpdateProxyBody : public PeriodicUpdate< TypedField< Field, Types< MFVec3f, MFRotation > > > {
    protected:
      virtual void update();
    };


    /// Constructor.
    ArticulatedRigidBody(
      Inst< SFNode > _metadata = 0,
      Inst< SFFloat > _angularDampingFactor = 0,
      Inst< SFVec3f > _angularVelocity = 0,
      Inst< SFBool > _autoDamp = 0,
      Inst< SFBool > _autoDisable = 0,
      Inst< SFVec3f > _centerOfMass = 0,
      Inst< SFFloat > _disableAngularSpeed = 0,
      Inst< SFFloat > _disableLinearSpeed = 0,
      Inst< SFFloat > _disableTime = 0,
      Inst< SFBool > _enabled = 0,
      Inst< SFVec3f > _finiteRotationAxis = 0,
      Inst< SFBool > _fixed = 0,
      Inst< MFVec3f > _forces = 0,
      Inst< MFArticulatedCollidables > _geometry = 0,
      Inst< SFMatrix3f > _inertia = 0,
      Inst< SFFloat > _linearDampingFactor = 0,
      Inst< SFVec3f > _linearVelocity = 0,
      Inst< SFFloat > _mass = 0,
      Inst< SFNode > _massDensityModel = 0,
      Inst< SFRotation > _orientation = 0,
      Inst< SFVec3f > _position = 0,
      Inst< MFVec3f > _torques = 0,
      Inst< SFBool > _useFiniteRotation = 0,
      Inst< SFBool > _useGlobalGravity = 0,
      Inst< SFBool > _kinematicControl = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0,
      Inst< MFRotation > _orientations = 0,
      Inst< MFVec3f > _positions = 0,
      Inst< MFProxyRigidBody  > _proxyBodies = 0,
      Inst< SFInt32  > _maxProjectionIterations = 0,
      Inst< SFFloat  > _separationTolerance = 0,
      Inst< SFFloat  > _jointInternalCompliance = 0,
      Inst< SFFloat  > _jointExternalCompliance = 0 );

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Render the collidables of the body.
    virtual void renderCollidable( bool render_only_enabled_collidables );

    /// The orientation and field is used to set the initial 
    /// conditions of this body's location in world space. After the initial
    /// conditions have been set, these fields are used to report the current
    /// information based on the most recent physics model evaluation. Setting
    /// new values will cause the objects to be moved to the new location 
    /// for the start of the next evaluation cycle. Care should be
    /// used in manually changing the orientation as the underlying
    /// physics models may cache information between time step evaluations 
    /// and sudden instantaneous changes may lead to numerical instability.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Rotation( 0, 0, 1, 0 )
    ///
    /// \dotfile ArticulatedRigidBody_orientations.dot
    auto_ptr< MFRotation > orientations;

    /// The position field is used to set the initial 
    /// conditions of this body's location in world space. After the initial
    /// conditions have been set, these fields are used to report the current
    /// information based on the most recent physics model evaluation. Setting
    /// new values will cause the objects to be moved to the new location 
    /// for the start of the next evaluation cycle. Care should be
    /// used in manually changing the position as the underlying
    /// physics models may cache information between time step evaluations 
    /// and sudden instantaneous changes may lead to numerical instability.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile ArticulatedRigidBody_positions.dot
    auto_ptr< MFVec3f > positions;

    /// The rigid proxyBodies. 
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile ArticulatedRigidBody_proxyBodies.dot
    auto_ptr< MFProxyRigidBody  > proxyBodies;

    /// The maximum number of iterations to run projection
    /// on the articulation to bring the links back together if the
    /// separation tolerance is exceeded.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 4
    ///
    /// \dotfile ArticulatedRigidBody_maxProjectionIterations.dot
    auto_ptr< SFInt32 > maxProjectionIterations;

    /// The maximum allowed separation of any joint in the articulation
    /// before projection is used
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.1
    ///
    /// \dotfile ArticulatedRigidBody_separationTolerance.dot
    auto_ptr< SFFloat > separationTolerance;

    /// Compliance determines the extent to which the joint resists acceleration.
    ///
    /// There are separate values for resistance to accelerations caused by external
    /// forces such as gravity and contact forces, and internal forces generated
    /// from other joints.
    ///
    /// A low compliance means that forces have little effect, a compliance of 1 means
    /// the joint does not resist such forces at all.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0
    ///
    /// \dotfile ArticulatedRigidBody_jointInternalCompliance.dot
    auto_ptr< SFFloat > jointInternalCompliance;

    /// Compliance determines the extent to which the joint resists acceleration.
    ///
    /// There are separate values for resistance to accelerations caused by external
    /// forces such as gravity and contact forces, and internal forces generated
    /// from other joints.
    ///
    /// A low compliance means that forces have little effect, a compliance of 1 means
    /// the joint does not resist such forces at all.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0
    ///
    /// \dotfile ArticulatedRigidBody_jointExternalCompliance.dot
    auto_ptr< SFFloat > jointExternalCompliance;

    /// Updates the proxy bodies
    auto_ptr< UpdateProxyBody > updateProxyBodies;

    /// Initialize the rigid body for the given PhysicsEngineThread. I.e. 
    /// create a new rigid body in the physics engine with the parameters
    /// of the rigid body fields. Returns 0 on success.
    //virtual bool initializeBody( H3D::PhysicsEngineThread& pt );

    /// Deletes this rigid body node from the given PhysicsEngineThread.
    //virtual bool deleteBody();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new concrete instance of H3DSoftBodyNodeParameters appropriate for this subtype of H3DSoftBodyNode
    ///
    /// For this node it returns new instance of SoftBodyParameters
    virtual PhysicsEngineParameters::RigidBodyParameters *createRigidBodyParameters();

    /// Returns a RigidBodyParameter to describe the rigid body. By default
    /// the function returns a RigidBodyParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::RigidBodyParameters *getRigidBodyParameters( bool all_params = false );

    bool proxy_bodies_initialized;

    void initializeProxyBodies( H3D::PhysicsEngineThread& pt );

  };
}
#endif
