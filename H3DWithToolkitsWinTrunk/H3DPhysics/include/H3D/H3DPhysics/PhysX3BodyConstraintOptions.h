//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
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
/// \file PhysX3BodyConstraintOptions.h
/// \brief Header file for PhysX3BodyConstraintOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSX3BODYCONSTRAINTOPTIONS__
#define __PHYSX3BODYCONSTRAINTOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFInt32.h>
#include <H3D/SFVec2f.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3BodyConstrainParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3BodyConstrainParameters : public EngineOptionParameters {
      /// Constructor
      PhysX3BodyConstrainParameters () {}
    };

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3JointParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3JointParameters : public PhysX3BodyConstrainParameters {
      /// Constructor
      PhysX3JointParameters() : PhysX3BodyConstrainParameters() {}
    };

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3JointParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3Joint6DOFLimitParameters : public PhysX3JointParameters {
      /// Constructor
      PhysX3Joint6DOFLimitParameters() :
        linear_x( Vec2f() ), linear_y( Vec2f() ), linear_z( Vec2f() ),
        angular_x( Vec2f() ), angular_y( Vec2f() ), angular_z( Vec2f() ),
        linear_spring( Vec2f() ), angular_spring( Vec2f() ),
        projection_tolerance( Vec2f() ), PhysX3JointParameters() {}

      // 'set' functions

      void setLinearX ( Vec2f _linear_x ) {
        update_bit_mask|= LINEAR_X;
        linear_x = _linear_x;
      }

      void setLinearY ( Vec2f _linear_y ) {
        update_bit_mask|= LINEAR_Y;
        linear_y = _linear_y;
      }

      void setLinearZ ( Vec2f _linear_z ) {
        update_bit_mask|= LINEAR_Z;
        linear_z = _linear_z;
      }

      void setAngularX ( Vec2f _angular_x ) {
        update_bit_mask|= ANGULAR_X;
        angular_x = _angular_x;
      }

      void setAngularY ( Vec2f _angular_y ) {
        update_bit_mask|= ANGULAR_Y;
        angular_y = _angular_y;
      }

      void setAngularZ ( Vec2f _angular_z ) {
        update_bit_mask|= ANGULAR_Z;
        angular_z = _angular_z;
      }

      void setLinearSpring ( Vec2f _linear_spring ) {
        update_bit_mask|= LINEAR_SPRING;
        linear_spring = _linear_spring;
      }

      void setAngularSpring ( Vec2f _angular_spring ) {
        update_bit_mask|= ANGULAR_SPRING;
        angular_spring = _angular_spring;
      }

      void setProjectionTolerance ( Vec2f _projection_tolerance ) {
        update_bit_mask|= PROJECTION_TOLERANCE;
        projection_tolerance = _projection_tolerance;
      }

      void setBreakForce ( Vec2f _break_force ) {
        update_bit_mask|= BREAK_FORCE;
        break_force = _break_force;
      }

      void setContactDistance ( Vec2f _contact_distance ) {
        update_bit_mask|= CONTACT_DISTANCE;
        contact_distance = _contact_distance;
      }

      // 'get' functions

      Vec2f getLinearX () {
        return linear_x;
      }

      Vec2f getLinearY () {
        return linear_y;
      }

      Vec2f getLinearZ () {
        return linear_z;
      }

      Vec2f getAngularX () {
        return angular_x;
      }

      Vec2f getAngularY () {
        return angular_y;
      }

      Vec2f getAngularZ () {
        return angular_z;
      }

      Vec2f getLinearSpring () {
        return linear_spring;
      }

      Vec2f getAngularSpring () {
        return angular_spring;
      }

      Vec2f getProjectionTolerance () {
        return projection_tolerance;
      }

      Vec2f getBreakForce() {
        return break_force;
      }
      
      Vec2f getContactDistance() {
        return contact_distance;
      }

      // 'have' functions

      bool haveLinearX() {
        return (update_bit_mask & LINEAR_X) != 0;
      }

      bool haveLinearY() {
        return (update_bit_mask & LINEAR_Y) != 0;
      }

      bool haveLinearZ() {
        return (update_bit_mask & LINEAR_Z) != 0;
      }

      bool haveAngularX() {
        return (update_bit_mask & ANGULAR_X) != 0;
      }

      bool haveAngularY() {
        return (update_bit_mask & ANGULAR_Y) != 0;
      }

      bool haveAngularZ() {
        return (update_bit_mask & ANGULAR_Z) != 0;
      }

      bool haveLinearSpring() {
        return (update_bit_mask & LINEAR_SPRING) != 0;
      }

      bool haveAngularSpring() {
        return (update_bit_mask & ANGULAR_SPRING) != 0;
      }

      bool haveProjectionTolerance() {
        return (update_bit_mask & PROJECTION_TOLERANCE) != 0;
      }

      bool haveBreakForce() {
        return (update_bit_mask & BREAK_FORCE) != 0;
      }

      bool haveContactDistance() {
        return (update_bit_mask & CONTACT_DISTANCE) != 0;
      }

      protected:
        // update bit mask flags
        static const unsigned int LINEAR_X            = 0x0001;
        static const unsigned int LINEAR_Y            = 0x0002;
        static const unsigned int LINEAR_Z            = 0x0004;
        static const unsigned int ANGULAR_X           = 0x0008;
        static const unsigned int ANGULAR_Y           = 0x0010;
        static const unsigned int ANGULAR_Z           = 0x0020;
        static const unsigned int LINEAR_SPRING       = 0x0040;
        static const unsigned int ANGULAR_SPRING      = 0x0080;
        static const unsigned int PROJECTION_TOLERANCE= 0x0100;
        static const unsigned int BREAK_FORCE         = 0x0200;
        static const unsigned int CONTACT_DISTANCE    = 0x0400;
        
        Vec2f linear_x;
        Vec2f linear_y;
        Vec2f linear_z;
        Vec2f angular_x;
        Vec2f angular_y;
        Vec2f angular_z;

        Vec2f linear_spring;
        Vec2f angular_spring;
        Vec2f projection_tolerance;
        Vec2f break_force;
        Vec2f contact_distance;

    };


    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3JointParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3SliderJointParameters : public PhysX3JointParameters {
      /// Constructor
      PhysX3SliderJointParameters() : explicitAnchorPoint( Vec3f() ),
        body1Offset( Vec3f() ), body2Offset( Vec3f() ), body2ForceScale( 1.0 ),
        forceType("eForce"),  PhysX3JointParameters() {}


      // 'set' functions

      void setExplicitAnchorPoint ( Vec3f _explicitAnchorPoint ) {
        update_bit_mask|= EXPLICIT_ANCHOR_POINT;
        explicitAnchorPoint = _explicitAnchorPoint;
      }

      void setBody1Offset ( Vec3f _body1Offset ) {
        update_bit_mask|= BODY1_OFFSET;
        body1Offset = _body1Offset;
      }

      void setBody2Offset ( Vec3f _body2Offset ) {
        update_bit_mask|= BODY2_OFFSET;
        body2Offset = _body2Offset;
      }

      void setBody2ForceScale ( float _forceScale ) {
        update_bit_mask|= BODY2_FORCESCALE;
        body2ForceScale = _forceScale;
      }

      void setForceType ( string _forceType ) {
        update_bit_mask|= FORCETYPE;
        forceType = _forceType;
      }

      // 'get' functions

      Vec3f getExplicitAnchorPoint () {
        return explicitAnchorPoint;
      }

      Vec3f getBody1Offset () {
        return body1Offset;
      }

      Vec3f getBody2Offset () {
        return body2Offset;
      }

      H3DFloat getBody2ForceScale () {
        return body2ForceScale;
      }

      string getForceType () {
        return forceType;
      }

      // 'have' functions

      bool haveExplicitAnchorPoint() {
        return (update_bit_mask & EXPLICIT_ANCHOR_POINT) != 0;
      }

      bool haveBody1Offset () {
        return (update_bit_mask & BODY1_OFFSET) != 0;
      }

      bool haveBody2Offset () {
        return (update_bit_mask & BODY2_OFFSET) != 0;
      }

      bool haveBody2ForceScale () {
        return (update_bit_mask & BODY2_FORCESCALE) != 0;
      }

      bool haveForceType () {
        return (update_bit_mask & FORCETYPE) != 0;
      }

      protected:
        // update bit mask flags
        static const unsigned int EXPLICIT_ANCHOR_POINT   = 0x0001;
        static const unsigned int BODY1_OFFSET            = 0x0002;
        static const unsigned int BODY2_OFFSET            = 0x0004;
        static const unsigned int BODY2_FORCESCALE        = 0x0008;
        static const unsigned int FORCETYPE               = 0x0010;

        Vec3f explicitAnchorPoint;
        Vec3f body1Offset;
        Vec3f body2Offset;
        H3DFloat body2ForceScale;
        string forceType;

    };

  }

  /// \ingroup PhysX3
  /// Node used to specify options relating to a SliderJoint that are specific to
  /// the PhysX3 physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an SliderJoint node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3SliderJointOptions.dot
  class H3DPHYS_API PhysX3SliderJointOptions : public H3DEngineOptions {
  public:

    /// Constructor.
    PhysX3SliderJointOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFVec3f > _explicitAnchorPoint = 0,
      Inst< SFVec3f > _body1Offset = 0,
      Inst< SFVec3f > _body2Offset = 0,
      Inst< SFFloat > _body2ForceScale = 0,
      Inst< SFString > _forceType = 0 );

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX3"
    virtual string getEngine () {
      return "PhysX3";
    }

    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    /// 
    /// By default the anchor point for the slider joint is set to the mid
    /// point between the two bodies. One can overwrite the anchor point explicitly.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_explicitAnchorPoint.dot
    auto_ptr < SFVec3f > explicitAnchorPoint;
    
    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    ///
    /// In case the axis of slider joint is changed, position of one of the bodies
    /// can be off than desired position. This parameter is the offset which needs
    /// to be added to the current body position s.t it would end up at the desired
    /// position.
    ///
    /// The slider joint forces this movement, for high offset values jumps and
    /// instabilities might occur.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body1Offset.dot
    auto_ptr < SFVec3f > body1Offset;
    
    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    ///
    /// In case the axis of slider joint is changed, position of one of the bodies
    /// can be off than desired position. This parameter is the offset which needs
    /// to be added to the current body position s.t it would end up at the desired
    /// position.
    ///
    /// The slider joint forces this movement, for high offset values jumps and
    /// instabilities might occur.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body2Offset.dot
    auto_ptr < SFVec3f > body2Offset;
    
    // TODO: If this proves to be useful, it makes more sense to move this one to
    // sliderJoint to be used in all engines.
    /// It is used only when to apply different forces, by using sliderForce, to
    /// the bodies of the slider joint. sliderForce is applied to body1, and it is
    /// scaled before being applied to body2.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> H3DFloat(1.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body2ForceScale.dot
    auto_ptr < SFFloat > body2ForceScale;
    
    /// Determines how the sliderForce is interpreted while being applied by the engine.
    ///
    /// eFORCE,				//!< parameter has unit of mass * distance/ time^2, i.e. a force
    ///	eIMPULSE,			//!< parameter has unit of mass * distance /time
    ///	eVELOCITY_CHANGE,	//!< parameter has unit of distance / time, i.e. the effect is mass independent: a velocity change.
    ///	eACCELERATION		//!< parameter has unit of distance/ time^2, i.e. an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> string(eFORCE) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_forceType.dot
    auto_ptr < SFString > forceType;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };


  /// \ingroup PhysX3
  /// Used to set limits to the joints in different axis of 6DOF.
  ///
  /// It could be either hard limits or soft limits, controlled by a spring.
  /// axis for which the limit parameters are not set are not affected.
  ///
  /// Mostly usefeul in BallJoint and UniversalJoint.
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3Joint6DOFLimitOptions.dot
  class H3DPHYS_API PhysX3Joint6DOFLimitOptions : public H3DEngineOptions {
  public:

    /// Constructor.
    PhysX3Joint6DOFLimitOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFVec2f > _linear_x = 0,
      Inst< SFVec2f > _linear_y = 0,
      Inst< SFVec2f > _linear_z = 0,
      Inst< SFVec2f > _angular_x = 0,
      Inst< SFVec2f > _angular_y = 0,
      Inst< SFVec2f > _angular_z = 0,
      Inst< SFVec2f > _linearSpring = 0,
      Inst< SFVec2f > _angularSpring = 0,
      Inst< SFVec2f > _projectionTolerance = 0,
      Inst< SFVec2f > _breakForce = 0,
      Inst< SFVec2f > _contactDistance = 0 );

    /// Limit on the translation along x-axis, (min,max)
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_x.dot
    auto_ptr < SFVec2f > linear_x;
    
    /// Limit on the translation along y-axis, (min,max)
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_y.dot
    auto_ptr < SFVec2f > linear_y;
    
    /// Limit on the translation along z-axis, (min,max)
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_z.dot
    auto_ptr < SFVec2f > linear_z;

    /// Limit on the rotation around x-axis, (min,max)-degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_x.dot
    auto_ptr < SFVec2f > angular_x;
    
    /// Limit on the rotation around y-axis, (min,max)-degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_y.dot
    auto_ptr < SFVec2f > angular_y;
    
    /// Limit on the rotation around z-axis, (min,max)-degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_z.dot
    auto_ptr < SFVec2f > angular_z;

    /// Instead of applying a hard limit one can apply a soft limit controlled
    /// by a spring. It applies to linear limits if its value is different
    /// than 0.
    /// 
    /// x - stiffness, y - daming of the spring
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_spring.dot
    auto_ptr < SFVec2f > linearSpring;

    /// Instead of applying a hard limit one can apply a soft limit controlled
    /// by a spring. It applies to angular limits if its value is different
    /// than 0.
    /// 
    /// x - stiffness, y - daming of the spring
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_spring.dot
    auto_ptr < SFVec2f > angularSpring;

    /// ProjectionTolerance is similar to limits, however, applied to
    /// axis which are not limited but locked by the physices engine itself.
    ///
    /// For each locked degree of freedom, NOT limited, If the joint separates
    /// by more than this distance(or angle) along its locked degrees of freedom,
    /// the solver will move the bodies to close the distance.
    /// It applies to only if its values are greater than 0
    /// 
    /// x - linear tolerance, y - angular tolerance - degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(-1.0f, -1.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_projectionTolerance.dot
    auto_ptr < SFVec2f > projectionTolerance;

    /// If the force or torque are greater than this value the joint is broken.
    ///
    /// x - max force, y - max torque
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(-1.0f, -1.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_breakForce.dot
    auto_ptr < SFVec2f > breakForce;

    /// The distance from the upper or lower limit at which the limit constraint becomes
    /// active.
    ///
    /// x - linear distance, y - angular distance - degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.01f, 5.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_contactDistance.dot
    auto_ptr < SFVec2f > contactDistance;

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX3"
    virtual string getEngine () {
      return "PhysX3";
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
