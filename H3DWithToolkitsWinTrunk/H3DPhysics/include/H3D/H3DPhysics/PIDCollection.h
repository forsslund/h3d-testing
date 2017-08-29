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
/// \file PIDCollection.h
/// \brief Header file for PIDCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PIDCOLLECTION__
#define __PIDCOLLECTION__

#include <H3D/H3DPhysics/H3DPIDNode.h>
#include <H3D/H3DPhysics/RigidBodyCollection.h>

namespace H3D{

 
  class H3DPHYS_API PIDCollection : public X3DChildNode{
  public:

	  typedef TypedMFNode< H3DPIDNode > MFJointPIDNode;
    typedef TypedSFNode< RigidBodyCollection > SFRigidBodyCollection;

    /// Constructor.
    PIDCollection( Inst< SFBool    > _enabled = 0,
                   Inst< SFNode    > _metadata = 0,
				           Inst< MFJointPIDNode        > _pids = 0,
                   Inst< SFRigidBodyCollection > _rbc = 0 );

    /// Destructor. 
    virtual ~PIDCollection();

    /// traverseSG function of this node.
    virtual void traverseSG ( H3D::TraverseInfo &ti );

	  /// Master enable for the PIDs in this collection
	  ///
    /// <b>Access type:</b> input and output
    auto_ptr< SFBool > enabled;

	  /// Collection of JointPID nodes
	  ///
    /// <b>Access type:</b> input and output
	  auto_ptr< MFJointPIDNode > pids;

    /// The RigidBodyCollection to which the joints being controlled belong to
    ///
    /// <b>Access type:</b> input and output
    auto_ptr< SFRigidBodyCollection > rbc;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    class UpdatePIDProperties : public PeriodicUpdate < Field > {
      virtual void update ();
    };

    /// Initializes for the given PhysicsEngineThread.
	  virtual void initialize ( H3D::PhysicsEngineThread& pt );

    /// Callback funtion for the physics thread to run all the PID loops at physics engine thread rate
	  static H3DUtil::PeriodicThread::CallbackCode updatePhysics ( void* data );
    
    /// The PhysicsEngineThread with which this PIDCollection is created
    H3D::PhysicsEngineThread *engine_thread;

	  /// Handle for the callback
	  int physicsCallbackId;

    /// A lock used to access the list of pid controllers and other rt properties
    MutexLock rt_pids_lock;

    /// Master enable for the PIDs in this collection shared by graphics and physics threads
    bool rt_enabled;

    vector<H3DPIDNode*> rt_pids;

    auto_ptr < UpdatePIDProperties > update_pid_properties;
  };
  
}
#endif