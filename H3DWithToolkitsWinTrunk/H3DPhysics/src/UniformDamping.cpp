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
/// \file UniformDamping.cpp
/// \brief Source file for UniformDamping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/UniformDamping.h>

using namespace H3D;

H3DNodeDatabase UniformDamping::database( "UniformDamping", 
                                         &(newInstance< UniformDamping >), 
                                         typeid( UniformDamping ),
                                         &H3DPhysicsDampingNode::database);

UniformDamping::UniformDamping(
                               Inst< SFNode > _metadata,
                               Inst< ValueUpdater  > _valueUpdater,
                               Inst< SFString > _unitType,
                               Inst< SFDamping > _damping ):
H3DPhysicsDampingNode( _metadata, _valueUpdater, _unitType, _damping ){

  type_name = "UniformDamping";
  database.initFields( this );
}
PhysicsEngineParameters::MaterialPropertyParameters* UniformDamping::createMaterialPropertyParameters() {
  return new PhysicsEngineParameters::DampingParameters();
}
PhysicsEngineParameters::MaterialPropertyParameters* UniformDamping::getMaterialPropertyParameters( bool all_params ){

  PhysicsEngineParameters::DampingParameters *params= 
    static_cast<PhysicsEngineParameters::DampingParameters*>(H3DPhysicsDampingNode::getMaterialPropertyParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( damping ) ) {
    params->setValue( damping->getValue() );
  }

  return params;
}