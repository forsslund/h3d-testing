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
/// \file CompositeGeometryNode.cpp
/// \brief Source file for CompositeGeometryNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/CompositeGeometryNode.h>
#include <H3D/Normal.h>
#include <H3D/Coordinate.h>

using namespace H3D;  

H3DNodeDatabase CompositeGeometryNode::database( "CompositeGeometryNode", 
                                                &(newInstance<CompositeGeometryNode>), 
                                                typeid( CompositeGeometryNode ),
                                                &X3DComposedGeometryNode::database);

namespace CompositeGeometryNodeInternals {
  FIELDDB_ELEMENT( CompositeGeometryNode, childGeometries, INPUT_OUTPUT );
}

CompositeGeometryNode::CompositeGeometryNode ( Inst< SFNode           > _metadata,
                                              Inst< SFBound          > _bound,
                                              Inst< DisplayList      > _displayList,
                                              Inst< SFColorNode      > _color,
                                              Inst< SFCoordinateNode > _coord,
                                              Inst< SFNormalNode     > _normal,
                                              Inst< SFTextureCoordinateNode > _texCoord,
                                              Inst< SFBool           > _ccw,
                                              Inst< SFBool           > _colorPerVertex,
                                              Inst< SFBool           > _normalPerVertex,
                                              Inst< SFBool           > _solid,
                                              Inst< MFVertexAttributeNode > _attrib,
                                              Inst< SFFogCoordinate     > _fogCoord,
                                              Inst< MFX3DGeometryNode   > _childGeometries,
                                              Inst< UpdateChildGeometries> _updateChildGeometries ) :
X3DComposedGeometryNode(  _metadata, _bound, _displayList, 
                        _color, _coord, _normal, _texCoord, 
                        _ccw, _colorPerVertex, _normalPerVertex,
                        _solid, _attrib, _fogCoord ),
                        childGeometries ( _childGeometries ),
                        updateChildGeometries ( _updateChildGeometries )                                      
{
  // init fields
  type_name = "CompositeGeometryNode";
  database.initFields( this );

  updateChildGeometries->setName( "updateChildGeometries" );
  updateChildGeometries->setOwner( this );

  coord->route( updateChildGeometries );
  //coord->route( bound );

  //updateChildGeometries->setValue ( true );
}

void CompositeGeometryNode::render ()
{
  for( std::vector<H3D::Transform*>::iterator i = childTransforms.begin();
    i != childTransforms.end(); ++i) {
      (*i)->render();
  }
}
void CompositeGeometryNode::initialize()
{
  childTransforms.clear();
  Coordinate *coords = dynamic_cast< Coordinate * >(coord->getValue());
  const NodeVector& geos = childGeometries->getValue();

  if( coords && geos[0] ) {

    std::vector<Vec3f> c_points = coords->point->getValue();
    if( c_points.size() == geos.size() ) {

      for( unsigned int i = 0; i<geos.size(); ++i) {

        // WARNINGUMUT: TODO: Implement.
        //H3D::Transform *t = new H3D::Transform();
        //t->addChildren->setValue( dynamic_cast< X3DChildNode* >(geos[i]));
        //t->translation->setValue( c_points.at(i) );
        //childTransforms.push_back( *t );
      }
    }
    else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The childGeometries and points of the coord field must have the same size." << endl;    
    }
  }
  else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }
}
void CompositeGeometryNode::UpdateChildGeometries::update() {

  Coordinate *coords = 
    dynamic_cast< Coordinate * >( static_cast< SFCoordinateNode * >
    ( routes_in[0] )->getValue() );

  if( coords )
  {
    CompositeGeometryNode *geoNode = 
      static_cast< CompositeGeometryNode * >( getOwner() );
    const NodeVector& geos = geoNode->childGeometries->getValue();
    std::vector<Vec3f> c_points = coords->point->getValue();

    if( c_points.size() == geos.size() ) {

      for( unsigned int i = 0; i<geos.size(); ++i) {
        geoNode->childTransforms.at( i )->translation->setValue( c_points.at(i) );

      }

    }
    else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The childGeometries and points of the coord field must have the same size." << endl;    
    }

  } else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }

}
