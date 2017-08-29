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
/// \file CompositeGeometryNode.h
/// \brief Header file for CompositeGeometryNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COMPOSITEGEOMETRYNODE__
#define __COMPOSITEGEOMETRYNODE__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/MFInt32.h>
#include <H3D/Transform.h>
#include <H3D/IndexedTriangleSet.h>   // Note: Include required to avoid error LNK2005 (vc8)
#include <H3D/X3DComposedGeometryNode.h>

namespace H3D{

  /// \ingroup SoftBody
  /// An X3DComposedGeometryNode which includes a set of child geometries
  /// which are X3DGeometryNodes. The coord field refers to the position
  /// of the childGeometries and the position of the childGeometries are
  /// kept up to date with the coord field. The childGeometries field can
  /// include any X3DGeometryNode such as Sphere, Box as well as IndexedFaceSet
  /// etc.
  /// \deprecated This node was never properly implemented and will be removed
  /// in the future. Use GeometryGroup from H3DAPI instead.
  /// 
  /// \par Internal routes:
  /// \dotfile CompositeGeometryNode.dot
  class H3DPHYS_API CompositeGeometryNode : public X3DComposedGeometryNode {
  public:


    /// Helper class to update the transformation of the childGeometries
    /// when the coord node is changed.
    class UpdateChildGeometries : public AutoUpdate< 
      TypedField< SFBool, Types< SFCoordinateNode > > > {
    protected:
      virtual void update();
    };

    typedef TypedMFNode< X3DGeometryNode > MFX3DGeometryNode;

    /// Constructor.
    CompositeGeometryNode(
      Inst< SFNode           > _metadata           = 0,
      Inst< SFBound          > _bound              = 0,
      Inst< DisplayList      > _displayList        = 0,
      Inst< SFColorNode      > _color              = 0,
      Inst< SFCoordinateNode > _coord              = 0,
      Inst< SFNormalNode     > _normal             = 0,
      Inst< SFTextureCoordinateNode > _texCoord    = 0,
      Inst< SFBool           > _ccw                = 0,
      Inst< SFBool           > _colorPerVertex     = 0,
      Inst< SFBool           > _normalPerVertex    = 0,
      Inst< SFBool           > _solid              = 0,
      Inst< MFVertexAttributeNode > _attrib        = 0,
      Inst< SFFogCoordinate     > _fogCoord        = 0,
      Inst< MFX3DGeometryNode   > _childGeometries = 0,
      Inst< UpdateChildGeometries> _updateChildGeometries = 0 );

    /// The list of X3DGeometries the positions of which are related
    /// with the coord field.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile CompositeGeometryNode_childGeometries.dot
    auto_ptr < MFX3DGeometryNode > childGeometries;

    /// The field updating the transformations of the childGeometries
    /// when the coord field is changed.
    /// Only accessable in C++.
    auto_ptr< UpdateChildGeometries  > updateChildGeometries;

    /// Render the children.
    virtual void render ();

    /// Adds the childGeometries to the childTransforms.
    virtual void initialize();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    typedef std::vector<H3D::Transform*> TransformList;
    TransformList childTransforms;

  };
}
#endif
