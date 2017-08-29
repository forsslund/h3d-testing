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
/// \file SOFAHexmshSoftBodyLoader.h
/// \brief Header file for SOFAHexmshSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFAHEXMSHTETRALOADER__
#define __SOFAHEXMSHTETRALOADER__

#include <H3D/H3DPhysics/H3DSoftBodyLoader.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/H3DPhysics/IndexedTetraSet.h>

namespace H3D{

  /// \ingroup SoftBody
  /// Node used to load a SoftBody representation from
  /// files in Gmsh's (http://geuz.org/gmsh/) output format.
  /// The SoftBody may then be used in the scene graph or saved in X3D format.
  ///
  /// the input file is expected: to be filename.msh, which should
  /// be output from Gmsh in Gmsh's Mesh format, where 'filename.msh' is the string specified 
  /// in the filename field.
  ///
  /// This loader is based on the Gmsh file specifications, as defined at
  /// http://geuz.org/gmsh/doc/texinfo/gmsh.html#MSH-ASCII-file-format
  ///
  /// Please note only the simulation mesh is loaded not the surface geometry.
  /// Thus surfaceGeomtry will be empty and has to loaded from a X3D file.
  class H3DPHYS_API SOFAHexmshSoftBodyLoader : public H3DSoftBodyLoader {
    public:

    typedef SpecializedSFNode < IndexedTetraSet, SFX3DGeometryNode > SFIndexedTetraSet;
    typedef SpecializedMFNode < X3DComposedGeometryNode, MFX3DGeometryNode > MFX3DComposedGeometryNode;

    /// Constructor.
    SOFAHexmshSoftBodyLoader(
                        Inst< SFNode            > _metadata       = 0,
                        Inst< SFString          > _output         = 0,
                        Inst< SFX3DGeometryNode > _geometry       = 0,
                        Inst< MFX3DComposedGeometryNode > _surfaceGeometry = 0,
                        Inst< MFX3DNBodyCollidableNode > _collisionGeometry = 0,
                        Inst< SFBool            > _success        = 0,
                        Inst< MFString          >  _url           = 0 );

    /// Initialize the node
    ///
    /// Override to load the SoftBody geometry from file
    virtual void initialize();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns true if the specified line is a valid line of input
    /// and not a comment
    bool validLine ( const string& line );

    /// Return the next logical input line from the specified input stream
    /// skipping invalid lines and comments
    string getGmshLine ( istream& input );

    /// Loads the *.node, *.ele and *.face Tetgen files with the specified base filename
    /// \return true iff successful
    bool load ( const vector< string > &urls );

    /// Loads the coordinates/nodes from the *.node file
    /// \return true iff successful
    bool loadNodes ( istream& input );

    /// Loads the tetrahedra from the *.topo file
    /// \return true iff successful
    bool loadTetra ( istream& input );

    /// Loads a transform matrix
    /// \return true iff successful
    bool loadTransform ( istream& input );

    /// Creates the suitable type of X3DComposedGeometry node for the concrete of the loader. 
    virtual X3DGeometryNode* createGeometry();

    /// Creates the suitable type of X3DGeometry nodes for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DGeometryNodeList* createSurfaceGeometry();

    /// Creates the suitable type of X3DNBodyCollidableNode's for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DNBodyCollidableNodeList* createCollisionGeometry();

    /// The list of surface geometries.
    H3DSoftBodyNode::X3DGeometryNodeList sgList;

    /// The list of collidable nodes.
    H3DSoftBodyNode::X3DNBodyCollidableNodeList cnList;

    Matrix4f transformMatrix;

  };

}
#endif
