//////////////////////////////////////////////////////////////////////////////
//    Copyright 2011-2014, SenseGraphics AB
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
/// \file H3DViewerPopupMenus.h
/// \brief Header file for H3DViewerPopupMenus.
/// Subclass of MenuContainer, which is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DViewerPopupMenus__
#define __H3DViewerPopupMenus__

#include "H3DViewerTreeViewDialog.h"
#include "H3DViewerAddChildDialog.h"

class H3DViewerPopupMenus : public MenuContainer {
public:
  /// Constructor.
  H3DViewerPopupMenus( H3DViewerTreeViewDialog *_treeview_dialog,
                       wxWindow* parent, wxWindowID id = wxID_ANY, 
                       const wxString& title = wxEmptyString, 
                       const wxPoint& pos = wxDefaultPosition, 
                       const wxSize& size = wxSize( 500,300 ), 
                       long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL ):
    MenuContainer( parent,id, title, pos, size, style ),
    treeview_dialog( _treeview_dialog ) {
#ifndef HAVE_TEEM
  // If we dont have TEEM, disable that menu item
  /// \todo This is untested because RightClickMenuTexture is not used anywhere at the moment.
  int item= RightClickMenuTexture->FindItem ( wxT("Save texture image in NRRD format..") );
  if ( item != wxNOT_FOUND ) {
    RightClickMenuTexture->Enable ( item, false );
  }
#endif
  }
        
  /// Callback for collapse all menu choice.
  virtual void OnTreeViewCollapseAll( wxCommandEvent& event );

  /// Callback for expand all menu choice.
  virtual void OnTreeViewExpandAll( wxCommandEvent& event );

  /// Callback for collapse children menu choice.
  virtual void OnTreeViewCollapseChildren( wxCommandEvent& event );

  /// Callback for node watch menu choice.
  virtual void OnTreeViewNodeWatch( wxCommandEvent& event );

  /// Callback for node save x3d menu choice.
  virtual void OnTreeViewSaveX3D( wxCommandEvent& event );

  /// Callback for node lookat menu choice.
  virtual void onTreeViewLookAt( wxCommandEvent& event );

  /// Callback for node save nrrd menu choice.
  virtual void OnTreeViewSaveNrrd( wxCommandEvent& event );

   /// Callback for node save png menu choice.
  virtual void OnTreeViewSavePng( wxCommandEvent& event );

  /// Callback for node show image menu choice.
  virtual void OnTreeViewShowImage( wxCommandEvent& event );

  /// Callback for node save VRML menu choice.
  virtual void OnTreeViewSaveVRML( wxCommandEvent& event );

  /// Callback for node save stl menu choice.
  virtual void OnTreeViewSaveSTL( wxCommandEvent& event );

  /// Callback for triangle save menu choice.
  virtual void OnTreeViewSaveTrianglesX3D( wxCommandEvent& event );
  
  /// Callback for delete node menu choice.
  virtual void OnTreeViewDeleteNode( wxCommandEvent& event );
  
  /// Callback for add child menu choice.
  virtual void OnTreeViewAddChildNode( wxCommandEvent& event );
 protected:
  H3DViewerTreeViewDialog *treeview_dialog;
};
#endif // __H3DViewerTreeViewDialog__
