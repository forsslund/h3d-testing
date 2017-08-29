//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file MedX3DDemoStyleDialog.h
/// \brief Header file for MedX3DDemoStyleDialog. Subclass of StyleDialog,
/// which is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MedX3DDemoStyleDialog__
#define __MedX3DDemoStyleDialog__

#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/ToneMappedVolumeStyle.h>

#include "MedX3DDemo.h"

/** Implementing StyleDialog */
class MedX3DDemoStyleDialog : public StyleDialog
{
protected:
  // Handlers for StyleDialog events.
  void OnRenderStyleChoice( wxCommandEvent& event );
  void OnNewStyleButton( wxCommandEvent& event );
  void OnDeleteStyleButton( wxCommandEvent& event );
  void OnSaveStyleButton( wxCommandEvent& event );
  void OnStyleTypeChoice( wxCommandEvent& event );

  void initializeAvailableRenderStyleList();
  wxPanel *current_render_style_sizer;
  
  void setStyleOptions( H3D::X3DVolumeRenderStyleNode *style ); 

   // Show a dialog asking if style changes should be changed. Returns false
  // if the user clicked cancel to abort, true otherwise.
  bool doSaveStyleChangesDialog();

  wxString selected_style;
public:
  /** Constructor */
  MedX3DDemoStyleDialog( wxWindow* parent );

  bool isStyleModified();
  void setStyleModified( bool modified );
  
  wxString getSelectedStyle() { return selected_style; }

  bool setSelectedStyleName( const wxString &name );
};

#endif // __MedX3DDemoStyleDialog__
