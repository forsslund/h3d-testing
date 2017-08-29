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
/// \file MedX3DDemoMainDialog.h
/// \brief Header file for MedX3DDemoMainDialog. Subclass of MainDialog, which
/// is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MedX3DDemoMainDialog__
#define __MedX3DDemoMainDialog__
#include <H3D/Image3DTexture.h>
#include <H3D/MedX3D/VolumeData.h>
#include <H3D/MedX3D/SegmentedVolumeData.h>
#include <H3D/MedX3D/ISOSurfaceVolumeData.h>

#include "MedX3DDemo.h"

/** Implementing MainDialog */
class MedX3DDemoMainDialog : public MainDialog
{
protected:
  // Handlers for MainDialog events.
  void OnCloseDialog( wxCloseEvent& event );
  void OnIdle( wxIdleEvent& event );
  void OnLoadButton( wxCommandEvent& event );
  void OnLoadRawButton( wxCommandEvent& event );
  void OnSaveAsNrrdButton( wxCommandEvent& event );
  void OnToothDataRangeMinChange( wxCommandEvent& event );
  void OnToothDataRangeMaxChange( wxCommandEvent& event );
  void OnRenderStyleChoice( wxCommandEvent& event );

  void OnDataFilterChoice( wxCommandEvent& event );
  void OnRendererChoice( wxCommandEvent& event );
  void OnBackgroundColorChanged( wxColourPickerEvent& event );
  void OnRayStep( wxCommandEvent& event );
  void OnUseEmptySpaceSkipping( wxCommandEvent& event );
  void OnShowNonEmptySpace( wxCommandEvent& event );
  void OnEmptySpaceResolution( wxCommandEvent& event );
  void OnStopRaysAtGeom( wxCommandEvent& event );
  void OnUseStochasticJittering( wxCommandEvent& event );
  void OnNrSlices( wxCommandEvent& event );
  void OnSaveAsX3DButton( wxCommandEvent& event );
  void OnVolumeNodeChoice( wxCommandEvent& event );
  void OnShowStyleEditor( wxCommandEvent& event );  

  void OnLoadSegmentDataButton( wxCommandEvent& event );
  void OnSegmentChoice( wxCommandEvent& event );
  void OnSegmentStyleChoice( wxCommandEvent& event );
  void OnSegmentEnabledCheck( wxCommandEvent& event );
  void OnIsoValueChange( wxCommandEvent& event );
  void OnIsoSurfaceChoice( wxCommandEvent& event );
  void OnIsoSurfaceStyleChoice( wxCommandEvent& event );
  void OnContourStepSizeChange( wxCommandEvent& event );
  void OnSurfaceToleranceChange( wxCommandEvent& event );

  wxSizer *current_render_style_sizer;

  void initializeAvailableRenderStyleList();

  std::vector< wxString > iso_surface_styles;

  // std::vector with the name of the rendering style and if the
  // segment is enabled.
  std::vector< std::pair< wxString, bool > > segment_styles;


public:
  /** Constructor */
  MedX3DDemoMainDialog( wxWindow* parent );

  void showVolumeInfo( bool v );
  void showStyleOptions( wxSizer *sizer, bool v );
  void setDensityDataVolumeInfo( H3D::Image *i );
  void updateSelectedRenderStyle();
  void updateIsoSurfaceRenderStyles();
  void updateSegmentRenderStyles();
  void deleteRenderStyle( const wxString &name );
  void addRenderStyle( const wxString &name );
  

  H3D::VolumeData *newVolumeDataFromOptions();
  H3D::SegmentedVolumeData *newSegmentedVolumeDataFromOptions();
  H3D::ISOSurfaceVolumeData *newIsoSurfaceVolumeDataFromOptions();

  void copyCommonParameters( H3D::X3DVolumeNode *dest, 
                             H3D::X3DVolumeNode *src );

  float getRayStep();
  void setRayStep( float step );

  unsigned int getNrSlices();
  void setNrSlices( unsigned nr_slices );

  

};

#endif // __MedX3DDemoMainDialog__
