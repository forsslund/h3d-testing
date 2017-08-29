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
/// \file MedX3DDemoShadedVolumeStyleOptions.h
/// \brief Header file for MedX3DDemoShadedVolumeStyleOptions. Subclass of
/// ShadedVolumeStyleOptions, which is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MedX3DDemoShadedVolumeStyleOptions__
#define __MedX3DDemoShadedVolumeStyleOptions__

#include <H3D/MedX3D/ShadedVolumeStyle.h>

#include "MedX3DDemo.h"

#include <H3D/MedX3D/ShadedVolumeStyle.h>
#include <H3D/Material.h>

/** Implementing ShadedVolumeStyleOptions */
class MedX3DDemoShadedVolumeStyleOptions : public ShadedVolumeStyleOptions
{
protected:
  // Handlers for ShadedVolumeStyleOptions events.
  void OnShadedDiffuseColorChanged( wxColourPickerEvent& event );
  void OnShadedEmissiveColorChanged( wxColourPickerEvent& event );
  void OnShadedSpecularColorChanged( wxColourPickerEvent& event );
  void OnShadedTransparencyScroll( wxScrollEvent& event );
  void OnShadedTransparencyText( wxCommandEvent& event );
  void OnShadedAmbientIntensityScroll( wxScrollEvent& event );
  void OnShadedAmbientIntensityText( wxCommandEvent& event );
  void OnShadedLightingBox( wxCommandEvent& event );
  void OnShadedShadowsBox( wxCommandEvent& event );
  void OnShadedUseMaterialCheck( wxCommandEvent& event );


  H3DUtil::AutoRef< H3D::ShadedVolumeStyle  > shaded_volume_style;
  H3DUtil::AutoRef< H3D::Material > shaded_volume_style_material;
  
public:
  /** Constructor */
  MedX3DDemoShadedVolumeStyleOptions( wxWindow* parent,
                                      H3D::ShadedVolumeStyle *style );

  /// Set to use the Material values of the style or not. Enables/disables
  /// the UI widgets for controlling the material values and use it in the style
  /// if needed.
  void useMaterialAsColor( bool v );
};

#endif // __MedX3DDemoShadedVolumeStyleOptions__
