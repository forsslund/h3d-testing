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
/// \file MedX3DDemoOpacityMapVolumeStyleOptions.h
/// \brief Header file for MedX3DDemoOpacityMapVolumeStyleOptions. Subclass of
/// OpacityMapVolumeStyleOptions, which is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MedX3DDemoOpacityMapVolumeStyleOptions__
#define __MedX3DDemoOpacityMapVolumeStyleOptions__

#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/WindowFunctionTexture.h>
#include <H3D/Image3DTexture.h>

#include "MedX3DDemo.h"

/** Implementing OpacityMapVolumeStyleOptions */
class MedX3DDemoOpacityMapVolumeStyleOptions : public OpacityMapVolumeStyleOptions
{
protected:
  H3DUtil::AutoRef< H3D::OpacityMapVolumeStyle > opacity_map_style;
  void OnTransferFunctionChoice( wxCommandEvent& event );
  void OnLoadImageButton( wxCommandEvent& event );
  void OnWindowCenterScroll( wxScrollEvent& event );
  void OnWindowWidthScroll( wxScrollEvent& event );
  void OnTypeChoice( wxCommandEvent& event );

  bool setTransferFunctionMode( wxString mode );

  H3DUtil::AutoRef< H3D::WindowFunctionTexture > window_texture;
  H3DUtil::AutoRef< H3D::Image3DTexture        > file_texture;
public:
  /** Constructor */
  MedX3DDemoOpacityMapVolumeStyleOptions( wxWindow* parent,
                                          H3D::OpacityMapVolumeStyle *style );
};

#endif // __MedX3DDemoOpacityMapVolumeStyleOptions__
