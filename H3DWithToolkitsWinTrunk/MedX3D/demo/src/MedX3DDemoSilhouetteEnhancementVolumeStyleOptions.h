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
/// \file MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.h
/// \brief Header file for MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.
/// Subclass of SilhouetteEnhancementVolumeStyleOptions, which is generated by
/// wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MedX3DDemoSilhouetteEnhancementVolumeStyleOptions__
#define __MedX3DDemoSilhouetteEnhancementVolumeStyleOptions__

#include <H3D/MedX3D/SilhouetteEnhancementVolumeStyle.h>

#include "MedX3DDemo.h"

/** Implementing SilhouetteEnhancementVolumeStyleOptions */
class MedX3DDemoSilhouetteEnhancementVolumeStyleOptions : public SilhouetteEnhancementVolumeStyleOptions
{
protected:
  // Handlers for SilhouetteEnhancementVolumeStyleOptions events.
  void OnBoundaryOpacityChanged( wxCommandEvent& event );
  void OnSharpnessChanged( wxCommandEvent& event );
  void OnRetainedOpacityScroll( wxScrollEvent& event );
  
  H3DUtil::AutoRef< H3D::SilhouetteEnhancementVolumeStyle > silhouette_style;
public:
  /** Constructor */
  MedX3DDemoSilhouetteEnhancementVolumeStyleOptions( wxWindow* parent,
                                                     H3D::SilhouetteEnhancementVolumeStyle *style );
};

#endif // __MedX3DDemoSilhouetteEnhancementVolumeStyleOptions__
