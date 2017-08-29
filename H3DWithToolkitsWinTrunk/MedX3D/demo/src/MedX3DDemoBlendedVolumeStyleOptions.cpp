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
/// \file MedX3DDemoBlendedVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoBlendedVolumeStyleOptions.
/// Subclass of BlendedVolumeStyleOptions, which is generated by
/// wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoBlendedVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

#include <H3D/ImageTexture.h>

using namespace H3D;

MedX3DDemoBlendedVolumeStyleOptions::MedX3DDemoBlendedVolumeStyleOptions( wxWindow* parent,
                                                                          BlendedVolumeStyle *style )
:
  BlendedVolumeStyleOptions( parent ),
  blended_style( style )
{
  
  H3DFloat weight_constant_1 = blended_style->weightConstant1->getValue(); 
  H3DFloat weight_constant_2 = blended_style->weightConstant2->getValue(); 
  string weight_function_1 = blended_style->weightFunction1->getValue();
  string weight_function_2 = blended_style->weightFunction2->getValue();

  X3DTextureNode *t1 = blended_style->weightTransferFunction1->getValue();
  X3DTextureNode *t2 = blended_style->weightTransferFunction2->getValue();

  X3DTexture3DNode *voxels = blended_style->voxels->getValue();
  X3DComposableVolumeRenderStyleNode *render_style = blended_style->renderStyle->getValue(); 

  string voxels_url = "None";
  if( Image3DTexture *t = dynamic_cast< Image3DTexture * >( voxels ) ) {
    const vector< string > &urls = t->url->getValue();
    if( urls.size() > 0 ) {
      voxels_url = urls[0];
    }
  }

  string table_1_url = "None";
  if( ImageTexture *t = dynamic_cast< ImageTexture * >( t1 ) ) {
    const vector< string > &urls = t->url->getValue();
    if( urls.size() > 0 ) {
      table_1_url = urls[0];
    }
  }

  string table_2_url = "None";
  if( ImageTexture *t = dynamic_cast< ImageTexture * >( t2 ) ) {
    const vector< string > &urls = t->url->getValue();
    if( urls.size() > 0 ) {
      table_2_url = urls[0];
    }
  }

  BlendFunction1Choice->SetStringSelection( wxString( weight_function_1.c_str(),wxConvUTF8 ) );
  BlendFunction2Choice->SetStringSelection( wxString( weight_function_2.c_str(),wxConvUTF8 ) );

  BlendConstant1Slider->SetValue( weight_constant_1 * 100 );
  stringstream wc1;
  wc1 << weight_constant_1;
  BlendConstant1Text->SetValue( wxString( wc1.str().c_str(),wxConvUTF8  ) );

  BlendConstant2Slider->SetValue( weight_constant_2 * 100 );
  stringstream wc2;
  wc2 << weight_constant_2;
  BlendConstant2Text->SetValue( wxString( wc2.str().c_str(),wxConvUTF8  ) );

  BlendTable1UrlText->SetLabel( wxString( table_1_url.c_str(),wxConvUTF8 ) );
  BlendTable2UrlText->SetLabel( wxString( table_2_url.c_str(),wxConvUTF8 ) );
  LoadedFileText->SetLabel( wxString( voxels_url.c_str(),wxConvUTF8 ) );

  // fill in available style choices
  BlendedVolumeStyleStyleChoice->Append( wxT("None") );
  wxString style_name = wxT( "None" );
  
  // the choices of volume styles are all that are composable except for blended
  // styles.
  for( MedX3DDemoApp::StyleNodeMap::iterator i = wxGetApp().style_nodes.begin();
       i != wxGetApp().style_nodes.end(); ++i ) {
    if( dynamic_cast< X3DComposableVolumeRenderStyleNode * >( (*i).second.get() ) &&
    !dynamic_cast< BlendedVolumeStyle * >((*i).second.get() ) ) {
      BlendedVolumeStyleStyleChoice->Append( (*i).first );
    }
  if( render_style == (*i).second.get() ) {
      style_name = (*i).first;
  }
  }

  BlendedVolumeStyleStyleChoice->SetStringSelection( style_name );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnLoadBlendDataButton( wxCommandEvent& event )
{
  wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    
    AutoRef< Image3DTexture > tex( new Image3DTexture );

    tex->url->push_back( string( openFileDialog->GetPath().mb_str() ) ); 
    tex->textureProperties->setValue( 
       X3D::createX3DNodeFromString( "<TextureProperties boundaryModeS=\"CLAMP_TO_EDGE\" \
                                              boundaryModeT=\"CLAMP_TO_EDGE\" \
                                              boundaryModeR=\"CLAMP_TO_EDGE\" \
                                             minificationFilter=\"AVG_PIXEL\" \
                                         magnificationFilter=\"AVG_PIXEL\" \
                     textureCompression=\"DEFAULT\"/>" ) );
    Image *i = tex->image->getValue();
    if( !i ) {
      wxMessageBox( wxT( "Could not read image" ) );
      return;
    }

    LoadedFileText->SetLabel( openFileDialog->GetPath() );
    blended_style->voxels->setValue( tex.get() );
    wxGetApp().style_dialog->setStyleModified( true );
  }
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendedVolumeStyleStyleChoice( wxCommandEvent& event )
{
  wxString selected_style = BlendedVolumeStyleStyleChoice->GetStringSelection();

  if( selected_style.IsSameAs( wxT( "None" ) ) ) {
    // nothing
    blended_style->renderStyle->setValue( NULL );
  } else { 
    blended_style->renderStyle->setValue( wxGetApp().style_nodes[ selected_style] );
  }
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendFunction1Choice( wxCommandEvent& event )
{
  wxString choice = event.GetString();
  blended_style->weightFunction1->setValue( string( choice.mb_str() ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendFunction2Choice( wxCommandEvent& event )
{
  wxString choice = event.GetString();
  blended_style->weightFunction2->setValue( string( choice.mb_str() ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendConstant1Scroll( wxScrollEvent& event )
{
  int slider_value = event.GetPosition();
  blended_style->weightConstant1->setValue( slider_value / 100.f );
  stringstream s;
  s << slider_value / 100.f; 
  BlendConstant1Text->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendConstant2Scroll( wxScrollEvent& event )
{
  int slider_value = event.GetPosition();
  blended_style->weightConstant2->setValue( slider_value / 100.f );
  stringstream s;
  s << slider_value / 100.f; 
  BlendConstant2Text->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendTable1LoadButton( wxCommandEvent& event )
{
   wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    wxString path = openFileDialog->GetPath();
    BlendTable1UrlText->SetLabel( path );
    AutoRef< ImageTexture > t( new ImageTexture );
    t->url->push_back( string( path.mb_str() ) );
    if( t->image->getValue() )
      blended_style->weightTransferFunction1->setValue( t );
    wxGetApp().style_dialog->setStyleModified( true );
  }
}

void MedX3DDemoBlendedVolumeStyleOptions::OnBlendTable2LoadButton( wxCommandEvent& event )
{
  wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    wxString path = openFileDialog->GetPath();
    BlendTable1UrlText->SetLabel( path );
    AutoRef< ImageTexture > t( new ImageTexture );
    t->url->push_back( string( path.mb_str() ) );
    if( t->image->getValue() )
      blended_style->weightTransferFunction2->setValue( t );
    wxGetApp().style_dialog->setStyleModified( true );
  }
}
