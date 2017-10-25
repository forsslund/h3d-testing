//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
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
/// \file H3DPluginActiveXCtrl.h
/// \brief Header file for CH3DPluginActiveXCtrl. File generated by Visual
/// Studio templates and then modified. Handle messages from window.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPLUGIN_ACTIVEX_CTRL_H
#define __H3DPLUGIN_ACTIVEX_CTRL_H

#include <string>

// exposing IObjectSafety
#include <objsafe.h>

#include "../H3DPluginInstance.h"


/// Web player ActiveX control.
class CH3DPluginActiveXCtrl : public COleControl
{
  DECLARE_DYNCREATE(CH3DPluginActiveXCtrl)
public:
  CH3DPluginActiveXCtrl();

  virtual void DoPropExchange(CPropExchange* pPX);
  virtual void OnResetState();
  virtual DWORD GetControlFlags();
  void setSizeOfContainedWindow( int cx, int cy );

  void setControlClass( bool _is_control_class );

private:
  void  render();

  H3DPluginInstance  m_InstanceThread;
  CString        m_src;
  bool        m_hasSrc;

  // Constants containing command ranges to be able to assign OnCommand
  // handlers
  enum {
    ID_NAVIGATION_MENU     = 40000,
    ID_NAVIGATION_MENU_MAX = 40099,  
    ID_VIEWPOINT_MENU      = 40100,
    ID_VIEWPOINT_MENU_MAX  = 40199
  };

protected:
  ~CH3DPluginActiveXCtrl();

  bool scene_root_removed;
  bool is_control_class;
  static list< CH3DPluginActiveXCtrl * > paxct_instances;

  DECLARE_OLECREATE_EX(CH3DPluginActiveXCtrl)    // Class factory and guid
  DECLARE_OLETYPELIB(CH3DPluginActiveXCtrl)    // GetTypeInfo
  DECLARE_OLECTLTYPE(CH3DPluginActiveXCtrl)    // Type name and misc status

  DECLARE_MESSAGE_MAP()

  // expose IObjectSafety
  BEGIN_INTERFACE_PART(ObjectSafety, IObjectSafety)
    STDMETHOD(GetInterfaceSafetyOptions)(REFIID riid,
                                         DWORD __RPC_FAR *pdwSupportedOptions,
                                         DWORD __RPC_FAR *pdwEnabledOptions);
    STDMETHOD(SetInterfaceSafetyOptions)(REFIID riid, DWORD dwOptionSetMask,
                                         DWORD dwEnabledOptions);
  END_INTERFACE_PART(ObjectSafety);
  DECLARE_INTERFACE_MAP();

  DECLARE_DISPATCH_MAP()

  DECLARE_EVENT_MAP()

public:
  // Dispatch and event IDs
  enum {
    dispidData = 1L,
  };

  // Message handlers
  int OnCreate( LPCREATESTRUCT lpCreateStruct );
  afx_msg void OnDestroy();
  afx_msg void OnTimer( UINT_PTR event );
  afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
  afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
  afx_msg void OnMouseMove(UINT nFlags, CPoint point);
  afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
  afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
  afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
  afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
  afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
  afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
  afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
  afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
  afx_msg void OnNavigationMenu( UINT nID );
  afx_msg void OnViewpointMenu( UINT nID );
  afx_msg void OnSize( UINT nType, int cx, int cy );
};

#endif