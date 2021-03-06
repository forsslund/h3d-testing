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
/// \file H3DPluginActiveXCtrlMisc.cpp
/// \brief CPP file generated by Microsoft Visual Studio template.
///
//
//////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "H3DPluginActiveX.h"
#include "H3DPluginActiveXCtrl.h"

/**
 * Most parts of the ActiveX specifics is in this file
 */

IMPLEMENT_DYNCREATE(CH3DPluginActiveXCtrl, COleControl)

// expose IObjectSafety
BEGIN_INTERFACE_MAP(CH3DPluginActiveXCtrl, COleControl)
  INTERFACE_PART(CH3DPluginActiveXCtrl, IID_IObjectSafety, ObjectSafety)
END_INTERFACE_MAP()

// Dispatch map
BEGIN_DISPATCH_MAP(CH3DPluginActiveXCtrl, COleControl)
  DISP_STOCKPROP_READYSTATE()
END_DISPATCH_MAP()

// Event map
BEGIN_EVENT_MAP(CH3DPluginActiveXCtrl, COleControl)
  EVENT_STOCK_READYSTATECHANGE()
END_EVENT_MAP()

// Initialize class factory and guid
// {51EEAA22-604D-4b28-9FD3-98EF05FDDEA0}
IMPLEMENT_OLECREATE_EX(CH3DPluginActiveXCtrl,
                       "H3DPluginActiveX.H3DPluginActiveXCtrl.1",
  0x51eeaa22, 0x604d, 0x4b28, 0x9f, 0xd3, 0x98, 0xef, 0x5, 0xfd, 0xde, 0xa0)

// Type library ID and version
IMPLEMENT_OLETYPELIB(CH3DPluginActiveXCtrl, _tlid, _wVerMajor, _wVerMinor)

// Control type information
static const DWORD BASED_CODE _dwH3DPluginActiveXOleMisc =
  OLEMISC_ACTIVATEWHENVISIBLE |
  OLEMISC_SETCLIENTSITEFIRST |
  OLEMISC_INSIDEOUT |
  OLEMISC_CANTLINKINSIDE |
  OLEMISC_RECOMPOSEONRESIZE;

IMPLEMENT_OLECTLTYPE( CH3DPluginActiveXCtrl, IDS_H3DPLUGINACTIVEX,
                      _dwH3DPluginActiveXOleMisc)


BOOL CH3DPluginActiveXCtrl::CH3DPluginActiveXCtrlFactory::
  UpdateRegistry(BOOL bRegister)
{
  if( bRegister )
    return AfxOleRegisterControlClass(
      AfxGetInstanceHandle(), 
      m_clsid, 
      m_lpszProgID,
      IDS_H3DPLUGINACTIVEX, 
      IDB_H3DPLUGINACTIVEX,
      afxRegApartmentThreading, 
      _dwH3DPluginActiveXOleMisc,
      _tlid, 
      _wVerMajor, 
      _wVerMinor );
  else
    return AfxOleUnregisterClass(m_clsid, m_lpszProgID);
}


// ---------------------------------------------------------------------------
//  implementation of IObjectSafety

const DWORD dwSupportedBits =
  INTERFACESAFE_FOR_UNTRUSTED_CALLER | INTERFACESAFE_FOR_UNTRUSTED_DATA;
const DWORD dwNotSupportedBits = ~dwSupportedBits;


STDMETHODIMP CH3DPluginActiveXCtrl::XObjectSafety::GetInterfaceSafetyOptions(
  REFIID riid, 
  DWORD __RPC_FAR *pdwSupportedOptions, 
  DWORD __RPC_FAR *pdwEnabledOptions )
{
  METHOD_PROLOGUE(CH3DPluginActiveXCtrl, ObjectSafety)

  if (!pdwSupportedOptions || !pdwEnabledOptions)
  {
    return E_POINTER;
  }
  
  HRESULT retval = ResultFromScode(S_OK);

  // Does interface exist?
  IUnknown FAR* punkInterface;
  retval = pThis->ExternalQueryInterface(&riid, (void**)&punkInterface );
  if( retval != E_NOINTERFACE ) {
    punkInterface->Release(); // release it - just checking
  }

  // We support both kinds of safety and have always both set,
  // regardless of interface.
  *pdwSupportedOptions = *pdwEnabledOptions = dwSupportedBits;

  return retval; // E_NOINTERFACE if QI failed
}

STDMETHODIMP CH3DPluginActiveXCtrl::XObjectSafety::SetInterfaceSafetyOptions(
  REFIID riid, 
  DWORD dwOptionSetMask, 
  DWORD dwEnabledOptions )
{
  METHOD_PROLOGUE(CH3DPluginActiveXCtrl, ObjectSafety)

  // Does interface exist?
  IUnknown FAR* punkInterface;
  pThis->ExternalQueryInterface(&riid, (void * *)&punkInterface);
  if (punkInterface)
  {
    punkInterface->Release(); // release it - just checking
  }
  else
  {
    return ResultFromScode(E_NOINTERFACE);
  }

  // Can't set bits we don't support.
  if( dwOptionSetMask & dwNotSupportedBits ) {
    return ResultFromScode(E_FAIL);
  }

  // Can't set bits we do support to zero
  dwEnabledOptions &= dwSupportedBits;
  // (We already know there are no extra bits in mask.)
  if( (dwOptionSetMask & dwEnabledOptions) != dwOptionSetMask )
  {
    return ResultFromScode(E_FAIL);
  }                                    

  // Don't need to change anything since we're always safe.
  return ResultFromScode(S_OK);
}

STDMETHODIMP_(ULONG) CH3DPluginActiveXCtrl::XObjectSafety::AddRef()
{
  METHOD_PROLOGUE_EX_(CH3DPluginActiveXCtrl, ObjectSafety)
  return (ULONG)pThis->ExternalAddRef();
}

STDMETHODIMP_(ULONG) CH3DPluginActiveXCtrl::XObjectSafety::Release()
{
  METHOD_PROLOGUE_EX_(CH3DPluginActiveXCtrl, ObjectSafety)
  return (ULONG)pThis->ExternalRelease();
}

STDMETHODIMP CH3DPluginActiveXCtrl::XObjectSafety::QueryInterface(
  REFIID iid, LPVOID* ppvObj)
{
  METHOD_PROLOGUE_EX_(CH3DPluginActiveXCtrl, ObjectSafety)
  return (HRESULT)pThis->ExternalQueryInterface(&iid, ppvObj);
}

// ---------------------------------------------------------------------------
