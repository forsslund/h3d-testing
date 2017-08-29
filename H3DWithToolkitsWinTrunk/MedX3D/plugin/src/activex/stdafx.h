#ifndef __H3DPLUGIN_STDAFX_H
#define __H3DPLUGIN_STDAFX_H

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN    // Exclude rarely-used stuff from Windows headers
#endif

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX 1

// NOTE 0x0501 == Windows XP or later.
// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#ifndef WINVER        // Allow use of features specific to Windows 95 and Windows NT 4 or later.
#define WINVER 0x0501    // Change this to the appropriate value to target Windows 98 and Windows 2000 or later.
#endif

#ifndef _WIN32_WINNT    // Allow use of features specific to Windows NT 4 or later.
#define _WIN32_WINNT 0x0501  // Change this to the appropriate value to target Windows 2000 or later.
#endif            

#ifndef _WIN32_WINDOWS    // Allow use of features specific to Windows 98 or later.
#define _WIN32_WINDOWS 0x0501 // Change this to the appropriate value to target Windows Me or later.
#endif

#ifndef _WIN32_IE      // Allow use of features specific to IE 4.0 or later.
#define _WIN32_IE 0x0501  // Change this to the appropriate value to target IE 5.0 or later.
#endif

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS  // some CString constructors will be explicit

#include <afxctl.h>         // MFC support for ActiveX Controls
#include <afxext.h>         // MFC extensions
#include <afxdtctl.h>    // MFC support for Internet Explorer 4 Comon Controls
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>      // MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT


#include "../CommonInclude.h"

#endif
