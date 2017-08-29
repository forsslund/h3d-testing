//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file H3DTimer.h
/// \brief Header for H3DTimer class.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DTIMER_H__
#define __H3DTIMER_H__

#include <sstream>
#include <H3DUtil/H3DUtil.h>

#ifdef HAVE_PROFILER
#include <sofa/helper/AdvancedTimer.h>

namespace H3DUtil{

  /// This is basically a wrapper class around sofa::helper::AdvancedTimer.
  /// Currently it does not add any new features to sofa::helper::AdvancedTimer
  /// but there might be some new ones in the future.
  class H3DUTIL_API H3DTimer: protected sofa::helper::AdvancedTimer {

  public:
    /// Constructor
    H3DTimer() : AdvancedTimer(  ) {}
    
    static void stepBegin(const std::string & idStr, const std::string &group = "GENERAL");
    static void stepNext(const std::string & idStr, const std::string & next, const std::string &group = "GENERAL" );
    static void stepEnd  (const std::string & idStr);
    static void begin(const std::string & id);
    static bool isEnabled(const std::string & id);
    static void setEnabled(const std::string & id, bool val);
    static void setInterval(const std::string & id, int val);
    static int  getInterval(const std::string & id);

    /// Call this function when ending profiling section.
    /// Call start function when starting a profiling section.
    static void end(const std::string & id, std::stringstream& profiledResult);
  };
}
#endif

// Macros for convenience.
//
// Using these macros means that client code can
// compile with and without the profiler enabled 
// and will not incur any overhead when it is disabled.
#ifdef HAVE_PROFILER
#define H3DTIMER_BEGIN(...)      H3DUtil::H3DTimer::stepBegin ( __VA_ARGS__ );
#define H3DTIMER_END(name)        H3DUtil::H3DTimer::stepEnd ( name );
#define H3DTIMER_NEXT(prev,next)  H3DUtil::H3DTimer::stepNext ( prev, next );
#else
#define H3DTIMER_BEGIN(...)
#define H3DTIMER_END(name)
#define H3DTIMER_NEXT(prev,next)
#endif

#endif
