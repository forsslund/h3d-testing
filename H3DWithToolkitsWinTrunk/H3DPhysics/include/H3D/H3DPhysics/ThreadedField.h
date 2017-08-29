//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file H3DPhysics/ThreadedField.h
/// \brief Threaded field types
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __THREADEDFIELD_RBP__
#define __THREADEDFIELD_RBP__
#include <H3D/PeriodicUpdate.h>

namespace H3D {
  
  /// A field template which allows an SFField type to be used in non-main threads
  /// The field value can be set, and accessed from a non-main thread. Simplified
  /// routing between multiple fields of this type is supported.
  ///
  /// The following cases are supported:
  ///  * Setting the field value from any thread, using setValue()
  ///  * Routing FROM the field to any other field
  ///  * Routing TO the field from another ThreadedField
  ///  * Getting the field value from the main thread using getValue()
  ///  * Getting the field value from any thread using getValueRT()
  ///
  template < class BaseField >
  class ThreadedField : public PeriodicUpdate < BaseField > {
  public:
    typedef PeriodicUpdate < BaseField > TField;

    ThreadedField () :
      ts_route_in ( NULL ),
      ts_event_pending ( false ) {}

    virtual typename TField::value_type getValueRT () {
      this->mutex.lock();
      typename TField::value_type tmp_value= this->ts_value;
      ThreadedField < BaseField >* tmp_route_in= this->ts_route_in;
      this->mutex.unlock();

      if ( !tmp_route_in ) {
        return tmp_value;
      } else {
        return tmp_route_in->getValueRT();
      }
    }

    virtual void setValue ( const typename TField::value_type& _value, int _id= 0 ) {
      bool in_main= H3DUtil::ThreadBase::inMainThread();
      if ( in_main ) {
        TField::setValue ( _value, _id );
      } else {
        this->ts_event_pending= true;
      }
      this->mutex.lock();
      this->ts_value= _value;
      this->mutex.unlock();
    }

    virtual void upToDate() {
      assert ( H3DUtil::ThreadBase::inMainThread() );
      TField::upToDate();

      this->mutex.lock();
      bool tmp_ts_event_pending= this->ts_event_pending;
      this->ts_event_pending= false;
      typename TField::value_type tmp_value= this->ts_value;
      this->mutex.unlock();

      // If the field value was set from non-main thread, then trigger
      // an event to update values in the main thread
      if ( tmp_ts_event_pending ) {
        TField::value= tmp_value;
        // reset the event pointer since we want to ignore any pending
        // events when the field is set to a new value.
        this->event.ptr = NULL;
        // generate an event.
        this->startEvent();
      }
    }
    
  protected:
    virtual void routeFrom ( Field* f, int id= 0 ) {
      assert ( H3DUtil::ThreadBase::inMainThread() );

      TField::routeFrom ( f, id );

      if ( ThreadedField < BaseField >* tf= dynamic_cast < ThreadedField < BaseField >* > ( f ) ) {
        this->mutex.lock();
        this->ts_route_in= tf;
        this->mutex.unlock();
      } else {
        Console(4) << "WARNING: Routing a non-threaded field to a threaded field is not supported! ( " << f->getFullName() << " -> " << this->getFullName() << "). "
                      "The result is thread-safe, but no events will be propagated." << endl;
      }
    }

    virtual void unrouteFrom( Field* f ) {
      assert ( H3DUtil::ThreadBase::inMainThread() );

      TField::unrouteFrom ( f );

      this->mutex.lock();
      if ( f == this->ts_route_in ) {
        this->ts_route_in= NULL;
      }
      this->mutex.unlock();
    }

    MutexLock mutex;

    typename TField::value_type ts_value;
    ThreadedField < BaseField >* ts_route_in;
    bool ts_event_pending;
  };
}
#endif
