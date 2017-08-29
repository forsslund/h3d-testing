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
/// \file AutoRefVector.h
/// Header file for AutoRefVector class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOREFVECTOR_H__
#define __AUTOREFVECTOR_H__

#include <H3DUtil/H3DUtil.h>
#include <vector>
#include <algorithm>

namespace H3DUtil {
  /// This class is similar to the AutoRef class in the vector elements
  /// are RefCountedClass * or pointers to subclasses of RefCountedClass. Reference counting 
  /// will be upheld on all RefCountedClasses in the vector.
  /// 
  template< class RefCountedClass >
  class AutoRefVector : private std::vector<RefCountedClass*> {
  public:
    /// The type of the RefCountedClass, stored in the vector.
    typedef typename std::vector<RefCountedClass*>::value_type value_type;
    /// Pointer to RefCountedClass.
    typedef typename std::vector<RefCountedClass*>::pointer pointer;
    /// Const reference to RefCountedClass.
    typedef typename std::vector<RefCountedClass*>::const_reference const_reference;
    /// An unsigned integral type.
    typedef typename std::vector<RefCountedClass*>::size_type size_type;
    /// A signed integral type.
    typedef typename std::vector<RefCountedClass*>::difference_type difference_type; 
    /// Const iterator used to iterate through a vector.
    typedef typename std::vector<RefCountedClass*>::const_iterator const_iterator;
    /// Iterator used to iterate backwards through a vector.
    typedef typename std::vector<RefCountedClass*>::const_reverse_iterator 
    const_reverse_iterator;

    /// Creates an empty vector.
    inline AutoRefVector() {}

    /// Copy constructor from a vector class.
    inline AutoRefVector( const std::vector<RefCountedClass *> &v ) :
      std::vector<RefCountedClass*>( v ) {
      refAll();
    }

    /// Copy constructor
    inline AutoRefVector( const AutoRefVector<RefCountedClass> &v ) :
      std::vector<RefCountedClass*>( v ) {
      refAll();
    }

    /// Creates a vector with n elements.
    inline AutoRefVector( size_type n ):
      std::vector< RefCountedClass * >( n ) {}

    /// Destructor.
    inline virtual ~AutoRefVector() {
      clear();
    }

    /// Assignement operator.
    inline AutoRefVector<RefCountedClass> 
    &operator=( const AutoRefVector<RefCountedClass> &v ) {
      if( this != &v ) {
        unrefAll();      
        std::vector<RefCountedClass*>::operator=( v );
        refAll();
      }
      return *this;
    }

    /// Assignement operator.
    inline AutoRefVector<RefCountedClass> &operator=(
                                       const std::vector<RefCountedClass *> &v ) {
      // temporarily add an extra reference to the refcounted class in v so
      // they are not accidentally removed in unrefAll()
      for( typename std::vector< RefCountedClass * >::const_iterator i = v.begin();
           i != v.end();
           ++i ) 
        if(*i) (*i)->ref();
      unrefAll();
      std::vector<RefCountedClass*>::operator=( v );
      refAll();

      // remove the temporary references.
      for( typename std::vector< RefCountedClass * >::const_iterator i = v.begin();
           i != v.end();
           ++i ) 
        if(*i) (*i)->unref();
      return *this;
    }

    /// Returns a const_iterator pointing to the beginning of the vector.
    inline const_iterator begin() const { 
      return std::vector<RefCountedClass*>::begin();
    }
      
    /// Returns a const_iterator pointing to the end of the vector.
    inline const_iterator end() const { return std::vector<RefCountedClass*>::end(); }

        
    /// Returns a const_reverse_iterator pointing to the beginning of the
    /// reversed vector.
    inline const_reverse_iterator rbegin() const { 
      return std::vector<RefCountedClass*>::rbegin();
    }
      
    /// Returns a const_reverse_iterator pointing to the end of the reversed 
    /// vector.
    inline const_reverse_iterator rend() const { 
      return std::vector<RefCountedClass*>::rend(); 
    }

    /// Returns the size of the vector.
    inline size_type size() const { 
      return std::vector<RefCountedClass*>::size(); 
    }

    /// Returns the largest possible size of the vector.
    inline size_type max_size() const {
      return std::vector<RefCountedClass*>::max_size();
    }
        
    /// Number of elements for which memory has been allocated. capacity() 
    /// is always greater than or equal to size().
    inline size_type capacity() const { 
      return std::vector<RefCountedClass*>::capacity(); 
    }
        
    /// Swaps the contents of two vectors.
    inline void swap( AutoRefVector<RefCountedClass> &x ) {
      std::vector<RefCountedClass*>::swap( x );
    }

    /// Swaps the contents of two vectors.
    inline void swap( std::vector<RefCountedClass*> &x ) {
      unrefAll();
      std::vector<RefCountedClass*>::swap( x );
      refAll();
    }
        
    /// A request for allocation of additional memory. If s is less than
    /// or equal to capacity(), this call has no effect. 
    /// Otherwise, it is a request for allocation of additional memory. 
    /// If the request is successful, then capacity() is greater than or 
    /// equal to s; otherwise, capacity() is unchanged. In either case, 
    /// size() is unchanged.
    /// 
    inline void reserve( size_t s ) { std::vector<RefCountedClass*>::reserve( s ); }

    /// Inserts or erases elements at the end such that the size becomes n.
    /// Note: when n is bigger than original size, the newly added value will
    /// point to the same location, unless you want that, otherwise do not specify
    /// value t (will use NULL then) and always set the value manually after resized.
    inline virtual void resize( size_t n, RefCountedClass * t = NULL ) {
      if( size() > n ) {
        for( size_t i = n; i < size(); ++i )
          unref( std::vector<RefCountedClass*>::operator[]( i ) );
      }
      if( size() < n ) {
        for( size_t j = 0; j < n-size(); j++ ) {
          ref( t );
        }
      }
      std::vector<RefCountedClass*>::resize( n, t );
    }

    /// true if the vector's size is 0.
    inline bool empty() const { return std::vector<RefCountedClass*>::empty(); }

    /// Returns the n'th element. We return a const_reference so that
    /// the values of the vector only can be changed using member 
    /// functions. To change the value of a specific index use
    /// the set( index, value ) function.
    inline const_reference operator[](size_type n) const {
      return std::vector<RefCountedClass*>::operator[]( n );
    }

    /// Set value at index i to v.
    inline void set( size_type i, const value_type &v ) {
      if( v != std::vector<RefCountedClass*>::operator[]( i ) ) {
        unref( std::vector<RefCountedClass*>::operator[]( i ) );
        ref( v );
        std::vector<RefCountedClass*>::operator[]( i ) = v;
      }
    }

    /// Returns the first element.
    inline const_reference front() const { return std::vector<RefCountedClass*>::front();}

    /// Returns the last element.
    inline const_reference back() const { return std::vector<RefCountedClass*>::back(); }

    /// Inserts a new element at the end.
    inline void push_back( const value_type &x ) {
      ref( x );
      std::vector< RefCountedClass * >::push_back( x );
    }

    /// Removed the last element.
    void pop_back() {
      unref( back() );
      std::vector< RefCountedClass * >::pop_back();
    }
        
    /// Erases all of the elements.
    inline void clear() {
      unrefAll();
      std::vector<RefCountedClass*>::clear();
    }

    /// Erase the first element equal to a.
    inline virtual void erase( RefCountedClass *a ) {
      typename std::vector<RefCountedClass * >::iterator i = 
        std::find( std::vector<RefCountedClass*>::begin(), 
                   std::vector<RefCountedClass*>::end(), 
                   a );
      if( i != end() ) {
        unref( *i );
        std::vector<RefCountedClass*>::erase( i );
      } 
    }

    /// Insert an element before the index given by pos.
    inline virtual void insert(unsigned int pos,
                               const value_type & x) {
      ref( x );
      std::vector< RefCountedClass*>::insert( std::vector< RefCountedClass*>::begin() + pos, x );
    }

    /// Removes the element at the index pos.
    inline virtual void erase(unsigned int pos ) {
      // nop if pos is outside range.
      if( pos >= size() ) return;

      unref( std::vector<RefCountedClass*>::operator[]( pos ) );
      std::vector< RefCountedClass*>::erase( std::vector< RefCountedClass*>::begin() + pos );
    }

  protected:
    /// Virtual function that is called when a RefCountedClass is added to 
    /// the vector.
    inline virtual void ref( RefCountedClass *n ) const {
      if( n ) {
        n->ref();
      }
    }

    /// Virtual function that is called when a RefCountedClass is removed from
    /// the vector.
    inline virtual void unref( RefCountedClass *n ) const {
      if( n ) {
        n->unref();
      }
    }

    /// Call ref () on all values in the vector.
    inline void refAll() const {
      for( const_iterator i = std::vector<RefCountedClass*>::begin(); 
           i != std::vector<RefCountedClass*>::end(); ++i ) 
        ref( *i );
    }

    /// Call unref () on all values in the vector.
    inline void unrefAll() const {
      for( const_iterator i = std::vector<RefCountedClass*>::begin(); 
           i != std::vector<RefCountedClass*>::end(); ++i ) 
        unref( *i );
    }
  };
}
    
#endif
