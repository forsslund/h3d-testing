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
/// \file DualQuaternion.h
/// \brief Header file for DualQuaternion.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __DUALQUATERNION_H__
#define __DUALQUATERNION_H__

#include <H3DUtil/H3DUtil.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <H3DUtil/H3DMath.h>
#include <H3DUtil/Quaternion.h>

namespace H3DUtil {
  namespace ArithmeticTypes {
    // forward declarations.
    class Matrix4f;
    class Matrix4d;
    class DualQuaterniond;
    class Quaterniond;
    class Rotation;

    /// Quaternion describes an arbitrary rotation.
    /// \ingroup H3DUtilBasicTypes
    class H3DUTIL_API DualQuaternion {
    public:
      /// Default constructor.
      DualQuaternion() {}

      /// Constructor. 
      DualQuaternion( const Quaternion &_q0,
                      const Quaternion &_qe ) :
        q0( _q0 ), qe( _qe ){}

      
      /// Constructor.
      explicit DualQuaternion( const Matrix4f &m );
      
      /// Constructor. From DualQuaterniond.
      explicit DualQuaternion( const DualQuaterniond &r );

      explicit DualQuaternion( const Matrix4d &m );

      /*      /// Returns the DualQuaternion norm.
      inline H3DFloat norm() {
        return w*w + v.dotProduct( v );
      }

            /// Normalize the DualQuaternion, i.e. scale it so that the magnitude
      /// is 1.
      inline void normalize() {
        H3DFloat n = norm();
        if (H3DAbs( n ) > Constants::f_epsilon ) {
          H3DFloat length = H3DSqrt( n );
          v = v / length;
          w = w / length;
        }
        }*/
            
      /// Returns the conjugate of the DualQuaternion.
      inline DualQuaternion conjugate() const {
        return DualQuaternion( q0.conjugate(), qe.conjugate() );
      }

      /// Returns the inverse of the DualQuaternion.
      inline DualQuaternion inverse();
      
      /// The DualQuaternion q0 part (rotation).
      Quaternion q0;
      /// The DualQuaternion qe part (translation).
      Quaternion qe;
    };

    /// \defgroup DualQuaternionOperators DualQuaternion operators.
    /// \brief Operators on DualQuaternion instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup H3DUtilBasicTypes
    /// \{

    /// Function for printing a DualQuaternion to an ostream.
    inline std::ostream& operator<<( std::ostream &os, const DualQuaternion &q ) {
      os << q.q0 << " " << q.qe;
      return os;
    }

    /// Equality between two DualQuaternion instances.
    inline bool operator==( const DualQuaternion &q1, const DualQuaternion &q2 ) {
      return q1.q0 == q2.q0 && q1.qe == q2.qe;
    }

    /*
    /// Multiplication of DualQuaternions. If q1 and q2 are unit quaternions, 
    /// then return value will also be a unit quaternion.
    inline DualQuaternion operator*( const DualQuaternion &q1, 
                                     const DualQuaternion &q2 ) {    
      return DualQuaternion( q1.w * q2.v + q2.w * q1.v + q1.v % q2.v,
                         q1.w * q2.w - q1.v * q2.v );
    }
    */
    /// Multiplication by a double.
    inline DualQuaternion operator*( const DualQuaternion &q, 
                                     double d ) {    
      return DualQuaternion( d *q.q0, d * q.qe );
    }

    /// Multiplication by a float.
    inline DualQuaternion operator*( const DualQuaternion &q, 
                                     float d ) {    
      return DualQuaternion( d *q.q0, d * q.qe );
    }

    /// Multiplication by an int.
    inline DualQuaternion operator*( const DualQuaternion &q, 
                                     int d ) {    
      return DualQuaternion( d *q.q0, d * q.qe );
    }

    /// Multiplication by a long.
    inline DualQuaternion operator*( const DualQuaternion &q, 
                                     long d ) {    
      return DualQuaternion( d *q.q0, d * q.qe );
    }

    /// Multiplication with float.
    inline DualQuaternion operator*( const float &a, 
                                     const DualQuaternion &b ) { 
      return b * a;
    }

    /// Multiplication with double.
    inline DualQuaternion operator*( const double &a, 
                                 const DualQuaternion &b ) { 
      return b * a;
    }

    /// Multiplication with int.
    inline DualQuaternion operator*( const int &a, 
                                 const DualQuaternion &b ) { 
      return b * a;
    }

    /// Multiplication with long.
    inline DualQuaternion operator*( const long &a, 
                                 const DualQuaternion &b ) { 
      return b * a;
    }

    /// Addition of DualQuaternions. The result is not necessarily a unit 
    /// quaternion even if  and  are unit quaternion
    inline DualQuaternion operator+( const DualQuaternion &q1, 
                                     const DualQuaternion &q2 ) {
      return DualQuaternion( q1.q0 + q2.q0,
                             q1.qe + q2.qe );
    }
    /// \}

    /// Unary minus.
    inline DualQuaternion operator-( const DualQuaternion &q ) { return q * -1; }
    
    /// Subtraction between two DualQuaternions. The result is not necessarily a unit 
    /// quaternion even if a and b are unit quaternion.
    inline DualQuaternion operator-( const DualQuaternion &a, const DualQuaternion &b ) { 
      return a + (-b); 
    }

    /// \}
    /*
    // Returns the inverse of the DualQuaternion.
    inline DualQuaternion DualQuaternion::inverse() {
      H3DFloat n = norm();
      if ( H3DAbs(n ) < Constants::f_epsilon ) {
        return DualQuaternion(0,0,0,0);
      } else {
        return conjugate() / n;
      }
    }*/
    typedef DualQuaternion DualQuaternionf;
  }
}

#endif
