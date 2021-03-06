//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2008, SenseGraphics AB
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
/// \file CartoonVolumeStyle_FragmentShader.glsl
/// \brief shader file for CartoonVolumeStyle, not used by default.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#define eps 1.0e-6

// UNIFORMS - X3DVolumeRenderStyleNode
uniform sampler3D voxels;
uniform mat4 textureMatrix;
uniform mat4 textureMatrixInverse;
uniform float rayStep;

// UNIFORMS - ToneMappedVolumeStyle
uniform sampler3D surfaceNormals;
uniform vec4 parallelColor;
uniform vec4 orthogonalColor;
uniform int colorSteps;

// VARYINGS - from X3DRenderStyleNode_VertexShader
varying vec3 rayStart;
varying vec3 rayDir;

// FUNCTIONS
// compute t such that r0+t*dir is the exit point of the [0,1] box
float rayexit(vec3 r0, vec3 dir) {
  vec3 q = 1.0/(dir+eps);
  vec3 t0 = -r0*q;
  vec3 tmax = max(t0,q+t0);
  return min(tmax.x,min(tmax.y,tmax.z));
}

// struct to return from traverseRay, 
// the resulting color and the point to use for depth calculation 
struct RayResult {
  vec4 color;
  vec4 zpoint;
};

// traverse ray
// Inputs:
//   r0  - ray start (xyz), ray exit time (w)
//   dir - ray direction (xyz), step length (-w)
// Output:
//   RayResult.color  - the computed RGBA color for this fragment
//   RayResult.zpoint - the point to use for depth calculations,
//                      if zpoint.x<0, no depth is computed
RayResult traverseRay(vec4 r0, vec4 dir) {
  // return value
  RayResult rr;
  
  // initial color of this fragment is black with zero alpha
  rr.color = vec4(0.0, 0.0, 0.0, 0.0);

  // the depth is entry point
  rr.zpoint = vec4(r0.xyz,1.0);
  
  mat4 tex_to_view =  gl_ModelViewMatrix*textureMatrixInverse;

  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {

    // values
    float sample = texture3D(voxels, r0.xyz).r;
    
    vec4 col = orthogonalColor;
    
    if( colorSteps >= 1 ) {
      // sample position in view coordinates
      vec3 sample_pos = (tex_to_view * vec4(r0.xyz,1.0)).xyz;

      // view vector
      vec3 viewv = normalize(-sample_pos);

      // the normal at the sample point  
      // normal vector (0-0.5 -> -1.0-0.0 and 0.5-1.0 -> 0.0-1.0) 
      vec3 normal = 2.0*texture3D(surfaceNormals,r0.xyz).xyz-1.0;
      normal.y = -normal.y;		
      normal.z = -normal.z;     
      normal = normalize( gl_NormalMatrix * normal );     
    
      float cos_a = abs( dot( normal, viewv ) );

      float step = 1.0 / float(colorSteps);

      float interval = floor( cos_a / step );
      if( interval >= float(colorSteps) ) interval = float(colorSteps) - 1.0;

      float w = interval * step;
    
      col = orthogonalColor * w + parallelColor * (1.0-w );
      // 2 -> 0-0.5-1           0.5
      // 3 -> 0-0.33-0.66-1     0.33
      // 4 -> 0-0.25-0.5-0.75-1 0.25
   }

    vec4 src = vec4( col.xyz, sample );
    src.rgb *= src.a;
    rr.color += (1.0-rr.color.a)*src;

    // step forward along ray  
    r0 += dir;
  }
  
  // return result 
  return rr;
}

// main function
void main() {
  // initialize ray
  vec4 raydir = vec4(normalize(rayDir), -1.0);
  float texit = rayexit(rayStart,raydir.xyz);
  raydir = rayStep*raydir; 
  vec4 raypos = vec4(rayStart, texit);  
  
  // traverse ray
  RayResult rr = traverseRay(raypos, raydir);
  
  // set color
  gl_FragColor = rr.color;
  
  // set depth if computed
  if( rr.zpoint.x>=0.0 ) {
    // set depth
    vec4 tmp =
      gl_ModelViewProjectionMatrix*textureMatrixInverse*rr.zpoint;
    // in window coordinates
    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +
			gl_DepthRange.near+gl_DepthRange.far);
  }
}

