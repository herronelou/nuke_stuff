// Some basic functions to make blinkscript a bit easier to use


// smoothstep
float smoothstep(float edge0, float edge1, float x) {
  // Scale, bias and saturate x to 0..1 range
  x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f); 
  // Evaluate polynomial
  return x * x * (3 - 2 * x);
}

// mix
float mix(float _X,float _Y,float _A){
  return _X*(1.0f-_A)+_Y*_A;
}
float3 mix(float3 _X,float3 _Y,float _A){
  return _X*(1.0f-_A)+_Y*_A;
}
float4 mix(float4 _X,float4 _Y,float _A){
  return _X*(1.0f-_A)+_Y*_A;
}

// Remap
float remap(float val, float ori_min, float ori_max, float new_min, float new_max)
{
  float ori_range = ori_max-ori_min;
  float new_range = new_max-new_min;
  return (((val - ori_min) * new_range) / ori_range) + new_min;
}

// Fract
float fract (float x) {return x-floor(x);}
float2 fract (float2 x) {return x-floor(x);}
float3 fract (float3 x) {return x-floor(x);}
float4 fract (float4 x) {return x-floor(x);}

// Random
random(float co) { return fract(sin(co*(91.3458f)) * 47453.5453f); }

// Make a quaternion out of 3 vectors, where these 3 axis vectors.
// Adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
inline float4 quaternion_from_axes(float3 x, float3 y, float3 z)
{
float tr = x.x + y.y + z.z;
float4 q = .0f;

if (tr > 0) { 
  float S = sqrt(tr+1.0f) * 2;
  q.w = 0.25 * S;
  q.x = (y.z - z.y) / S;
  q.y = (z.x - x.z) / S; 
  q.z = (x.y - y.x) / S; 
} else if ((x.x > y.y)&(x.x > z.z)) { 
  float S = sqrt(1.0f + x.x - y.y - z.z) * 2; // S=4*qx 
  q.w = (y.z - z.y) / S;
  q.x = 0.25 * S;
  q.y = (y.x + x.y) / S; 
  q.z = (z.x + x.z) / S; 
} else if (y.y > z.z) { 
  float S = sqrt(1.0f + y.y - x.x - z.z) * 2; // S=4*qy
  q.w = (z.x - x.z) / S;
  q.x = (y.x + x.y) / S; 
  q.y = 0.25 * S;
  q.z = (z.y + y.z) / S; 
} else { 
  float S = sqrt(1.0f + z.z - x.x - y.y) * 2; // S=4*qz
  q.w = (x.y - y.x) / S;
  q.x = (z.x + x.z) / S;
  q.y = (z.y + y.z) / S;
  q.z = 0.25 * S;
}
// Where most quaternions expect the format to be WXYZ, Nuke uses the order XYZW for float 4s.
q = float4(q.w, q.x, q.y, q.z);
return q;
}
