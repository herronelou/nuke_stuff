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
