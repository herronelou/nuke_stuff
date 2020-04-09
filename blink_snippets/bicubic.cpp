// Partial function for bicubic sampling. Requires pixel coordinates to be defined as uv and an input Image called texture.
// Have not found the way to pass texture as agument, Image type is not recognized.

float2 fract (float2 x) {return x-floor(x);}
float4 mix(float4 _X,float4 _Y,float _A){
  return _X*(1.0f-_A)+_Y*_A;
}

float4 cubic(float v)
{
  float4 n = float4(1.0, 2.0, 3.0, 4.0) - v;
  float4 s = n * n * n;
  float x = s.x;
  float y = s.y - 4.0 * s.x;
  float z = s.z - 4.0 * s.y + 6.0 * s.x;
  float w = 6.0 - x - y - z;
  return float4(x, y, z, w) * (1.0/6.0);
}

// Below needs to be in the kernel

float4 bicubic(float2 uv)
{
  float2 fxy = fract(uv);
  uv -= fxy;

  float4 xcubic = cubic(fxy.x);
  float4 ycubic = cubic(fxy.y);

  float4 c = float4(uv.x - .5f, uv.x + 1.5f, uv.y - .5f, uv.y + 1.5f);

  float4 s = float4(xcubic.x + xcubic.y, xcubic.z + xcubic.w, ycubic.x + ycubic.y, ycubic.z + ycubic.w);
  float4 offset = c + float4 (xcubic.y, xcubic.w, ycubic.y, ycubic.w) / s;

  float4 sample0 = bilinear(texture, offset.x, offset.z);
  float4 sample1 = bilinear(texture, offset.y, offset.z);
  float4 sample2 = bilinear(texture, offset.x, offset.w);
  float4 sample3 = bilinear(texture, offset.y, offset.w);

  float sx = s.x / (s.x + s.y);
  float sy = s.z / (s.z + s.w);

  return mix(mix(sample3, sample2, sx), mix(sample1, sample0, sx), sy);
}
