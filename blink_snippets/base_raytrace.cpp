kernel BaseRayTrace : ImageComputationKernel<ePixelWise>
{
  Image<eRead, eAccessPoint, eEdgeClamped> src; // the input image
  Image<eWrite> dst; // the output image

  param:
    float2 resolution; // This parameter is made available to the user.
    float focal_length, h_aperture;
    float4x4 cam;

  void define() {
    defineParam(resolution, "resolution", float2(1920.0f, 1080.0f));
    defineParam(cam, "cam", float4x4( 1.0f,0.0f,0.0f,0.0f,
                                      0.0f,1.0f,0.0f,0.0f,
                                      0.0f,0.0f,1.0f,0.0f,
                                      0.0f,0.0f,0.0f,1.0f));
  }

  void process(int2 pos) {

    float2 stmap = float2(pos.x + 0.5f, pos.y + 0.5f)/resolution;
    float aspect = resolution.x / resolution.y;
    float2 cam_space = stmap*2-1;
    cam_space.y /= aspect;
    //cam_space = cam_space*(h_aperture/focal_length);
    float fov = h_aperture / (2.0f * focal_length);
    cam_space *= fov;
    float4 world_space = cam * float4(cam_space.x, cam_space.y, -1.0f, 1.0);
    float3 ro = float3(cam[0][3], cam[1][3], cam[1][3]); // Ray Origin
    float3 rd = normalize(float3(world_space.x, world_space.y, world_space.z)-ro); // Ray Direction
    
    // Write the result to the output image
    dst() = float4(rd.x, rd.y, rd.z, 1.0f);
  }
};
