// Based on https://www.raywenderlich.com/100-unreal-engine-4-paint-filter-tutorial

kernel KuwaharaFilterKernel : ImageComputationKernel<ePixelWise>
{
  Image<eRead, eAccessRandom, eEdgeConstant> src; // the input image
  Image<eRead, eAccessPoint, eEdgeConstant> orientation;
  Image<eWrite> dst; // the output image

  param:
    int samples; // This parameter is made available to the user.
    float2 brush_size;
    
  local:
    float2 spacing;
    float total_samples;


  // In define(), parameters can be given labels and default values.
  void define() {
    defineParam(samples, "Samples", 6);
    defineParam(brush_size, "Brush Size", float2(10.0, 10.0));
  }
  
  void init() {
    spacing = brush_size / samples;
    total_samples = (samples + 1) * (samples + 1);
  }

  inline float2 rotate_by_vec2(float2 pos, float2 vec)
  {
    return float2(vec.x  * pos.x + vec.y * -1.f * pos.y, vec.y  * pos.x + vec.x * pos.y);
  }


  void process(int2 pos) {
    float4 vector_input = orientation();
    float2 orient_vec = float2(vector_input.x, vector_input.y);
    float2 sample_pos;
    float4 color;
    float4 mean0 = 0, mean1 = 0, mean2 = 0, mean3 = 0;
    float4 ssum0 = 0, ssum1 = 0, ssum2 = 0, ssum3 = 0; // Squared Sum for pseudo variance
    
    // Loop over the image 4 times
    for (int j = -samples; j <= 0; j++) {
        for (int i = -samples; i <= 0; i++) {
            sample_pos = rotate_by_vec2(float2(i, j)*spacing, orient_vec);
            color = bilinear(src, pos.x + sample_pos.x, pos.y + sample_pos.y);
            mean0 += color;
            ssum0 += color * color;
        } 
    }

    for (int j = -samples; j <= 0; j++) {
        for (int i = 0; i <= samples; i++) {
            sample_pos = rotate_by_vec2(float2(i, j)*spacing, orient_vec);
            color = bilinear(src, pos.x + sample_pos.x, pos.y + sample_pos.y);
            mean1 += color;
            ssum1 += color * color;
        } 
    }

    for (int j = 0; j <= samples; j++) {
        for (int i = 0; i <= samples; i++) {
            sample_pos = rotate_by_vec2(float2(i, j)*spacing, orient_vec);
            color = bilinear(src, pos.x + sample_pos.x, pos.y + sample_pos.y);
            mean2 += color;
            ssum2 += color * color;
        } 
    }

    for (int j = 0; j <= samples; j++) {
        for (int i = -samples; i <=0; i++) {
            sample_pos = rotate_by_vec2(float2(i, j)*spacing, orient_vec);
            color = bilinear(src, pos.x + sample_pos.x, pos.y + sample_pos.y);
            mean3 += color;
            ssum3 += color * color;
        } 
    }

    float variance; // Pseudo variance
    float min_variance;
    
    mean0 /= total_samples;
    ssum0 = fabs(ssum0 / total_samples - mean0 * mean0);
    variance = ssum0.x + ssum0.y + ssum0.z;
    min_variance = variance;
    dst() = mean0;
    
    mean1 /= total_samples;
    ssum1 = fabs(ssum1 / total_samples - mean1 * mean1);
    variance = ssum1.x + ssum1.y + ssum1.z;
    if (variance < min_variance) {
        min_variance = variance;
        dst() = mean1;
    }
    
    mean2 /= total_samples;
    ssum2 = fabs(ssum2 / total_samples - mean2 * mean2);
    variance = ssum2.x + ssum2.y + ssum2.z;
    if (variance < min_variance) {
        min_variance = variance;
        dst() = mean2;
    }
    
    mean3 /= total_samples;
    ssum3 = fabs(ssum3 / total_samples - mean3 * mean3);
    variance = ssum3.x + ssum3.y + ssum3.z;
    if (variance < min_variance) {
        min_variance = variance;
        dst() = mean3;
    }
  }
};
