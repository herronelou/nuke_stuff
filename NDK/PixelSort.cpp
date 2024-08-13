// PixelSort.cpp
// Copyright (c) 2024 Erwan Leroy

static const char* const HELP =
"This Node sorts the pixels in an image based from brightest to darkest, or vice-versa";

#include "DDImage/Iop.h"
#include "DDImage/Row.h"
#include "DDImage/Knobs.h"
#include "DDImage/DDMath.h"

using namespace DD::Image;

// Enums

enum {
    THRESHOLD = 0, MAX_LENGTH, BOTH
};

static const char* const mask_mode_names[] = {
    "Threshold", "Max Length", "Threshold and Max Length", nullptr
};

// Definition

class PixelSortIop : public Iop
{
    double threshold;
    bool dynamic_sort_order;
    bool reverse_sort_order;
    int max_length;
    int mask_mode;

public:
    PixelSortIop(Node* node) : Iop(node)
    {
        threshold = 1.0;
        dynamic_sort_order = false;
        reverse_sort_order = false;
        max_length = 0;
    }

    void knobs(Knob_Callback f) override;
    void _validate(bool for_real) override;
    void _request(int x, int y, int r, int t, ChannelMask channels, int count) override;
    void engine(int y, int x, int r, ChannelMask channels, Row& out) override;
    void sort_and_flush(std::vector<std::array<float, 2>>& vec, float*& rOut, float*& gOut, float y);

    static const Iop::Description d;
    const char* Class() const override { return d.name; }
    const char* node_help() const override { return HELP; }
};

static Iop* build(Node* node)
{
    return new PixelSortIop(node);
}

const Iop::Description PixelSortIop::d("PixelSort", "Transform/PixelSort", build);

// Implementation

void PixelSortIop::knobs(Knob_Callback f)
{
    Double_knob(f, &threshold, IRange(0, 4), "threshold");
    Tooltip(f, "The threshold above which a new sorting zone will be started.");
    Bool_knob(f, &dynamic_sort_order, "dynamic_sort_order", "Dynamic Sort Order");
    Tooltip(f, "If Checked, the sort order will be determined by the first and last pixel of the sorting zone. If the first pixel is brighter than the last, we will sort from bright to dark, and vice-versa (or the opposite if reverse is enabled)");
    Bool_knob(f, &reverse_sort_order, "reverse_sort_order", "Reverse Sort Order");
    Tooltip(f, "If Checked, the sort order will be reversed. If the sort order is dynamic, the sort order will be reversed after the dynamic sort order has been determined.");
    Int_knob(f, &max_length, "max_length", "Max Length");
    Tooltip(f, "The maximum length of a sorting zone. If the length of a sorting zone exceeds this value, the zone will be sorted and flushed.");
    Enumeration_knob(f, &mask_mode, mask_mode_names, "mask_mode", "Mask affects");
    Tooltip(f, "If threshold is picked, the mask will act as a multiplier on the threshold value.\n"
        "If max length is picked, the mask will act as a multiplier on the max length value.\n"
        "If both is picked, the mask will act as a multiplier on both the threshold and max length values.");
}

void PixelSortIop::_validate(bool for_real)
{
    copy_info();
    info_.turn_off(Mask_All);
    info_.turn_on(Mask_Red | Mask_Green);
    set_out_channels((Mask_Red | Mask_Green));
}

void PixelSortIop::_request(int x, int y, int r, int t, ChannelMask channels, int count)
{
    input0().request(info_.box().x(), y, info_.box().r(), t, Mask_RGB, count);
}

void PixelSortIop::engine(int y, int x, int r, ChannelMask channels, Row& out)
{
    // Get our input pixels
    int real_left = info_.box().x();
    int real_right = info_.box().r();
    Row in(real_left, real_right);
    input0().get(y, real_left, real_right, Mask_RGB, in);

    // Read all the input pixels and store them in a vector
    std::vector<std::array<float, 2>> vec;
    vec.reserve(real_right - real_left); // Reserve memory to avoid repeated allocations

    // Create Output Pointers
    // Write the coordinates of the sorted pixels to the output
    out.range(real_left, real_right);
    float* rOut = out.writable(Chan_Red) + real_left;
    float* gOut = out.writable(Chan_Green) + real_left;

    float previous_zone = 0.0;
    float local_threshold = threshold;
    int local_max_length = max_length;

    for (int i = real_left; i < real_right; i++)
    {
        float value = in[Chan_Red][i];
        float zone_value = in[Chan_Green][i];
        float mask_value = in[Chan_Blue][i];
        float pos_x = float(i);

        // Apply the mask
        switch (mask_mode)
        {
        case THRESHOLD:
            local_threshold = threshold * mask_value;
            break;
        case MAX_LENGTH:
            local_max_length = max_length * mask_value;
            break;
        case BOTH:
            local_threshold = threshold * mask_value;
            local_max_length = max_length * mask_value;
            break;
        }

        if ((std::abs(zone_value - previous_zone) > local_threshold) || (max_length && vec.size() >= local_max_length))
        {
            sort_and_flush(vec, rOut, gOut, y);
        }

        vec.push_back({value, pos_x});
        previous_zone = zone_value;
    }

    sort_and_flush(vec, rOut, gOut, y);
}

void PixelSortIop::sort_and_flush(std::vector<std::array<float, 2>>& vec, float*& rOut, float*& gOut, float y)
{
    // Return if the vector is empty
    if (vec.empty())
    {
        return;
    }

    bool dark_to_bright = true;
    if (dynamic_sort_order)
    {
        // Determine the sort order
        dark_to_bright = vec[0][0] < vec.back()[0];
    }

    if (reverse_sort_order)
    {
        dark_to_bright = !dark_to_bright;
    }

    // Sort all the input pixels
    if (dark_to_bright)
    {
        std::stable_sort(vec.begin(), vec.end(), [](const std::array<float, 2>& a, const std::array<float, 2>& b)
            {
                return a[0] < b[0];
            });
    }
    else
    {
        std::stable_sort(vec.begin(), vec.end(), [](const std::array<float, 2>& a, const std::array<float, 2>& b)
            {
                return a[0] > b[0];
            });
    }

    // Write the coordinates of the sorted pixels to the output
    for (const auto& elem : vec)
    {
        *rOut++ = elem[1];
        *gOut++ = float(y);
    }

    // Empty the vector
    vec.clear();
}
