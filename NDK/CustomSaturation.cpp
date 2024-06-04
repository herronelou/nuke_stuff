// CustomSaturation.cpp
// The comments in this file are meant for people following the tutorials at https://www.erwanleroy.com
// Part 1: Compilation: https://erwanleroy.com/intro-to-writing-nuke-c-plugins-a-k-a-the-ndk-part-1-intro-compiling/
// Part 2: Architecture, knobs, validate and NoIop: https://erwanleroy.com/writing-nuke-c-plugins-a-k-a-the-ndk-part-2-architecture-the-validate-and-knobs-functions-and-first-simple-plugins/
// Part 3: Engine and pixel engine functions: https://erwanleroy.com/writing-nuke-c-plugins-a-k-a-the-ndk-part-3-engine-and-pixel-engine-functions/

static const char* const HELP =
"This Iop changes the saturation (color intensity) of the incoming "
"image data usign custom weights.";


// As in part 2, we need to include the headers for the classes we are using:
// First, the PixelIop class we are deriving from:
#include "DDImage/PixelIop.h"
// We will need the Row class to access the pixel data in our pixel_engine:
#include "DDImage/Row.h"
// We need to create knobs for the user interface:
#include "DDImage/Knobs.h"
// and of couse the cross platform math:
#include "DDImage/DDMath.h"

using namespace DD::Image;

// CustomSaturationIop is derived from PixelIop. PixelIops must implement
// the pixel_engine as their engine call, as well as validate.
// For this example, we'll implement functions directly in the class definition to keep things simple.
class CustomSaturationIop : public PixelIop
{
    // this is where the knobs store the user selected saturation:
    float saturation;

    // we also store an array of 3 floats, which will be our custom weights:
    float weights[3];

public:

    CustomSaturationIop(Node* node) : PixelIop(node)
    {
        saturation = 1.0;
        // We default to the Rec. 709 weights for the RGB to Y conversion:
        weights[0] = 0.2126f; // red
        weights[1] = 0.7152f; // green
        weights[2] = 0.0722f; // blue    
    }

    // Foundry's saturation node allows you to pick which channels to work on. Ours will only deal with RGB.
    // This function is called by Nuke to find out which channels we need to request from the input.
    // We need all three color channels, so we will turn them on if they are not already on. We also don't need any other channels so we turn them off.
    void in_channels(int input_number, ChannelSet& channels) const override
    {
        channels = Mask_RGB;
    }

    // Add our knobs to the user interface. This is called by Nuke to build the control panel.
    void knobs(Knob_Callback f) override
    {
        // Add a knob to control the saturation. This is a double knob with a range from 0 to 4.
        Float_knob(f, &saturation, IRange(0, 4), "saturation", "Saturation");
        // I also try to always add a tooltip to my knobs, as it can be very useful for the user.
        Tooltip(f, "Set the saturation of the image. 1.0 is no change, 0.0 is greyscale, 2.0 is double saturation, etc.");
        // Also add an RGB color knob to set the weights for the RGB to Y conversion:
        Color_knob(f, weights, "weights", "RGB to Y weights");
        Tooltip(f, "Set the weights for the RGB to Y conversion. The default is Rec. 709.\n"
                   "Note that weights will be normalized so that you do not need to ensure the 3 values add - up to 1.");
    }

    // Set the output channels and then call the base class validate.
    // If saturation is 1, the image won't change. By saying there are no
    // changed channels, Nuke's caching will completely skip this operator,
    // saving time. Also the GUI indicator will turn off, which
    // is useful and informative...
    void _validate(bool for_real) override
    {
        if (saturation != 1)
            set_out_channels(Mask_RGB);
        else
            set_out_channels(Mask_None);
        PixelIop::_validate(for_real);
    }

    // The pixel_engine function is called by Nuke to process the image data.
    void pixel_engine(const Row& in, int y, int x, int r, ChannelMask channels, Row& out) override
    {
        // We have told Nuke that we only need the RGB channels, by setting the out channels in _validate.
        // We also told it that we only need the RGB channels from the input in in_channels.
        // Thanks to this, we know that the in row will contain the RGB channels already filled in.
        const float* rIn = in[Chan_Red] + x;
        const float* gIn = in[Chan_Green] + x;
        const float* bIn = in[Chan_Blue] + x;

        // We want to write into the channels. This is done with a different
        // call that returns a non-const float* pointer. We must call this
        // *after* getting the in pointers into local variables. This is
        // because in and out may be the same row structure, and calling
        // these may change the pointers from const buffers (such as a cache
        // line) to allocated writable buffers:
        float* rOut = out.writable(Chan_Red) + x;
        float* gOut = out.writable(Chan_Green) + x;
        float* bOut = out.writable(Chan_Blue) + x;

        // We will need a variable to store the luma value:
        float custom_luma;
        // We also need to normalize the weights so that they add up to 1:
        // We need to be careful, if the user sets all weights to 0, we will have a division by 0, so we need to check for that.
        // As a workaround, if all the weights are zero, we consider that all channels have equal weight, and set them to 1/3.
        float n_weights[3];
        float sum_weights = weights[0] + weights[1] + weights[2];
        if (sum_weights == 0)
        {
            n_weights[0] = 1.0f / 3.0f;
            n_weights[1] = 1.0f / 3.0f;
            n_weights[2] = 1.0f / 3.0f;
        }
        else
        {
            n_weights[0] = weights[0] / sum_weights;
            n_weights[1] = weights[1] / sum_weights;
            n_weights[2] = weights[2] / sum_weights;
        }
        
        // Pointer to when the loop is done:
        const float* END = rIn + (r - x);

        // Start the loop:
        while (rIn < END) {
			// Calculate the luma value using the custom weights:
            // Note how we do not increment the input pointers here, as we will need the original values for the lerp.
			custom_luma = n_weights[0] * *rIn + n_weights[1] * *gIn + n_weights[2] * *bIn;
			// Lerp the color channels towards the luma value:
            // Here we increment the input pointers, as we are done with the original values, so that next iteration we get the next pixel.
			*rOut++ = lerp(custom_luma, *rIn++, saturation);
			*gOut++ = lerp(custom_luma, *gIn++, saturation);
			*bOut++ = lerp(custom_luma, *bIn++, saturation);
		}
    }

    // The constructor for this object tells Nuke about its existence.
    // Making this a class member and not just a static variable may be
    // necessary for some systems to call the constructor when the plugin
    // is loaded, though this appears to not be necessary for Linux or Windows.
    static const Iop::Description d;

    // let Nuke know the command name used to create this op:
    const char* Class() const override { return d.name; }

    // Provide something for the [?] button in the control panel:
    const char* node_help() const override { return HELP; }
};

// Define the build function that Nuke will use to create this operator, and register it with Nuke using the Description class.

static Iop* build(Node* node)
{
    return (new CustomSaturationIop(node));
}

const Iop::Description CustomSaturationIop::d("CustomSaturation", "Color/CustomSaturation", build);
