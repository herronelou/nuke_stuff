// ChannelFlood.cpp
// Copyright (c) 2022 Erwan Leroy

static const char* const RCLASS = "ChannelFlood";

static const char* const HELP = "Shuffle channels from one layer into every layer.";

#include "DDImage/PixelIop.h"
#include "DDImage/Row.h"
#include "DDImage/Knobs.h"
#include "DDImage/IopInfo.h"
#include <iostream>

using namespace DD::Image;

class ChannelFlood : public PixelIop
{
    ChannelSet channels_to_copy;
    bool repeat;
    Channel chan_set[4];

public:
    void _validate(bool) override;
    ChannelFlood(Node* node) : PixelIop(node)
    {
        channels_to_copy = Mask_RGBA;
    }
    void knobs(Knob_Callback) override;
    void in_channels(int input, ChannelSet& mask) const;
    void pixel_engine(const Row& in, int y, int x, int r, ChannelMask, Row& out);
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    OpHints opHints() const override;
    static Iop::Description d;
};

void ChannelFlood::_validate(bool for_real)
{
    // Prepare our source channels for later
    int i = 0;
    foreach(z, channels_to_copy)
    {
        if (i < 4) chan_set[i++] = z;
    }

    // If the array wasn't fully filled, fill with either balc or the last channel
    for (i; i < 4; ++i) {
        if (repeat) chan_set[i] = channels_to_copy.last();
        else chan_set[i] = Chan_Black;
    }


    // Actual validate
    copy_info();

    ChannelSet current_chans = info_.channels();
    set_out_channels(current_chans - channels_to_copy); // Tell Nuke we're changing all current channels except the ones we're copying
    return;
}

void ChannelFlood::knobs(Knob_Callback f)
{
    Input_ChannelMask_knob(f, &channels_to_copy, 0, "channels_to_copy", "Flood source");
    Tooltip(f, "Choose channels will be copied to every other channel.");
    Bool_knob(f, &repeat, "repeat_mode", "Repeat last channel");
    Tooltip(f, "If the flood source has less channels than the layer being flooded, we can either fill it with black, "
        "or copy the last available channel. \nCheck this box to copy the last available channel.");
    //TODO, really we could also use a constant color instead of black, or we could leave the channel untouched, that would be 4 options.
}

OpHints ChannelFlood::opHints() const
{
    return OpHints::eChainable;
}

void ChannelFlood::in_channels(int input, ChannelSet& mask) const {
    mask += channels_to_copy;
}

void ChannelFlood::pixel_engine(const Row& in, int y, int l, int r, ChannelMask channels, Row& out) {
    foreach(z, channels)
    {
        int i = colourIndex(z);
        //std::cout << "sourcing" << z << "from: " << chan_set[i]  << std::endl;
        //input0().get(y, l, r, chan_set[i], out);  // This doesn't work somehow, so do it pixel by pixel
        const float* inptr = in[chan_set[i]];
        float* outptr = out.writable(z);
        for (int x = l; x < r; x++)
        {
            outptr[x] = inptr[x];
        }
    }
    
    
}

static Iop* build(Node* node) { return new ChannelFlood(node); }
Iop::Description ChannelFlood::d(RCLASS, "Channel/ChannelFlood", build);
