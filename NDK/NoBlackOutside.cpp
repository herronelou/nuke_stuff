// NoBlackOutside.cpp
// Copyright (c) 2024 Erwan Leroy

static const char* const RCLASS = "NoBlackOutside";

static const char* const HELP = "Removes BlackOutside from an image";

#include "DDImage/NoIop.h"
#include "DDImage/Knobs.h"

using namespace DD::Image;

class NoBlackOutside : public NoIop
{
public:
    NoBlackOutside(Node* node) : NoIop(node){}
    void _validate(bool) override;
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    static Iop::Description d;
};

void NoBlackOutside::_validate(bool for_real)
{

    copy_info();
    // Check if back_outside was set, and unset it if it was.
    if (!info_.black_outside())
    {
        set_out_channels(Mask_None);
        return;
    }
    info_.black_outside(false);
    info_.expand(-1);
}

static Iop* build(Node* node) { return new NoBlackOutside(node); }
Iop::Description NoBlackOutside::d(RCLASS, "Transform/NoBlackOutside", build);
