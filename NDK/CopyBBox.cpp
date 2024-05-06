// MergeBBox.cpp
// Copyright (c) 2024 Erwan Leroy

static const char* const RCLASS = "MergeBBox";

static const char* const HELP = "Removes BlackOutside from an image";


#include "DDImage/NoIop.h"
#include "DDImage/Knobs.h"


using namespace DD::Image;



class MergeBBox : public NoIop
{
    int operation;
public:
    MergeBBox(Node* node) : NoIop(node) 
    {
        operation = 0;
        inputs(2);
    }
    void _validate(bool) override;
    void knobs(Knob_Callback) override;
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    static Iop::Description d;
};


static const char* const enums[] = {
  "union", "intersection", "A", "B", nullptr
};

void MergeBBox::_validate(bool for_real)
{

    copy_info();
    if (operation == 3) 
    {
        // If setting Bbox to B we might as well return now
        set_out_channels(Mask_None);
        return;
    }
    set_out_channels(Mask_All);
    input(1)->validate(for_real);
    Info input_info = input1().info();
    // Based on the operation chosen, edit the BBox.
    switch (operation)
    {
    case 0:
        info_.merge(input_info);
        return;
    case 1:
        info_.intersect(input_info);
        return;
    case 2:
        info_.set(input_info);
        return;
    }
}

void MergeBBox::knobs(Knob_Callback f)
{
    Enumeration_knob(f, &operation, enums, "operation");
    Tooltip(f, "Output Bounding box. Any data outside this region is clipped off");
}


static Iop* build(Node* node) { return new MergeBBox(node); }
Iop::Description MergeBBox::d(RCLASS, "Transform/MergeBBox", build);
