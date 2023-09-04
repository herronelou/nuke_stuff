// WildcardCopy.cpp
// Copyright (c) 2022 Erwan Leroy

static const char* const RCLASS = "WildcardCopy";

static const char* const HELP = "Copy channels from the A input based on wildcard expressions.";

/* Remove channels from the input. This is really simple to implement
   as it does nothing except change the info so the channels are not
   there.
 */

#include "DDImage/PixelIop.h"
#include "DDImage/Row.h"
#include "DDImage/Knobs.h"
#include "DDImage/IopInfo.h"
#include <iostream>
#include <regex>

using namespace DD::Image;

class WildcardCopy : public PixelIop
{
    std::string pattern = "";
    bool regex_mode;
    bool copy_existing;

public:
    void _validate(bool) override;
    WildcardCopy(Node* node) : PixelIop(node)
    {
        inputs(2);
    }
    void in_channels(int input, ChannelSet& mask) const;
    void pixel_engine(const Row& in, int y, int x, int r, ChannelMask, Row& out);
    void knobs(Knob_Callback) override;
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    OpHints opHints() const override;
    static Iop::Description d;
private:
    const std::regex escape{ R"([.^$()\[\]{}+\\])" };  // Any of: . ^ $ ( ) [ ] { } + and backslash
};

void WildcardCopy::_validate(bool for_real)
{
    // Remove all spaces from pattern
    size_t pos = pattern.find(' ');
    while (pos != std::string::npos)
    {
        pattern.erase(pos, 1);
        pos = pattern.find(' ');
    }

    copy_info();

    // Skip on empty patterns
    if (pattern.empty())
    {
        //std::cout << "Early Exit" << std::endl;
        set_out_channels(Mask_None); // Tell Nuke we didn't touch anything.
        return;
    }

    if (!regex_mode) {
        //std::cout << "Wildcard mode"<< std::endl;
        // Convert the Wildcard format to a Regex
        pattern = std::regex_replace(pattern, escape, R"(\$&)");
        pattern = std::regex_replace(pattern, std::regex(R"(\*)"), R"(.*)");  // * means any number of any characters
        pattern = std::regex_replace(pattern, std::regex(R"(\?)"), R"(.)");  // ? means one instance of any character
        pattern = std::regex_replace(pattern, std::regex(R"(#)"), R"(\d)");  // # means one instance of a digit
    }

    //std::cout << "pattern is: " << pattern << std::endl;

    ChannelSet current_chans = info_.channels();
    ChannelSet matching_chans = Mask_None;

    // Compile Regex
    std::regex regex_pattern;
    try {
        regex_pattern = std::regex(pattern, std::regex::icase);
    }
    catch (std::regex_error& e) {
        error(e.what());
        return;
    }

    // Get input 1 channels
    input(1)->validate(for_real);
    Info input_info = input1().info();
    ChannelSet input_chans = input_info.channels();
    if (!copy_existing) {
        input_chans -= current_chans;  // We don't want to copy the channels we already have ?? TODO: Expose as a control?
    }
    if (!input_chans) {
        set_out_channels(Mask_None); // Tell Nuke we didn't touch anything.
        return;
    }
    
    // Compare all channels
    for (Channel channel = input_chans.first(); channel; channel = input_chans.next(channel)) {
        //std::cout << getName(channel) << std::endl;
        if (std::regex_match(getName(channel), regex_pattern)) {
            // std::cout << "Match!!!" << std::endl;
            matching_chans += channel;
        }
    }

    // Merge info
    merge_info(1, matching_chans);
    set_out_channels(matching_chans);
}

void WildcardCopy::in_channels(int input, ChannelSet& mask) const {
    //mask is unchanged
}

void WildcardCopy::pixel_engine(const Row& in, int y, int x, int r, ChannelMask channels, Row& out) {
    // get the colors from the A input:
    input1().get(y, x, r, channels, out);
  }

void WildcardCopy::knobs(Knob_Callback f)
{
    // Pattern knob to type the regex or pattern
    String_knob(f, &pattern, "pattern"); 
    Tooltip(f, "Pattern to use for choosing which channels to keep or remove.\n"
        "In normal mode, use \"?\" to match any character, \"#\" to match any digit character and \"*\" to match multiple characters.\n"
        "In Regex Mode, all symbols supported by C++ ECMAScript regexes are supported.\n"
        "Pattern is case-insensitive. Multiple patterns can be joined with a \"|\" character.");
    Bool_knob(f, &regex_mode, "regex_mode", "Use Regex");
    Tooltip(f, "Use Regex syntax.\nIf not checked, simple wildcard syntax will be used.");
    // Add a dropdown to bypass certain channels 
    Divider(f);
    Bool_knob(f, &copy_existing, "copy_existing", "Copy channels already in B");
    Tooltip(f, "By default, channels that are already in the B input will be left untouched.\n"
        "Check this box is you want to copy all the channels matching the pattern from A, even if the channel already exists in B.");
}

OpHints WildcardCopy::opHints() const
{
    return OpHints::eChainable;
}

static Iop* build(Node* node) { return new WildcardCopy(node); }
Iop::Description WildcardCopy::d(RCLASS, "Channels/WildcardCopy", build);
