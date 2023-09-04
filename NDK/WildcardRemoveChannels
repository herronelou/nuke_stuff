// WildcardRemoveChannels.cpp
// Copyright (c) 2022 Erwan Leroy

static const char* const RCLASS = "WildcardRemoveChannels";

static const char* const HELP = "Removes/keeps color channels from the image based on wildcard expressions. ";

/* Remove channels from the input. This is really simple to implement
   as it does nothing except change the info so the channels are not
   there.
 */

#include "DDImage/NoIop.h"
#include "DDImage/Knobs.h"
#include <iostream>
#include <regex>

using namespace DD::Image;

class WildcardRemoveChannels : public NoIop
{
    ChannelSet channels_to_keep;
    std::string pattern = "";
    bool regex_mode;
    int operation; // 0 = remove, 1 = keep

public:
    void _validate(bool) override;
    WildcardRemoveChannels(Node* node) : NoIop(node)
    {
        channels_to_keep = Mask_RGBA;
        operation = 0;
    }
    void knobs(Knob_Callback) override;
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    OpHints opHints() const override;
    static Iop::Description d;
private:
    const std::regex escape{ R"([.^$()\[\]{}+\\])" };  // Any of: . ^ $ ( ) [ ] { } + and backslash
};

void WildcardRemoveChannels::_validate(bool for_real)
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
    
    // Compare all channels
    for (Channel channel = current_chans.first(); channel; channel = current_chans.next(channel)) {
        //std::cout << getName(channel) << std::endl;
        if (std::regex_match(getName(channel), regex_pattern)) {
            // std::cout << "Match!!!" << std::endl;
            matching_chans += channel;
        }
    }
    

    if (operation) {  //  Keep
        if (channels_to_keep) {
            matching_chans += channels_to_keep;
        }
        info_.channels() &= matching_chans;
        set_out_channels(info_.channels());  // Should this be inverted to indicate the channels that were removed?
    }
    else {  // Remove
        if (channels_to_keep) {
            matching_chans -= channels_to_keep;
        }
        info_.turn_off(matching_chans);
        set_out_channels(matching_chans);
    }
}

static const char* const enums[] = {
  "remove", "keep", nullptr
};

void WildcardRemoveChannels::knobs(Knob_Callback f)
{
    Enumeration_knob(f, &operation, enums, "operation");
    Tooltip(f, "Remove: the named channels are deleted\n"
        "Keep: all but the named channels are deleted");
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
    Input_ChannelMask_knob(f, &channels_to_keep, 0, "channels_to_keep", "always keep");
    Tooltip(f, "Choose channels to always keep, even if the wildcard would have normally removed them.\n"
        "For example, you may want to always keep RGBA.");
}

OpHints WildcardRemoveChannels::opHints() const
{
    return OpHints::eChainable;
}

static Iop* build(Node* node) { return new WildcardRemoveChannels(node); }
Iop::Description WildcardRemoveChannels::d(RCLASS, "Color/WildcardRemoveChannels", build);
