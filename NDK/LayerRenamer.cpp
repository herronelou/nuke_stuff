// LayerRenamer.cpp
// Copyright (c) 2023 Erwan Leroy

static const char* const RCLASS = "LayerRenamer";

static const char* const HELP = "Renamer layers, optionally using a regex pattern.";

/* Rename layers from your input, controlled by a Table.
 */

#include "DDImage/NoIop.h"
#include "DDImage/Knobs.h"
#include "DDImage/TableKnobI.h"
#include "DDImage/Row.h"
#include <iostream>
#include <unordered_map>
#include <regex>


// Mini class to create a Bidirectional mapping for my layers
class BidirectionalMap {
public:
    void clear() {
        keyToValueMap.clear();
        valueToKeyMap.clear();
    }

    void insert(const std::string& key, const std::string& value) {
        // Check if the key already exists. If it does, but the value differs, throw an error.
        if (keyToValueMap.find(key) != keyToValueMap.end()) {
            std::string existingValue = getValue(key);
            if (value != existingValue)
            {
                throw std::invalid_argument(key + " 'layer is already mapped to " + existingValue);
            }
            return;
        }

        // Check if the value already exists
        if (valueToKeyMap.find(value) != valueToKeyMap.end()) {
            std::string existingKey = getKey(value);
            throw std::invalid_argument("Cannot map layer " + key + " to " + value + " as it is already used by layer " + existingKey);
        }

        keyToValueMap[key] = value;
        valueToKeyMap[value] = key;
    }

    // Get Value from Key
    const std::string& getValue(const std::string& key) const {
        // Check if the key exists
        if (keyToValueMap.find(key) == keyToValueMap.end()) {
            throw std::out_of_range("Key not found in the map.");
        }

        return keyToValueMap.at(key);
    }

    // Get Key from Value
    const std::string& getKey(const std::string& value) const {
        // Check if the value exists
        if (valueToKeyMap.find(value) == valueToKeyMap.end()) {
            throw std::out_of_range("Value not found in the map.");
        }

        return valueToKeyMap.at(value);
    }

private:
    std::unordered_map<std::string, std::string> keyToValueMap;
    std::unordered_map<std::string, std::string> valueToKeyMap;
};


using namespace DD::Image;

class LayerRenamer : public NoIop
{
    BidirectionalMap mapping;
public:
    void engine(int y, int x, int r, ChannelMask channels, Row& row) override;
    void _request(int x, int y, int r, int t, ChannelMask channels, int count) override;
    void _validate(bool) override;
    LayerRenamer(Node* node) : NoIop(node) {}
    void append(Hash& hash) override;
    void knobs(Knob_Callback) override;
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    OpHints opHints() const override;
    static Iop::Description d;
private:
    const std::regex validator{ R"([A-Za-z_][A-Za-z0-9_]*)"};
    Channel getSourceChannel(const Channel& targetChannel);
    ChannelSet getSourceChannels(const ChannelMask& channels);
};

// Function to split a const char* into two strings based on the first dot
std::pair<std::string, std::string> splitChannelName(const Channel& channel) 
{
    std::pair<std::string, std::string> result;

    const char* fullName = getName(channel);

    // Check if the input is not null
    if (fullName) {
        // Find the position of the first dot
        const char* dotPosition = std::strchr(fullName, '.');

        if (dotPosition) {
            // Calculate the length before the dot
            std::size_t lengthBeforeDot = dotPosition - fullName;

            // Assign the substring before the dot to the first string
            result.first.assign(fullName, lengthBeforeDot);

            // Assign the substring after the dot to the second string
            result.second = std::string(dotPosition + 1); // Skip the dot
        } else {
            throw std::out_of_range("No dot in input.");
        }
    }

    return result;
}

void LayerRenamer::_validate(bool for_real)
{

    copy_info();

    auto* tableKnob = knob("mapping");
    Table_KnobI* tableKnobI = tableKnob->tableKnob();

    // Skip on empty table
    if (tableKnobI->getRowCount() == 0)
    {
        set_out_channels(Mask_None); // Tell Nuke we didn't touch anything.
        return;
    }

    mapping.clear();

    ChannelSet current_chans = info_.channels();
    ChannelSet removed_chans = Mask_None;
    ChannelSet added_chans = Mask_None;

    std::vector<std::regex> compiled_patterns;

    // TODO: We need to protect against possible redguards. If we rename a layer with chans to a default layer with different chans it could be bad!
    
    // Compare all channels
    for (Channel channel = current_chans.first(); channel; channel = current_chans.next(channel)) {
        std::cout << getName(channel) << std::endl;
        // Iterate over the table
        for (int row = 0; row < tableKnobI->getRowCount(); row++)
        {
            // Get the in value regex, or compile it
            std::regex in_regex;
            try
            {
                in_regex = compiled_patterns.at(row);
            }
            catch (const std::exception&)
            {
                std::string raw_pattern = tableKnobI->getCellString(row, 1);
                try {
                    in_regex = std::regex(raw_pattern, std::regex::icase);
                }
                catch (std::regex_error& e) {
                    error(("Regex Error Row " + std::to_string(row+1) + ": " + std::string(e.what())).c_str());
                    return;
                }
                compiled_patterns.push_back(in_regex);
            }


            // If the row is not enabled, skip it. We still want to compile it so that our pre-compiled patterns array is correct
            if (!tableKnobI->getValue(row, 0))
            {
                continue;
            }
            
            std::pair<std::string, std::string> fullname = splitChannelName(channel);
            std::string layer = fullname.first;
            std::string chan = fullname.second;

            // Check if the regex matches, and skip if not
            if (!std::regex_match(layer, in_regex)) {
                continue;
            }
            // std::cout << "Match!!!" << std::endl;
            removed_chans += channel;

            std::string new_name;
            try {
                new_name = std::regex_replace(layer, in_regex, tableKnobI->getCellString(row, 2), std::regex_constants::match_continuous | std::regex_constants::format_first_only);
            }
            catch (std::regex_error& e) {
                error(("Regex Error Row " + std::to_string(row + 1) + ": " + std::string(e.what())).c_str());
                return;
            }

            if (!std::regex_match(new_name, validator)) {
                error(("Invalid Channel name: " + new_name + ". Must only use alphanumerical characters and underscores. Cannot start with a digit. (row " + std::to_string(row + 1) + ").").c_str());
                return;
            }
            
            std::cout << layer << " will be renamed to " << new_name << std::endl;
            try
            {
                mapping.insert(layer, new_name);
            }
            catch (const std::invalid_argument& e)
            {
                error(("Invalid Target Channel: " + std::string(e.what()) + " (row " + std::to_string(row + 1) + ").").c_str());
            }
            

            Channel new_chan = getChannel((new_name + "." + chan).c_str());
            added_chans += new_chan;
            
            break; // This channel is processed, no need to continue iterating our table
        }
        
        
    }
    
    info_.turn_off(removed_chans);
    info_.turn_on(added_chans);
    set_out_channels(removed_chans + added_chans);

}


void LayerRenamer::knobs(Knob_Callback f)
{
    Knob* tableKnob = Table_knob(f, "mapping", "");
    Table_KnobI* tableKnobI = tableKnob->tableKnob();
    if (f.makeKnobs())
    {
        //Table_KnobI* tableKnobI = tableKnob->tableKnob();
        tableKnobI->addColumn("enabled", "e", Table_KnobI::AnimBoolColumn, true, 20);
        tableKnobI->addColumn("old_name", "Old Name", Table_KnobI::StringColumn, true, 140);
        tableKnobI->addColumn("new_name", "New Name", Table_KnobI::StringColumn, true, 140);
        tableKnobI->setEditingWidgetFlags(tableKnobI->AddRowWidget | tableKnobI->DeleteRowsWidget);
    }
    Tooltip(f, "On the left, indicate the channel to rename, and on the right its new name.\n"
               "You may use regex patterns to rename multiple layers at once, you may use capture groups in your pattern, and reference back to them using $1, $2, etc... "
               "referring to 1st capture group, second group, etc...\n\n"
               "Table is processed top to bottom, so if a layer matches multiple entires, the first one encountered will be used.\n"
               "You may not rename multiple layers to the same name, as each layer can only receive the channels from a single source (there is no merge operation performed).");
}

void LayerRenamer::append(Hash& hash)
{
    // Table knob doesn't do its own hashing, so implement it here.
    auto* tableKnob = knob("mapping");
    Table_KnobI* tableKnobI = tableKnob->tableKnob();
    for (int i = 0; i < tableKnobI->getRowCount(); i++)
    {
        hash.append(tableKnobI->getValue(i, 0));
        hash.append(tableKnobI->getCellString(i, 1));
        hash.append(tableKnobI->getCellString(i, 2));
    }
}

Channel LayerRenamer::getSourceChannel(const Channel& targetChannel)
{
    std::pair<std::string, std::string> fullname = splitChannelName(targetChannel);
    std::string layerName = fullname.first;
    std::string channelName = fullname.second;

    try
    {
        std::string desired_layer = mapping.getKey(layerName);
        Channel source_channel = getChannel((desired_layer + "." + channelName).c_str());
        return source_channel;
    }
    catch (const std::out_of_range&)
    {
        return targetChannel;
    }
}

ChannelSet LayerRenamer::getSourceChannels(const ChannelMask& channels)
{
    ChannelSet desired_channels = Mask_None;
    foreach(channel, channels) {
        desired_channels += getSourceChannel(channel);
    }
    return desired_channels;
}

void LayerRenamer::_request(int x, int y, int r, int t, ChannelMask channels, int count)
{
    ChannelSet desired_channels = getSourceChannels(channels);
    input0().request(x, y, r, t, desired_channels, count);
}

void LayerRenamer::engine(int y, int x, int r, ChannelMask channels, Row& row)
{
    // ChannelSet desired_channels = getSourceChannels(channels);
    input0().get(y, x, r, channels, row);

    Row source_row(x, r);
    foreach(channel, channels) {
        Channel source_channel = getSourceChannel(channel);
        if (channel == source_channel)
        {
            continue; // Unmodified channel
        }
        

        input0().get(y, x, r, ChannelSet(source_channel), source_row);
        row.pre_copy(channel, source_row, source_channel);
        row.copy(channel, source_row, source_channel, x, r);
    }
}

OpHints LayerRenamer::opHints() const
{
    return OpHints::eChainable;
}

static Iop* build(Node* node) { return new LayerRenamer(node); }
Iop::Description LayerRenamer::d(RCLASS, "Channel/LayerRenamer", build);
