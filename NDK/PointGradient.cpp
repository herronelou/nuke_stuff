// PointGradient.cpp
// Copyright (c) 2023-2026 Erwan Leroy
// Modified to use custom Delaunay implementation (Removed CGAL Dependency)

#include <Python.h> // Must be included first to avoid _POSIX_C_SOURCE macro conflicts

// Game plan:
/*
- [x] Get the table setup
- [x] Get the table +/- buttons hooked up
- [x] Show the points in the viewer
- [x] Manipulate points in the viewer
- [x] Make sure we can undo the manipulations
- [x] Draw a marquee for selection by dragging
- [x] Create points by clicking when the create button is enabled, and by ctrl/alt/click when not
- [x] Setup knob to allow color picking for selected points
  - [x] The knob is clashing with ctrl+alt+click
  - [x] Add a color picker too, like in the roto
  - [x] Allow users to not have to always sample
- [x] Changing the color chips in the table should adjust the RGBA values
- [x] Display the animated color live in the table's color chip
- [x] Baked color picking, with options for sampling bilinear, impulse, 3*3 or 5*5 -> Maybe use python
- [x] Implement a simple voronoi algorithm
- [x] Implement the different pixel algorithms based on https://en.wikipedia.org/wiki/Multivariate_interpolation#Irregular_grid_(scattered_data)
- [x] Ensure it works in proxy mode. See addStoreCallback on the table knob for the X Y positions.
- [x] When deleting/moving points, give a better label to the Undo operations.
- [ ] Optimization: Upstream changes should not trigger a redraw, except for bbox changes. Not sure if there's a way to have the hash ignore upstream changes.
- [x] Implement a Python API to interact with the node.
- [x] On the tracker node, it's cool, when you press Ctrl+Alt, the cursor changes. See if we can reproduce that.

Not implemented:
- It would have been cool to be able to do live-sampling of the input (as-in a point would always be the color of the input), but this clashes with the idea to build the triangulation in the validate method, as that would force the validation to process the input image, which would be bad for performance.

Bugs:
- [ ] Might be a table knob bug, but when you go from having a selection to no selection, or the other way around, there is a significant delay > the same happens on Tracker so it's a definite Nuke bug... Reported as bug 601176
*/

#include "DDImage/PixelIop.h"
#include "DDImage/Pixel.h"
#include "DDImage/Row.h"
#include "DDImage/Box.h"
#include "DDImage/Knobs.h"
#include "DDImage/TableKnobI.h"
#include "DDImage/DDMath.h"
#include "DDImage/Black.h"
#include "DDImage/gl.h"
#include "DDImage/Matrix4.h"
#include "DDImage/Vector2.h"

#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <vector>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

// Include our Delaunay implementation
#include "Delaunay.h"

using namespace DD::Image;

// -----------------------------------------------------------------------------
// SECTION 1: CONSTANTS & ENUMS
// -----------------------------------------------------------------------------
// RCLASS is the internal name Nuke uses to register the node.
static const char* const RCLASS = "PointGradient";
// HELP is the text that appears when you click the '?' icon in the node properties.
static const char* const HELP = "Generate a gradient from multiple points using various interpolation algorithms.";

namespace {
    // Internal names for knobs (properties). 
    // It's good practice to store these as constants to avoid typos when referencing them later.
    const char* kPointsKnobName = "points";
    const char* kAddPointKnobName = "add_point";
    const char* kDeletePointsKnobName = "delete_points";
    const char* kShowRGBKnobName = "show_rgba_columns";
    const char* kShowLabelsKnobName = "show_labels";
    const char* kPointsLockKnobName = "lock_points";
    const char* kCreateModeName = "create_mode";
    const char* kPickerKnob = "points_color";
    const char* kAutoGrab = "grab_color_from_input";
    const char* kIDWMaxRadius = "max_radius";
    const char* kIDWPower = "power";
    const char* kBboxMode = "bbox_mode";
    const char* kOperation = "algorithm";
    const char* kNNInterpolation = "nn_interpolation";
    const char* kLogInterpolation = "log_interpolation";
    const char* kAntiAliasing = "anti_aliasing";

    // Python script for the "Sample..." button. 
    // We embed this Python code directly in C++. When the user clicks the button,
    // this script runs in Nuke, popping up a UI dialog. It then explicitly calls 
    // back into our custom C++ function to do the heavy lifting of sampling frames.
    const char* kPyScript =
        "import nuke\n"
        "node = nuke.thisNode()\n"
        "if not node.input(0):\n"
        "    nuke.message('Please connect an input to sample from.')\n"
        "else:\n"
        "    p = nuke.Panel('Sample Colors')\n"
        "    p.addSingleLineInput('Frames', '{}-{}'.format(nuke.root().firstFrame(), nuke.root().lastFrame()))\n"
        "    p.addEnumerationPulldown('Filter', 'Impulse Bilinear 2x2 3x3 4x4')\n"
        "    if p.show():\n"
        "        try:\n"
        "            f_range = nuke.FrameRange(p.value('Frames'))\n"
        "            filter_str = p.value('Filter')\n"
        "            dx, dy = 1.0, 1.0\n"
        "            if filter_str == 'Impulse':\n"
        "                dx, dy = 0.0, 0.0\n"
        "            elif filter_str == '2x2':\n"
        "                dx, dy = 2.0, 2.0\n"
        "            elif filter_str == '3x3':\n"
        "                dx, dy = 3.0, 3.0\n"
        "            elif filter_str == '4x4':\n"
        "                dx, dy = 4.0, 4.0\n"
        "            task = nuke.ProgressTask('Sampling Colors')\n"
        "            knob = node['table_interface']\n"
        "            with nuke.Undo('Bake Sample'):\n"
        "                try:\n"
        "                    for i, f in enumerate(f_range):\n"
        "                        if task.isCancelled():\n"
        "                            break\n"
        "                        task.setMessage('Sampling frame {}'.format(f))\n"
        "                        task.setProgress(int(100.0 * i / f_range.frames()))\n"
        "                        # Explicitly calling our custom PyMethodDef C-API function attached to the knob\n"
        "                        knob.sample(float(f), float(dx), float(dy))\n"
        "                finally:\n"
        "                    task.setProgress(100)\n"
        "                    del task\n"
        "        except ValueError:\n"
        "            nuke.message('Invalid frame range provided.')\n";
}

// Dropdown Menus for the UI
static const char* const modes[] = { "Natural Neighbor", "Nearest Neighbor (Voronoi)", "Triangles (Delaunay)", "Inverse Distance Weighting", nullptr };
static const char* const nn_modes[] = { "Linear", "Quadratic", "Sibson", "Sibson Square", "Farin", nullptr };
static const char* const bbox_modes[] = { "Normal", "Minimal", "All", nullptr };
static const char* const aa_modes[] = { "None", "2x2", "3x3", "4x4", nullptr };

// Column indices for the table knob.
// Using an enum makes code highly readable instead of writing table->getValue(row, 4),
// which becomes a nightmare if you ever add or reorder a column later!
namespace Column {
    enum Type { ENABLED, NAME, X, Y, R, G, B, A, COLOR };
}

// -----------------------------------------------------------------------------
// SECTION 2: PYTHON BINDING
// -----------------------------------------------------------------------------
// This section is slightly advanced. Nuke allows us to attach custom Python functions
// to our C++ Knobs. This means a Python script (like the one above) can call `knob.sample()`, 
// and it will run the C++ function we define below, bypassing the normal UI limitations.

class PointsTableKnob; // Forward declare

// Structure defining our custom Python object
struct PointsTableKnobObject {
    PyObject_HEAD
        PointsTableKnob* _knob;
};

static PyObject* PointsTableKnobNew(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    Py_RETURN_NONE;
}

static void PointsTableKnobDealloc(PyObject* self) {
    // tp_free handles deletion natively
}

// Forward declare the custom Python bridging method
static PyObject* PointsTableKnobSample(PointsTableKnobObject* self, PyObject* args, PyObject* kwds);

// Map the Python method name "sample" to our C++ function `PointsTableKnobSample`
static PyMethodDef PointsTableKnobMethods[] = {
    {"sample", reinterpret_cast<PyCFunction>(PointsTableKnobSample), METH_VARARGS, "Evaluates upstream pixel values natively bridging C++ math safely off main thread."},
    {nullptr, nullptr, 0, nullptr}
};

// Initialize the Python type object leveraging C++11 Lambda Initialization
static PyTypeObject PointsTableKnobPythonType = []() {
    PyTypeObject result;
    memset(&result, 0, sizeof(PyTypeObject));

    result.tp_name = "PointsTableKnob";
    result.tp_basicsize = sizeof(PointsTableKnobObject);
    result.tp_itemsize = 0;
    result.tp_dealloc = (destructor)PointsTableKnobDealloc;
    result.tp_getattro = PyObject_GenericGetAttr;
    result.tp_setattro = PyObject_GenericSetAttr;
    result.tp_flags = Py_TPFLAGS_DEFAULT;
    result.tp_methods = PointsTableKnobMethods;
    result.tp_alloc = PyType_GenericAlloc;
    result.tp_new = PointsTableKnobNew;
    result.tp_free = PyObject_Del;
    return result;
    }();

// -----------------------------------------------------------------------------
// SECTION 3: HELPER FUNCTIONS
// -----------------------------------------------------------------------------

/**
 * @brief Helper Function to map [0.0 - 1.0] RGBA parameters into a 32-bit graphical Unsigned Integer safely.
 * @param r Red Value
 * @param g Green Value
 * @param b Blue Value
 * @param a Alpha Value
 * @return unsigned int Bit-shifted representation of the color.
 * * Nuke's 'ColorPickerColumn' in a Table Knob expects colors to be provided as a single
 * 32-bit unsigned integer (packing R, G, B, and A into 8 bits each). This helper does that math.
 */
unsigned int RGBToUnsignedInt(double r, double g, double b, double a = 0.0f) {
    int red = static_cast<int>(std::round(r * 255.0));
    int green = static_cast<int>(std::round(g * 255.0));
    int blue = static_cast<int>(std::round(b * 255.0));
    int alpha = static_cast<int>(std::round(a * 255.0));
    red = std::min(255, std::max(0, red));
    green = std::min(255, std::max(0, green));
    blue = std::min(255, std::max(0, blue));
    alpha = std::min(255, std::max(0, alpha));
    return (red << 24) | (green << 16) | (blue << 8) | alpha;
}

/**
 * @brief Helper Function to extract raw floating point [0.0 - 1.0] RGBA values out of an aggregated 32-bit Integer.
 * @param color Source 32-bit Unsigned Integer.
 * @param r Reference modified to the Red Value.
 * @param g Reference modified to the Green Value.
 * @param b Reference modified to the Blue Value.
 * @param a Reference modified to the Alpha Value.
 * * The reverse of the above function. Unpacks the integer back into separate floating point values.
 */
void UnsignedIntToRGBA(unsigned int color, double& r, double& g, double& b, double& a) {
    r = static_cast<double>((color >> 24) & 0xFF) / 255.0;
    g = static_cast<double>((color >> 16) & 0xFF) / 255.0;
    b = static_cast<double>((color >> 8) & 0xFF) / 255.0;
    a = static_cast<double>(color & 0xFF) / 255.0;
}

// -----------------------------------------------------------------------------
// SECTION 4: POINTGRADIENT PLUGIN DEFINITION
// -----------------------------------------------------------------------------

/**
 * @class PointGradient
 * @brief Core PixelIop class responsible for generating gradients from multi-point configurations.
 * * Inheriting from PixelIop means this node operates purely on a pixel-by-pixel basis,
 * making it ideal for rendering gradients. PixelIop is optimized for multi-threading.
 */
class PointGradient : public PixelIop
{
private:
    // UI Settings mapped directly to node properties (knobs)
    bool show_rgba_columns;
    bool show_point_labels;
    bool create_mode;
    bool points_locked;
    bool sample_on_create;
    float _pickerValue[4]{};

    // Math/Algorithm Settings
    int bbox_mode;
    int operation;
    int nn_interpolation; // Stores the Natural Neighbor method enumeration
    bool log_interpolation;
    int anti_aliasing;
    float idw_max_radius;
    float idw_power;

    // Internal Geometry State
    bool is_dragging;
    DD::Image::Matrix4 _transform; // To handle the internal 'Transform' tab mathematically
    Delaunay::Triangulation triangulation;


protected:
    // Interaction Buffers for drawing and selecting points in the Viewer
    bool _isMarqueeSelecting;
    DD::Image::Vector3 _marqueeStart;
    DD::Image::Vector3 _marqueeEnd;
    std::set<int> _selectionBuffer;

public:
    PointGradient(Node* node) : PixelIop(node)
    {
        operation = 0; // Default to Natural Neighbor
        nn_interpolation = 0; // Default to Linear for Natural Neighbor
        show_rgba_columns = false;
        show_point_labels = true;
        create_mode = false;
        sample_on_create = true;
        points_locked = false;
        bbox_mode = 0;
        is_dragging = false;
        idw_max_radius = 0.0f;
        idw_power = 4.0f;
        _isMarqueeSelecting = false;
        _marqueeStart = DD::Image::Vector3(0.0f, 0.0f, 0.0f);
        _marqueeEnd = DD::Image::Vector3(0.0f, 0.0f, 0.0f);

        log_interpolation = false;
        anti_aliasing = 0;
        _transform.makeIdentity();
    }

    friend class PointsTableKnob;

    /**
     * @brief Custom hashing implementation ensuring Nuke reliably detects property updates.
     * @param hash The Foundry hash object to append logic into.
     * * Nuke relies on hashes to know if an image needs recalculating. By default, Nuke
     * might not realize the node changed if we only modify internal table cells.
     * Here, we explicitly feed the state of every cell into Nuke's hash.
     */
    void append(Hash& hash) {
        auto* tableKnob = knob(kPointsKnobName);
        Table_KnobI* tableKnobI = tableKnob->tableKnob();
        for (int i = 0; i < tableKnobI->getRowCount(); i++) {
            hash.append(tableKnobI->getValue(i, Column::ENABLED));
            hash.append(tableKnobI->getValue(i, Column::X));
            hash.append(tableKnobI->getValue(i, Column::Y));
            hash.append(tableKnobI->getValue(i, Column::R));
            hash.append(tableKnobI->getValue(i, Column::G));
            hash.append(tableKnobI->getValue(i, Column::B));
            hash.append(tableKnobI->getValue(i, Column::A));
        }
    }

    /**
     * @brief Transforms coordinates using standard Nuke Transform Matrix logic natively.
     */
    Delaunay::Vector2D transformPoint(double x, double y) const {
        DD::Image::Vector2 v(static_cast<float>(x), static_cast<float>(y));
        return _transform.transform(v); // Implicitly casts back to our generic Delaunay Vector2D via template
    }

    /**
     * @brief Translates screen-space dragged elements logically backwards into untouched Table format values.
     * Used so that when you drag a point in the Viewer, it properly counter-transforms the math
     * to save the correct raw coordinate in the table.
     */
    Delaunay::Vector2D inverseTransformPoint(double x, double y) const {
        DD::Image::Vector2 v(static_cast<float>(x), static_cast<float>(y));
        return _transform.inverse().transform(v);
    }

    // Core Iop Functions (Required overrides for a Nuke Node)
    void _validate(bool) override;
    virtual void pixel_engine(const Row& in, int y, int l, int r, ChannelMask channels, Row& out) override;
    void knobs(Knob_Callback) override;
    virtual int knob_changed(Knob*) override;
    bool updateUI(const OutputContext& context) override;

    // Helper Methods for custom node logic
    bool handle(ViewerContext* ctx, int index);
    void addPoint(float x = 0.0f, float y = 0.0f, bool from_viewer = false);
    void setSelectedPointsColor(float r, float g, float b, float a = 1.0f);
    void refreshColorChip(Table_KnobI* tableKnobI);
    void setColorFromChip(Table_KnobI* tableKnobI, int row);
    void toggle_columns(Table_KnobI* tableKnobI);
    void perform_sample(float frame, float dx, float dy);

    // Metadata Nuke uses to identify the plugin
    const char* Class() const override { return RCLASS; }
    const char* node_help() const override { return HELP; }
    OpHints opHints() const override;
    static Iop::Description d;
};

// -----------------------------------------------------------------------------
// SECTION 5: IMPLEMENTATION - VALIDATE & PIXEL ENGINE
// -----------------------------------------------------------------------------

/**
 * @brief The node validation method called natively by Nuke.
 * @param for_real True if Nuke is demanding an explicit render pass execution.
 * * Nuke evaluates graphs in two main steps: `_validate` and `engine`.
 * `_validate` runs FIRST, once per frame. This is where we calculate our Bounding Box,
 * set our output channels, and construct heavy data structures (like our Triangulation mesh)
 * so we don't have to build them thousands of times per-pixel.
 */
void PointGradient::_validate(bool for_real)
{
    // Tell Nuke we'll output the same size as our input, and we are working in RGBA.
    copy_info();
    set_out_channels(Mask_RGBA);

    auto* tableKnob = knob(kPointsKnobName);
    Table_KnobI* tableKnobI = tableKnob->tableKnob();

    // Reset our geometric mesh for the new frame
    triangulation = Delaunay::Triangulation();
    double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9;

    // OutputContext tells us things like current frame and proxy scales
    OutputContext oc = outputContext();

    // Reversible robust algebraic logging directly matching ARRI Log C v4 (Linear to Log).
    // Doing interpolation in Log space prevents ugly dark bands when fading between bright colors.
    auto safe_log = [](double L) {
        const double p = 95.0 / 1023.0;
        const double q = (1023.0 - 95.0) / 1023.0;
        const double A = (std::pow(2.0, 18.0) - 16.0) / 117.45;
        const double f_inv_0 = (std::pow(2.0, (14.0 * (-p / q) + 6.0)) - 64.0) / A;

        if (L >= f_inv_0) {
            return (std::log2(L * A + 64.0) - 6.0) / 14.0 * q + p;
        }
        else {
            return -((std::log2((2.0 * f_inv_0 - L) * A + 64.0) - 6.0) / 14.0 * q + p);
        }
        };

    // Iterate over the UI table, extract every point, and add it to our mesh data structure.
    for (int i = 0; i < tableKnobI->getRowCount(); i++) {
        if (!tableKnobI->getValue(i, Column::ENABLED)) continue;

        double px = tableKnobI->getValue(i, Column::X);
        double py = tableKnobI->getValue(i, Column::Y);

        // Pass X and Y variables through Nuke matrix projection (from the Transform Tab).
        Delaunay::Vector2D transformed = transformPoint(px, py);

        double r = tableKnobI->getValue(i, Column::R);
        double g = tableKnobI->getValue(i, Column::G);
        double b = tableKnobI->getValue(i, Column::B);
        double a = tableKnobI->getValue(i, Column::A);

        if (log_interpolation && operation == 0) {
            r = safe_log(r); g = safe_log(g); b = safe_log(b); a = safe_log(a);
        }

        // Handle Proxy scale conversions. If the user is viewing the script at half-res, 
        // we need to scale the point positions down by half so they match the image data!
        Delaunay::Vector2D p(static_cast<double>(oc.to_proxy_x(static_cast<float>(transformed.x))),
            static_cast<double>(oc.to_proxy_y(static_cast<float>(transformed.y))));
        Delaunay::VertexColor vc;
        vc.color = { r, g, b, a };
        vc.pid = i;

        triangulation.addPoint(p, vc);

        // Keep track of the extremities of our points to calculate our Bounding Box later
        minX = std::min<double>(minX, p.x); minY = std::min<double>(minY, p.y);
        maxX = std::max<double>(maxX, p.x); maxY = std::max<double>(maxY, p.y);
    }

    // Hand-off calculation math purely to the custom Delaunay Implementation 
    // to build the actual triangulation network for this frame.
    triangulation.build();

    // Natural Neighbor modes (operation == 0) that are non-linear require gradients
    if (operation == 0 && nn_interpolation > 0 && triangulation.vertices.size() >= 2) {
        triangulation.computeGradients(static_cast<double>(idw_max_radius));
    }

    // Bounding Box Logic: We figure out exactly what region of the image actually has data.
    Info input_info = input0().info();
    if (input(0)->isBlackIop()) { // If nothing is connected to the input
        input_info.setBox(Box(0, 0, input_info.format().width(), input_info.format().height()));
        info_.channels(Mask_RGBA);
    }

    Box bbox(static_cast<int>(std::floor(minX)), static_cast<int>(std::floor(minY)),
        static_cast<int>(std::ceil(maxX)), static_cast<int>(std::ceil(maxY)));
    if (minX > maxX) bbox = Box(0, 0, 1, 1); // Empty handling

    switch (bbox_mode) {
    case 0: break;
    case 1: input_info.intersect(bbox); break; // Minimal mode shrinks the box
    case 2: input_info.merge(bbox); break;     // All mode expands the box
    }

    // Commit the final calculated Box back to Nuke
    info_.setBox(input_info);
}

/**
 * @brief Threaded per-pixel processing loop execution invoked by Nuke.
 * @param in Row data input.
 * @param y Absolute vertical target location (Current row).
 * @param l Absolute horizontal start limit index (Left edge).
 * @param r Absolute horizontal end limit index (Right edge).
 * @param channels Nuke rendering channel mask.
 * @param out Output row pointer to write our generated colors to.
 * * Nuke calls `pixel_engine` across multiple threads concurrently.
 * NO UI interactions or data-structure building should happen here! Just pure, fast math.
 */
void PointGradient::pixel_engine(const Row& in, int y, int l, int r, ChannelMask channels, Row& out)
{
    int picked_mode = operation;
    size_t nver = triangulation.vertices.size();

    // Bail early if there are no points to draw
    if (nver == 0) {
        out.erase(channels);
        return;
    }
    // If we only have 1 point, force Nearest Neighbor logic unless evaluating Triangle mode
    else if (nver == 1) {
        if (picked_mode != 2) picked_mode = 1;
    }

    // Exact LogC4 spec inverse logic ensuring 0 bounds round trip safely (Log to Linear).
    auto safe_exp = [](double x) {
        if (x >= 0.0) {
            return ((std::pow(2.0, (14.0 * (((x)-(95.0 / 1023.0)) / ((1023.0 - 95.0) / 1023.0)) + 6.0)) - 64.0) / ((std::pow(2.0, 18.0) - 16.0) / 117.45));
        }
        else {
            return (-((std::pow(2.0, (14.0 * (((-x) - (95.0 / 1023.0)) / ((1023.0 - 95.0) / 1023.0)) + 6.0)) - 64.0) / ((std::pow(2.0, 18.0) - 16.0) / 117.45)) + 2.0 * ((std::pow(2.0, (14.0 * (-(95.0 / 1023.0) / ((1023.0 - 95.0) / 1023.0))) + 6.0) - 64.0) / ((std::pow(2.0, 18.0) - 16.0) / 117.45)));
        }
        };

    // The primary loop traversing horizontally across the row, pixel by pixel.
    for (int x = l; x < r; x++) {
        std::array<double, 4> color = { 0.0, 0.0, 0.0, 0.0 };

        // Handle MSAA (Anti-Aliasing) if requested specifically inside the sharp-edged modes.
        if (anti_aliasing > 0 && (picked_mode == 1 || picked_mode == 2)) {
            int samples = anti_aliasing + 1; // 1->2x2, 2->3x3, 3->4x4
            int num_samples = samples * samples;
            std::array<double, 4> accum = { 0.0, 0.0, 0.0, 0.0 };

            // Sub-pixel sampling loop
            for (int sy = 0; sy < samples; ++sy) {
                for (int sx = 0; sx < samples; ++sx) {
                    Delaunay::Vector2D query(x + (sx + 0.5) / samples - 0.5, y + (sy + 0.5) / samples - 0.5);
                    std::array<double, 4> s_color;

                    if (picked_mode == 1) s_color = triangulation.interpolateNearestNeighbor(query);
                    else s_color = triangulation.interpolateNearestTriangle(query);

                    for (int c = 0; c < 4; ++c) accum[c] += s_color[c];
                }
            }
            for (int c = 0; c < 4; ++c) color[c] = accum[c] / num_samples;
        }
        else {
            Delaunay::Vector2D query(static_cast<double>(x), static_cast<double>(y));

            // Delegate mathematical evaluation to the Delaunay struct based on UI dropdown
            switch (picked_mode) {
            case 0: // Natural Neighbor
                color = triangulation.interpolateNaturalNeighbor(query, static_cast<Delaunay::NaturalNeighborMethod>(nn_interpolation));
                break;
            case 1: // Nearest Neighbor (Voronoi)
                color = triangulation.interpolateNearestNeighbor(query);
                break;
            case 2: // Triangles
                color = triangulation.interpolateNearestTriangle(query);
                break;
            case 3: // IDW
                color = triangulation.interpolateIDW(query, static_cast<double>(idw_max_radius), static_cast<double>(idw_power));
                break;
            }
        }

        // Apply reversable EXP modifier if log calculation was requested.
        if (log_interpolation && picked_mode == 0) {
            for (int c = 0; c < 4; ++c) color[c] = safe_exp(color[c]);
        }

        // Map computed {r, g, b, a} back directly to Nuke's channels.
        // Nuke's foreach(z, channels) is a macro that iterates over the active channels (R, G, B, A).
        // out.writable(z)[x] grabs the pointer to that specific pixel so we can write our result.
        foreach(z, channels) {
            out.writable(z)[x] = static_cast<float>(color[colourIndex(z)]);
        }
    }
}

// -----------------------------------------------------------------------------
// SECTION 6: UI LOGIC - KNOBS & PROPERTY PANEL
// -----------------------------------------------------------------------------

// ==============================================
// SECTION 6A: CUSTOM KNOB FOR TABLE INTERACTIONS
// ==============================================

/**
 * @class PointsTableKnob
 * @brief Handles custom viewer interactions (clicking, dragging) related to point modifications.
 * * To catch Mouse clicks and Keyboard shortcuts directly inside the Viewer, we must
 * build a custom Knob class. This class acts as a bridge between the Viewer UI and our Node.
 */
class PointsTableKnob : public Knob, public DD::Image::PluginPython_KnobI {
public:
    PointGradient* theOp;

    const char* Class() const override { return "PointsTable"; }

    // Required override mapping Nuke native API directly into our custom Python bindings
    DD::Image::PluginPython_KnobI* pluginPythonKnob() override {
        return this;
    }

    static bool handle_cb(ViewerContext* ctx, Knob* knob, int index);
    void draw_handle(ViewerContext* ctx);
    bool build_handle(ViewerContext* ctx);
    static bool nothing(ViewerContext* ctx, Knob* knob, int index);

    PointsTableKnob(Knob_Closure* kc, PointGradient* t, const char* n) : Knob(kc, n) {
        theOp = t;
        setPythonType(&PointsTableKnobPythonType);
    }
};

/**
 * @brief Universal Python C-API extension method routing C++ operations directly from bound Nuke scripting instances.
 * * This is the destination function called when the Python script (from Section 1) runs `knob.sample()`.
 */
static PyObject* PointsTableKnobSample(PointsTableKnobObject* self, PyObject* args, PyObject* kwds) {
    float frame, dx, dy;
    if (!PyArg_ParseTuple(args, "fff", &frame, &dx, &dy)) {
        return nullptr;
    }

    PointsTableKnob* myKnob = self->_knob;
    if (myKnob != nullptr && myKnob->theOp != nullptr) {
        myKnob->theOp->perform_sample(frame, dx, dy);
    }

    Py_RETURN_NONE;
}

// ========================================
// SECTION 6B: KNOB DEFINITIONS & CALLBACKS
// ========================================


/**
 * @brief Declares visual UI inputs attached to the plugin node.
 * @param f System knob callback.
 * * This function builds the Properties Panel inside Nuke. It maps C++ variables (like 'operation')
 * directly to UI elements (like Dropdowns or Checkboxes).
 */
void PointGradient::knobs(Knob_Callback f)
{
    Enumeration_knob(f, &operation, modes, kOperation, "Algorithm");
    Tooltip(f, "Choose the primary interpolation algorithm to use.");

    Enumeration_knob(f, &nn_interpolation, nn_modes, kNNInterpolation, "Interpolation");
    Tooltip(f, "Select the secondary blending type specifically for the Natural Neighbor algorithm.");

    Bool_knob(f, &log_interpolation, kLogInterpolation, "Log Interpolation");
    Tooltip(f, "Perform interpolation in log space to yield smoother non-linear gradients for Natural Neighbor.");
    SetFlags(f, Knob::STARTLINE);

    Enumeration_knob(f, &anti_aliasing, aa_modes, kAntiAliasing, "Anti Aliasing");
    Tooltip(f, "Multi-sample anti-aliasing. Highly useful for feathering the sharp boundaries in Nearest Neighbor and Triangle modes.");

    Float_knob(f, &idw_max_radius, kIDWMaxRadius, "Max Radius");
    SetRange(f, 0.0, 1000.0);
    Tooltip(f, "Maximum radius for the inverse distance weighting.\nIf set to 0, no maximum radius will be used.\n");
    Float_knob(f, &idw_power, kIDWPower, "Power");
    SetRange(f, 0.0, 10.0);
    Tooltip(f, "Power for the inverse distance weighting.\nIf set to a positive value, the algorithm will use that power to affect how much weight to use for each point.\nGenerally, the higher the value, the more the image will ressemble a voronoi diagram.");
    Divider(f);

    Enumeration_knob(f, &bbox_mode, bbox_modes, kBboxMode, "BBox");
    Tooltip(f, "Choose the bounding box mode to use:\nNormal: The bounding box is the BBox of the input image if the input is connected, or the format if it isn't.\nMinimal: The bounding box is the smallest rectangle that contains all the points within the format (or the input BBox).\nAll: The output BBox will be the union of the format (or input BBox) and the BBox of the points.");

    // This creates our invisible custom knob to handle drawing and viewer interactions.
    Knob* tableInterfaceKnob = CustomKnob1(PointsTableKnob, f, this, "table_interface");
    if (f.makeKnobs()) {
        f(PLUGIN_PYTHON_KNOB, Custom, tableInterfaceKnob, nullptr, nullptr, nullptr);
    }

    // Creating the large Table element containing all the points.
    Knob* tableKnob = Table_knob(f, kPointsKnobName, "");
    Table_KnobI* tableKnobI = tableKnob->tableKnob();
    if (f.makeKnobs()) {
        tableKnobI->addColumn("enabled", "e", Table_KnobI::AnimBoolColumn, true, 20);
        tableKnobI->addColumn("name", "name", Table_KnobI::StringColumn, true, 140);
        tableKnobI->addColumn("x", "x", Table_KnobI::AnimCurveColumn, true, 60);
        tableKnobI->addColumn("y", "y", Table_KnobI::AnimCurveColumn, true, 60);
        tableKnobI->addColumn("r", "r", Table_KnobI::AnimCurveColumn, true, 70);
        tableKnobI->addColumn("g", "g", Table_KnobI::AnimCurveColumn, true, 70);
        tableKnobI->addColumn("b", "b", Table_KnobI::AnimCurveColumn, true, 70);
        tableKnobI->addColumn("a", "a", Table_KnobI::AnimCurveColumn, true, 70);
        tableKnobI->addColumn("color", "color", Table_KnobI::ColorPickerColumn, true, 60);
    }

    Button(f, kAddPointKnobName, "Add Point");
    SetFlags(f, Knob::STARTLINE);
    Button(f, kDeletePointsKnobName, "Delete Points");
    Tooltip(f, "Delete selected points from the table.");

    // Attaching our python script to a button.
    PyScript_knob(f, kPyScript, "sample_colors_btn", "Sample...");
    Tooltip(f, "Open a dialog to sample colors from the input over a specified frame range with different filter options. Samples the selected Points, or all Points if none are selected.\nExisting animation is not cleared before sampling.");

    Bool_knob(f, &show_rgba_columns, kShowRGBKnobName, "Show RGBA columns");
    Tooltip(f, "Display or Hide the Columns in the Table keeping track of the animated RGB values.");
    SetFlags(f, Knob::NO_RERENDER); // Telling Nuke pushing this button shouldn't clear the image cache.

    // Using icons in the viewer toolbar, much like a Roto or Tracker node.
    BeginToolbar(f, "PointGradienTopToolbar", "", Knob::TOOLBAR_TOP);
    Spacer(f, 10);
    Bool_knob(f, &create_mode, kCreateModeName, "@Tracker/AddTrackOff;@Tracker/AddTrackOn;");
    Tooltip(f, "Enable or disable the create mode.\nWhen this is enabled, you can add points by clicking in the viewer.\nWhen disabled, you can create points using Ctrl+Alt+Click");
    SetFlags(f, Knob::NO_RERENDER);
    Bool_knob(f, &sample_on_create, kAutoGrab, "@Tracker/GrabPattern");
    Tooltip(f, "Automatically grab the color of the point from the image.\nWhen this is enabled, the color of the point will be set to the input color under the point when creating it or right-clicking it.\nWhen disabled, the color will be set to the default color (from the color picker knob).");
    SetFlags(f, Knob::NO_RERENDER);
    Spacer(f, 10);
    Bool_knob(f, &show_point_labels, kShowLabelsKnobName, "@Roto/LabelPoints");
    Tooltip(f, "Show or hide the point labels in the viewer.");
    SetFlags(f, Knob::NO_RERENDER);
    Bool_knob(f, &points_locked, kPointsLockKnobName, "@Roto/FeatherLink");
    Tooltip(f, "Lock the points positions and creation.\nWhen this is enabled, you will not be able to edit the point positions nor add new points using the viewer controls, but still can using the properties.\nYou can still change the colors.\nThis is useful when color picking as otherwise some mouse / keyboard actions may clash(for example, ctrl + alt + click to sample input, vs ctrl + alt + click to add a new point).");

    AColor_knob(f, _pickerValue, kPickerKnob, "");
    SetFlags(f, Knob::NO_RERENDER | Knob::NO_ANIMATION | Knob::NO_COLOR_DROPDOWN | Knob::NO_NUMERIC_FIELDS | Knob::DO_NOT_WRITE | Knob::NO_UNDO | Knob::NO_CURVE_EDITOR | Knob::COLOURCHIP_HAS_UNSET);
    ClearFlags(f, Knob::SLIDER | Knob::MAGNITUDE | Knob::STARTLINE);
    Tooltip(f, "Change the color of all the selected points.\n\nPoints added via the properties panel will also default to this color.");
    Spacer(f, 0); Spacer(f, 0);
    EndToolbar(f);

    Tab_knob(f, "Transform");
    Transform2d_knob(f, &_transform, "transform");
}

/**
 * @brief Manages live UI updates unprompted by knob clicks.
 * @param context Environment properties context.
 * @return true execution successfully processed.
 */
bool PointGradient::updateUI(const OutputContext& context) {
    auto* tableKnob = knob(kPointsKnobName);
    Table_KnobI* tableKnobI = tableKnob->tableKnob();
    refreshColorChip(tableKnobI);
    return true;
}

/**
 * @brief Dynamic handler evaluating which property was manipulated and its corresponding action.
 * @param k The UI knob that fired the trigger.
 * @return Result execution identifier natively expected by Nuke.
 * * When a user changes a setting in the Properties Panel, Nuke fires `knob_changed`.
 * Here we catch specific clicks (like pressing the "Delete Points" button) and
 * write custom logic to handle them.
 */
int PointGradient::knob_changed(Knob* k) {
    if (k->is(kPointsKnobName)) {
        Table_KnobI* tableKnobI = k->tableKnob();
        // If the user modified a row manually (like dragging a color picker)
        if (tableKnobI->getModificationType() == Table_KnobI::eEditRow) {
            std::set<std::pair<int, int>> cells = tableKnobI->getModifiedCells();
            for (std::pair<int, int> cell : cells) {
                if (cell.second == Column::COLOR) setColorFromChip(tableKnobI, cell.first);
            }
        }
        return 1;
    }
    if (k->is(kAddPointKnobName)) { addPoint(); return 1; }
    if (k->is(kDeletePointsKnobName)) {
        knob(kPointsKnobName)->force_new_undo("Delete Point(s)"); // Custom undo string
        Table_KnobI* tableKnobI = knob(kPointsKnobName)->tableKnob();
        tableKnobI->deleteSelectedRows();
        return 1;
    }
    if (k->is(kOperation)) {
        // Toggle UI dynamically based on chosen algorithm (hide properties that don't apply to the mode)
        knob(kNNInterpolation)->visible(operation == 0); // Natural Neighbor
        knob(kLogInterpolation)->visible(operation == 0); // Log implementation
        knob(kAntiAliasing)->visible(operation == 1 || operation == 2); // MSAA
        knob(kIDWMaxRadius)->visible(operation == 3);    // IDW
        knob(kIDWPower)->visible(operation == 3);        // IDW
        return 1;
    }
    if (k->is(kShowRGBKnobName)) {
        Table_KnobI* tableKnobI = knob(kPointsKnobName)->tableKnob();
        toggle_columns(tableKnobI);
        return 1;
    }
    if (k->is(kCreateModeName)) {
        if (k->get_value()) knob(kPointsLockKnobName)->set_value(false);
        return 1;
    }
    if (k->is(kPointsLockKnobName)) {
        if (k->get_value()) knob(kCreateModeName)->set_value(false);
        return 1;
    }
    if (k->is(kPickerKnob)) {
        setSelectedPointsColor(_pickerValue[0], _pickerValue[1], _pickerValue[2], _pickerValue[3]);
        return 1;
    }
    if (k->is("showPanel")) { // "showPanel" fires when the UI window itself opens
        toggle_columns(knob(kPointsKnobName)->tableKnob());
        knob(kNNInterpolation)->visible(operation == 0);
        knob(kLogInterpolation)->visible(operation == 0);
        knob(kAntiAliasing)->visible(operation == 1 || operation == 2);
        knob(kIDWMaxRadius)->visible(operation == 3);
        knob(kIDWPower)->visible(operation == 3);
        return 0;
    }
    // Fallback to base class behavior
    return PixelIop::knob_changed(k);
}

// -----------------------------------------------------------------------------
// SECTION 7: DATA MANAGEMENT & SAMPLING LOGIC
// -----------------------------------------------------------------------------

/**
 * @brief Discrete evaluation request called dynamically by Python script binding.
 * Allows safe evaluation without relying on heavy main thread GUI graph lockups.
 * @param frame Sub-frame time execution requested.
 * @param dx Extrapolated multi-pixel bounding shift X.
 * @param dy Extrapolated multi-pixel bounding shift Y.
 * * This temporarily pushes Nuke to evaluate the input graph at a specific frame
 * so we can rip the color data off the upstream node.
 */
void PointGradient::perform_sample(float frame, float dx, float dy) {
    Knob* table = knob(kPointsKnobName);
    Table_KnobI* tableKnobI = table->tableKnob();
    std::vector<int> target_rows = tableKnobI->getSelectedRows();

    // Sample all enabled points if none are actively highlighted.
    if (target_rows.empty()) {
        for (int i = 0; i < tableKnobI->getRowCount(); ++i) {
            if (tableKnobI->getValue(i, Column::ENABLED)) target_rows.push_back(i);
        }
    }

    Iop* in = input(0);
    if (!in || in->isBlackIop() || target_rows.empty()) return;

    // OutputContext defines 'where and when' we are looking in Nuke. We spoof it 
    // to jump to our target frame for this loop without visually shifting the user's timeline.
    OutputContext old_oc = outputContext();
    OutputContext oc = old_oc;
    oc.setFrame(static_cast<double>(frame));

    in->gotoContext(oc, true);
    try {
        in->validate(true);

        double min_x = 1e9, min_y = 1e9;
        double max_x = -1e9, max_y = -1e9;

        // Find the extents of all selected points so we don't request Nuke to render 
        // the entire image, only the bounding box encompassing our points.
        for (int point : target_rows) {
            double raw_x = tableKnobI->getValueAt(frame, point, Column::X);
            double raw_y = tableKnobI->getValueAt(frame, point, Column::Y);
            Delaunay::Vector2D world = transformPoint(raw_x, raw_y);

            min_x = std::min(min_x, world.x);
            min_y = std::min(min_y, world.y);
            max_x = std::max(max_x, world.x);
            max_y = std::max(max_y, world.y);
        }

        // Convert to proxy space, pad by the sample radius, and build the box
        int px = static_cast<int>(std::floor(oc.to_proxy_x(static_cast<float>(min_x)) - dx));
        int py = static_cast<int>(std::floor(oc.to_proxy_y(static_cast<float>(min_y)) - dy));
        int pr = static_cast<int>(std::ceil(oc.to_proxy_x(static_cast<float>(max_x)) + dx));
        int pt = static_cast<int>(std::ceil(oc.to_proxy_y(static_cast<float>(max_y)) + dy));

        DD::Image::Box sample_box(px, py, pr, pt);

        // Intersect with the actual image info box so we don't request outside the format
        sample_box.intersect(in->info().box());

        // 3. Make exactly ONE request for the merged region
        if (sample_box.w() > 0 && sample_box.h() > 0) {
            in->request(sample_box, Mask_RGBA, 1);
        }
    }
    catch (...) {}

    tableKnobI->suspendKnobChangedEvents();

    // Now run the sample loop

    tableKnobI->suspendKnobChangedEvents();

    for (int point : target_rows) {
        // Dynamically fetch point coords using Table_KnobI's native getValueAt
        double raw_x = tableKnobI->getValueAt(frame, point, Column::X);
        double raw_y = tableKnobI->getValueAt(frame, point, Column::Y);

        Delaunay::Vector2D world = transformPoint(raw_x, raw_y);

        Pixel sample_pix(Mask_RGBA);
        in->sample(oc.to_proxy_x(static_cast<float>(world.x)), oc.to_proxy_y(static_cast<float>(world.y)), dx, dy, sample_pix);

        // Commit the data sequentially backwards using Table_KnobI's native setValueAt
        // This is what adds the animation keyframes directly into the table!
        tableKnobI->setValueAt(point, Column::R, frame, sample_pix[Chan_Red]);
        tableKnobI->setValueAt(point, Column::G, frame, sample_pix[Chan_Green]);
        tableKnobI->setValueAt(point, Column::B, frame, sample_pix[Chan_Blue]);
        tableKnobI->setValueAt(point, Column::A, frame, sample_pix[Chan_Alpha]);

        if (std::abs(frame - old_oc.frame()) < 1e-5) {
            tableKnobI->setCellColor(point, Column::COLOR, RGBToUnsignedInt(sample_pix[Chan_Red], sample_pix[Chan_Green], sample_pix[Chan_Blue], sample_pix[Chan_Alpha]));
        }
    }

    // Reset Context back to normal viewing
    in->gotoContext(old_oc, true);
    tableKnobI->resumeKnobChangedEvents(true);
}

/**
 * @brief Generates and positions a new graphical point inside the Nuke viewer.
 * @param x World X Coordinate.
 * @param y World Y Coordinate.
 * @param from_viewer Flag triggering color sampling directly from input pixel bounds.
 */
void PointGradient::addPoint(float x, float y, bool from_viewer) {
    knob(kPointsKnobName)->force_new_undo("Add Point");
    Table_KnobI* tableKnobI = knob(kPointsKnobName)->tableKnob();
    const int newPointIndex = tableKnobI->addRow();
    OutputContext oc = outputContext();

    std::string name = "Point " + std::to_string(newPointIndex);
    double r, g, b, a;

    Iop* const node_input = input(0);
    if (from_viewer && !node_input->isBlackIop()) {
        Pixel sample(Mask_RGBA);
        node_input->sample(oc.to_proxy_x(x), oc.to_proxy_y(y), 1.0f, 1.0f, sample);
        r = sample[Chan_Red]; g = sample[Chan_Green]; b = sample[Chan_Blue]; a = sample[Chan_Alpha];
    }
    else {
        r = _pickerValue[0]; g = _pickerValue[1]; b = _pickerValue[2]; a = _pickerValue[3];
    }

    // Un-project world mouse coordinate back to mathematically un-scaled table coordinate
    Delaunay::Vector2D table_pos(x, y);
    if (from_viewer) {
        table_pos = inverseTransformPoint(static_cast<double>(x), static_cast<double>(y));
    }

    // Suspending events prevents Nuke from trying to redraw midway through us building a row
    tableKnobI->suspendKnobChangedEvents();
    tableKnobI->setCellBool(newPointIndex, Column::ENABLED, true);
    tableKnobI->setCellString(newPointIndex, Column::NAME, name);
    tableKnobI->setValue(newPointIndex, Column::X, static_cast<double>(table_pos.x));
    tableKnobI->setValue(newPointIndex, Column::Y, static_cast<double>(table_pos.y));
    tableKnobI->setValue(newPointIndex, Column::R, r);
    tableKnobI->setValue(newPointIndex, Column::G, g);
    tableKnobI->setValue(newPointIndex, Column::B, b);
    tableKnobI->setValue(newPointIndex, Column::A, a);
    tableKnobI->setCellColor(newPointIndex, Column::COLOR, 0);

    tableKnobI->selectRow(newPointIndex);
    tableKnobI->resumeKnobChangedEvents(true);
}

/**
 * @brief Commits selected solid color onto all actively highlighted table rows.
 * @param r Red parameter to apply.
 * @param g Green parameter to apply.
 * @param b Blue parameter to apply.
 * @param a Alpha parameter to apply.
 */
void PointGradient::setSelectedPointsColor(float r, float g, float b, float a) {
    Table_KnobI* tableKnobI = knob(kPointsKnobName)->tableKnob();
    std::vector<int> selected_rows = tableKnobI->getSelectedRows();

    tableKnobI->suspendKnobChangedEvents();
    for (int point : selected_rows) {
        tableKnobI->setValue(point, Column::R, static_cast<double>(r));
        tableKnobI->setValue(point, Column::G, static_cast<double>(g));
        tableKnobI->setValue(point, Column::B, static_cast<double>(b));
        tableKnobI->setValue(point, Column::A, static_cast<double>(a));
        tableKnobI->setCellColor(point, Column::COLOR, RGBToUnsignedInt(r, g, b));
    }
    tableKnobI->resumeKnobChangedEvents(true);
}

/**
 * @brief Triggers a synchronized extraction of component values down to a singular representation for 32-bit graphical displays in the UI.
 * @param tableKnobI Reference Table object rendering values.
 */
void PointGradient::refreshColorChip(Table_KnobI* tableKnobI) {
    double r, g, b, a;
    tableKnobI->suspendKnobChangedEvents();
    for (int i = 0; i < tableKnobI->getRowCount(); i++) {
        r = tableKnobI->getValue(i, Column::R); g = tableKnobI->getValue(i, Column::G);
        b = tableKnobI->getValue(i, Column::B); a = tableKnobI->getValue(i, Column::A);
        tableKnobI->setCellColor(i, Column::COLOR, RGBToUnsignedInt(r, g, b, a));
    }
    tableKnobI->resumeKnobChangedEvents(false);
}

/**
 * @brief Unwraps the 32-bit Integer representation stored within color pickers directly back into usable RGB doubles.
 * @param tableKnobI Reference Table object rendering values.
 * @param row Target table row requiring value extraction.
 */
void PointGradient::setColorFromChip(Table_KnobI* tableKnobI, int row) {
    unsigned int color = tableKnobI->getCellColor(row, Column::COLOR);
    double r, g, b, a;
    UnsignedIntToRGBA(color, r, g, b, a);
    tableKnobI->suspendKnobChangedEvents();
    tableKnobI->setValue(row, Column::R, r);
    tableKnobI->setValue(row, Column::G, g);
    tableKnobI->setValue(row, Column::B, b);
    tableKnobI->setValue(row, Column::A, a);
    tableKnobI->resumeKnobChangedEvents(false);
}

/**
 * @brief Modifies visibility of expanded discrete RGB columns in UI matrix context.
 * @param tableKnobI Parent component controlling parameters.
 */
void PointGradient::toggle_columns(Table_KnobI* tableKnobI) {
    if (tableKnobI->getColumnCount() > 0) {
        tableKnobI->suspendKnobChangedEvents();
        tableKnobI->setColumnVisibility(Column::R, show_rgba_columns);
        tableKnobI->setColumnVisibility(Column::G, show_rgba_columns);
        tableKnobI->setColumnVisibility(Column::B, show_rgba_columns);
        tableKnobI->setColumnVisibility(Column::A, show_rgba_columns);
        tableKnobI->resumeKnobChangedEvents(false);
    }
}

// -----------------------------------------------------------------------------
// SECTION 8: VIEWER INTERACTION (THE HANDLE METHOD)
// -----------------------------------------------------------------------------

/**
 * @brief Processes direct graphical manipulations driven by the user on elements located inside the viewplane.
 * @param ctx Native Nuke tracking structure governing interaction.
 * @param index Contextual identity of what object was clicked (e.g. -1 for background, >=0 for a specific point).
 * @return True on positive action recognition.
 * * Nuke routes Viewer inputs (Clicks, Drags) to this method. The `ctx->event()` dictates
 * what action is happening (PUSH = click, DRAG = mouse move while clicked, RELEASE = let go).
 */
bool PointGradient::handle(ViewerContext* ctx, int index) {
    Table_KnobI* tableKnobI;
    Knob* table = knob(kPointsKnobName);
    OutputContext oc = outputContext();

    switch (ctx->event()) {
    case PUSH: {
        // Checking for standard 'Add Point' combinations (Ctrl+Alt+Click) or custom create_mode button
        if (((ctx->state() & CTRL) && (ctx->state() & ALT) && !(ctx->state() & SHIFT)) || (create_mode && index < 0)) {
            if (points_locked) return false;
            // Add a point, making sure to convert from proxy scale back to real coordinates
            addPoint(static_cast<float>(oc.from_proxy_x(static_cast<float>(ctx->x()))),
                static_cast<float>(oc.from_proxy_y(static_cast<float>(ctx->y()))), sample_on_create);
            return true;
        }
        if (ctx->state() & CTRL) return false;

        tableKnobI = table->tableKnob();
        // If the user clicked on a valid existing point...
        if (index >= 0) {
            std::vector<int> selected_rows = tableKnobI->getSelectedRows();
            auto selected = std::find(selected_rows.begin(), selected_rows.end(), index);

            // Re-sample specific point if right clicked (button 3 is typical native right-click identifier)
            if (ctx->button() == 3 && sample_on_create && !input(0)->isBlackIop()) {
                double tx = tableKnobI->getValue(index, Column::X);
                double ty = tableKnobI->getValue(index, Column::Y);
                Delaunay::Vector2D world = transformPoint(tx, ty);

                Pixel sample(Mask_RGBA);
                input(0)->sample(oc.to_proxy_x(static_cast<float>(world.x)), oc.to_proxy_y(static_cast<float>(world.y)), 1.0f, 1.0f, sample);

                tableKnobI->suspendKnobChangedEvents();
                tableKnobI->setValue(index, Column::R, sample[Chan_Red]);
                tableKnobI->setValue(index, Column::G, sample[Chan_Green]);
                tableKnobI->setValue(index, Column::B, sample[Chan_Blue]);
                tableKnobI->setValue(index, Column::A, sample[Chan_Alpha]);
                tableKnobI->setCellColor(index, Column::COLOR, RGBToUnsignedInt(sample[Chan_Red], sample[Chan_Green], sample[Chan_Blue], sample[Chan_Alpha]));
                tableKnobI->resumeKnobChangedEvents(true);
            }

            // Handle SHIFT-Click to add/remove from selection
            if (ctx->state() & SHIFT) {
                if (selected == selected_rows.end()) selected_rows.push_back(index);
                else selected_rows.erase(selected);
                tableKnobI->selectRows(selected_rows);
                return true;
            }
            // Standard click to isolate selection
            else if (selected == selected_rows.end()) {
                tableKnobI->selectRow(index);
                return true;
            }
        }
        else {
            // Clicked empty space. Initiate marquee selection drag buffer.
            _isMarqueeSelecting = true;
            _marqueeStart = ctx->pos();
            _marqueeEnd = ctx->pos();
            if (ctx->state() & SHIFT) {
                std::vector<int> selected_rows = tableKnobI->getSelectedRows();
                _selectionBuffer.insert(selected_rows.begin(), selected_rows.end());
            }
            return true;
        }
        break;
    }
    case DRAG: {
        tableKnobI = table->tableKnob();
        std::vector<int> selected_rows = tableKnobI->getSelectedRows();

        // Dragging a Marquee Box across the screen
        if (_isMarqueeSelecting) {
            _marqueeEnd = ctx->pos();
            double x_min = static_cast<double>(std::min(_marqueeStart.x, _marqueeEnd.x));
            double y_min = static_cast<double>(std::min(_marqueeStart.y, _marqueeEnd.y));
            double x_max = static_cast<double>(std::max(_marqueeStart.x, _marqueeEnd.x));
            double y_max = static_cast<double>(std::max(_marqueeStart.y, _marqueeEnd.y));

            _selectionBuffer.clear();
            if (ctx->state() & SHIFT) _selectionBuffer.insert(selected_rows.begin(), selected_rows.end());

            std::set<int> _newSelectionBuffer;
            for (int i = 0; i < tableKnobI->getRowCount(); ++i) {
                if (!tableKnobI->getValue(i, Column::ENABLED)) continue;

                Delaunay::Vector2D transformed = transformPoint(tableKnobI->getValue(i, Column::X), tableKnobI->getValue(i, Column::Y));
                double px = static_cast<double>(oc.to_proxy_x(static_cast<float>(transformed.x)));
                double py = static_cast<double>(oc.to_proxy_y(static_cast<float>(transformed.y)));

                // If point is mathematically inside the rectangle, highlight it
                if (px >= x_min && px <= x_max && py >= y_min && py <= y_max) _newSelectionBuffer.insert(i);
            }

            if (_selectionBuffer.empty() || _newSelectionBuffer.empty() || std::any_of(_newSelectionBuffer.begin(), _newSelectionBuffer.end(), [&](int point) { return _selectionBuffer.find(point) == _selectionBuffer.end(); })) {
                _selectionBuffer.insert(_newSelectionBuffer.begin(), _newSelectionBuffer.end());
            }
            else {
                for (int point : _newSelectionBuffer) _selectionBuffer.erase(point);
            }
            return true;
        }
        // Dragging actual points around
        else if (!selected_rows.empty() && !points_locked) {
            if (!is_dragging) {
                is_dragging = true;
                table->force_new_undo("Move Point(s)");
            }
            else {
                table->new_undo("Move Point(s)");
            }

            // Getting the delta means checking how far the mouse has moved since the last evaluation
            DD::Image::Vector3 delta = ctx->dPos();
            oc.from_proxy_wh(delta.x, delta.y);

            tableKnobI->suspendKnobChangedEvents();
            for (int point : selected_rows) {
                double current_table_x = tableKnobI->getValue(point, Column::X);
                double current_table_y = tableKnobI->getValue(point, Column::Y);

                // Compute exact transformed shift mapping so handles flow perfectly with the mouse drag
                Delaunay::Vector2D current_world = transformPoint(current_table_x, current_table_y);
                Delaunay::Vector2D new_world(current_world.x + static_cast<double>(delta.x), current_world.y + static_cast<double>(delta.y));
                Delaunay::Vector2D new_table = inverseTransformPoint(new_world.x, new_world.y);

                tableKnobI->setValue(point, Column::X, new_table.x);
                tableKnobI->setValue(point, Column::Y, new_table.y);

                // Continuous grab integration when dragging a point interactively over input imagery
                if (ctx->button() == 3 && sample_on_create && !input(0)->isBlackIop()) {
                    Pixel sample(Mask_RGBA);
                    input(0)->sample(oc.to_proxy_x(static_cast<float>(new_world.x)), oc.to_proxy_y(static_cast<float>(new_world.y)), 1.0f, 1.0f, sample);

                    tableKnobI->setValue(point, Column::R, sample[Chan_Red]);
                    tableKnobI->setValue(point, Column::G, sample[Chan_Green]);
                    tableKnobI->setValue(point, Column::B, sample[Chan_Blue]);
                    tableKnobI->setValue(point, Column::A, sample[Chan_Alpha]);
                    tableKnobI->setCellColor(point, Column::COLOR, RGBToUnsignedInt(sample[Chan_Red], sample[Chan_Green], sample[Chan_Blue], sample[Chan_Alpha]));
                }
            }
            tableKnobI->resumeKnobChangedEvents(true);
            return true;
        }
        break;
    }
    case RELEASE: {
        // Letting go of the mouse. Apply the marquee selection permanently to the table rows.
        if (_isMarqueeSelecting) {
            _isMarqueeSelecting = false;
            tableKnobI = table->tableKnob();
            std::vector<int> selected_rows(_selectionBuffer.begin(), _selectionBuffer.end());
            tableKnobI->selectRows(selected_rows);
            _selectionBuffer.clear();
            return true;
        }
        is_dragging = false;
        break;
    }
    }
    return false;
}

// Intermediary hook to bridge between the standard Viewer system and our custom object
bool PointsTableKnob::handle_cb(ViewerContext* ctx, Knob* knob, int index) {
    return ((PointsTableKnob*)knob)->theOp->handle(ctx, index);
}

bool PointsTableKnob::nothing(ViewerContext* ctx, Knob* knob, int index) {
    return false;
}

/**
 * @brief Draws OpenGL UI overlays natively inside the active Nuke Viewer.
 * * Nuke lets developers inject Raw OpenGL commands to visually assist users.
 * Here we draw the points as glowing crosses/dots, text names, and the selection boxes.
 */
void PointsTableKnob::draw_handle(ViewerContext* ctx) {
    if (!ctx->draw_lines()) return;

    if (ctx->event() == DRAW_LINES || ctx->event() == DRAW_SHADOW || ctx->event() == PUSH || ctx->event() == DRAG || ctx->event() == CURSOR) {
        Table_KnobI* tableKnobI = knob(kPointsKnobName)->tableKnob();
        std::vector<int> selected_rows = tableKnobI->getSelectedRows();
        OutputContext oc = theOp->outputContext();

        for (int i = 0; i < tableKnobI->getRowCount(); i++) {
            if (!tableKnobI->getValue(i, Column::ENABLED)) continue;

            if (ctx->event() != DRAW_SHADOW) {
                // If it's selected, draw it in bright red/orange, otherwise fallback to the Node Color.
                bool is_selected = theOp->_isMarqueeSelecting ?
                    (theOp->_selectionBuffer.find(i) != theOp->_selectionBuffer.end()) :
                    (std::find(selected_rows.begin(), selected_rows.end(), i) != selected_rows.end());
                glColor(is_selected ? 0xff900000 : ctx->node_color());
            }
            else {
                glColor(ctx->node_color());
            }

            // Convert math coordinates to on-screen proxy coordinates
            Delaunay::Vector2D transformed = theOp->transformPoint(tableKnobI->getValue(i, Column::X), tableKnobI->getValue(i, Column::Y));
            float x = static_cast<float>(oc.to_proxy_x(static_cast<float>(transformed.x)));
            float y = static_cast<float>(oc.to_proxy_y(static_cast<float>(transformed.y)));

            // begin_handle tells Nuke "Hey, the area around this point should be clickable"
            begin_handle(ctx, handle_cb, i, 0, 0, 0, ViewerContext::kMovePointCursor);
            if (knob(kShowLabelsKnobName)->get_value()) {
                gl_text(tableKnobI->getCellString(i, Column::NAME).c_str(), x, y);
            }
            // Standard raw OpenGL syntax to draw a tiny dot representing the point
            glBegin(GL_POINTS);
            glVertex2f(x, y);
            glEnd();
            end_handle(ctx);
        }

        // Draws the white dotted/solid line box when click+dragging empty space
        if (theOp->_isMarqueeSelecting) {
            glColor(0xffffffff);
            glBegin(GL_LINE_LOOP);
            glVertex2f(theOp->_marqueeStart.x, theOp->_marqueeStart.y);
            glVertex2f(theOp->_marqueeEnd.x, theOp->_marqueeStart.y);
            glVertex2f(theOp->_marqueeEnd.x, theOp->_marqueeEnd.y);
            glVertex2f(theOp->_marqueeStart.x, theOp->_marqueeEnd.y);
            glEnd();
        }

        // Determines which mouse cursor Nuke displays (like the crosshair vs standard arrow)
        ViewerContext::Cursor cursor = ViewerContext::kArrowCursor;
        if (ctx->event() == CURSOR) {
            if (!theOp->points_locked && (theOp->create_mode || ((ctx->state() & CTRL) && (ctx->state() & ALT) && !(ctx->state() & SHIFT)))) {
                cursor = ViewerContext::kAddPointCursor;
            }
        }

        // Invisible fallback handle across the whole screen allowing for clicks on empty space
        begin_handle(Knob::ANYWHERE, ctx, handle_cb, -1, 0, 0, 0, cursor);
        end_handle(ctx);
    }
}

bool PointsTableKnob::build_handle(ViewerContext* ctx) {
    return (ctx->transform_mode() == VIEWER_2D);
}


// -----------------------------------------------------------------------------
// SECTION 9: REGISTRATION
// -----------------------------------------------------------------------------
// This section acts as the entry point when Nuke launches. It registers the node
// into Nuke's internal factory, linking our Class name ("PointGradient") to the 
// build function so Nuke knows how to instantiate it.

static Iop* build(Node* node) { return new PointGradient(node); }
Iop::Description PointGradient::d(RCLASS, "Draw/PointGradient", build);

OpHints PointGradient::opHints() const {
    return OpHints::eChainable;
}
