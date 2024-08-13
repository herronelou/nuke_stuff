// CurveRenderer.cpp

static const char* const CLASS = "CurveRenderer";
static const char* const HELP =
"Render an animation curve as pixels. Note that while mst knobs can be animated, "
"Nuke doesn't support animated animation curves, therefore the rendered curve will be the same in every frame.";

// this header file contains our base class 'DrawIop'
#include "DDImage/DrawIop.h"

// Knobs are the user interface elements that appear in the node panels
#include "DDImage/Knobs.h"

// this is a collection of math functions, and cross platform
// compatibility of math:
#include "DDImage/DDMath.h"

#include <vector>
#include <array>

using namespace DD::Image;


class CurveRenderIop : public DrawIop
{
    double thickness, _thickness; // Underscored value is for the effective thickness, taking into account proxy scaling
    // bounding box of the final drawing
    double x, y, r, t;
    // bounding box of the curve
    double x1, y1, r1, t1;
    // softness of the curve
    double soft;
    // curve to draw
    double _placeholder;
    // Proxy scale multipliers
    double par;

    // Make a vector to store the baked curve for each X pixel in our render area
    std::vector<std::array<float, 2>> baked_curve;
public:
    void append(Hash& hash);
    void _validate(bool) override;
    bool draw_engine(int y, int x, int r, float* buffer) override;
    // make sure that all members are initialised
    CurveRenderIop(Node* node) : DrawIop(node)
    {
        thickness = _thickness = 5.0;
        x = y = r = t = x1 = y1 = r1 = t1 = 0.0;
        soft = 1.0;
        _placeholder = 0.0;
        par = 1.0;
    }
    void knobs(Knob_Callback) override;
    float distance_to_segment_squared(float x, float y, float x1, float y1, float x2, float y2);

    const char* Class() const override { return CLASS; }
    const char* node_help() const override { return HELP; }
    static const Iop::Description d;
};

// The knobs function creates and maintains the user interface elementes in
// the Node panels and the interactive handles in the Nuke viewer:

void CurveRenderIop::knobs(Knob_Callback f)
{
    // allow DrawIop to add its own knobs to handle input
    input_knobs(f);

    // The curve that we want to draw
    Double_knob(f, &_placeholder, "target_curve", "Curve to Draw");
    SetFlags(f, Knob::NO_RERENDER); // We hash this one ourselves, so remove it from the default hash
    Divider(f);

    // this knob provides controls for the position and size of our drawing.
    // It also manages the rectangular handle box in all connected viewers.
    BBox_knob(f, &x, "drawing_area", "Drawing Area");

    // this knob provides controls for the position and size of our curve.
    BBox_knob(f, &x1, "curve_area", "Curve Area");
    SetFlags(f, Knob::NO_HANDLES | Knob::NO_PROXYSCALE);

    // This knob manages the thickness of the curve, in pixels.
    Double_knob(f, &thickness, "thickness");
    // This knob manages user input for the rectangles edge softness
    Double_knob(f, &soft, "softness");

    // allow DrawIop to add its own knobs to handle output
    output_knobs(f);
}

inline float remap(float value, float low1, float high1, float low2, float high2) {
    if (high1 == low1) {
        return low2; // or return high2, this is to prevent a divide by zero and either value would be correct.
    }
    return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
}

inline float distance_squared(float x1, float y1, float x2, float y2) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

// Append is called to add to the hash that uniquely identifies this node
// and its current state. This is used to determine if the node needs to
// be re-executed. The hash is a unique number that changes when the node
// changes. We need to hash our whole curve, as the Op draws the entire curve.
void CurveRenderIop::append(Hash& hash)
{
    // Handle degenerate case
    if (x1 > r1) {
		return;
	}
	// Get the curve to draw
	Knob* curve_knob = knob("target_curve");
	if (!curve_knob) {
		return;
	}

    // We sample the curve at each integer value in the range of the curve 
    // that's in the render area. This will not catch sub-frame changes in
    // the curve, but it should be good enough.
    for (int i = floor(x1); i < ceil(r1); i++) {
		hash.append(curve_knob->get_value_at(i, 0));
	}
}

// _validate (note the underscore!) in DrawIop's should set the
// desired drawing area as a bounding box. If there is nothing to draw
// (for example, the user requested a zero width rectangle), _validate
// should call disable(). As long as an Iop is disabled, Nuke will not
// perform any engine calls to this node.

void CurveRenderIop::_validate(bool for_real)
{
    // don't bother calling the engine in degenerate cases
    if (x >= r || y >= t || thickness <= 0) {
        set_out_channels(Mask_None);
        copy_info();
        return;
    }
    // Get the curve to draw
    Knob* curve_knob = knob("target_curve");
    if (!curve_knob) {
        set_out_channels(Mask_None);
        copy_info();
        return;
    }

    set_out_channels(Mask_All);
    // make sure that we get enough pixels to build our curve
    DrawIop::_validate(for_real,
        int(floor(x)),
        int(floor(y)),
        int(ceil(r)),
        int(ceil(t)));

    // Get our effective thickness, based on the proxy scale
    _thickness = outputContext().to_proxy_w(thickness);
    par = format().pixel_aspect();
    // Bake the curve into the baked_curve vector
    float time;
    // Calculate how much time is represented by a one pixel distance in the render area
    float time_offset = (r1 - x1) / (r - x);
    int handles = (ceil(_thickness) + 1) / 2;
    int curve_samples = int(ceil(r)) - int(floor(x)) + (handles * 2) + 1;
    if (curve_samples != baked_curve.size())
    {
        // I'm not sure if the condition is necessary, but adding is as I'm trying to prevent a crash caused by memory allocation
        baked_curve.resize(curve_samples);
    }
    
    for (int i = 0; i < baked_curve.size(); i++) {
        // We need to get the value of the curve at this point.
        // We can use the get_value_at function of the knob to get the value of the curve at a certain time.
        // The time should be remapped to the range of the curve (x1, r1) and the render area (x, r).
        time = remap(i+x-handles, x, r, x1, r1);
        std::array<float, 2> segment;
        segment[0] = remap(curve_knob->get_value_at(time - time_offset, 0), y1, t1, y, t);
        segment[1] = remap(curve_knob->get_value_at(time, 0), y1, t1, y, t);
        baked_curve[i] = segment;
	}
}

// This is the finally the function that does some actual drawing.
//
// Warning: This function may be called by many different threads at
// the same time for different lines in the image. Do not modify any
// non local variable!  Lines will be called up in a random order at
// random times!
//
// This is the plugin's chance to put useful drawings into the pixel
// buffer.  All drawing is done in floating point space for a single
// component. Color may be added later by other operators in the
// graph.

bool CurveRenderIop::draw_engine(int Y, int X, int R, float* buffer)
{
    // lets see if there is anything to draw at all
    if (Y < (int)floor(y))
        return false;
    if (Y >= (int)ceil(t))
        return false;

    float value;
    float distance_to_curve;
    int thick = ceil(_thickness) + 1;
    std::array<float, 2> segment;
    float fully_opaque_threshold = (1.0 - soft) * (_thickness / 2);

    // We need to keep in mind that X is not always corresponding to the whole BBOX, it could be a subregion of it.
    // As we built our baked curve to be the entire BBOX, we need to make sure that we use the correct starting point and don't go out of bounds.
    // R is the right side of the ROI + 1.
    // X is the left side of the ROI - 1 if we see the left edge, otherwise it's the left side of the ROI.
    // DrawIop seems to add black outside, so the first and last pixel can be any value, as it seems to kill them. We ignore them by only iterating based on our own BBOX.
    int start = X - int(floor(x)) + (thick/2);
    int end = R - int(floor(x)) + (thick/2);

    
    for (int i = start; i < end; i++) 
    {
        value = _thickness*_thickness*2;  // Use a large value to start with, anything above squared thickness will do as it will be made black later
        // Iterate over the values in the baked curve that may be in range of the thickness, and keep the nearest one
        for (int j = -thick/2+1; j < thick/2+1; j++) 
		{
            // If the value is out of bounds, skip it
            if (i + j < 0 || i + j >= baked_curve.size()) continue;
			// Calculate the distance to the curve
            segment = baked_curve[i+j];
            distance_to_curve = distance_to_segment_squared(X, Y, X-1+j, segment[0], X+j, segment[1]);
            //distance_to_curve = distance_squared(X, Y, X-1+j, segment[0]);
			if (distance_to_curve < value) 
			{
				value = distance_to_curve;
                // If the value is below the fully opaque threshold, we can break the loop as we know that the pixel will be fully opaque
                // This increases performance for large thickness values.
                if (value <= fully_opaque_threshold)
                {
                    break;
                }
			}
		}
        // Using the softness (as a float multiplier of the thickness) and the thickness, use smoothstep to calculate the value of the pixel
        // The value will be 1 if the pixel is on the curve, and 0 if it is outside the curve
        value = smoothstep(float(_thickness/2 + 0.001), fully_opaque_threshold, sqrt(value));
        buffer[X] = value;
        X++;
	}

    return true;
}

// This function calculates the distance from a point to a line segment, taking into account the pixel aspect ratio.
float CurveRenderIop::distance_to_segment_squared(float x, float y, float x1, float y1, float x2, float y2) {

    // Before calculating the distance, we need to take into account the pixel aspect ratio
    if (par != 1.0) 
    {
        x *= par;
        x1 *= par;
        x2 *= par;
	}
    

    float A = x - x1;
    float B = y - y1;
    float C = x2 - x1;
    float D = y2 - y1;

    float dot = A * C + B * D;
    float len_sq = C * C + D * D;
    float param = -1;
    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;

    float xx, yy;

    if (param < 0) {
        xx = x1;
        yy = y1;
    }
    else if (param > 1) {
        xx = x2;
        yy = y2;
    }
    else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }

    return distance_squared(x, y, xx, yy);
}

// Nuke will call this function to add a new Rectangle operator to the scene
// graph. Should you want to use the Nuke Wrapper, this would be the place to
// add it.
static Op* build(Node* node) { return new CurveRenderIop(node); }


// The Description class gives Nuke access to information about the plugin.
// The first element is the name under which the plugin will be known. The
// second argument is the constructor callback. The optional third argument
// is the License structure that you can use to make sure the caller is
// authorized to use your plugin.

const Op::Description CurveRenderIop::d(CLASS, "Draw/CurveRenderer", build);
