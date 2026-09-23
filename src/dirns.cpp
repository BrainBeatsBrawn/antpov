#include <iostream>
#include <cstdint>
#include <memory>
#include <tuple>
#include <cmath>
#include <string>
#include <tuple>

import sm.vec;
import sm.vvec;
import cater.helpers;
import mplot.visual;
import mplot.graphvisual;
import mplot.colourbarvisual;

constexpr std::int32_t glver = mplot::gl::version_4_3;

struct myvisual final : public mplot::Visual<glver>
{
    myvisual (int width, int height, const std::string& title) : mplot::Visual<glver> (width, height, title) {}
    float speedmag = 45.0f;
    bool needsupdate = true;
    bool smooth = false;
protected:
    void key_callback_extra (int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) override
    {
        if (key == mplot::key::n1 && action == mplot::keyaction::press) {
            speedmag /= 2.0f;
            needsupdate = true;
        }
        if (key == mplot::key::n2 && action == mplot::keyaction::press) {
            speedmag *= 2.0f;
            needsupdate = true;
        }
        if (key == mplot::key::n3 && action == mplot::keyaction::press) {
            speedmag /= 1.2f;
            needsupdate = true;
        }
        if (key == mplot::key::n4 && action == mplot::keyaction::press) {
            speedmag *= 1.2f;
            needsupdate = true;
        }
        if (key == mplot::key::s && action == mplot::keyaction::press) {
            smooth = !smooth;
            needsupdate = true;
        }
        if (key == mplot::key::h && action == mplot::keyaction::press) {
            std::cout << "Use '1'/'2' to change multiplier by multiples of 2\n";
            std::cout << "Use '3'/'4' to change multiplier by multiples of 1.2\n";
            std::cout << "Use 's' to toggle smoothing\n";
        }
    }
};

// Draw/re-draw the GraphVisual
std::tuple<mplot::GraphVisual<float, glver>*, mplot::ColourBarVisual<float, glver>*>
draw (myvisual& v,
      mplot::GraphVisual<float, glver>* gptr,
      mplot::ColourBarVisual<float, glver>* cbptr,
      sm::vvec<sm::vec<float, 2>>& positions,
      sm::vvec<sm::vec<float, 2>>& dirns,
      sm::vvec<std::uint32_t>& antflags,
      sm::vvec<float>& _ant_speed)
{
    if (gptr != nullptr) { v.removeVisualModel (gptr); }
    if (cbptr != nullptr) { v.removeVisualModel (cbptr); }

    sm::vvec<float> ant_speed = _ant_speed;

    if (v.smooth) {
        // Fix speed gaps
        for (std::uint32_t i = 0u; i < antflags.size() && i < positions.size(); ++i) {
            if ((antflags[i] & 16u) == 16u) {
                if (i > 0u) { ant_speed[i] = ant_speed[i-1]; }
            }
        }
        // then smooth
        using wrapdata = sm::vvec<float>::wrapdata;
        ant_speed.smooth_gauss_inplace<wrapdata::none> (2.0f, 2);
    }

    // Get colour from speed
    sm::vvec<float> clr = ant_speed;
    clr *= v.speedmag;
    // Threshold down
    clr.threshold_inplace (0.0f, 1.0f);
    // Replace flagged ones, too
    for (std::uint32_t i = 0; i < antflags.size() && i < positions.size(); ++i) {
        if ((antflags[i] & 16u) == 16u) {
            clr[i] = 1.0f;
        }
    }

    auto cmap = mplot::ColourMapType::CET_R1;

    const float sz = 0.5f;
    // Set up quiver dataset style
    mplot::DatasetStyle dsq (mplot::stylepolicy::markers);
    dsq.markerstyle = mplot::markerstyle::quiver_fromcoord;
    dsq.markersize /= sz * 8.0f;
    dsq.colourmap.setType (cmap); // Plasma is the default
    dsq.quiver_flagset.reset (mplot::quiver_flags::colour_fixed);
    dsq.quiver_flagset.set (mplot::quiver_flags::thickness_fixed);
    dsq.quiver_flagset.reset (mplot::quiver_flags::show_zeros);
    dsq.quiver_flagset.reset (mplot::quiver_flags::marker_sphere);
    dsq.linewidth /= sz * 5.0f;
    // Create the graph
    sm::vec<float> offset = { -1.5f, -1.0f, 0.0f };
    auto gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setsize (3, 2);
    gv->setdata (positions, dirns, clr, dsq);
    gv->finalize();
    mplot::GraphVisual<float, glver>* ptr = v.addVisualModel (gv);

    offset[0] += 3.1;
    auto cbv = std::make_unique<mplot::ColourBarVisual<float, glver>>(offset);
    cbv->set_parent (v.get_id());
    cbv->orientation = mplot::colourbar_orientation::vertical;
    cbv->tickside = mplot::colourbar_tickside::right_or_below;
    cbv->width = 0.06f;
    cbv->length = 0.4f;
    cbv->framelinewidth = 0.003f;
    cbv->tf.fontsize = 0.03f;
    cbv->cm.setType (cmap);
    constexpr float samples_per_second = 50.0f;
    cbv->scale.compute_scaling (0, samples_per_second / v.speedmag);
    cbv->label = "m/s";
    cbv->finalize();
    mplot::ColourBarVisual<float, glver>* cptr = v.addVisualModel (cbv);

    return { ptr, cptr };
}

std::int32_t main (std::int32_t argc, char* argv[])
{
    std::string path = {};
    if (argc > 1) { path = std::string (argv[1]); }

    std::uint32_t block = 3;
    if (argc > 2) { block = std::stoi (argv[2]); }
    float max_delta_phi = 2.8f; // a little less than pi
    if (argc > 3) { max_delta_phi = std::stof (argv[3]); }
    sm::vvec<std::uint32_t> antflags;
    sm::vvec<sm::vec<float, 2>> positions;
    if (cater::helpers::read_csv (path, positions, antflags) == false) {
        std::cout << "Failed to read\n";
        return -1;
    } else {
        std::cout << "Read " << positions.size() << " ant positions from CSV\n";
        // Invert y
        for (auto& p : positions) { p[1] *= -1; }
    }
    // Process the positions
    sm::vvec<sm::vec<float, 2>> dirns (positions.size(), sm::vec<float, 2>{});
    sm::vvec<float> ant_speed;
    const auto[pos_orig, dirn_orig] = cater::helpers::process_positions<false> (positions, antflags, dirns, ant_speed, block, max_delta_phi);

    // Visualize
    myvisual v(1024, 768, "Ant direction analysis");
    mplot::GraphVisual<float, glver>* gptr = nullptr;
    mplot::ColourBarVisual<float, glver>* cptr = nullptr;
    while (!v.readyToFinish()) {
        v.waitevents(0.017);
        if (v.needsupdate) {
            std::cout << "Re-draw with multiplier " << v.speedmag << "\n";
            std::tie(gptr, cptr) = draw (v, gptr, cptr, positions, dirns, antflags, ant_speed);
            v.needsupdate = false;
        }
        v.render();
    }
}
