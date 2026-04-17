#include <iostream>
#include <cstdint>
#include <memory>
#include <tuple>
#include <cmath>

import sm.vec;
import sm.vvec;
import antpov.helpers;
import mplot.visual;
import mplot.graphvisual;

constexpr std::int32_t glver = mplot::gl::version_4_3;

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
    if (antpov::read_csv (path, positions, antflags) == false) {
        std::cout << "Failed to read\n";
        return -1;
    } else {
        std::cout << "Read " << positions.size() << " ant positions from CSV\n";
    }
    /*
     * Process the positions
     */
    sm::vvec<sm::vec<float, 2>> dirns (positions.size(), sm::vec<float, 2>{});
    const auto[pos_orig, dirn_orig] = antpov::process_positions (positions, antflags, dirns, block, max_delta_phi);

    // Get colour from antflags
    sm::vvec<float> clr (positions.size(), 0.0f);
    for (std::uint32_t i = 0; i < antflags.size() && i < positions.size(); ++i) {
        if ((antflags[i] & 16u) == 16u) { clr[i] = 1.0f; }
    }

    /*
     * Plot the results
     */

    mplot::Visual<glver> v(1024, 768, "Ant direction analysis");
    // Set up quiver dataset style
    mplot::DatasetStyle dsq (mplot::stylepolicy::markers);
    dsq.markerstyle = mplot::markerstyle::quiver_fromcoord;
    dsq.markersize /= 16;
    dsq.quiver_colourmap.setType (mplot::ColourMapType::Jet);
    dsq.quiver_flagset.reset (mplot::quiver_flags::colour_fixed);
    dsq.quiver_flagset.set (mplot::quiver_flags::thickness_fixed);
    dsq.quiver_flagset.reset (mplot::quiver_flags::show_zeros);
    dsq.quiver_flagset.reset (mplot::quiver_flags::marker_sphere);
    dsq.linewidth /= 10;
    // Create the graph
    sm::vec<float> offset = { -1.5f, -1.0f, 0.0f };
    auto gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setsize (3, 2);
    gv->setdata (positions, dirns, clr, dsq);
    if (!pos_orig.empty()) {
        dsq.quiver_flagset.set (mplot::quiver_flags::colour_fixed);
        dsq.linecolour = mplot::colour::purple;
        gv->setdata (pos_orig, dirn_orig, dsq);
    }
    gv->finalize();
    v.addVisualModel (gv);

    v.keepOpen();
}
