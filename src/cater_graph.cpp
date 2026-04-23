#include <iostream>
#include <cstdint>
#include <memory>
#include <tuple>
#include <cmath>
#include <string>

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
        // Invert y
        for (auto& p : positions) { p[1] *= -1; }
    }
    /*
     * Process the positions
     */
    sm::vvec<std::uint32_t> antflags_ = antflags;
    sm::vvec<sm::vec<float, 2>> dirns (positions.size(), sm::vec<float, 2>{});
    const auto[pos_orig, dirn_orig] = antpov::process_positions<false, true> (positions, antflags_, dirns, block, max_delta_phi);

    // Get colour from antflags to plot visibility/invisibility
    sm::vvec<float> clr (positions.size(), 0.2f);
    for (std::uint32_t i = 0; i < antflags.size() && i < positions.size(); ++i) {
        if ((antflags[i] & 8u) == 8u) {
            clr[i] = 0.7f;
        }
    }

    /*
     * Plot the results
     */
    mplot::Visual<glver> v(1024, 768, "Ant direction analysis");
    // Create the graph
    sm::vec<float> offset = { -1.5f, -1.0f, 0.0f };
    auto gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setsize (3, 2);
    mplot::DatasetStyle ds (mplot::stylepolicy::markers);
    ds.markersize = 0.01f;
    gv->setdata (positions, clr, ds);

    gv->finalize();
    v.addVisualModel (gv);

    v.keepOpen();
}
