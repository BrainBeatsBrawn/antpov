#include <iostream>
#include <cstdint>
#include <memory>

import sm.vec;
import sm.vvec;
import antpov.helpers;
import mplot.visual;
import mplot.graphvisual;

constexpr int32_t glver = mplot::gl::version_4_3;

std::int32_t main (std::int32_t argc, char* argv[])
{
    std::string path = {};
    if (argc > 1) { path = std::string (argv[1]); }
    sm::vvec<std::uint32_t> antflags;
    sm::vvec<sm::vec<float, 2>> positions;
    if (antpov::read_csv (path, positions, antflags) == false) {
        std::cout << "Failed to read\n";
        return -1;
    } else {
        std::cout << "Read " << positions.size() << " ant positions from CSV\n";
    }

    // Maths time.

    // Find our instantaneous directions using the next datum
    sm::vvec<sm::vec<float, 2>> dirns (positions.size());
    sm::vvec<float> d_x (positions.size());
    sm::vvec<float> d_y (positions.size());

    for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {
        dirns[i] = positions[i+1] - positions[i];
        d_x[i] = dirns[i][0];
        d_y[i] = dirns[i][1];
    }

    sm::vvec<float> kern (3, 0.0f);
    kern.set_from (1.0f / kern.size());
    d_x.convolve_inplace (kern);
    d_y.convolve_inplace (kern);

    sm::vvec<sm::vec<float, 2>> dirns_smth (positions.size());
    for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {
        dirns_smth[i] = { d_x[i], d_y[i] };
    }

    // Now plot
    mplot::Visual<glver> v(1024, 768, "Ant direction analysis");

    // Set up quiver dataset style
    mplot::DatasetStyle dsq (mplot::stylepolicy::markers);
    dsq.markerstyle = mplot::markerstyle::quiver_fromcoord;
    dsq.markersize /= 32;
    dsq.quiver_colourmap.setType (mplot::ColourMapType::MonochromeRed);
    dsq.quiver_flagset.set (mplot::quiver_flags::colour_fixed);
    dsq.quiver_flagset.set (mplot::quiver_flags::thickness_fixed);
    dsq.quiver_flagset.reset (mplot::quiver_flags::show_zeros);
    dsq.linewidth /= 10;

    sm::vec<float> offset = { -1.5f, -1.0f, 0.0f };
    auto gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setthickness (0.0002);
    gv->setsize (3, 2);
    dsq.linecolour = mplot::colour::black;
    gv->setdata (positions, dirns, dsq);
    gv->finalize();
    v.addVisualModel (gv);

    gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setthickness (0.0002);
    gv->setsize (3, 2);
    dsq.linecolour = mplot::colour::crimson;
    gv->setdata (positions, dirns_smth, dsq);
    gv->finalize();
    v.addVisualModel (gv);

    v.keepOpen();
}
