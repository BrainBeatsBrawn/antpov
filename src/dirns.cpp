#include <iostream>
#include <cstdint>
#include <memory>
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
    float turnratiothresh = 0.001f;
    if (argc > 2) { turnratiothresh = std::stof (argv[2]); }
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
    sm::vvec<sm::vec<float, 2>> dirns (positions.size(), sm::vec<float, 2>{});
    sm::vvec<float> d_x (positions.size(), 0.0f);
    sm::vvec<float> d_y (positions.size(), 0.0f);

    sm::vvec<float> phi (positions.size(), 0.0f);
    sm::vvec<float> turnlen (positions.size(), 0.0f);
    sm::vvec<float> turnlen_over_phi (positions.size(), 0.0f);
    sm::vvec<float> clr (positions.size(), 0.0f);

    sm::vvec<sm::vec<float, 2>> pos_uncertain;
    sm::vvec<sm::vec<float, 2>> dirns_uncertain;

    for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {
        dirns[i] = positions[i+1] - positions[i];
        d_x[i] = dirns[i][0];
        d_y[i] = dirns[i][1];
        if (i > 0) {
            phi[i] = dirns[i].angle (dirns[i-1]);
            if (std::isnan (phi[i])) { phi[i] = 0.0f; } // caused if dirns[i] or [i-1] had zero length

            turnlen[i] = /*dirns[i].length() + */ dirns[i-1].length();
            std::cout << "Angle " << phi[i] << ", length " << turnlen[i] << std::endl;

            if (std::abs(phi[i]) > 0) {
                //turnlen_over_phi[i] = std::log (turnlen[i] / std::abs(phi[i])) + 9.2103;
                //turnlen_over_phi[i] = turnlen_over_phi[i] < 0 ? 0 : turnlen_over_phi[i];
                turnlen_over_phi[i] = turnlen[i] / std::abs(phi[i]);

            } else {
                turnlen_over_phi[i] = 1;
            }

            if (turnlen_over_phi[i] < turnratiothresh) {
                pos_uncertain.push_back (positions[i]);
                dirns_uncertain.push_back (dirns[i]);
                clr[i] = 0.2f;
            } else {
                clr[i] = 0.7f;
            }
        }
    }
    clr[0] = 0.0f;
    clr[1] = 1.0f;

    std::cout << "turnlen_over_phi range: " << turnlen_over_phi.range() << std::endl;

    sm::vvec<float> kern (3, 0.0f);
    kern.set_from (1.0f / kern.size());
    d_x.convolve_inplace (kern);
    d_y.convolve_inplace (kern);

    clr.convolve_inplace (kern);

    [[maybe_unused]] sm::vvec<sm::vec<float, 2>> dirns_smth (positions.size());
    for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {
        dirns_smth[i] = { d_x[i], d_y[i] };
    }

    // Now plot
    mplot::Visual<glver> v(1024, 768, "Ant direction analysis");

    // Set up quiver dataset style
    mplot::DatasetStyle dsq (mplot::stylepolicy::markers);
    dsq.markerstyle = mplot::markerstyle::quiver_fromcoord;
    dsq.markersize /= 16;
    dsq.quiver_colourmap.setType (mplot::ColourMapType::Jet);
    //dsq.quiver_flagset.set (mplot::quiver_flags::colour_fixed);
    dsq.quiver_flagset.set (mplot::quiver_flags::thickness_fixed);
    dsq.quiver_flagset.reset (mplot::quiver_flags::show_zeros);
    dsq.linewidth /= 5;

    sm::vec<float> offset = { -1.5f, -1.0f, 0.0f };
    auto gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setthickness (0.0002);
    gv->setsize (3, 2);
    //gv->setlimits (0.15, 0.5, 0.48, 0.59);
    //dsq.linecolour = mplot::colour::black;
    gv->setdata (positions, dirns, clr, dsq); // or cdata is another arg here
    gv->finalize();
    v.addVisualModel (gv);

#if 0
    gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setthickness (0.0002);
    gv->setsize (3, 2);
    gv->setlimits (0.15, 0.5, 0.48, 0.59);
    dsq.linecolour = mplot::colour::crimson;
    gv->setdata (pos_uncertain, dirns_uncertain, dsq);
    gv->finalize();
    v.addVisualModel (gv);
#endif

#if 0
    gv = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    gv->set_parent (v.get_id());
    gv->setthickness (0.0002);
    gv->setsize (3, 2);
    dsq.linecolour = mplot::colour::crimson;
    gv->setdata (positions, dirns_smth, dsq);
    gv->finalize();
    v.addVisualModel (gv);
#endif

    v.keepOpen();
}
