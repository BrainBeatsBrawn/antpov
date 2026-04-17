#include <iostream>
#include <cstdint>
#include <memory>
#include <cmath>

import sm.vec;
import sm.vvec;
import sm.algo;
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
     * Process the positions and directions
     */

    // Find our instantaneous directions using the next datum
    sm::vvec<sm::vec<float, 2>> dirns (positions.size(), sm::vec<float, 2>{});
    sm::vvec<float> d_x (positions.size(), 0.0f);
    sm::vvec<float> d_y (positions.size(), 0.0f);

    sm::vvec<float> phi (positions.size(), 0.0f); // Absolute angle of direction
    sm::vvec<float> dphi (positions.size(), 0.0f); // angle change from one movement to the next
    sm::vvec<float> turnlen (positions.size(), 0.0f);
    sm::vvec<float> turnlen_over_dphi (positions.size(), 0.0f);
    sm::vvec<float> clr (positions.size(), 0.0f);

    for (std::uint32_t i = 0; i < positions.size(); ++i) {
        dirns[i] = positions[i+1] - positions[i];
        d_x[i] = dirns[i][0];
        d_y[i] = dirns[i][1];
        phi[i] = dirns[i].angle(); // initially -pi to pi
        if (std::isnan (phi[i])) { std::cout << "have nan\n"; }
        //sm::algo::zero_to_twopi (phi[i-1]); // constrain here?
        if (i > 0) {
            dphi[i] = dirns[i].angle (dirns[i-1]);
            if (std::isnan (dphi[i])) {
                dphi[i] = 0.0f;
                phi[i] = phi[i - 1];
            } // caused if dirns[i] or [i-1] had zero length

            turnlen[i] = /*dirns[i].length() + */ dirns[i-1].length();
            std::cout << "Angle " << i << " " << phi[i] << ", delta angle: " << dphi[i] << ", length " << turnlen[i] << std::endl;

            if (std::abs(dphi[i]) > 0) {
                turnlen_over_dphi[i] = turnlen[i] / std::abs(dphi[i]);
            } else {
                turnlen_over_dphi[i] = 1;
            }
        }
    }

    std::cout << "turnlen_over_dphi range: " << turnlen_over_dphi.range() << std::endl;

    // Uncertain/changed directions
    sm::vvec<sm::vec<float, 2>> pos_orig;
    sm::vvec<sm::vec<float, 2>> dirn_orig;

    // Analyse the angle change during blocks of movements. If angle change is greater than
    // threshold then we're milling about.

    // Should we replace milling directions?
    constexpr bool change_milling = false;

    sm::vvec<float> blk_angles (block, 0.0f);
    for (std::uint32_t i = 0; i < positions.size(); ++i) {

        // Get mean vector for block starting at i
        sm::vec<float, 2> dav = {};
        for (std::uint32_t j = 0; j < block; ++j) {
            dav += dirns[i + j];
            blk_angles[j] = dirns[i + j].angle();
        }
        dav /= block; // dav is now the direction average

        blk_angles -= dav.angle(); // subtract mean angle from blk_angles...
        for (std::uint32_t j = 0; j < block; ++j) { sm::algo::minus_pi_to_pi (blk_angles[j]); } // and offset, so they cluster around 0

        // Is the span of the range of angles in the block above threshold?
        if (blk_angles.range().span() > max_delta_phi) { // then mark this block as 'milling'
            for (std::uint32_t j = 0; j < block && i + j < positions.size(); ++j) {
                clr[i + j] = 0.9f;
                if constexpr (change_milling) {
                    pos_orig.push_back (positions[i + j]); // now these are 'original'
                    dirn_orig.push_back (dirns[i + j]);
                    if (j > 0) { dirns[i + j] = dirns[i]; } // you could change the direction in some way
                }
            }
            i += block - 1; // because this block is milling, jump all the way to the next block, instead of just ++i
        }
    }

    // Ensure useful colours by having at least one 0 and one 1
    clr[0] = 0.0f;
    clr[positions.size() - 1] = 1.0f;

#if 0 // convolution
    sm::vvec<float> kern (3, 0.0f);
    kern.set_from (1.0f / kern.size());
    d_x.convolve_inplace (kern);
    d_y.convolve_inplace (kern);
    [[maybe_unused]] sm::vvec<sm::vec<float, 2>> dirns_smth (positions.size());
    for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {
        dirns_smth[i] = { d_x[i], d_y[i] };
    }
#endif
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
    if constexpr (change_milling) {
        dsq.quiver_flagset.set (mplot::quiver_flags::colour_fixed);
        dsq.linecolour = mplot::colour::purple;
        gv->setdata (pos_orig, dirn_orig, dsq);
    }
    gv->finalize();
    v.addVisualModel (gv);

    v.keepOpen();
}
