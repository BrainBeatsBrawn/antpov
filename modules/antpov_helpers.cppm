module;

#include <iostream>
#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
#include <tuple>

export module antpov.helpers;

import sm.vvec;
import sm.vec;
import sm.algo;
import mplot.tools;

export namespace antpov
{
    // The flags recorded by the experimenters
    enum class antflags : std::uint8_t
    {
        bush,
        cookie,
        shadow,
        invisible,
        direction_uncertain
    };

    // Read a simple csv with 2D coordinates. Should also read flags. Ah - this is ant specific
    bool read_csv (const std::string& path, sm::vvec<sm::vec<float, 2>>& positions, sm::vvec<std::uint32_t>& antflags)
    {
        std::ifstream f (path.c_str(), std::ios::in);
        if (f.is_open() == false) { return false; }
        std::string line;
        std::vector<std::string> tokens;
        while (std::getline (f, line)) {
            sm::vec<float, 2> twodpos;
            // Tokenize line into the coordinates and the flags
            try {
                twodpos.set_from_str (line, ","); // may throw
            } catch (const std::exception& e) {
                std::cout << "Failed to read csv line " << line << std::endl;
                continue;
            }
            positions.push_back (twodpos);
            // Get flags from third entry
            tokens.clear();
            tokens = mplot::tools::stringToVector (line, ",");
            std::uint32_t fl = 0u;
            if (tokens.size() == 6) {
                // We have flags as four columns: 1,0,1,0
                fl = std::stoi (tokens[2]) + 2 * std::stoi (tokens[3])+ 4 * std::stoi (tokens[4])+ 8 * std::stoi (tokens[5]);
            } else if (tokens.size() == 3) {
                // We have flags compressed into a single integer, so 1,0,1,0 would become 10 (decimal)
                fl = std::stoi (tokens[2]);
            } // else fl is 0u
            antflags.push_back (fl);
        }
        return true;
    }

    /*
     * Process the 2D positions into 2D antflags and directions
     *
     * \tparam replace_uncertain_directions If true, then attempt to replace uncertain directions
     * with an average direction. If false, simply mark the uncertainty
     *
     * \param positions input. the coordinates or each agent location
     *
     * \param antflags input/output. existing antflags, which may be updated with 'direction uncertain'
     *
     * \param dirns output. the computed agent directions
     *
     * \param block input. number of elements per block. A parameter
     *
     * \param max_delta_phi input. A threshold parameter
     *
     * \return tuple containing pos_orig (original positions for debug vis) and dirn_orig (original directions for debug vis)
     */
    template <bool replace_uncertain_directions = false, bool fix_positions_with_invisibility_flag = false>
    std::tuple<sm::vvec<sm::vec<float, 2>>, sm::vvec<sm::vec<float, 2>>>
    process_positions (sm::vvec<sm::vec<float, 2>>& positions, sm::vvec<std::uint32_t>& antflags,
                       sm::vvec<sm::vec<float, 2>>& dirns,
                       const std::uint32_t block = 3, const float max_delta_phi = 2.8f)
    {
        // Find our instantaneous directions using the next datum

        sm::vvec<float> phi (positions.size(), 0.0f); // Absolute angle of direction
        sm::vvec<float> dphi (positions.size(), 0.0f); // angle change from one movement to the next

        if constexpr (fix_positions_with_invisibility_flag == true) {
            // use antflags::invisible to interpolate between visible locations
            for (std::uint32_t i = 1; i < positions.size(); ++i) {
                if ((antflags[i] & 8u) == 8u) {
                    // positions[i] is invisible. Find the next *visible* position before interpolating
                    for (std::uint32_t j = i; j < positions.size(); ++j) {
                        if ((antflags[j] & 8u) != 8u) {
                            // Ant is visible again. Update positions from i to j by linear interpolation
                            sm::vec<float, 2> v = positions[j] - positions[i-1];
                            float intvl = static_cast<float>(j - i + 1);
                            for (std::uint32_t k = i; k < j; ++k) {
                                positions[k] = positions[i-1] + (v * ((static_cast<float>(k - i)) / intvl));
                            }
                            i = j - 1;
                            break;
                        }
                    }
                } // else positions[i] is visible, move to next
            }
        }

        for (std::uint32_t i = 0; i < positions.size() - 1; ++i) {

            dirns[i] = positions[i+1] - positions[i];
            phi[i] = dirns[i].angle(); // initially -pi to pi

            if (i > 0) {
                dphi[i] = dirns[i].angle (dirns[i-1]);
                if (std::isnan (dphi[i])) {
                    dphi[i] = 0.0f;
                    phi[i] = phi[i - 1];
                } // caused if dirns[i] or [i-1] had zero length
            }
        }

        // Analyse the angle change during blocks of movements. If angle change is greater than
        // threshold then we're milling about.

        sm::vvec<sm::vec<float, 2>> pos_orig;
        sm::vvec<sm::vec<float, 2>> dirn_orig;

        sm::vvec<float> blk_angles (block, 0.0f);
        std::uint32_t blk_needs_update = std::numeric_limits<std::uint32_t>::max();
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
            if (blk_angles.range().span() > max_delta_phi) { // then mark this block as 'direction uncertain'
                if constexpr (replace_uncertain_directions) {
                    blk_needs_update = i;
                }
                for (std::uint32_t j = 0; j < block && i + j < positions.size(); ++j) {
                    antflags[i + j] |= 16u; // mark as 'direction uncertain'
                    if constexpr (replace_uncertain_directions) {
                        pos_orig.push_back (positions[i + j]); // now these are 'original'
                        dirn_orig.push_back (dirns[i + j]);
                        // Want to replace dirns[i + j] with the average of the start direction and the
                        // direction at the start of the next direction-certain block. That means storing the
                        // start direction and coming back to update.
                    }
                }
                i += block - 1; // because this block is direction-uncertain, jump all the way to the next block, instead of just ++i
            } else {
                if constexpr (replace_uncertain_directions) {
                    // This is a 'good block' so update any preceding bad blocks
                    if (blk_needs_update < std::numeric_limits<std::uint32_t>::max()) {
                        auto newdirn = (dirns[blk_needs_update] + dirns[i]) * 0.5f;
                        for (std::uint32_t j = blk_needs_update; j < i; ++j) { dirns[j] = newdirn; }
                        blk_needs_update = std::numeric_limits<std::uint32_t>::max();
                    }
                }
            }
        }

        // Post-process antflags uncertainty - blur it, then re-signum it, so that clusters of uncertainty become larger

        // Convert flags to floats
        sm::vvec<float> uncer (positions.size(), 0.0f);
        for (std::uint32_t i = 0; i < uncer.size(); ++i) {
            uncer[i] = (antflags[i] & 16u) == 16u ? 1.0f : 0.0f;
        }
        // Perform the convolution then signum
        sm::vvec<float> kern (5, 0.0f); // 5 is a parameter really
        kern.set_from (1.0f / kern.size());
        uncer.convolve_inplace (kern);
        uncer.signum_inplace();
        // Convert floats back to flags
        for (std::uint32_t i = 0; i < uncer.size(); ++i) {
            if (uncer[i] > 0.0f) { antflags[i] |= 16u; }
        }

        return { pos_orig, dirn_orig };
    }

} // namespace antpov
