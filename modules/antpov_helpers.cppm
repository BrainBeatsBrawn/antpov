module;

#include <iostream>
#include <string>
#include <cstdint>
#include <vector>

export module antpov.helpers;

import sm.vvec;
import sm.vec;

import mplot.tools;
import mplot.compoundray.eyevisual;

export namespace antpov
{
    // The flags recorded by the experimenters
    enum class antflags : std::uint8_t
    {
        bush,
        cookie,
        shadow,
        visibility
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
            twodpos.set_from_str (line, ",");
            positions.push_back (twodpos);
            // Get flags from third entry
            tokens.clear();
            tokens = mplot::tools::stringToVector (line, ",");
            if (tokens.size() > 2) {
                std::uint32_t fl = std::stoi (tokens[2]);
                antflags.push_back (fl);
            } else {
                antflags.push_back (0u);
            }
        }
        return true;
    }

} // namespace antpov
