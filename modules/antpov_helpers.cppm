module;

#include <iostream>
#include <string>
#include <cstdint>
#include <vector>

export module antpov.helpers;

import sm.vvec;
import sm.vec;
import sm.quaternion;

import mplot.tools;
import mplot.compoundray.eyevisual;

import craysim.visual;

export namespace antpov
{
    // Parse cmd line to find the path and set app specific options. Return hoverheight
    std::string parse_inputs (int argc, char* argv[])
    {
        std::string hovh = "";
        for (int i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-H") { hovh = std::string(argv[++i]); }
        }
        return hovh;
    }

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

    // Add a suitable 2D projection to show our ant eye in a flat fiew
    template <int glver>
    void add_ant_eye_spherical_projection (craysim::visual<glver>& v, mplot::compoundray::EyeVisual<glver>* eyevm2)
    {
        // First eye of eye pair (one spherical projection)
        std::uint32_t sz = 1024;
        float ps_rad = 0.0001f;                  // projection sphere radius
        sm::vec<> centre = { -0.00002f, 0, 0 };  // projection sphere centre

        if (v.oces_reader.read_success == true) {
            sz = v.oces_reader.position.size();
            ps_rad = 0.0002f;
            centre = { -0.00056, 0.00005, -0.00005 };
        }

        sm::mat<float, 4> twod_tr;                            // twod projection transformation
        float twod_scale = 1.0f;                              // twod projection scaling
        sm::vec<> twod_offset = { 0.0001f, 0.0f, 0.0f };      // twod projection translation to move to centre
        sm::vec<> twod_offset2 = { -0.0004f, 0.0007f, 0.0f }; // post scale/rotate translation
        sm::vec<> twod_shift = {0,0.0006,0};
        float rotn = -sm::mathconst<float>::pi_over_8;
        auto ptype = mplot::compoundray::EyeVisual<glver>::projection_type::mercator;
        if (v.oces_reader.read_success == true) {
            std::cout << "Read from oces file!!\n";
            ptype = mplot::compoundray::EyeVisual<glver>::projection_type::equirectangular;
            twod_tr.translate (twod_shift);
        } else {
            twod_tr.translate (twod_offset2);
            twod_tr.scale (twod_scale);
            twod_tr.rotate (sm::vec<>::uy(), rotn);
            twod_tr.translate (twod_offset);
        }

        // Projection sphere rotation about x axis by 0.2 radians. Numbers determined using oces_viewer
        sm::quaternion<float> psrotn (sm::vec<>::ux(), 0.2f);

        eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn, 0, sz/2);

        // Second eye of the eye pair (another spherical projection)
        if (v.oces_reader.read_success == true) {
            if (v.oces_reader.mirrors.empty() == false) {
                centre = (v.oces_reader.mirrors[0] * centre).less_one_dim();
                sm::vec<> twod_shift_left = twod_shift;
                twod_shift_left[0] *= -1.0f;
                twod_tr.set_identity();
                twod_tr.translate (twod_shift_left);
                eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn.invert(), sz/2, sz);
            }
        }
    }

} // namespace antpov
