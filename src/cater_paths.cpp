// Graph all the CATER paths from  mosaic and encoded in files like
//
// Ant03R01.csv
// Ant03R01_labels.csv etc

#include <iostream>
#include <cstdint>
#include <memory>
#include <tuple>
#include <cmath>
#include <format>
#include <map>
#include <vector>
#include <string>
#include <cstdlib>

import sm.vec;
import sm.vvec;
import sm.quaternion;
import antpov.helpers;
import mplot.visual;
import mplot.graphvisual;
import mplot.tools;

constexpr std::int32_t glver = mplot::gl::version_4_3;

std::int32_t main (std::int32_t argc, char* argv[])
{
    // Base dir
    std::string path = "./data/seville/orig_paths";
    if (argc > 1) { path = std::string (argv[1]); }

    float markersz = 0.003f;
    float markermult = 1.0f;
    if (argc > 2) { markermult = std::atof (argv[2]); }

    std::vector<std::string> labels_files;
    {
        std::vector<std::string> files;
        mplot::tools::readDirectoryTree (files, path);
        for (auto f : files) {
            if (f.find ("_labels.csv") != std::string::npos) {
                labels_files.push_back (f);
            }
        }
    }

    mplot::Visual<glver> v(1920, 1080, "CATER paths");
    v.setSceneTrans (sm::vec<float,3>{ float{-5.32225}, float{-3.8499}, float{-17.7575} });
    v.setSceneRotation (sm::quaternion<float>{ float{1}, float{0}, float{0}, float{0} });

    std::map<std::uint32_t, std::unique_ptr<mplot::GraphVisual<float, glver>>> graphs;
    sm::vec<float> offset = {};
    constexpr float spc = 2.8f;

    std::vector<std::array<float, 3>> ai_colours = {
        mplot::colour::black,         // There was no Ant 0
        mplot::colour::grey70,
        mplot::colour::grey70,
        mplot::colour::dodgerblue3,   // Ant 3
        mplot::colour::crimson,       // Ant 4
        mplot::colour::mediumpurple1, // Ant 5
        mplot::colour::brick,         // Ant 6
        mplot::colour::plum1,         // Ant 7
        mplot::colour::purple3,       // Ant 8
        mplot::colour::yellow3,       // Ant 9
        mplot::colour::turquoise2,    // Ant 10
        mplot::colour::maroon2,       // Ant 11
        mplot::colour::darkorange2,   // Ant 12
        mplot::colour::grey70,
        mplot::colour::grey70
    };

    std::uint32_t num = 0;
    for (auto f : labels_files) {

        std::cout << "Graphing file " << f << std::endl;
        // Get Ant index from position p to posn before 'R'
        std::string::size_type lstart = f.find ("Ant") + 3;
        std::string::size_type lend = f.find ("R", lstart);
        if (lend == std::string::npos) {
            lend = f.find ("Z", lstart);
        }
        if (lend == std::string::npos) {
            std::cout << "Uh oh\n";
            return -1;
        }
        std::string ais = f.substr (lstart, lend - lstart);
        std::cout << "Ant index substr: " << ais << std::endl;
        std::uint32_t antidx = std::stoi (ais);
        std::cout << "Ant index: " << antidx << std::endl;

        std::uint32_t routeidx = 0;
        if (f.find("ZVOP") != std::string::npos) {
            // ZVOP Zero Vector Opposite Side - Ant is allowed to go to nest but before arrival is
            // placed in opposite side of the feeding areay
            routeidx = 9999;
        } else if (f.find("ZVSF") != std::string::npos) {
            // ZVOP Zero Vector Semi Familiar - Ant is allowed to go to nest but before arrival is
            // placed near but not at the feeding area
            routeidx = 8888;
        } else if (f.find("ZVF") != std::string::npos) {
            // ZVOP Zero Vector Familiar - Ant is allowed to go to nest but before arrival is
            // placed back at the feeding area
            routeidx = 999;
        } else {
            std::string::size_type iend = f.find_first_of ('_');
            routeidx = std::stoi (f.substr (lend + 1, iend - (lend + 1)));
        }
        std::cout << "Route index: " << routeidx << std::endl;

        sm::vvec<std::uint32_t> antflags;
        sm::vvec<sm::vec<float, 2>> positions;
        std::string filepath = f;
        if (antpov::read_csv (filepath, positions, antflags) == false) {
            std::cout << "Failed to read " << filepath << " continue..." << std::endl;
            continue;
        } else {
            std::cout << "Read " << positions.size() << " ant positions from CSV\n";
            if (positions.size() == 0) {
                throw std::runtime_error ("Yikes");
            }
            // Invert y
            for (auto& p : positions) { p[1] *= -1; }
        }

        mplot::DatasetStyle ds (mplot::stylepolicy::markers);

        if (routeidx == 9999) {
            ds.datalabel = std::format ("Ant{:02d}ZVOP", antidx);
            ds.setcolour (mplot::colour::grey50);
        } else if (routeidx == 8888) {
            ds.datalabel = std::format ("Ant{:02d}ZVSF", antidx);
            ds.setcolour (mplot::colour::grey70);
        } else if (routeidx == 999) {
            ds.datalabel = std::format ("Ant{:02d}ZVF", antidx);
            ds.setcolour (mplot::colour::black);
        } else {
            ds.datalabel = std::format ("Ant{:02d}R{:02d}", antidx, routeidx);
            ds.setcolour (ai_colours [antidx]);
        }
        ds.markersize = markersz * markermult;

        std::cout << "2\n";
        if (graphs.contains (antidx) == false) {
            std::uint32_t ix = (antidx - 3) % 4u;
            std::uint32_t iy = (antidx - 3) / 4u;
            offset = { spc * ix, spc * iy, 0.0f };
            graphs[antidx] = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
            graphs[antidx]->set_parent (v.get_id());
            graphs[antidx]->setsize (2, 2);
            // Same limits on each graph
            graphs[antidx]->setlimits (10000, 40000, -28000, 2000);
            graphs[antidx]->setdata (positions, ds);
            num++;

        } else {
            graphs[antidx]->setdata (positions, ds);
            num++;
        }
    }

    for (std::uint32_t i = 0; i < 16; ++i) {
        if (graphs.contains (i)) {
            graphs[i]->finalize();
            v.addVisualModel (graphs[i]);
        }
    }

    std::cout << "Plotted " << num << " datasets\n";

    v.keepOpen();
}
