// Graph selected CATER paths from mosaic and encoded in files like
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

import sm.vec;
import sm.vvec;
import sm.grid;
import sm.quaternion;
import antpov.helpers;
import mplot.visual;
import mplot.graphvisual;
import mplot.gridvisual;
import mplot.tools;
import mplot.loadpng;

constexpr std::int32_t glver = mplot::gl::version_4_3;

std::int32_t main (std::int32_t argc, char* argv[])
{
    // Base dir
    std::string path = "./data/seville/orig_paths";
    if (argc > 1) { path = std::string (argv[1]); }

    float markersz = 0.003f;
    float markermult = 2.5f;
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

    // Just one graph this time
    sm::vec<float> offset = {-1.5, -1.5, 0};

    // Grid/graph width/height
    constexpr float gw = 3.0f;
    constexpr float gh = 3.0f;

    // Create a grid here to match an image
    std::string fn = "./data/mosaic_one_sixteenth.png";
    sm::vvec<sm::vec<float>> image_data_tlbr;
    sm::vec<unsigned int, 2> dims = mplot::loadpng (fn, image_data_tlbr, sm::vec<bool, 2>{false,false});
    std::cout << "Image dims: " << dims << std::endl;

    sm::vec<float, 2> dx = { gw / dims[0], gh / dims[1] };
    sm::vec<float, 2> nul = { 0.0f, gh };
    sm::grid g1(dims[0], dims[1], dx, nul, sm::griddomainwrap::horizontal, sm::gridorder::topleft_to_bottomright);

    // Now visualise with a GridVisual
    auto gv1 = std::make_unique<mplot::GridVisual<float, std::uint32_t, float, glver>>(&g1, offset);
    gv1->set_parent (v.get_id());
    gv1->gridVisMode = mplot::GridVisMode::Triangles;
    gv1->setVectorData (&image_data_tlbr);
    gv1->cm.setType (mplot::ColourMapType::RGB); // inverse greyscale is good for a monochrome image
    gv1->zScale.set_params (0, 0); // As it's an image, we don't want relief, so set the zScale to have a zero gradient
    gv1->finalize();
    v.addVisualModel (gv1);

    // If graph is 3, 3 then what is the 'active' dims for the graph axes?
    std::unique_ptr<mplot::GraphVisual<float, glver>> graph;
    graph = std::make_unique<mplot::GraphVisual<float, glver>> (offset);
    graph->set_parent (v.get_id());
    graph->twodimensional(false);
    graph->setsize (gw, gh);
    graph->setdataaxisdist (0.0f);
    graph->axisstyle = mplot::axisstyle::none;
    graph->legend = false;
    graph->setlimits (0, 41248, -30351, 0);

    std::vector<std::array<float, 3>> ai_colours = {
        mplot::colour::black,         // There was no Ant 0
        mplot::colour::grey70,
        mplot::colour::grey70,
        mplot::colour::dodgerblue3,   // Ant 3
        mplot::colour::crimson,       // Ant 4
        mplot::colour::mediumpurple1, // Ant 5
        mplot::colour::springgreen2,  // Ant 6 // brick was close to original but indistinct
        mplot::colour::plum1,         // Ant 7
        mplot::colour::purple3,       // Ant 8
        mplot::colour::yellow3,       // Ant 9
        mplot::colour::turquoise2,    // Ant 10
        mplot::colour::maroon2,       // Ant 11
        mplot::colour::darkorange2,   // Ant 12
        mplot::colour::grey70,
        mplot::colour::grey70
    };

    constexpr bool show_all = false; // else show all data, not just selected

    for (auto f : labels_files) {

        if constexpr (show_all == false) {
            if (f.find ("Ant12") == std::string::npos
                && f.find ("Ant11") == std::string::npos
                && f.find ("Ant03") == std::string::npos
                && f.find ("Ant06") == std::string::npos) {
                continue;
            }
        }
        std::cout << "Adding file data " << f << std::endl;
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
        if constexpr (true) {
            if (routeidx == 9999) {
                ds.datalabel = std::format ("Ant{:02d}ZVOP", antidx);
            } else if (routeidx == 8888) {
                ds.datalabel = std::format ("Ant{:02d}ZVSF", antidx);
            } else if (routeidx == 999) {
                ds.datalabel = std::format ("Ant{:02d}ZVF", antidx);
            } else {
                ds.datalabel = std::format ("Ant{:02d}R{:02d}", antidx, routeidx);
            }
            ds.setcolour (ai_colours [antidx]);
        } else {
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
        }
        ds.markersize = markersz * markermult;

        graph->setdata (positions, ds);
    }

    graph->finalize();
    v.addVisualModel (graph);

    v.keepOpen();
}
