#include <iostream>
#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>
#include <stdexcept>
#include <format>
#include <filesystem>
#include <fstream>

import sm.flags;
import sm.vvec;
import sm.grid;

import mplot.gl.version;
import mplot.compoundray.interop; // mathplot <--> compoundray interoperability
import mplot.compoundray.eyevisual;
import mplot.tools;
import mplot.gridvisual;
import mplot.scattervisual;

import craysim.visual;
import craysim.antbody;

import antpov.helpers;

// OpenGL 4.3 for Instanced VisualModels
constexpr std::int32_t glver = mplot::gl::version_4_3;

void read_csv (const std::string& csv, sm::vvec<sm::vec<float, 3>>& points)
{
    std::ifstream fin (csv, std::ios::in);
    if (!fin.is_open()) {
        std::cout << "Failed to open " << csv << " for reading in read_csv\n";
    } else {
        std::string line = {};
        while (getline (fin, line)) {
            std::vector<std::string> vals = mplot::tools::stringToVector (line, ",");
            if (vals.size() > 2) {
                sm::vec<float, 3> coord = { std::stof (vals[0]), std::stof (vals[1]), std::stof (vals[2]) };
                points.push_back (coord);
            }
        }
        fin.close();
    }
}

std::int32_t main (std::int32_t argc, char* argv[])
{
    craysim::parsed_inputs prog_opts = craysim::parse_inputs (argc, argv);
    if (prog_opts.opts.test (craysim::options::can_exit)) { return 1; }

    std::int32_t _w = prog_opts.w > 0 ? prog_opts.w : 1920;
    std::int32_t _h = prog_opts.h > 0 ? prog_opts.h : 1080;
    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (_w, _h, "AntPOV", prog_opts);
    v.set_hoverheight (prog_opts.hovh, 0.002f); // 2 mm is good for C. velox model
    v.find_landscape ("Landscape.003,ground_inner_high_res");
    v.frame_tau = 0.017;
    v.set_horizontal_fov (26.0f);
    v.ambient_intensity = 0.6f; // override the 0.4/0.6 ambient/diffuse intensity
    v.diffuse_position = { 5, 5, -15 }; // Set light source position suitable for Seville
    v.setup_landscape();
    v.setSceneTrans (sm::vec<float,3>{ float{0.682335}, float{0.47893}, float{-8.38334} });
    v.setSceneRotation (sm::quaternion<float>{ float{0.73946}, float{0.6732}, float{0.00036425}, float{0.000331613} });

    // An ant body to go in the scene
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->draw_ring = true;
    av->finalize();
    v.agent_body = v.addVisualModel (av);
    v.agent_body->name = "ant";
    v.agent_body->setViewMatrix (v.initial_camera_space);

    // Scatter plot some csv coords
    if (!prog_opts.csv_path.empty()) {
        sm::vvec<sm::vec<float, 3>> points;
        read_csv (prog_opts.csv_path, points);
        auto sv = std::make_unique<mplot::ScatterVisual<float, glver>> (sm::vec<float>{});
        sv->set_parent (v.get_id());
        sv->setDataCoords (&points);
        sv->radiusFixed = 0.02f;
        sv->cm.setType (mplot::ColourMapType::Plasma);
        sv->finalize();
        v.addVisualModel (sv);
    }

    // The main program loop
    while (!v.readyToFinish()) {
        v.start_loop_timer(); // It's important to call this line at the start of the loop
        v.render_and_poll(); // Does all the render computations
        v.end_loop_timer(); // Mark that we got to the end of the loop
    }

    v.complete_recording();
}
