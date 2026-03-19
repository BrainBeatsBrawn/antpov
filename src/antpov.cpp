#include <iostream>
#include <cstdint>
#include <memory>
#include <vector>
#include <stdexcept>

import sm.flags;
import sm.vvec;

import mplot.gl.version;
import mplot.compoundray.interop; // mathplot <--> compoundray interoperability
import mplot.compoundray.eyevisual;

import craysim.visual;
import craysim.antbody;

import antpov.helpers;

// OpenGL 4.3 for Instanced VisualModels
constexpr int32_t glver = mplot::gl::version_4_3;

std::int32_t main (std::int32_t argc, char* argv[])
{
    // craysim-common options parsing
    sm::flags<craysim::options> opts;
    auto[gltf_path, csv_path, h5_path, hovh] = craysim::parse_inputs (argc, argv, opts);

    // Perhaps we printed options help and can now exit
    if (opts.test (craysim::options::can_exit)) { return 1; }

    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (2000, 2000, "Compound-ray sim", gltf_path, h5_path, opts);
    // Set the agent hoverheight from our inputs if necessary
    v.set_hoverheight (hovh, 0.002f); // 2 mm is good for C. velox model
    // Find the model from the glTF that you want to be the landscape
    v.find_landscape ("Landscape.003,ground_inner_high_res");

    // APP-SPECIFIC csv reading (comes between find_landscape and setup_landscape)
    sm::vvec<std::uint32_t> csv_antflags;
    if (v.sim_opts.test (craysim::options::path_from_csv)) {
        // Use antpov::read_csv instead of craysim::read_csv as we are also reading flags
        // Note that v.csv_positions is populated.
        if (antpov::read_csv (csv_path, v.csv_positions, csv_antflags) == false) {
            throw std::runtime_error ("Failed to read CSV file");
        } else { std::cout << "Read " << v.csv_positions.size() << " ant positions from CSV\n"; }
    }
    // Turn antflags into colour info, all at the start:
    sm::flags<antpov::antflags> aflags;
    v.bc_clr.resize (csv_antflags.size());
    v.bc_alpha.resize (csv_antflags.size());
    v.bc_scale.resize (csv_antflags.size());
    for (std::uint32_t i = 0; i < csv_antflags.size(); ++i) {
        aflags = csv_antflags[i];
        v.bc_clr[i] = aflags.test (antpov::antflags::cookie) ? mplot::colour::deepskyblue2 : mplot::colour::flesh;
        if (i % 4 == 0) {
            v.bc_alpha[i] = 1.0f;
            v.bc_scale[i] = 1.0f;
        } else {
            v.bc_alpha[i] = 0.7f;
            v.bc_scale[i] = 0.5f;
        }
    }
    // Once CSV has be read (if you are using that feature) do some setup on the landscape
    v.setup_landscape();
    // Enable random walking. n_steps, a_tau, kappa are the params
    v.setup_random_walk (1500, 150, 100);

    // APP-SPECIFIC. A window for the 2D eye view projection
    mplot::Visual<glver> veye (920, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{0}, float{0}, float{-4.1} });
    veye.setSceneTrans (sm::vec<float,3>{ float{-0.00859182}, float{-0.616208}, float{-0.972577} });

    // APP-SPECIFIC. A window for the Ant body view
    mplot::Visual<glver> vant (920, 920, "Ant view");
    vant.setSceneTrans (sm::vec<float,3>{ float{0.113123}, float{0.0217872}, float{-3.7961} });
    vant.setSceneRotation (sm::quaternion<float>{ float{0.937372}, float{0.106131}, float{0.330499}, float{0.0289824} });

    // APP-SPECIFIC. Ant body, plotted in its own window; first the eyes for the body
    auto eyevm1 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &v.ommatidiaData, v.get_ommatidia_ptr(), v.get_head_mesh());
    eyevm1->set_parent (vant.get_id());
    eyevm1->name = "Ant Eyes";
    eyevm1->show_3d = true;
    eyevm1->finalize();
    mplot::compoundray::EyeVisual<glver>* ep1 = vant.addVisualModel (eyevm1);
    // Scale this model up, so it's not tiny like the one in the main scene
    ep1->scaleViewMatrix (1000);
    // The ant body for the separate window
    auto av1 = std::make_unique<craysim::AntBodyVisual<glver>>();
    av1->set_parent (vant.get_id());
    av1->finalize();
    mplot::VisualModel<glver>* ant_ptr1 = vant.addVisualModel (av1);
    ant_ptr1->name = "ant";
    ant_ptr1->scaleViewMatrix (1000);

    // APP-SPECIFIC. 2D eye representation (goes in the other window)
    auto eyevm2 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &v.ommatidiaData, v.get_ommatidia_ptr(), nullptr);
    eyevm2->set_parent (veye.get_id());
    eyevm2->name = "2D Ant Eyes";
    antpov::add_ant_eye_spherical_projection<glver> (v, eyevm2.get());
    eyevm2->show_3d = false;
    eyevm2->twodimensional (true);
    eyevm2->show_sphere = false;
    eyevm2->show_rays = false;
    sm::mat<float, 4> mflip (sm::quaternion<float>{0, 0, 1, 0});
    eyevm2->setViewMatrix (mflip);
    eyevm2->finalize();
    mplot::compoundray::EyeVisual<glver>* ep2 = veye.addVisualModel (eyevm2);
    ep2->scaleViewMatrix (1000);

    // APP-SPECIFIC. An ant body to go in the scene
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->finalize();
    v.agent_body = v.addVisualModel (av);
    v.agent_body->name = "ant";
    v.agent_body->setViewMatrix (v.initial_camera_space);

    // APP-SPECIFIC how we replay a crashed movement for debug
    if (v.sim_opts.test (craysim::options::debug_mv)) {
        try {
            v.do_crashed_movement ();
        } catch (const std::exception& e) {
            std::cout << "Exception moving: " << e.what() << std::endl;
            throw e;
        }
    }

    // APP-SPECIFIC. Put all our 'other windows' in a container, which we pass in to render_and_poll()
    std::vector<mplot::Visual<glver>*> other_windows = { &vant, &veye };
    // Similar for our other eyes
    std::vector<mplot::compoundray::EyeVisual<glver>*> other_eyes = { ep1, ep2 };

    // The main program loop
    while (!v.readyToFinish()) {
        v.start_loop_timer(); // It's important to call this line at the start of the loop

        v.render_and_poll (other_windows, other_eyes); // Does all the render computations

        // Here is where you would work on the data for the last view in v.ommatidiaData;

        v.end_loop_timer(); // Mark that we got to the end of the loop
    }

    v.complete_recording();
}
