#include <iostream>
#include <cstdint>
#include <vector>
#include <array>
#include <deque>
#include <chrono>
#include <stdexcept>

#include <sampleConfig.h>
#include "MulticamScene.h"
#include "libEyeRenderer.h"

import sm.mathconst;
import sm.flags;
import sm.vvec;
import sm.hdfdata;
import sm.random;

import mplot.gl.version;
import mplot.compoundray.interop; // mathplot <--> compoundray interoperability
import mplot.compoundray.eyevisual;

import craysim.visual;
import craysim.antbody;

import antpov.helpers;

// OpenGL 4.3 for Instanced VisualModels
constexpr int32_t glver = mplot::gl::version_4_3;

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

std::int32_t main (std::int32_t argc, char* argv[])
{
    // craysim-common options parsing
    sm::flags<craysim::options> opts;
    auto[gltf_path, csv_path, h5_path, hovh] = craysim::parse_inputs (argc, argv, opts);

    // Perhaps we printed options help and can now exit
    if (opts.test (craysim::options::can_exit)) { return 1; }

    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (2000, 2000, "Compound-ray sim", gltf_path, opts);
    // Set the agent hoverheight from our inputs if necessary
    v.set_hoverheight (hovh, 0.002f); // 2 mm is good for C. velox model
    // Find the model from the glTF that you want to be the landscape
    v.find_landscape ("Landscape.003,ground_inner_high_res");

    // APP-SPECIFIC csv reading (comes between find_landscape and setup_landscape)
    sm::vvec<std::uint32_t> csv_antflags;
    if (opts.test (craysim::options::path_from_csv)) {
        // Use antpov::read_csv instead of craysim::read_csv as we are also reading flags
        // Note that v.csv_positions is populated.
        if (antpov::read_csv (csv_path, v.csv_positions, csv_antflags) == false) {
            throw std::runtime_error ("Failed to read CSV file");
        } else { std::cout << "Read " << v.csv_positions.size() << " ant positions from CSV\n"; }
    }
    // Turn antflags into colour info, all at the start:
    sm::flags<antpov::antflags> aflags;
    sm::vvec<std::array<float, 3>> bc_clr (csv_antflags.size());
    sm::vvec<float> bc_alpha (csv_antflags.size());
    sm::vvec<float> bc_scale (csv_antflags.size());
    for (std::uint32_t i = 0; i < csv_antflags.size(); ++i) {
        aflags = csv_antflags[i];
        bc_clr[i] = aflags.test (antpov::antflags::cookie) ? mplot::colour::deepskyblue2 : mplot::colour::flesh;
        if (i % 4 == 0) {
            bc_alpha[i] = 1.0f;
            bc_scale[i] = 1.0f;
        } else {
            bc_alpha[i] = 0.7f;
            bc_scale[i] = 0.5f;
        }
    }
    // Once CSV has be read (if you are using that feature) do some setup on the landscape
    v.setup_landscape (opts);
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

    // APP-SPECIFIC. Put all our 'other eyes' in a container, so that if the eye needs reinit, this can be applied.
    std::vector<mplot::compoundray::EyeVisual<glver>*> other_eyes = { ep1, ep2 };

    // APP-SPECIFIC. An ant body to go in the scene
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->finalize();
    v.agent_body = v.addVisualModel (av);
    v.agent_body->name = "ant";
    v.agent_body->setViewMatrix (v.initial_camera_space);

    // APP-SPECIFIC how we replay a crashed movement for debug
    if (opts.test (craysim::options::debug_mv)) {
        try {
            v.do_crashed_movement ();
        } catch (const std::exception& e) {
            // In this app, we'll catch the exception from the crash and then keep the app open.
            std::cout << "Exception moving: " << e.what() << std::endl;
            while (!v.readyToFinish()) {
                v.agent_coords->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
                v.waitevents (1.0);
                v.render();
                vant.render();
                veye.render();
            }
            throw e; // Then re-throw after user presses Ctrl-q
        }
    }

    /**
     * The main program loop
     */
    sm::hdfdata record (h5_path, std::ios::out | std::ios::trunc);
    while (!v.readyToFinish()) {

        // Tell the fps_profiler that we're at the start of a loop
        v.fps_profiler.at_begin (craysim::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));

        // The current camera may have changed, this subroutine deals with any changes
        v.detect_camera_changes (other_eyes);

        // Now render the mathplot window
        v.render();
        // Change label after render (it needs v's context, not veye's)
        if (v.move_counter % 100 == 0) {
            //v.render();
            v.fps_label_update();
            //vant.render();
        }
        // Save some electricity while developing - limit to 60 FPS. For max speed use v.poll() (-x)
        if (opts.test (craysim::options::max_fps)) { v.poll(); } else { v.wait (0.0167); }


        // Render the eye-only window
        vant.render();
        veye.render();
        // Deal with any movements commanded by key press events (including reset)

        v.setContext(); // right now key move over land needs v's context

        // tmp profile
        if (v.vstate.test (craysim::visual<glver>::state::paused) == false) {
            if (v.vstate.test (craysim::visual<glver>::state::walk)) {
                v.walk_over_land (v.fps_profiler.fps_mean);
            } else if (opts.test (craysim::options::path_from_csv)) { // Construct path from csv file of 2D ant locations
                if (v.subr_csv_playback (v.fps_profiler.fps_mean, bc_clr, bc_alpha, bc_scale) == false) {
                    // no more movements, so switch off path_from_csv mode
                    opts.set (craysim::options::path_from_csv, false);
                }
            } else {
                v.key_move_over_land (v.fps_profiler.fps_mean);
            }
        }
        // tmp profile

        // Do the compound-ray ray casting to recompute the scene
        renderFrame();
        // Access data so that a brain model could be fed
        if (isCompoundEyeActive()) {
            getCameraData (v.ommatidiaData);
            v.ommatidia = &scene->m_ommVecs[scene->getCameraIndex()];

            // if csv mode, then save the data
            if (opts.test (craysim::options::path_from_csv) && opts.test (craysim::options::save_hdf5)) {
                std::cout << "Saving frame...\n";
                std::string ommframe = "/ommatidiaData/frame_" + std::to_string (v.move_counter);
                try {
                    record.add_contained_vals (ommframe.c_str(), v.ommatidiaData);
                } catch (const std::exception& e) {
                    // Probably didn't move this time.
                }
            }
        }

        // Scale size of breadcrumbs based on distance
        float iscl = 2.0f * std::log (1.0f + v.get_d_to_rotation_centre());
        //if (v.move_counter % 50 == 0) { std::cout << "iscl = " << iscl << std::endl; }
        v.isvp->set_instance_scale (iscl);

        // Mark that we got to the end of the loop
        v.fps_profiler.at_end();
    }

    if (opts.test (craysim::options::path_from_csv)) {
        // convert std::vector<Ommatidium>* ommatidia into vvecs that can be h5 saved
        auto ommat = v.get_ommatidia_ptr();
        sm::vvec<sm::vec<float, 3>> o_pos;
        sm::vvec<sm::vec<float, 3>> o_dir;
        sm::vvec<float> o_aa;
        sm::vvec<float> o_fo;
        for (auto o : *ommat) {
            o_pos.push_back (o.relativePosition);
            o_dir.push_back (o.relativeDirection);
            o_aa.push_back (o.acceptanceAngleRadians);
            o_fo.push_back (o.focalPointOffset);
        }
        std::cout << "Pos\n";
        record.add_contained_vals ("/ommatidia/relativePosition", o_pos);
        std::cout << "Dir\n";
        record.add_contained_vals ("/ommatidia/relativeDirection", o_dir);
        std::cout << "AA\n";
        record.add_contained_vals ("/ommatidia/acceptanceAngleRadians", o_aa);
        std::cout << "FO\n";
        record.add_contained_vals ("/ommatidia/focalPointOffset", o_fo);
    }

    return 0;
}
