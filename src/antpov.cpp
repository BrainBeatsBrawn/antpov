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
import sm.grid;
import sm.hdfdata;
import sm.random;

import mplot.gl.version;
import mplot.compoundray.interop; // mathplot <--> compoundray interoperability
import mplot.compoundray.eyevisual;
import mplot.coordarrows;
import mplot.gridvisual;
import mplot.rodvisual;
import mplot.vectorvisual;

import craysim.visual;
import craysim.antbody;
import craysim.random_walk;

import antpov.helpers;

// OpenGL 4.3 for Instanced VisualModels
constexpr int32_t glver = mplot::gl::version_4_3;

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

std::int32_t main (std::int32_t argc, char* argv[])
{
    double waittime = 0.0167; // for debug, so I can make playback slow in a simple way

    // craysim-common options parsing
    sm::flags<craysim::options> opts;
    auto[path, csv_path, h5_path, hovh] = craysim::parse_inputs (argc, argv, opts);

    // Perhaps we printed options help and can now exit
    if (opts.test (craysim::options::can_exit)) { return 1; }

    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (2000, 2000, "Compound-ray sim", path, opts);
    // Set the agent hoverheight from our inputs if necessary
    v.set_hoverheight (hovh, 0.002f); // 2 mm is good for C. velox model

    // A window for the 2D eye view projection
    mplot::Visual<glver> veye (920, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{0}, float{0}, float{-4.1} });
    veye.setSceneTrans (sm::vec<float,3>{ float{-0.00859182}, float{-0.616208}, float{-0.972577} });

    // A window for the Ant body view
    mplot::Visual<glver> vant (920, 920, "Ant view");
    vant.setSceneTrans (sm::vec<float,3>{ float{0.113123}, float{0.0217872}, float{-3.7961} });
    vant.setSceneRotation (sm::quaternion<float>{ float{0.937372}, float{0.106131}, float{0.330499}, float{0.0289824} });

    // Ant body, plotted in its own window; first the eyes for the body
    auto eyevm1 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &v.ommatidiaData, v.get_ommatidia_ptr(), v.get_head_mesh());
    eyevm1->set_parent (vant.get_id());
    eyevm1->name = "Ant Eyes";
    // Visualization options ant body
    eyevm1->show_3d = true;
    eyevm1->finalize();
    mplot::compoundray::EyeVisual<glver>* ep1 = vant.addVisualModel (eyevm1);
    // Scale this model up, so it's not tiny like the one in the scene
    ep1->scaleViewMatrix (1000);
    // The ant body for the separate window
    auto av1 = std::make_unique<craysim::AntBodyVisual<glver>>();
    av1->set_parent (vant.get_id());
    av1->finalize();
    auto ant_ptr1 = vant.addVisualModel (av1);
    ant_ptr1->name = "ant";
    ant_ptr1->scaleViewMatrix (1000);

    // 2D eye representation (goes in the other window)
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

    // An ant body to go in the scene
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->finalize();
    auto ant_ptr = v.addVisualModel (av);
    ant_ptr->name = "ant";
    ant_ptr->setViewMatrix (v.initial_camera_space);

    v.find_landscape ("Landscape.003,ground_inner_high_res");

    // App-specific csv reading (comes between find_landscape and setup_landscape)
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

    v.setup_landscape (opts);

    // Random route generation object
    craysim::random_walk<float> rrg(1500, 150, 100);

    // We keep a track of the eye size. Used in subr_detect_camera_changes
    std::size_t last_eye_size = 0u;

    // For debug saving:
    sm::mat<float, 4> tm1_cam_to_scene;
    sm::vec<float> tm1_mv_camframe = {};
    std::uint32_t tm1_ti0 = 0u;

    std::uint32_t render_counter = 0u;
    auto subr_detect_camera_changes = [&v, &last_eye_size, &ep1, &ep2, &render_counter] ()
    {
        std::size_t curr_eye_size = last_eye_size;
        // Detect changes in the camera and update eye model as necessary
        if (v.ommatidiaData.size() == 0) {
            if (isCompoundEyeActive()) { getCameraData (v.ommatidiaData); }
        } // else no need to re-get data

        // Update eyevm model (or just update colours)
        v.eye->ommatidia = v.get_ommatidia_ptr();
        ep1->ommatidia = v.get_ommatidia_ptr();
        ep2->ommatidia = v.get_ommatidia_ptr();

        static constexpr std::uint32_t render_every = 1u; // set to 1 for max update, 60 to reduce compute
        if (v.ommatidia != nullptr) {
            curr_eye_size = v.ommatidia->size();
            if (curr_eye_size != last_eye_size) {
                if (render_counter % render_every == 0u) { v.eye->reinit(); }
                ep1->reinit();
                ep2->reinit();
                last_eye_size = curr_eye_size;
            } else {
                if (render_counter % render_every == 0u) { v.eye->reinitColours(); }
                ep1->reinitColours(); // 4x faster to just reinitColours
                ep2->reinitColours();
            }
            ++render_counter;
        }
    };

    // Helper subroutine used by all the movement subroutines
    auto subr_reset_camspace = [&v] (sm::mat<float, 4>& cam_to_scene)
    {
        // reset to initial camera space if requested
        if (v.vstate.test (craysim::visual<glver>::state::campose_reset_request) == true) {
            v.stop(); // cancel any active movements
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (v.initial_camera_space));
            sm::mat<float, 4> camspace = mplot::compoundray::getCameraSpace (scene);
            auto[hp_scene, _ti0] = v.land->navmesh->find_triangle_hit (camspace, v.land_to_scene);
            cam_to_scene = v.land->navmesh->position_camera (hp_scene, v.land_to_scene, v.hoverheight);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            v.vstate.reset (craysim::visual<glver>::state::campose_reset_request);
        }
    };

    auto subr_key_move_over_land = [&v, &ant_ptr, &rrg, subr_reset_camspace,
                                    &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0] (const float fps)
    {
        v.agent_coords->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
        sm::mat<float, 4> cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        if (v.isActivelyRotating()) {
            // Up-down (pitch) is rotation about local camera frame axis x
            rotateCamerasLocallyAround (v.getVerticalRotationAngle(), 1.0f, 0.0f, 0.0f);
            // Left-and-right (yaw) is rotation about local camera frame axis y
            rotateCamerasLocallyAround (v.getHorizontalRotationAngle(), 0.0f, 1.0f, 0.0f);
            // Roll
            rotateCamerasLocallyAround (v.getRollRotationAngle(), 0.0f, 0.0f, 1.0f);
            cam_to_scene = mplot::compoundray::getCameraSpace (scene); // update
        }
        if (v.isActivelyTranslating()) {
            if (v.move_state.test (craysim::visual<glver>::move_sense::up)) {
                v.hoverheight += 0.0001f;
            } else if (v.move_state.test (craysim::visual<glver>::move_sense::down)) {
                v.hoverheight -= 0.0001f;
                if (v.hoverheight < 0.0f) { v.hoverheight = 0.0f; }
            }
            // Obtain the commanded movement vector and turn this into a translation matrix
            rrg.step();
            //sm::vec<float> mv_camframe = v.getMovementVector (1.0f / rrg.speed); // confusing, can have -ve speed
            sm::vec<float> mv_camframe = v.getMovementVector (60);
            sm::vec<float> lastloc = cam_to_scene.translation();
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            std::uint32_t ti0_sv = v.land->navmesh->ti0;
            try {
                cam_to_scene = v.land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, v.land_to_scene, v.hoverheight);

                tm1_ti0 = ti0_sv;
                tm1_mv_camframe = mv_camframe;
                tm1_cam_to_scene = cam_to_scene_sv;

            } catch (const std::exception& e) {
                std::string msg (e.what());
                std::cout << "Exception: " << msg << std::endl;
                if (msg.find ("off-edge:") == 0) {
                    std::cout << "We went off the edge. Key move not possible. Don't crash.\n";
                    v.land->navmesh->ti0 = ti0_sv;
                } else {
                    std::cout << "key-command move was not possible...\n";
                    {
                        std::cout << "Saving compute_mesh_movement data\n";
                        std::cout << "mv_camframe: " << mv_camframe << " and tm1_mv_camframe: " << tm1_mv_camframe << std::endl;
                        std::cout << "cam_to_scene_sv is\n" << cam_to_scene_sv
                                  << "\nand tm1_cam_to_scene:\n" << tm1_cam_to_scene << std::endl;
                        sm::hdfdata dsv ("./antpov.h5", std::ios::out | std::ios::trunc);
                        dsv.add_contained_vals ("/mv_camframe", mv_camframe);
                        dsv.add_contained_vals ("/cam_to_scene", cam_to_scene_sv.arr);
                        dsv.add_contained_vals ("/land_to_scene", v.land_to_scene.arr);
                        dsv.add_val ("/hoverheight", v.hoverheight);
                        dsv.add_val ("/ti0", ti0_sv);
                        // Also save t-1 values:
                        dsv.add_contained_vals ("/tm1_mv_camframe", tm1_mv_camframe);
                        dsv.add_contained_vals ("/tm1_cam_to_scene", tm1_cam_to_scene.arr);
                        dsv.add_val ("/tm1_ti0", tm1_ti0);
                    }
                    throw e;
                }
            }

            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));

            v.add_breadcrumb (lastloc);
        }
        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        v.eye->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        v.agent_coords->setViewMatrix (cam_to_scene);
    };

    auto subr_walk_over_land = [&v, &ant_ptr, &rrg, &opts, subr_reset_camspace,
                                &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0](const float fps)
    {
        v.agent_coords->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
        sm::mat<float, 4> cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        // A random walk mode
        if (v.vstate.test (craysim::visual<glver>::state::walk) == false) { return; }

        // set rotation and step length according to the Stone paper
        rrg.step();
        // rrg.omega is the angular speed rrg.speed is the linear speed
        //std::cout << "rotating in this step by " << rrg.omega << " and moving forward by " << rrg.speed << std::endl;
        rotateCamerasLocallyAround (rrg.omega, 0.0f, 1.0f, 0.0f);
        cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        sm::vec<float> mv_camframe = { 0, 0, rrg.speed };
        sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
        std::uint32_t ti0_sv = v.land->navmesh->ti0;
        try {
            // Note that even if the last mesh movement would land on a triangle, a further
            // rotation might mean that we get a 'no triangle intersection' exception (esp. if
            // we are on the edge of a

            // ti0, mv_camframe, cam_to_scene to save.
            cam_to_scene = v.land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, v.land_to_scene, v.hoverheight);
            tm1_ti0 = ti0_sv;
            tm1_mv_camframe = mv_camframe;
            tm1_cam_to_scene = cam_to_scene_sv;
            v.add_breadcrumb (cam_to_scene_sv.translation());

        } catch (const std::exception& e) {
            std::string msg (e.what());
            std::cout << "Exception: " << msg << std::endl;
            if (msg.find ("off-edge:") == 0) {
                std::cout << "We went off the edge. Change direction (rrg.about_turn()).\n";
                rrg.about_turn();
                v.land->navmesh->ti0 = ti0_sv;
            } else {
                //cam_to_scene = cam_to_scene_sv;
                opts.set (craysim::options::max_fps, false); // don't burn electricity after exception
                v.vstate.set (craysim::visual<glver>::state::walk, false);
                {
                    std::cout << "Saving compute_mesh_movement data\n";
                    std::cout << "mv_camframe: " << mv_camframe << " and tm1_mv_camframe: " << tm1_mv_camframe << std::endl;
                    std::cout << "cam_to_scene_sv is\n" << cam_to_scene_sv
                              << "\nand tm1_cam_to_scene:\n" << tm1_cam_to_scene << std::endl;
                    sm::hdfdata dsv ("./antpov.h5", std::ios::out | std::ios::trunc);
                    dsv.add_contained_vals ("/mv_camframe", mv_camframe);
                    dsv.add_contained_vals ("/cam_to_scene", cam_to_scene_sv.arr);
                    dsv.add_contained_vals ("/land_to_scene", v.land_to_scene.arr);
                    dsv.add_val ("/hoverheight", v.hoverheight);
                    dsv.add_val ("/ti0", ti0_sv);
                    // Also save t-1 values:
                    dsv.add_contained_vals ("/tm1_mv_camframe", tm1_mv_camframe);
                    dsv.add_contained_vals ("/tm1_cam_to_scene", tm1_cam_to_scene.arr);
                    dsv.add_val ("/tm1_ti0", tm1_ti0);
                }
                throw e;
            }
        }
        setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        v.eye->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        v.agent_coords->setViewMatrix (cam_to_scene);
    };

    auto subr_csv_playback = [&v, &ant_ptr, &opts, bc_clr, bc_alpha, bc_scale, subr_reset_camspace] (const float fps, std::uint32_t& _last_ti)
    {
        v.agent_coords->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
        sm::mat<float, 4> cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        if (v.csv_positions.size() > v.move_counter) {
            /*
             * With a csv path, teleport between each location (and then estimate the heading of
             * the ant). CSV positions are relative to the landscape model.
             */
            sm::vec<float> lastcamloc = cam_to_scene.translation();

            sm::vec<float> nextloc = { v.csv_positions[v.move_counter][0], 0, v.csv_positions[v.move_counter][1] };
            sm::vec<float> lastloc = { v.csv_positions[v.move_counter - 1][0], 0, v.csv_positions[v.move_counter - 1][1] };
            //std::cout << "Teleport a distance " << (lastloc - nextloc).length() << std::endl;

            sm::vec<float> ltstr = v.land_to_scene.translation(); // always the same
            sm::vec<float> cam_nextloc = nextloc;
            cam_nextloc[0] += ltstr[0];
            cam_nextloc[2] += ltstr[2]; // update only x and z
            //std::cout << "--> cam_nextloc: " << cam_nextloc << std::endl;

            sm::mat<float, 4> cnl;
            cnl.translate (cam_nextloc);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cnl));
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);

            // Find triangle hits using the scene's 'up' direction.
            sm::vec<float> camloc_mf = (v.land_to_scene.inverse() * cam_to_scene).translation();
            sm::vec<float> vnrm = v.scene_up;
            vnrm *= 4.0f;
            auto[hp_scene, _ti0] = v.land->navmesh->find_triangle_hit (v.land_to_scene, camloc_mf + (vnrm / 2.0f), -2.0f * vnrm, _last_ti);
            _last_ti = _ti0;
            //std::cout << "--> Got hp_scene: " << hp_scene << std::endl;

            if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {
                sm::vec<float> fwds = nextloc - lastloc;
                // Set up our camera using the data obtained from find_triangle_hit()
                cam_to_scene = v.land->navmesh->position_camera (hp_scene, v.land_to_scene, v.hoverheight, fwds);
                if (cam_to_scene != sm::mat<float, 4>::identity()) {
                    setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
                } // else what to do if cam_to_scene is identity?
            } else {
                // Rather than throwing, could just move on to next in csv?
                // throw std::runtime_error ("Failed to find the landscape so can't teleport to that location!?!");
                cam_to_scene = mplot::compoundray::getCameraSpace (scene);
                std::cout << "Omit csv_positions[v.move_counter] = csv_positions[" << v.move_counter << "] = "
                          << v.csv_positions[v.move_counter] << " (failed to find triangle hit)\n";
            }

            v.add_breadcrumb (lastcamloc, &bc_clr, &bc_alpha, &bc_scale);

        } else {
            // else no more movements, so switch off path_from_csv mode
            opts.set (craysim::options::path_from_csv, false);
        }

        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        v.eye->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        v.agent_coords->setViewMatrix (cam_to_scene);
    };

    if (opts.test (craysim::options::debug_mv)) {

        std::cout << "Loading compute_mesh_movement data from crash file\n";

        sm::mat<float, 4> _cam_to_scene = {{}};
        sm::mat<float, 4> _land_to_scene = {{}};
        sm::vec<float> _mv_camframe = {};
        float _hoverheight = 0.0f;
        std::uint32_t _ti0 = 0u;

        sm::hdfdata dsv ("./antpov.h5", std::ios::in);
        dsv.read_contained_vals ("/mv_camframe", _mv_camframe);
        dsv.read_contained_vals ("/cam_to_scene", _cam_to_scene.arr);
        dsv.read_contained_vals ("/land_to_scene", _land_to_scene.arr);
        dsv.read_val ("/hoverheight", _hoverheight);
        dsv.read_val ("/ti0", _ti0);
        dsv.read_contained_vals ("/tm1_mv_camframe", tm1_mv_camframe);
        dsv.read_contained_vals ("/tm1_cam_to_scene", tm1_cam_to_scene.arr);
        dsv.read_val ("/tm1_ti0", tm1_ti0);

        try {
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (tm1_cam_to_scene));
            v.eye->setViewMatrix (tm1_cam_to_scene);
            ant_ptr->setViewMatrix (tm1_cam_to_scene);
            v.agent_coords->setViewMatrix (tm1_cam_to_scene);

            std::cout << "First compute_mesh_movement from saved data:\n";

            v.land->navmesh->ti0 = tm1_ti0;
            sm::mat<float, 4> _cam_to_scene_1 = v.land->navmesh->compute_mesh_movement (tm1_mv_camframe, tm1_cam_to_scene, _land_to_scene, _hoverheight);

            std::cout << "\ncompute_mesh_movement for time t-1 returned cam_to_scene:\n" << _cam_to_scene_1 << "\n";

            //if (_cam_to_scene_1 != _cam_to_scene) {
            // Random walk may have rotated the camera, to further alter cam_to_scene
            //}

            std::cout << "Running second compute_mesh_movement from saved data:\n";

            v.land->navmesh->ti0 = _ti0;
            _cam_to_scene = v.land->navmesh->compute_mesh_movement (_mv_camframe, _cam_to_scene, _land_to_scene, _hoverheight);

            std::cout << "compute_mesh_movement for time t returned!\n";

            // Set the new position for camera and ant models
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (_cam_to_scene));
            v.eye->setViewMatrix (_cam_to_scene);
            ant_ptr->setViewMatrix (_cam_to_scene);
            v.agent_coords->setViewMatrix (_cam_to_scene);

        } catch (const std::exception& e) {
            std::cout << "Exception moving: " << e.what() << std::endl;
            while (!v.readyToFinish()) {
                v.agent_coords->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
                v.waitevents (1.0);
                v.render();
                vant.render();
                veye.render();
            }
            throw e;
        }
    }

    /**
     * The main program loop
     */
    v.render();
    vant.render();
    veye.render();

    sm::hdfdata record (h5_path, std::ios::out | std::ios::trunc);
    while (!v.readyToFinish()) {

        // Tell the fps_profiler that we're at the start of a loop
        v.fps_profiler.at_begin (craysim::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));

        // The current camera may have changed, this subroutine deals with any changes
        subr_detect_camera_changes();

        // Now render the mathplot window
        v.render();
        // Change label after render (it needs v's context, not veye's)
        if (v.move_counter % 100 == 0) {
            //v.render();
            v.fps_label_update();
            //vant.render();
        }
        // Save some electricity while developing - limit to 60 FPS. For max speed use v.poll() (-x)
        if (opts.test (craysim::options::max_fps)) { v.poll(); } else { v.wait (waittime); }
        // Render the eye-only window
        vant.render();
        veye.render();
        // Deal with any movements commanded by key press events (including reset)

        v.setContext(); // right now key move over land needs v's context

        // tmp profile
        if (v.vstate.test (craysim::visual<glver>::state::paused) == false) {
            if (v.vstate.test (craysim::visual<glver>::state::walk)) {
                subr_walk_over_land (v.fps_profiler.fps_mean);
            } else if (opts.test (craysim::options::path_from_csv)) { // Construct path from csv file of 2D ant locations
                subr_csv_playback (v.fps_profiler.fps_mean, v.last_ti);
            } else {
                subr_key_move_over_land (v.fps_profiler.fps_mean);
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
