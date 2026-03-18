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
import mplot.instancedscattervisual;
import mplot.normalsvisual;

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
    auto[path, csv_path, h5_path] = craysim::parse_inputs (argc, argv, opts);
    std::string hovh = antpov::parse_inputs (argc, argv);

    // Perhaps we printed options help and can now exit
    if (opts.test (craysim::options::can_exit)) { return 1; }

    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (2000, 2000, "Compound-ray sim", path, opts);

    // A window for the 2D eye view projection
    mplot::Visual<glver> veye (920, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{0}, float{0}, float{-4.1} });
    veye.setSceneTrans (sm::vec<float,3>{ float{-0.00859182}, float{-0.616208}, float{-0.972577} });

    // A window for the Ant body view
    mplot::Visual<glver> vant (920, 920, "Ant view");
    vant.setSceneTrans (sm::vec<float,3>{ float{0.113123}, float{0.0217872}, float{-3.7961} });
    vant.setSceneRotation (sm::quaternion<float>{ float{0.937372}, float{0.106131}, float{0.330499}, float{0.0289824} });

    // Ant body window
    auto eyevm1 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &v.ommatidiaData, v.get_ommatidia_ptr(), v.get_head_mesh());
    eyevm1->set_parent (vant.get_id());
    eyevm1->name = "Ant Eyes";
    // Visualization options ant body
    eyevm1->show_3d = true;
    eyevm1->finalize();
    mplot::compoundray::EyeVisual<glver>* ep1 = vant.addVisualModel (eyevm1);
    // Scale this model up, so it's not tiny like the one in the scene
    ep1->scaleViewMatrix (1000);
    // The ant body itself
    auto av1 = std::make_unique<craysim::AntBodyVisual<glver>>();
    av1->set_parent (vant.get_id());
    av1->finalize();
    auto ant_ptr1 = vant.addVisualModel (av1);
    ant_ptr1->name = "ant";
    ant_ptr1->scaleViewMatrix (1000);

    // 2D eye representation
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

    // The ant body
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->finalize();
    auto ant_ptr = v.addVisualModel (av);
    ant_ptr->name = "ant";
    ant_ptr->setViewMatrix (v.initial_camera_space);

    // Breadcrumb trail
    std::uint64_t move_counter = 0u;
    std::uint64_t max_bc = 32000;//00; // 32000
    sm::vvec<sm::vec<float, 3>> breadcrumb_coords = {};
    sm::vvec<float> breadcrumb_data = {};

    auto isv = std::make_unique<mplot::InstancedScatterVisual<glver>> (sm::vec<>{});
    isv->set_parent (v.get_id());
    isv->max_instances = max_bc;
    isv->radiusFixed = 0.004f;
    isv->finalize();
    mplot::InstancedScatterVisual<glver>* isvp = v.addVisualModel (isv);

    // Make CoordArrows axes to show our camera's localspace (and to help find our tiny ant)
    auto antca = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<>{});
    antca->set_parent (v.get_id());
    antca->em = 0.0f; // labels don't work so well
    float len = 2.0f;
    antca->lengths = { len, len, len };
    antca->thickness = 1.0f;
    antca->endsphere_size = 1.2f;
    antca->finalize();
    auto antca_ptr = v.addVisualModel (antca);
    antca_ptr->name = "ant";
    antca_ptr->setViewMatrix (v.initial_camera_space);

    // Get access to the landscape VisualModel by searching for a selection of model names
    mplot::VisualModel<glver>* land = nullptr;
    {
        mplot::VisualModel<glver>* vmp = nullptr;
        v.init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
        while ((vmp = v.get_next_vm_accessor()) != nullptr) {
            if (vmp->name == "Landscape.003" || vmp->name == "ground_inner_high_res") {
                land = vmp;
                land->make_navmesh (v.basepath);
                // normals for debug
                auto nrm = std::make_unique<mplot::NormalsVisual<glver>> (land);
                nrm->set_parent (v.get_id());
                nrm->scale_factor = 0.01f;
                // Set options to show just the boundary edge
                nrm->options.set (mplot::normalsvisual_flags::show_tri_normals, false);
                nrm->options.set (mplot::normalsvisual_flags::show_gl_normals, false);
                nrm->options.set (mplot::normalsvisual_flags::show_boundary_halfedges, true);
                nrm->options.set (mplot::normalsvisual_flags::show_inner_halfedges, false); // Heavy lifting
                nrm->options.set (mplot::normalsvisual_flags::show_boundary_next, false);
                nrm->options.set (mplot::normalsvisual_flags::show_boundary_prev, false);
                nrm->nextprev_offset = sm::vec<float>::uy() * 0.01f;
                nrm->finalize();
                v.addVisualModel (nrm);
            } else { std::cout << "Model name " << vmp->name << std::endl; }
        }
    }

    // Load data from csv file for pre-defined paths
    sm::vvec<sm::vec<float, 2>> csv_positions;
    sm::vvec<std::uint32_t> csv_antflags;
    // When reproducing csv paths, it's useful to keep a record of the last triangle, because the
    // most likely next triangle is the last triangle.
    std::uint32_t last_ti = std::numeric_limits<std::uint32_t>::max();

    if (opts.test (craysim::options::path_from_csv)) {
        //waittime = 0.25; // make it slow
        if (antpov::read_csv (csv_path, csv_positions, csv_antflags) == false) {
            throw std::runtime_error ("Failed to read CSV file");
        } else {
            std::cout << "Read " << csv_positions.size() << " ant positions from CSV\n";
        }
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

    sm::mat<float, 4> land_to_scene;  // land's viewmatrix. converts land model to scene

    float hoverheight = 0.002f; // 2 mm is good for C. velox model
    if (!hovh.empty()) {
        hoverheight = std::atof (hovh.c_str());
        std::cout << "Set user-supplied hoverheight to " << hoverheight << std::endl;
    }

    if (land) {
        std::cout << "Landscape name: " << land->name << " was found [" << (land->vpos_size() / 3) << " vertices]\n";

        land_to_scene = land->getViewMatrix();

        sm::mat<float, 4> camspace = mplot::compoundray::getCameraSpace (scene);

        if (opts.test (craysim::options::path_from_csv) && !csv_positions.empty()) {
            // Initial position from first entry in the csv
            std::cout << "Set initial position from csv\n";
            sm::vec<float> nextloc = { csv_positions[0][0], 0.0f, csv_positions[0][1] };
            nextloc -= sm::vec<>{ 0.5f, 0.0f, 0.5f };
            std::cout << "Initial position is " << nextloc << std::endl;
            // Change camspace based on nextloc. nextloc in landscape coords, so cam_nextloc = landscape.location + nextloc;
            sm::vec<float> ltstr = land_to_scene.translation();
            sm::vec<float> cam_nextloc = nextloc;
            cam_nextloc[0] += ltstr[0];
            cam_nextloc[2] += ltstr[2]; // update only x and z
            std::cout << "cam_nextloc = land locn (" << ltstr << ") + nextloc [xz ONLY] (" << nextloc << ") = " << cam_nextloc << std::endl;
            std::cout << "cf from-gltf camera location: " << camspace.translation() << std::endl;

            sm::mat<float, 4> cnl;
            cnl.translate (cam_nextloc);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cnl));

            ++move_counter;
        }

        auto[hp_scene, _ti0] = land->navmesh->find_triangle_hit (camspace, land_to_scene, 100.0f);
        if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {
            // Set up our camera using the data obtained from find_triangle_hit()
            sm::mat<float, 4> cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight);
            if (cam_to_scene != sm::mat<float, 4>::identity()) {
                std::cout << "Set camera pose matrix from\n" << cam_to_scene << std::endl;
                setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            } else {
                std::cout << "cam_to_scene is identity??\n";
            }
        } else {
            std::cout << "Failed to find the landscape; Camera position unchanged from glTF\n";
        }

        sm::mat<float, 4> _cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        std::cout << "Got camera pose matrix from scene:\n" << _cam_to_scene << std::endl;
        sm::vec<float> _lastloc = _cam_to_scene.translation();
        std::cout << "lastloc = " << _lastloc << " [this is cam_to_scene.translation()]" << std::endl;
    }
    std::cout << "*****\n";

    // Random route generation
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
    auto subr_reset_camspace = [&v, &hoverheight, land, land_to_scene] (sm::mat<float, 4>& cam_to_scene)
    {
        // reset to initial camera space if requested
        if (v.vstate.test (craysim::visual<glver>::state::campose_reset_request) == true) {
            v.stop(); // cancel any active movements
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (v.initial_camera_space));
            sm::mat<float, 4> camspace = mplot::compoundray::getCameraSpace (scene);
            auto[hp_scene, _ti0] = land->navmesh->find_triangle_hit (camspace, land_to_scene);
            cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            v.vstate.reset (craysim::visual<glver>::state::campose_reset_request);
        }
    };

    auto subr_key_move_over_land = [&v, &ant_ptr, &antca_ptr, &move_counter,
                                    &breadcrumb_coords, &isvp, &hoverheight, &rrg,
                                    max_bc, land, land_to_scene, subr_reset_camspace,
                                    &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0](const float fps)
    {
        antca_ptr->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
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
                hoverheight += 0.0001f;
            } else if (v.move_state.test (craysim::visual<glver>::move_sense::down)) {
                hoverheight -= 0.0001f;
                if (hoverheight < 0.0f) { hoverheight = 0.0f; }
            }
            // Obtain the commanded movement vector and turn this into a translation matrix
            rrg.step();
            //sm::vec<float> mv_camframe = v.getMovementVector (1.0f / rrg.speed); // confusing, can have -ve speed
            sm::vec<float> mv_camframe = v.getMovementVector (60);
            sm::vec<float> lastloc = cam_to_scene.translation();
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            std::uint32_t ti0_sv = land->navmesh->ti0;
            try {
                cam_to_scene = land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, land_to_scene, hoverheight);

                tm1_ti0 = ti0_sv;
                tm1_mv_camframe = mv_camframe;
                tm1_cam_to_scene = cam_to_scene_sv;

            } catch (const std::exception& e) {
                std::string msg (e.what());
                std::cout << "Exception: " << msg << std::endl;
                if (msg.find ("off-edge:") == 0) {
                    std::cout << "We went off the edge. Key move not possible. Don't crash.\n";
                    land->navmesh->ti0 = ti0_sv;
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
                        dsv.add_contained_vals ("/land_to_scene", land_to_scene.arr);
                        dsv.add_val ("/hoverheight", hoverheight);
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

            move_counter++;
            // This should be the right place to update breadcrumbs
            if (breadcrumb_coords.size() < max_bc) {
                breadcrumb_coords.push_back (lastloc);
            } else {
                breadcrumb_coords[move_counter % max_bc] = lastloc;
            }
            isvp->set_instance_data (breadcrumb_coords);
        }
        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        v.eye->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    auto subr_walk_over_land = [&v, &ant_ptr, &antca_ptr, &rrg,
                                &opts, &move_counter, max_bc, &breadcrumb_coords, &breadcrumb_data,
                                &isvp, land, land_to_scene,
                                &hoverheight, subr_reset_camspace,
                                &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0](const float fps)
    {
        antca_ptr->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
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
        std::uint32_t ti0_sv = land->navmesh->ti0;
        try {
            // Note that even if the last mesh movement would land on a triangle, a further
            // rotation might mean that we get a 'no triangle intersection' exception (esp. if
            // we are on the edge of a

            // ti0, mv_camframe, cam_to_scene to save.
            cam_to_scene = land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, land_to_scene, hoverheight);
            ++move_counter;

            tm1_ti0 = ti0_sv;
            tm1_mv_camframe = mv_camframe;
            tm1_cam_to_scene = cam_to_scene_sv;

            if (breadcrumb_coords.size() < max_bc) {
                breadcrumb_coords.push_back (cam_to_scene_sv.translation());
                breadcrumb_data.push_back (0.0f); // dummy for now
            } else {
                breadcrumb_coords[move_counter % max_bc] = cam_to_scene_sv.translation();
                // breadcrumb_data.push_back (0.0f); // dummy for now, to be flags.
            }
            isvp->set_instance_data (breadcrumb_coords);

        } catch (const std::exception& e) {
            std::string msg (e.what());
            std::cout << "Exception: " << msg << std::endl;
            if (msg.find ("off-edge:") == 0) {
                std::cout << "We went off the edge. Change direction (rrg.about_turn()).\n";
                rrg.about_turn();
                land->navmesh->ti0 = ti0_sv;
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
                    dsv.add_contained_vals ("/land_to_scene", land_to_scene.arr);
                    dsv.add_val ("/hoverheight", hoverheight);
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
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    auto subr_csv_playback = [&v, &ant_ptr, &antca_ptr,
                              &move_counter, &breadcrumb_coords, &isvp, &hoverheight, &opts,
                              max_bc, csv_positions, bc_clr, bc_alpha, bc_scale,
                              land, land_to_scene, subr_reset_camspace]
    (const float fps, std::uint32_t& _last_ti)
    {
        antca_ptr->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
        sm::mat<float, 4> cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        if (csv_positions.size() > move_counter) {
            /*
             * With a csv path, teleport between each location (and then estimate the heading of
             * the ant). CSV positions are relative to the landscape model.
             */
            sm::vec<float> lastcamloc = cam_to_scene.translation();

            sm::vec<float> nextloc = { csv_positions[move_counter][0], 0, csv_positions[move_counter][1] };
            sm::vec<float> lastloc = { csv_positions[move_counter - 1][0], 0, csv_positions[move_counter - 1][1] };
            //std::cout << "Teleport a distance " << (lastloc - nextloc).length() << std::endl;

            sm::vec<float> ltstr = land_to_scene.translation(); // always the same
            sm::vec<float> cam_nextloc = nextloc;
            cam_nextloc[0] += ltstr[0];
            cam_nextloc[2] += ltstr[2]; // update only x and z
            //std::cout << "--> cam_nextloc: " << cam_nextloc << std::endl;

            sm::mat<float, 4> cnl;
            cnl.translate (cam_nextloc);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cnl));
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);

            // Find triangle hits using the scene's 'up' direction.
            sm::vec<float> camloc_mf = (land_to_scene.inverse() * cam_to_scene).translation();
            sm::vec<float> vnrm = v.scene_up;
            vnrm *= 4.0f;
            auto[hp_scene, _ti0] = land->navmesh->find_triangle_hit (land_to_scene, camloc_mf + (vnrm / 2.0f), -2.0f * vnrm, _last_ti);
            _last_ti = _ti0;
            //std::cout << "--> Got hp_scene: " << hp_scene << std::endl;

            if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {
                sm::vec<float> fwds = nextloc - lastloc;
                // Set up our camera using the data obtained from find_triangle_hit()
                cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight, fwds);
                if (cam_to_scene != sm::mat<float, 4>::identity()) {
                    setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
                } // else what to do if cam_to_scene is identity?
            } else {
                // Rather than throwing, could just move on to next in csv?
                // throw std::runtime_error ("Failed to find the landscape so can't teleport to that location!?!");
                cam_to_scene = mplot::compoundray::getCameraSpace (scene);
                std::cout << "Omit csv_positions[move_counter] = csv_positions[" << move_counter << "] = "
                          << csv_positions[move_counter] << " (failed to find triangle hit)\n";
            }

            move_counter++;
            if (breadcrumb_coords.size() < max_bc) {
                breadcrumb_coords.push_back (lastcamloc);
            } else {
                breadcrumb_coords[move_counter % max_bc] = lastcamloc;
            }
            isvp->set_instance_data (breadcrumb_coords, bc_clr, bc_alpha, bc_scale);

        } else {
            // else no more movements, so switch off path_from_csv mode
            opts.set (craysim::options::path_from_csv, false);
        }

        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        v.eye->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
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
            antca_ptr->setViewMatrix (tm1_cam_to_scene);

            std::cout << "First compute_mesh_movement from saved data:\n";

            land->navmesh->ti0 = tm1_ti0;
            sm::mat<float, 4> _cam_to_scene_1 = land->navmesh->compute_mesh_movement (tm1_mv_camframe, tm1_cam_to_scene, _land_to_scene, _hoverheight);

            std::cout << "\ncompute_mesh_movement for time t-1 returned cam_to_scene:\n" << _cam_to_scene_1 << "\n";

            //if (_cam_to_scene_1 != _cam_to_scene) {
            // Random walk may have rotated the camera, to further alter cam_to_scene
            //}

            std::cout << "Running second compute_mesh_movement from saved data:\n";

            land->navmesh->ti0 = _ti0;
            _cam_to_scene = land->navmesh->compute_mesh_movement (_mv_camframe, _cam_to_scene, _land_to_scene, _hoverheight);

            std::cout << "compute_mesh_movement for time t returned!\n";

            // Set the new position for camera and ant models
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (_cam_to_scene));
            v.eye->setViewMatrix (_cam_to_scene);
            ant_ptr->setViewMatrix (_cam_to_scene);
            antca_ptr->setViewMatrix (_cam_to_scene);

        } catch (const std::exception& e) {
            std::cout << "Exception moving: " << e.what() << std::endl;
            while (!v.readyToFinish()) {
                antca_ptr->setHide (!v.vstate.test(craysim::visual<glver>::state::show_camframe));
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
        if (move_counter % 100 == 0) {
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
                subr_csv_playback (v.fps_profiler.fps_mean, last_ti);
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
                std::string ommframe = "/ommatidiaData/frame_" + std::to_string (move_counter);
                try {
                    record.add_contained_vals (ommframe.c_str(), v.ommatidiaData);
                } catch (const std::exception& e) {
                    // Probably didn't move this time.
                }
            }
        }

        // Scale size of breadcrumbs based on distance
        float iscl = 2.0f * std::log (1.0f + v.get_d_to_rotation_centre());
        //if (move_counter % 50 == 0) { std::cout << "iscl = " << iscl << std::endl; }
        isvp->set_instance_scale (iscl);

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
