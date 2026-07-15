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

import craysim.visual;
import craysim.antbody;

import antpov.helpers;

// OpenGL 4.3 for Instanced VisualModels
constexpr std::int32_t glver = mplot::gl::version_4_3;

std::int32_t main (std::int32_t argc, char* argv[])
{
    craysim::parsed_inputs prog_opts = craysim::parse_inputs (argc, argv);
    if (prog_opts.opts.test (craysim::options::can_exit)) { return 1; }

    std::int32_t _w = prog_opts.w > 0 ? prog_opts.w : 1920;
    std::int32_t _h = prog_opts.h > 0 ? prog_opts.h : 1080;
    // Create a craysim main window to render the eye/sensor. This loads in the models from gltf file at path
    craysim::visual<glver> v (_w, _h, "AntPOV", prog_opts);
    // Set the agent hoverheight from our inputs if necessary
    v.set_hoverheight (prog_opts.hovh, 0.002f); // 2 mm is good for C. velox model
    // Find the model from the glTF that you want to be the landscape
    v.find_landscape ("Landscape.003,ground_inner_high_res");

    v.frame_tau = 0.017;

    // Match the approximate field of view of the original camera
    v.set_horizontal_fov (26.0f);
    v.ambient_intensity = 0.6f; // override the 0.4/0.6 ambient/diffuse intensity

    // Set light source position suitable for Seville
    v.diffuse_position = { 5, 5, -15 };

    // Label options
    v.sim_opts.set (craysim::options::show_fps, false);
    v.sim_opts.set (craysim::options::show_movenum, false);
    v.fps_label_update_period = 1u;

    // Uncomment to save out 3D version of csv_positions as a CSV file
    v.sim_opts.set (craysim::options::save_csv_positions, true);

    std::uint32_t antid = 0u;
    std::uint32_t routeidx = 0u;

    // Some extra options for antpov only
    bool colour_by_route = false; // colour by route or ant?
    bool apply_colour_labels = false;
    for (std::int32_t i = 0; i < argc; i++) {
        std::string arg = std::string(argv[i]);
        if (arg == "-R") {
            colour_by_route = true; // I have a slightly hacky way to colour by route ID
        } else if (arg == "-L") {
            apply_colour_labels = true; // Set true to apply green-for-bush; grey-for-invisible
        }
    }

    // csv reading (comes between find_landscape and setup_landscape)
    if (v.sim_opts.test (craysim::options::path_from_csv)) {
        // Check if path encodes several paths
        std::vector<std::string> cpaths = mplot::tools::stringToVector (prog_opts.csv_path, ",");
        // Use antpov::read_csv instead of craysim::read_csv as we are also reading flags
        // Note that v.csv_positions is populated.
        for (auto cpath : cpaths) {

            std::cout << "Reading csv path " << cpath << std::endl;
            if (cpath.find ("Ant12") != std::string::npos) {
                antid = 12;
            } else if (cpath.find ("Ant11") != std::string::npos) {
                antid = 11;
            } else if (cpath.find ("Ant06") != std::string::npos) {
                antid = 6;
            } else if (cpath.find ("Ant03") != std::string::npos) {
                antid = 3;
            }

            // Get Ant index from position p to posn before 'R'
            std::string::size_type lstart = cpath.find ("Ant") + 3;
            std::string::size_type lend = cpath.find ("R", lstart);
            if (lend == std::string::npos) {
                lend = cpath.find ("Z", lstart);
            }
            if (lend == std::string::npos) {
                std::cout << "Uh oh\n";
                return -1;
            }
            if (cpath.find("ZVOP") != std::string::npos) {
                // ZVOP Zero Vector Opposite Side - Ant is allowed to go to nest but before arrival is
                // placed in opposite side of the feeding areay
                routeidx = 9999;
            } else if (cpath.find("ZVSF") != std::string::npos) {
                // ZVOP Zero Vector Semi Familiar - Ant is allowed to go to nest but before arrival is
                // placed near but not at the feeding area
                routeidx = 8888;
            } else if (cpath.find("ZVF") != std::string::npos) {
                // ZVOP Zero Vector Familiar - Ant is allowed to go to nest but before arrival is
                // placed back at the feeding area
                routeidx = 999;
            } else {
                std::string::size_type iend = cpath.find_first_of ('_');
                routeidx = std::stoi (cpath.substr (lend + 1, iend - (lend + 1)));
            }
            std::cout << "Route index: " << routeidx << std::endl;

            std::uint64_t existing = v.csv_positions.size();

            std::uint32_t _routeidx = std::numeric_limits<std::uint32_t>::max(); // colour-by-antid is default
            if (colour_by_route) { _routeidx = routeidx; }

            if (antpov::read_csv (cpath, v.csv_positions, v.csv_flags, antid, _routeidx) == false) {
                throw std::runtime_error ("Failed to read CSV file");
            } else { std::cout << "Read " << (v.csv_positions.size() - existing) << " ant positions from CSV\n"; }
        }

        // Now process the positions to generate directions.
        std::uint32_t block = 3;
        float max_delta_phi = 2.8f;
        // maybe process_positions belongs in craysim.visual?
        sm::vvec<sm::vec<float, 2>> dirns (v.csv_positions.size(), sm::vec<float, 2>{}); // dummy, unused
        antpov::process_positions<false, true> (v.csv_positions, v.csv_flags, dirns, block, max_delta_phi);
        // for each antflag, set dirn uncertain flag
    }
    v.setup_breadcrumbs (32000); // enough to show a whole path/all paths from csv
    v.bc_mult = 2.0f; // 1 is default
    v.breadcrumb_every = 10;

    // Turn antflags into colour info, all at the start:
    sm::flags<antpov::antflags> aflags;
    v.bc_clr.resize (1 + v.csv_flags.size() / v.breadcrumb_every);
    v.bc_alpha.resize (1 + v.csv_flags.size() / v.breadcrumb_every);
    v.bc_scale.resize (1 + v.csv_flags.size() / v.breadcrumb_every);
    std::uint32_t i = 0;
    for (std::uint32_t j = 0; j < v.csv_flags.size(); ++j) {
        if (j % v.breadcrumb_every == 0u) {
            aflags = v.csv_flags[j];
            // Use a 'base ant index' colour:
            if (aflags.test (antpov::antflags::ant15)) { // There was no ant 15, this is used to flag for ZVF colour
                v.bc_clr[i] = mplot::colour::bisque2; // ZVF. A white.
            } else if (aflags.test (antpov::antflags::ant14)) { // Hack to colour by routeID
                //v.bc_clr[i] = aflags.test (antpov::antflags::cookie) ? mplot::colour::red : mplot::colour::hotpink2;
                v.bc_clr[i] = mplot::colour::darkorange2;
            } else if (aflags.test (antpov::antflags::ant13)) { // Hack to colour by routeID
                //v.bc_clr[i] = aflags.test (antpov::antflags::cookie) ? mplot::colour::blue : mplot::colour::deepskyblue2;
                v.bc_clr[i] = mplot::colour::orangered2;

            } else if (aflags.test (antpov::antflags::ant3)) {
                v.bc_clr[i] = mplot::colour::dodgerblue3;
            } else if (aflags.test (antpov::antflags::ant6)) {
                v.bc_clr[i] = mplot::colour::springgreen2;
            } else if (aflags.test (antpov::antflags::ant11)) {
                v.bc_clr[i] = mplot::colour::maroon2;
            } else if (aflags.test (antpov::antflags::ant12)) {
                v.bc_clr[i] = mplot::colour::darkorange2;
            } else {
                // Default to Out/back colour selection (ant0)
                v.bc_clr[i] = aflags.test (antpov::antflags::cookie) ? mplot::colour::deepskyblue2 : mplot::colour::flesh;
            }
            if (apply_colour_labels) {
                if (aflags.test (antpov::antflags::direction_uncertain)) {
                    v.bc_clr[i] = mplot::colour::grey40;
                } else if (aflags.test (antpov::antflags::invisible)) {
                    v.bc_clr[i] = mplot::colour::grey60;
                } else if (aflags.test (antpov::antflags::bush)) {
                    v.bc_clr[i] = mplot::colour::darkgreen;
                }
            }
            if (i % 2 == 0) {
                v.bc_alpha[i] = 1.0f;
                v.bc_scale[i] = 1.0f;
            } else {
                v.bc_alpha[i] = 1.0f;
                v.bc_scale[i] = 1.0f;
            }
            ++i;
        }
    }
    // Once CSV has been read (if you are using that feature) do some setup on the landscape
    v.setup_landscape();

    // From cmd line output (after ctrl-z) set the initial view
    v.setSceneTrans (sm::vec<float,3>{ float{0.682335}, float{0.47893}, float{-8.38334} });
    v.setSceneRotation (sm::quaternion<float>{ float{0.73946}, float{0.6732}, float{0.00036425}, float{0.000331613} });

    // Enable random walking. n_steps, a_tau, kappa are the params
    v.setup_random_walk (1500, 150, 100.0f, 0.05f);

    // A window for the 2D eye view projection
    mplot::Visual<glver> veye (920, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{-0.00859182}, float{-0.616208}, float{-1.18557} });
    veye.setSceneRotation (sm::quaternion<float>{ float{1}, float{0}, float{0}, float{0} });

    // A window for the Ant body view (or cylindrical eye)
    mplot::Visual<glver> vant (920, 920, "Ant view");
    vant.setSceneTrans (sm::vec<float,3>{ float{0.113123}, float{0.0217872}, float{-3.7961} });
    vant.setSceneRotation (sm::quaternion<float>{ float{0.937372}, float{0.106131}, float{0.330499}, float{0.0289824} });

    // Ant body, plotted in its own window; first the eyes for the body
    auto eyevm1 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &v.ommatidia_datas[0], v.get_ommatidia_ptr(0), v.get_head_mesh(0));
    eyevm1->set_parent (vant.get_id());
    eyevm1->name = "Ant Eyes";
    eyevm1->show_3d = true;
    eyevm1->setGamma (0.45f);
    eyevm1->finalize();
    mplot::compoundray::EyeVisual<glver>* ep1 = vant.addVisualModel (eyevm1);
    // Scale this model up, so it's not tiny like the one in the main scene
    ep1->scaleViewMatrix (1000);
    // The ant body for the separate window
    auto av1 = std::make_unique<craysim::AntBodyVisual<glver>>();
    av1->set_parent (vant.get_id());
    av1->draw_antennae = true;
    av1->draw_body = true;
    av1->finalize();
    mplot::VisualModel<glver>* ant_ptr1 = vant.addVisualModel (av1);
    ant_ptr1->name = "ant";
    ant_ptr1->scaleViewMatrix (1000);

    mplot::GridVisual<float, std::uint32_t, float, glver>* gv1p = nullptr;
    mplot::compoundray::EyeVisual<glver>* ep2 = nullptr;

    sm::vec<float, 2> dx = { 0.0035f, 0.003f };
    sm::vec<float, 2> nul = { 0.0f, 0.0f };
    std::uint32_t cyl_w = 360; // must match cyl.eye
    std::uint32_t cyl_h = 90;
    sm::grid g1(cyl_w, cyl_h, dx, nul, sm::griddomainwrap::horizontal, sm::gridorder::bottomleft_to_topright);

    constexpr bool twodee = true;

    // Showing a cylindrical representation, if it is present
    if (v.efpaths.size() > 1 && v.efpaths[1].find ("cyl.eye") != std::string::npos) {
        // We have a 2D cylindrical representation in camera 1. Make a GridVisual.
        auto gv1 = std::make_unique<mplot::GridVisual<float, std::uint32_t, float, glver>>(&g1, sm::vec<>{-g1.width_of_pixels() / 2.0f, 1, 0});
        gv1->set_parent (veye.get_id());
        gv1->gridVisMode = mplot::GridVisMode::RectInterp;
        gv1->setVectorData (reinterpret_cast<std::vector<sm::vec<float>>*>(&v.ommatidia_datas[1]));
        gv1->cm.setType (mplot::ColourMapType::RGB);
        gv1->setGamma (0.45f);
        gv1->zScale.set_params (0, 0); // As it's an image, we don't want relief, so set the zScale to have a zero gradient
        gv1->twodimensional (twodee);
        gv1->finalize();
        gv1p = veye.addVisualModel (gv1);
        gv1p->scaleViewMatrix (1);

        veye.setSceneTrans (sm::vec<float,3>{ float{-0.0245425}, float{-0.876597}, float{-1.56183} });
        veye.setSceneRotation (sm::quaternion<float>{ float{1}, float{0}, float{0}, float{0} });
    }

    // 2D eye representation (goes in the other window)
    auto eyevm2 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{},
                                                                          &v.ommatidia_datas[0], v.get_ommatidia_ptr(0),
                                                                          nullptr);
    eyevm2->set_parent (veye.get_id());
    eyevm2->name = "2D Ant Eyes";
    craysim::add_ant_eye_spherical_projection<glver> (v, eyevm2.get(), 0);
    eyevm2->show_3d = false;
    eyevm2->setGamma (0.45f);
    eyevm2->twodimensional (twodee);
    eyevm2->show_sphere = false;
    eyevm2->show_rays = false;
    sm::mat<float, 4> mflip (sm::quaternion<float>{0, 0, 1, 0});
    eyevm2->setViewMatrix (mflip);
    eyevm2->finalize();
    ep2 = veye.addVisualModel (eyevm2);
    ep2->scaleViewMatrix (1000);


    // An ant body to go in the scene
    auto av = std::make_unique<craysim::AntBodyVisual<glver>>();
    av->set_parent (v.get_id());
    av->draw_ring = true;
    av->finalize();
    v.agent_body = v.addVisualModel (av);
    v.agent_body->name = "ant";
    v.agent_body->setViewMatrix (v.initial_camera_space);

    // how we replay a crashed movement for debug
    if (v.sim_opts.test (craysim::options::debug_mv)) {
        try {
            v.do_crashed_movement ();
        } catch (const std::exception& e) {
            std::cout << "Exception moving: " << e.what() << std::endl;
            throw e;
        }
    }

    // Put all our 'other windows' in a container, which we pass in to render_and_poll()
    v.other_windows = { &vant, &veye };
    // Similar for our other eyes
    if (ep2 == nullptr) {
        v.other_eyes[0] = std::vector<mplot::compoundray::EyeVisual<glver>*>{ ep1 };
    } else {
        v.other_eyes[0] = std::vector<mplot::compoundray::EyeVisual<glver>*>{ ep1, ep2 };
    }

    if (prog_opts.make_movie) {
        std::filesystem::create_directories (std::format ("./movies/Ant{:02d}R{:02d}/scene", antid, routeidx));
        std::filesystem::create_directories (std::format ("./movies/Ant{:02d}R{:02d}/ant", antid, routeidx));
        std::filesystem::create_directories (std::format ("./movies/Ant{:02d}R{:02d}/eyes", antid, routeidx));
    }

    // The main program loop
    while (!v.readyToFinish()) {
        v.start_loop_timer(); // It's important to call this line at the start of the loop

        if (v.move_counter < v.csv_flags.size()) {
            // Greyscale the eyes when we're in a section that was marked invisible
            if (ep2 != nullptr) {
                ep2->greyscale ((v.csv_flags[v.move_counter] & 8u) == 8u ? true : false);
            }
            // Also the ant head/eyes and body
            ep1->greyscale ((v.csv_flags[v.move_counter] & 8u) == 8u ? true : false);
            ant_ptr1->greyscale ((v.csv_flags[v.move_counter] & 8u) == 8u ? true : false);
        } else {
            if (ep2 != nullptr) { ep2->greyscale (false); }
            // Also the ant head/eyes and body
            ep1->greyscale (false);
            ant_ptr1->greyscale (false);
        }
        v.render_and_poll(); // Does all the render computations

        // How to access the eye data.
        //
        // In this program, we pass the eye data to a mathplot VisualModel to be rendered on
        // screen. However, we might want to save these data to a file, or pass them into a brain
        // model. This is a description to get you started if you need to do this.
        //
        // It is placed here, after v.render_and_poll(), because in that function the new eye values
        // will have been computed.
        //
        // Eye data lives in craysim_visual's v.ommatidia_datas which has type:
        //
        // std::map<std::uint32_t, std::vector<std::array<float, 3>>> ommatidia_datas;
        //
        // This structure is keyed by the compound-ray camera ID. In compound-ray cameras are
        // specified in the glTF file, and so any camera may have any ID.
        //
        // In the Seville scene, ground_and_veg_inner_circular.gltf, the biologically realistic eye
        // camera is always camera 0, i.e. v.ommatidia_datas[0].
        //
        // If the file has been modified so that there is also a cylindrical eye camera present,
        // then that is expected (by this program) to be v.ommatidia_datas[1].
        //
        // v.ommatidia_datas[0] is std::vector<std::array<float, 3>> - a vector of RGB values, with
        // one RGB for each ommatidium.
        //
        // Match the RGB values up with the ommatidia position information in craysim_visual's
        // v.ommatidias which is a camera ID-keyed map of vectors of Ommatidium objects.

        if (gv1p != nullptr) {
            //gv1p->reinitColours();
            gv1p->setVectorData (reinterpret_cast<std::vector<sm::vec<float>>*>(&v.ommatidia_datas[1]));
            gv1p->reinit();
            gv1p->render();
        }
        // Save frames
        if (prog_opts.make_movie && v.move_counter > 2) { // Ignore first couple of locations, as the system takes a couple of moves to get ready
            v.saveImage (std::format ("./movies/Ant{:02d}R{:02d}/scene/{:06d}.pnm", antid, routeidx, v.move_counter));
            vant.saveImage (std::format ("./movies/Ant{:02d}R{:02d}/ant/{:06d}.pnm", antid, routeidx, v.move_counter));
            veye.saveImage (std::format ("./movies/Ant{:02d}R{:02d}/eyes/{:06d}.pnm", antid, routeidx, v.move_counter));
        }

        // Here is where you would work on the data for the last view in v.ommatidia_data;

        v.end_loop_timer(); // Mark that we got to the end of the loop
    }

    v.complete_recording();

    // Manually create 6D positions with flags
    if (!v.csv_found_positions.empty() && !v.first_csv.empty()) {
        std::string fp_filename = v.first_csv + ".6d.csv";
        std::cout << "Write out found 3D positions and directions to " << fp_filename << "\n";
        std::ofstream fout (fp_filename, std::ios::out | std::ios::trunc);
        if (fout.is_open()) {
            fout << "# x,y,z,dir_x,dir_y,dir_z,bush,cookie,shadow,invisible,dirn_uncertain\n";
            std::uint32_t i = 0;
            sm::vec<float> last_p = v.csv_found_positions[0];
            sm::vec<float> last_d = {};
            for (auto p : v.csv_found_positions) {
                // Flags columns order originally was: Bush, Cookie, Shadow, Visibility; We add "Dirn Uncertain"
                sm::flags<antpov::antflags> aflags;
                aflags = v.csv_flags[i];

                // need to compute the 3D direction (giving ant pose) again.
                sm::vec<float> pose_direction = {};
                pose_direction = p - last_p;
                last_p = p;
                bool dirn_uncertain = pose_direction.length() < std::numeric_limits<float>::epsilon();
                if (dirn_uncertain) {
                    // Use last dirn
                    pose_direction = last_d;
                } else {
                    pose_direction.renormalize();
                    last_d = pose_direction;
                }
                fout << p.str_comma_separated() << ","
                     << pose_direction.str_comma_separated() << ","
                     << (aflags.test (antpov::antflags::bush) ? "1" : "0") << ","
                     << (aflags.test (antpov::antflags::cookie) ? "1" : "0") << ","
                     << (aflags.test (antpov::antflags::shadow) ? "1" : "0") << ","
                     << (aflags.test (antpov::antflags::invisible) ? "1" : "0") << ","
                     << (dirn_uncertain ? "1" : "0")
                     << std::endl;

                ++i;
            }
            fout.close();
        } else {
            std::cout << "Failed to open " << fp_filename << " to write out 3D csv positions\n";
        }
    } else {
        std::cout << "No found positions to write out\n";
    }
}
