#include <iostream>
#include <vector>
#include <array>
#include <deque>
#include <chrono>

#include <sm/flags>
#include <sm/vvec>
#include <sm/grid>

#include <sampleConfig.h>

#include "MulticamScene.h"
#include "libEyeRenderer.h"

#include "eye3dvisual.h"
#include <mplotext/fpsprofiler.h>
#include <mplot/compoundray/interop.h> // mathplot <--> compoundray interoperability

#include <mplot/compoundray/EyeVisual.h>
#include <mplotext/FourpiVisual.h>
#include <mplot/VoronoiVisual.h>
#include <mplot/CoordArrows.h>
#include <mplot/GridVisual.h>
#include <mplot/SphereVisual.h> // debug really

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

// When the program starts, how many samples per ommatidium/element do you want?
constexpr int samples_per_omm_default = 64;

namespace eye3d
{
    // Your application-specific help message
    void printHelp()
    {
        std::cout << "USAGE:\neye3d -f <path to gltf scene>" << std::endl << std::endl;
        std::cout << "\t-h\tDisplay this help information." << std::endl;
        std::cout << "\t-f\tPath to a gltf scene file (absolute or relative to current "
                  << "working directory, e.g. './data/axis_coloured_blocks.gltf')." << std::endl;
    }
    // Helper to plot coords
    mplot::CoordArrows<>* plot_axes (mplot::Visual<>* thevisual)
    {
        auto cavm = std::make_unique<mplot::CoordArrows<>> (sm::vec<float>{0.0f});
        thevisual->bindmodel (cavm);
        cavm->em = 0.0f; // labels don't work so well
        cavm->finalize();
        return thevisual->addVisualModel (cavm);
    }
    // Flags class
    enum class options : uint32_t
    {
        prefer_voronoi,   // Set true to prefer voronoi visualization over EyeVisual
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
        have_fourpi,      // If true, then a fourpi eye was found
        disable_fourpi,   // If true then use EyeVisual even if a fourpi eye was found
        keep_moving,      // If true, movements keep moving
        max_fps,          // If true, poll, instead of fps
        can_exit          // Can exit the program
    };
    // Parse cmd line to find the path and set options
    std::string parse_inputs (int argc, char* argv[], sm::flags<eye3d::options>& opts)
    {
        std::string path = "";
        for (int i=0; i<argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-h") {
                eye3d::printHelp();
                opts |= eye3d::options::can_exit;
            } else if (arg == "-f") {
                i++;
                path = std::string(argv[i]);
            } else if (arg == "-v") {
                opts |= eye3d::options::prefer_voronoi;
            } else if (arg == "-b") {
                opts |= eye3d::options::blender_axes;
            } else if (arg == "-x") {
                opts |= eye3d::options::max_fps;
            }
        }
        if (path.empty()) {
            eye3d::printHelp();
            opts |= eye3d::options::can_exit;
        }
        return path;
    }
} // namespace eye3d

int main (int argc, char* argv[])
{
    using mc = sm::mathconst<float>;

    // Program options and boolean state
    sm::flags<eye3d::options> opts;
    std::string path = eye3d::parse_inputs (argc, argv, opts);
    if (opts.test (eye3d::options::can_exit)) { return 1; }

    // Boilerplate memory alloc for compound-ray
    multicamAlloc();

    std::vector<sm::vec<float, 3>> ommatidiaPositions;
    std::vector<std::array<float, 3>> ommatidiaData;
    std::vector<Ommatidium>* ommatidia = nullptr;

    // Turn off verbose logging
    setVerbosity (false);
    // Load the file
    std::cout << "Loading glTF file \"" << path << "\"..." << std::endl;
    loadGlTFscene (path.c_str(), (opts.test(eye3d::options::blender_axes)
                                  ? mplot::compoundray::blender_transform() : sutil::Matrix4x4::identity()));

    // Create a mathplot window to render the eye/sensor
    eye3dvisual v (2000, 1200, "Eye 3D (mathplot graphics)", opts.test(eye3d::options::blender_axes));
    v.showCoordArrows (true);
    // Choose how fast the camera should move for key press and mouse events
    v.speed = 0.05f;
    v.angularSpeed = 2.0f * mc::two_pi / 360.0f;

    v.lightingEffects (true);

    // Use a non-default zFar as we use large environments
    v.zFar = 2400;
    // Rotate about the nearest VisualModel
    v.rotateAboutNearest (true);
    // Rotate about a scene vertical axis
    v.rotateAboutVertical (true);
    if (opts.test(eye3d::options::blender_axes)) {
        v.switch_scene_vertical_axis(); // to uz up
    }

    // Use a FPS profiling with a text object on screen
    mplotext::fps::profiler fps_profiler;
    mplot::VisualTextModel<>* fps_label;
    v.addLabel ("0 FPS", {0.63f, -0.43f, 0.0f}, fps_label);

    // We get the eye data path from the glTF file
    std::string efpath("");
    int ncam = static_cast<int>(getCameraCount());
    int num_compound_cameras = 0;
    int my_compound_camera = -1;
    for (int ci = 0; ci < ncam; ++ci) {
        gotoCamera (ci);
        efpath = getEyeDataPath();
        if (!efpath.empty()) {
            ++num_compound_cameras;
            my_compound_camera = ci;
            if (efpath.find ("fourpi") != std::string::npos) {
                // Check number of pixels and what the 'nside' is for this fourpi
                ommatidia = &scene->m_ommVecs[scene->getCameraIndex()];
                int npix = static_cast<int>(ommatidia->size());
                int _nside = static_cast<int>(std::sqrt((static_cast<double>(npix) - 2.0) / 24.0));
                if (hp::am::is_power_of_two (_nside)) { opts |= eye3d::options::have_fourpi; } // nside must be power of 2
            }
        }
    }
    if (num_compound_cameras > 1) {
        throw std::runtime_error ("This program works for only one compound eye camera in your gltf.");
    }
    // Now switch to our compound ray camera and set the samples per ommatidium/element
    if (my_compound_camera != -1) {
        gotoCamera (my_compound_camera);
        int csamp = getCurrentEyeSamplesPerOmmatidium();
        std::cout << "Current eye samples per ommatidium is " << csamp << std::endl;
        if (csamp < 32000) { changeCurrentEyeSamplesPerOmmatidiumBy (samples_per_omm_default - csamp); }
    }

    // We get the initial camera localspace. This also serves to reset the camera pose. This is set in the GLTF file.
    sm::mat44<float> initial_camera_space = mplot::compoundray::getCameraSpace (scene);

    // Plot the visual models
    mplot::compoundray::scene_to_visualmodels (scene, &v);

    // Create a FourpiVisual, EyeVisual or VoronoiVisual 'eye' in our mathplot scene, v.
    sm::vec<float, 3> offset = { 0.0f };
    mplot::compoundray::EyeVisual<>* eyevm_ptr = nullptr;
    mplotext::FourpiVisual<float>* fourpi_ptr = nullptr;
    mplot::VoronoiVisual<float, 0>* vvm_ptr = nullptr;
    if (opts.test (eye3d::options::have_fourpi) == true && opts.test (eye3d::options::disable_fourpi) == false) {
        auto fpvm = std::make_unique<mplotext::FourpiVisual<float>> (offset);
        v.bindmodel (fpvm);
        int npix = static_cast<int>(ommatidia->size());
        int _nside = static_cast<int>(std::sqrt((static_cast<double>(npix) - 2.0) / 24.0));
        fpvm->set_nside (_nside);
        fpvm->cm.setType (mplot::ColourMapType::RGB);
        fpvm->setColourData (&ommatidiaData);
        fpvm->setViewMatrix (initial_camera_space);
        fpvm->name = "fourpi";
        fpvm->finalize();
        fourpi_ptr = v.addVisualModel (fpvm);
    } else {
        if (opts.test(eye3d::options::prefer_voronoi) == true) {
            // The VoronoiVisual is set up later
        } else {
            auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<>> (offset, &ommatidiaData, ommatidia);
            v.bindmodel (eyevm);
            eyevm->setViewMatrix (initial_camera_space);
            eyevm->name = "EyeVisual";
            eyevm->finalize();
            eyevm_ptr = v.addVisualModel (eyevm);
        }
    }

    // Make CoordArrows axes to show our camera's localspace
    mplot::CoordArrows<>* cam_cs_ptr = eye3d::plot_axes (&v);
    cam_cs_ptr->name = "eye frame";
    cam_cs_ptr->setViewMatrix (initial_camera_space);

    /**
     * Get access to the landscape VisualModel
     */

    mplot::VisualModel<>* land = nullptr;
    {
        mplot::VisualModel<>* vmp = nullptr;
        v.init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
        while ((vmp = v.get_next_vm_accessor()) != nullptr) {
            // The 'land' is a cube for now
            if (vmp->name == "Cube.002") { land = vmp; }
            //if (vmp->name == "Landscape.003") { land = vmp; }
        }
    }
    sm::vec<float, 3> hp = {};
    mplot::SphereVisual<>* svp = nullptr;

    sm::mat44<float> vm;
    sm::mat44<float> vmi;
    if (land) {
        std::cout << "Landscape name: " << land->name << " was found\n";
        std::cout << "It has bounding box " << land->get_viewmatrix_modelbb() << std::endl;
        std::cout << "It has " << (land->vpos_size() / 3) << " vertices\n";

        auto loc1 = sm::vec<>{8.9f, -1.0f, 0.0f};
        uint32_t idx1 = land->find_vp1_nearest (loc1);
        std::cout << "Index " << idx1 << " " << land->vp1[idx1] << " is nearest to loc1: " << loc1 << std::endl;

        // Vector towards land?
        std::cout << "bb centre is " << land->get_viewmatrix_bb_centre() << std::endl;
        sm::vec<> to_land = land->get_viewmatrix_bb_centre() - loc1;
        std::cout << "bb centre - loc1 = " << to_land << std::endl;

        // Neighbours of idx1?
        sm::vvec<uint32_t> nb = land->neighbours (idx1);
        std::cout << "neighbours of idx1=" << idx1 << ": " << nb << std::endl;
        sm::vvec<std::array<uint32_t, 3>> nbt = land->neighbour_triangles (idx1);
        std::cout << "neighbour tris:\n";
        for (auto t : nbt) {
            std::cout << "  " << t[0] << "," << t[1] << "," << t[2] << std::endl;
        }

        // idx1 indices?
        std::cout << "equivalent indices to idx1: " << land->vp1_to_indices[idx1] << std::endl;
        for (auto i : land->vp1_to_indices[idx1]) {
            std::cout << "pos: " << land->get_position(i) << ", norm " << land->get_normal(i) << std::endl;
        }

        vm = land->get_viewmatrix();
        vmi = vm.inverse();
        sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);
        auto camloc = camspace * sm::vec<>{0,0,0};
        auto aa = (vmi * camloc).less_one_dim();
        auto [hit, ti, tn] = land->find_triangle_crossing (aa);
        hp = (vm * hit).less_one_dim();
        std::cout << "or with viewmatrix of model: " << hp << std::endl;

        auto sv = std::make_unique<mplot::SphereVisual<>>(hp, 0.1, mplot::colour::goldenrod3);
        v.bindmodel (sv);
        sv->finalize();
        svp = v.addVisualModel (sv);

        // Let's 'draw' the camera towards the land and then arrange its normal upwards wrt to the normal of the land.
        if (ti[0] == std::numeric_limits<uint32_t>::max()) {
            std::cout << "No hit\n";
        } else {
            std::cout << "Hit at " << hit << std::endl;
            std::cout << "In scene coordinates, hit = " << hp << std::endl;

            // Turn the hit point into a translation matrix
            sm::mat44<float> hitlocn;
            hitlocn.translate (hp);

            // Re-draw sphere
            svp->setViewMatrix (hitlocn);

            // The camera frame always has y up. Choose a random vector in the plane for 'x'
            // and then set z from this random x and the triangle norm (y).
            sm::vec<float> rand_vec;
            rand_vec.randomize();
            sm::vec<float> _x = rand_vec.cross (tn);
            _x.renormalize();
            sm::vec<float> _z = _x.cross (tn);
            auto coord_rotn = sm::mat44<float>::frombasis (_x, tn, _z);

            // Want to place camera just 'above'  hp.
            coord_rotn.translate (0.15f * tn);

            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (hitlocn * coord_rotn));
        }
    }

    /**
     * Done with landscape
     */

    // We keep a track of the eye size. Used in subr_detect_camera_changes
    size_t last_eye_size = 0u;

    /**
     * Subroutine lambda: Detect changes in the camera (there can be multiple cameras, some
     * compound ray, some non-compound ray). The complexity here results from this complexity in
     * compound-ray and the fact that we have multiple ways to visualize the eye's ommatidia
     * (fourpivisual, voronoivisual and compoundrayvisual)
     */
    auto subr_detect_camera_changes = [&v, &ommatidia, &ommatidiaData, &ommatidiaPositions,
                                       &last_eye_size, &eyevm_ptr, &vvm_ptr, &fourpi_ptr, opts] ()
    {
        size_t curr_eye_size = last_eye_size;
        // Detect changes in the camera and update eye model as necessary
        if (opts.test (eye3d::options::have_fourpi) == true && opts.test (eye3d::options::disable_fourpi) == false) {
            if (ommatidia != nullptr) {
                curr_eye_size = ommatidia->size();
                if (curr_eye_size == static_cast<size_t>(fourpi_ptr->n_pixels())) {
                    if (ommatidiaData.size() == 0) {
                        if (isCompoundEyeActive()) { getCameraData (ommatidiaData); }
                    } // else no need to re-get data

                    if (ommatidiaData.size() > 0) {
                        fourpi_ptr->setColourData (&ommatidiaData);
                        fourpi_ptr->reinit();
                    }
                }
            }
        } else { // Generic compound eye
            if (ommatidiaData.size() == 0) {
                if (isCompoundEyeActive()) { getCameraData (ommatidiaData); }
            } // else no need to re-get data

            if (opts.test (eye3d::options::prefer_voronoi) == false) {
                // Change showing the 'cones' of the compound eye visual model?
                if (eyevm_ptr->show_cones != v.vstate.test(eye3dvisual::state::show_cones)) {
                    eyevm_ptr->show_cones = v.vstate.test(eye3dvisual::state::show_cones);
                    eyevm_ptr->reinit();
                }
                // Change the length of the cones?
                if (eyevm_ptr->get_cone_length() != v.manual_cone_length) {
                    eyevm_ptr->set_cone_length (v.manual_cone_length);
                }
                // Update eyevm model (or just update colours)
                eyevm_ptr->ommatidia = ommatidia;
            }
            if (ommatidia != nullptr) {
                curr_eye_size = ommatidia->size();
                if (curr_eye_size != last_eye_size) {
                    if (opts.test (eye3d::options::prefer_voronoi) == false) {
                        eyevm_ptr->reinit();
                    } else {
                        constexpr auto voronoi_z_dirn = sm::vec<float>::uz(); // Hack, your z dirn may need to be different
                        sm::vec<float, 3> offset = { 0.0f };
                        auto vvm = std::make_unique<mplot::VoronoiVisual<float, 0>> (offset);
                        v.bindmodel (vvm);
                        for (size_t i = 0u; i < ommatidia->size(); ++i) {
                            float3 rpos = (*ommatidia)[i].relativePosition;
                            sm::vec<float> rposv = {rpos.x, rpos.y, rpos.z};
                            ommatidiaPositions.push_back (rposv);
                        }
                        vvm->cm.setType (mplot::ColourMapType::RGB);
                        vvm->zoom = 1.0f;
                        vvm->setDataCoords (&ommatidiaPositions);
                        vvm->setVectorData (reinterpret_cast<std::vector<sm::vec<float>>*>(&ommatidiaData));
                        vvm->data_z_direction = voronoi_z_dirn;
                        vvm->setViewMatrix (mplot::compoundray::getCameraSpace (scene));
                        vvm->name = "VoronoiVisual";
                        vvm->finalize();
                        vvm_ptr = v.addVisualModel (vvm);
                    }
                    last_eye_size = curr_eye_size;
                } else {
                    if (opts.test (eye3d::options::prefer_voronoi) == false) {
                        eyevm_ptr->reinitColours(); // 4x faster to just reinitColours
                    } else {
                        vvm_ptr->reinitColours();
                    }
                }
            }
        }
    };

    /**
     * Subroutine: Move the camera according to key events in the mathplot window
     */
    auto subr_key_move_camera = [&v, &eyevm_ptr, &vvm_ptr, &cam_cs_ptr, &fourpi_ptr, &initial_camera_space, opts, land, &svp, vm, vmi]()
    {
        cam_cs_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));

        sm::mat44<float> camera_space;
        if (v.isActivelyMoving()) {
            sm::vec<float, 3> t = v.getMovementVector (opts.test(eye3d::options::keep_moving));
            translateCamerasLocally (t.x(), t.y(), t.z());
            // Up-down (pitch) is rotation about local camera frame axis x
            rotateCamerasLocallyAround (v.getVerticalRotationAngle (opts.test(eye3d::options::keep_moving)), 1.0f, 0.0f, 0.0f);
            // Left-and-right (yaw) is rotation about local camera frame axis y
            rotateCamerasLocallyAround (v.getHorizontalRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 1.0f, 0.0f);
            // Roll
            rotateCamerasLocallyAround (v.getRollRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 0.0f, 1.0f);

            camera_space = mplot::compoundray::getCameraSpace (scene);
            if (land) {
                // Let's 'draw' the camera towards the land and then arrange its normal upwards wrt to the normal of the land.
                auto camloc = camera_space * sm::vec<>{0,0,0};
                // Update vm/vmi?
                auto aa = (vmi * camloc).less_one_dim();

                auto [hit0, ti0, tn0] = land->find_triangle_crossing (aa);
                auto [hit, ti, tn] = land->find_triangle_crossing (aa, -tn0);

                if (ti[0] == std::numeric_limits<uint32_t>::max()) {
                    std::cout << "No hit - off the edge!\n";
                } else {
                    std::cout << "Hit at " << hit << std::endl;
                    sm::vec<float, 3> hp = (vm * hit).less_one_dim();
                    std::cout << "In scene coordinates, hit = " << hp << std::endl;

                    // Turn the hit point into a translation matrix
                    sm::mat44<float> hitlocn;
                    hitlocn.translate (hp);

                    // Re-draw sphere
                    svp->setViewMatrix (hitlocn);

#if 0
                    // The camera frame always has y up. Choose a random vector in the plane for 'x'
                    // and then set z from this random x and the triangle norm (y).
                    sm::vec<float> rand_vec;
                    rand_vec.randomize();
                    sm::vec<float> _x = rand_vec.cross (tn);
                    _x.renormalize();
                    sm::vec<float> _z = _x.cross (tn);
                    auto coord_rotn = sm::mat44<float>::frombasis (_x, tn, _z);

                    // Want to place camera just 'above'  hp.
                    coord_rotn.translate (0.1f * tn);

                    setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (hitlocn * coord_rotn));

                    // Update camera_space
                    camera_space = hitlocn; // mplot::compoundray::getCameraSpace (scene);
#endif
                }
            }

        } else {
            // Get the camera space and update our eye and camera-frame models
            camera_space = mplot::compoundray::getCameraSpace (scene);
        }

        // reset to initial camera space if requested
        if (v.vstate.test (eye3dvisual::state::campose_reset_request) == true) {
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (initial_camera_space));
            v.stop(); // cancel any active movements
            camera_space = initial_camera_space;
            v.vstate.reset (eye3dvisual::state::campose_reset_request);
        }

        // Update the view matrix of eye and eye localspace axes
        if (opts.test (eye3d::options::have_fourpi) == true && opts.test (eye3d::options::disable_fourpi) == false) {
            fourpi_ptr->setViewMatrix (camera_space);
        } else {
            if (opts.test (eye3d::options::prefer_voronoi) == false) {
                eyevm_ptr->setViewMatrix (camera_space);
            } else {
                if (vvm_ptr != nullptr) { vvm_ptr->setViewMatrix (camera_space); }
            }
        }
        cam_cs_ptr->setViewMatrix (camera_space);
    };

#if 0 // From c_ray_mushscan:

    // Sets the height above the landscape, but *needs to search full_landscape* to do this, so
    // there's a comp. hit Uses a container of vertices and a container of normals. In mushscan,
    // these were extracted from compound ray, but here, we have VisualModels (VerticesVisuals) that
    // contain these data
    auto subr_set_height = [land, cam_height, cam_rotnzero]
    (sm::vec<float>& cam_posn, sm::vec<float>& last_posn)
    {
        if (cam_posn == last_posn) { return; }
        // Figure out nearest landscape element and...
        size_t closest_idx = std::numeric_limits<size_t>::max();
        float min_lsq = std::numeric_limits<float>::max();
        // Use land->vertexPositions
        for (size_t i = 0; i < full_landscape.size(); ++i) {
            float dx = full_landscape[i].x() - cam_posn[0];
            float dz = full_landscape[i].z() - cam_posn[2];
            float dl = dx * dx + dz * dz; // no need for sqrt, just want closest
            min_lsq = dl < min_lsq ? closest_idx = i, dl : min_lsq;
        }
        cam_posn[1] = full_landscape[closest_idx].y() + cam_height;
        mushscan::setCameraPose (cam_posn, cam_rotnzero);
    };

#endif

    /**
     * The main program loop
     */

    while (!v.readyToFinish()) {

        // Tell the fps_profiler that we're at the start of a loop
        fps_profiler.at_begin (getCurrentEyeSamplesPerOmmatidium());
        fps_label->setupText (fps_profiler.fps_txt);
        // The current camera may have changed, this subroutine deals with any changes
        subr_detect_camera_changes();
        // Now render the mathplot window
        v.render();
        // Save some electricity while developing - limit to 60 FPS. For max speed use v.poll() (-x)
        if (opts.test (eye3d::options::max_fps)) { v.poll(); } else { v.waitevents (0.018); }
        // Deal with any movements commanded by key press events (including reset)
        subr_key_move_camera();

        // Do the compound-ray ray casting to recompute the scene
        renderFrame();
        // Access data so that a brain model could be fed
        if (isCompoundEyeActive()) {
            getCameraData (ommatidiaData);
            ommatidia = &scene->m_ommVecs[scene->getCameraIndex()];
        }
        // Mark that we got to the end of the loop
        fps_profiler.at_end();
    }

    stop(); // stop compound-ray from running
    multicamDealloc(); // De-allocate compound-ray memory

    return 0;
}
