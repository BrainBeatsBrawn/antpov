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
#include <mplot/CoordArrows.h>
#include <mplot/GridVisual.h>
#include <mplot/SphereVisual.h> // debug really
#include <mplot/RodVisual.h>    // also debug

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
        auto cavm = std::make_unique<mplot::CoordArrows<>> (sm::vec<>{});
        thevisual->bindmodel (cavm);
        cavm->em = 0.0f; // labels don't work so well
        cavm->finalize();
        return thevisual->addVisualModel (cavm);
    }
    // Flags class
    enum class options : uint32_t
    {
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
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

    std::vector<sm::vec<>> ommatidiaPositions;
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
#if 0 // turned off during cube debug
    // Rotate about a scene vertical axis
    v.rotateAboutVertical (true);
    if (opts.test(eye3d::options::blender_axes)) {
        v.switch_scene_vertical_axis(); // to uz up
    }
#endif
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

    // Create an EyeVisual 'eye' in our mathplot scene, v.
    sm::vec<> offset = {};
    mplot::compoundray::EyeVisual<>* eyevm_ptr = nullptr;
    auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<>> (offset, &ommatidiaData, ommatidia);
    v.bindmodel (eyevm);
    eyevm->setViewMatrix (initial_camera_space);
    eyevm->name = "EyeVisual";
    eyevm->finalize();
    eyevm_ptr = v.addVisualModel (eyevm);

    // Make CoordArrows axes to show our camera's localspace
    mplot::CoordArrows<>* cam_cs_ptr = eye3d::plot_axes (&v);
    cam_cs_ptr->name = "eye frame";
    cam_cs_ptr->setViewMatrix (initial_camera_space);

    /**
     * Get access to the landscape VisualModel
     *
     * May want to make a class to manage the agent/camera's position wrt a VisualModel.
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
    sm::vec<> hp_scene = {};
    mplot::SphereVisual<>* svp = nullptr;
    mplot::SphereVisual<>* svp1 = nullptr;
    mplot::SphereVisual<>* svp2 = nullptr;
    mplot::SphereVisual<>* svp3 = nullptr;
    mplot::SphereVisual<>* svp4 = nullptr;

    mplot::SphereVisual<>* svp_t0 = nullptr;
    mplot::SphereVisual<>* svp_t1 = nullptr;
    mplot::SphereVisual<>* svp_t2 = nullptr;

    mplot::RodVisual<>* rvp1 = nullptr;
    mplot::RodVisual<>* rvp2 = nullptr;

    sm::mat44<float> land_to_scene;  // land's viewmatrix. converts land model to scene
    sm::mat44<float> scene_to_land;  // inverse of land_to_scene, converts scene to land model

    std::array<uint32_t, 3> ti0 = {}; // Current triangle indices
    sm::vvec<std::array<uint32_t, 3>> ti0_neighbours;
    sm::vec<> tn0_land = {}; // Current triangle normal (in landframe) that our agent/camera is 'next to'

    constexpr float hoverheight = 0.08f;

    if (land) {
        std::cout << "Landscape name: " << land->name << " was found\n";
        std::cout << "It has bounding box " << land->get_viewmatrix_modelbb() << std::endl;
        std::cout << "It has " << (land->vpos_size() / 3) << " vertices\n";

        auto loc1 = sm::vec<>{8.9f, -1.0f, 0.0f};
        uint32_t idx1 = land->find_vp1_nearest (loc1);
        std::cout << "Index " << idx1 << " " << land->vp1[idx1] << " is nearest to loc1: " << loc1 << std::endl;

        // Neighbours of idx1?
        std::cout << "neighbours of idx1=" << idx1 << ": " << land->neighbours (idx1) << std::endl;
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

        land_to_scene = land->get_viewmatrix();
        scene_to_land = land_to_scene.inverse();
        sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);
        auto camloc = camspace * sm::vec<>{0,0,0};
        auto camloc_landframe = (scene_to_land * camloc).less_one_dim();
        auto [hit, ti, tn] = land->find_triangle_crossing (camloc_landframe);
        ti0 = ti;
        // Populate neighbours of ti0 with something like:
        // ti0_neighbours = land->neighbour_triangles (ti0); // ??
        tn0_land = tn; // landframe
        // Can I make hit the centre of the triangle?
        constexpr bool hit_tri_centre = true;
        if constexpr (hit_tri_centre) {
            sm::vec<sm::vec<>, 3> tv_landframe = land->triangle_vertices (ti0);
            hit = tv_landframe.mean();
        }
        hp_scene = (land_to_scene * hit).less_one_dim();

        auto sv = std::make_unique<mplot::SphereVisual<>>(hp_scene, 0.005, mplot::colour::goldenrod3);
        v.bindmodel (sv);
        sv->finalize();
        svp = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.005, mplot::colour::deepskyblue3);
        v.bindmodel (sv);
        sv->finalize();
        svp1 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.005, mplot::colour::magenta3);
        v.bindmodel (sv);
        sv->finalize();
        svp2 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.005, mplot::colour::springgreen2);
        v.bindmodel (sv);
        sv->finalize();
        svp3 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.005, mplot::colour::black);
        v.bindmodel (sv);
        sv->finalize();
        svp4 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.012, mplot::colour::orangered1);
        v.bindmodel (sv);
        sv->finalize();
        svp_t0 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.012, mplot::colour::darkgreen);
        v.bindmodel (sv);
        sv->finalize();
        svp_t1 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.012, mplot::colour::blue2);
        v.bindmodel (sv);
        sv->finalize();
        svp_t2 = v.addVisualModel (sv);

        auto rv = std::make_unique<mplot::RodVisual<>>(land->get_viewmatrix_origin(), sm::vec<>{}, sm::vec<>{2,2,2}, 0.001f, mplot::colour::blue2);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp1 = v.addVisualModel (rv);

        rv = std::make_unique<mplot::RodVisual<>>(land->get_viewmatrix_origin(), sm::vec<>{}, sm::vec<>{-2,2,2}, 0.001f, mplot::colour::crimson);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp2 = v.addVisualModel (rv);

        // Let's 'draw' the camera towards the land and then arrange its normal upwards wrt to the normal of the land.
        if (ti[0] == std::numeric_limits<uint32_t>::max()) {
            std::cout << "No hit\n";
        } else {
            // In this case, place the camera on the land, and orient it randomly in the 'land plane'
            std::cout << "Hit at " << hit << std::endl;
            std::cout << "In scene coordinates, hit = " << hp_scene << std::endl;
            // Turn the hit point into a translation matrix
            sm::mat44<float> hitlocn_mat;
            hitlocn_mat.translate (hp_scene);
            svp->setViewMatrix (hitlocn_mat); // reposition sphere
            // The camera frame always has y up. Choose a random vector in the plane for 'x'
            // and then set z from this random x and the triangle norm (y).
            sm::vec<> rand_vec;
            rand_vec.randomize();
            sm::vec<> _x = rand_vec.cross (tn);
            _x.renormalize();
            sm::vec<> _z = _x.cross (tn);
            auto coord_rotn = sm::mat44<float>::frombasis (_x, tn, _z);

            // Want to place camera just 'above'  hp.
            coord_rotn.translate (hoverheight * tn);

            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (hitlocn_mat * coord_rotn));
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
     */
    auto subr_detect_camera_changes = [&v, &ommatidia, &ommatidiaData, &ommatidiaPositions,
                                       &last_eye_size, &eyevm_ptr, opts] ()
    {
        size_t curr_eye_size = last_eye_size;
        // Detect changes in the camera and update eye model as necessary
        if (ommatidiaData.size() == 0) {
            if (isCompoundEyeActive()) { getCameraData (ommatidiaData); }
        } // else no need to re-get data

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

        if (ommatidia != nullptr) {
            curr_eye_size = ommatidia->size();
            if (curr_eye_size != last_eye_size) {
                eyevm_ptr->reinit();
                last_eye_size = curr_eye_size;
            } else {
                eyevm_ptr->reinitColours(); // 4x faster to just reinitColours
            }
        }
    };

    /**
     * Subroutine: Move the camera according to key events in the mathplot window
     */
    auto subr_key_move_camera = [&v, &eyevm_ptr, &cam_cs_ptr, &initial_camera_space,
                                 opts, land, &svp, &svp1, &svp2, &svp3, &svp4, &svp_t0, &svp_t1, &svp_t2, &rvp1, &rvp2, land_to_scene, scene_to_land, &tn0_land, &ti0, hoverheight]()
    {
        cam_cs_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));

        /**
         * A subroutine to find the part of mv_inplane that gets us to the triangle boundary
         */
        auto subr_compute_mv_part = [land_to_scene, &svp1, &svp2, &svp3, &svp4, &rvp1, &rvp2]
        (const sm::vec<>& edge_s, const sm::vec<>& edge_e, const sm::vec<>& mv_inplane, const sm::vec<>& t_norm, const sm::vec<>& hovlocn)
        {
            constexpr bool debug = false;

            sm::vec<> edge = edge_e - edge_s;

            sm::vec<> u_y = edge;
            u_y.renormalize();
            sm::vec<> u_z = t_norm;
            u_z.renormalize();
            sm::vec<> u_x = u_y.cross (u_z);
            if constexpr (debug) {
                std::cout << "edge = " << edge << std::endl;
                std::cout << "Basis: " << u_x << " " << u_y << " " << u_z << std::endl;
            }

            // Create a matrix to convert from land frame movements to the triangle frame of ref.
            sm::mat44<float> from_triangle_frame = sm::mat44<float>::frombasis (u_x, u_y, u_z);

            // Idea (no good)
            // from_triangle_frame.pretranslate (edge_s);

            sm::mat44<float> to_triangle_frame = from_triangle_frame.inverse();

            // Use Edge as our 'y' and the orthogonal as our 'x', then express mv_inplane in terms
            // of these two unit vectors. We also have our 'z' which is the triangle normal.
            sm::vec<float, 4> mv_inplane4d = to_triangle_frame * mv_inplane;
            sm::vec<float, 2> mv_inplane2d = { mv_inplane4d[0], mv_inplane4d[1] };

            sm::vec<float, 4> h_4d = to_triangle_frame * hovlocn;
            sm::vec<float, 2> h_2d =  { h_4d[0], h_4d[1] };

            sm::vec<float, 4> edge_4d = to_triangle_frame * edge;
            sm::vec<float, 2> edge_2d =  { edge_4d[0], edge_4d[1] };

            sm::vec<float, 4> edge_s_4d = to_triangle_frame * edge_s;
            sm::vec<float, 2> edge_s_2d =  { edge_s_4d[0], edge_s_4d[1] };

            sm::vec<float, 4> edge_e_4d = to_triangle_frame * edge_e;
            sm::vec<float, 2> edge_e_2d =  { edge_e_4d[0], edge_e_4d[1] };

            if constexpr (debug) {
                std::cout << "mv_inplane2d: " << mv_inplane2d  << std::endl;
                std::cout << "h_2d: " << h_2d  << std::endl;
                std::cout << "edge_2d: " << edge_2d  << std::endl;
                std::cout << "edge_s_2d: " << edge_s_2d  << std::endl;
                std::cout << "edge_e_2d: " << edge_e_2d  << std::endl;
            }
            // A 2d null vector for the origin in 2D
            constexpr sm::vec<float, 2> orig_2d = {};

            sm::vec<> mv_part = {};
            // Can now apply algo to find crossing point
            if constexpr (debug) {
                std::cout << "intersection test for lines: " << orig_2d << " --> " << edge_2d
                          << " and " << h_2d << " --> " << (h_2d + mv_inplane2d) << "\n";
            }

            // Let's transform these back for vis
            rvp1->update ((from_triangle_frame * (sm::vec<float, 4>{} + edge_s_4d)).less_one_dim(),
                          (from_triangle_frame * (edge_4d + edge_s_4d)).less_one_dim());
            rvp2->update ((from_triangle_frame * h_4d).less_one_dim(),
                          (from_triangle_frame * (h_4d + mv_inplane4d)).less_one_dim());


            std::bitset<2> si = sm::algo::segments_intersect<float> (orig_2d + edge_s_2d, edge_2d + edge_s_2d, h_2d, h_2d + mv_inplane2d);
            if (si.test(1)) {
                throw std::runtime_error ("Deal with colinear movement and triangle edge!\n");
            } else {
                if (si.test(0)) {
                    // Intersects as expected
                    sm::vec<float, 2> cross_point_2d = sm::algo::crossing_point<float> (orig_2d + edge_s_2d, edge_2d + edge_s_2d, h_2d, h_2d + mv_inplane2d);
                    if constexpr (debug) { std::cout << "Cross point (2d) is " << cross_point_2d << std::endl; }
                    // Now go from cross point 2d to a point in landscape coordinates?
                    sm::vec<> cross_point = (from_triangle_frame * cross_point_2d.plus_one_dim()).less_one_dim();
                    if constexpr (debug) { std::cout << "Cross point in land frame: " << cross_point << std::endl; }
                    svp2->setViewTranslation (land_to_scene * hovlocn);     // last hover locn is magenta
                    svp4->setViewTranslation (land_to_scene * cross_point); // cross point is black (not yet in right location)
                    mv_part = cross_point - hovlocn;
                } else {
                    std::cout << "Huh?!? Got no intersection across edge?\n";
                    // Hmm, don't expect a lack of intersection
                    // set mv_part to be mv_inplane?
                    mv_part = mv_inplane;
                }
            }

            return mv_part;
        };

        sm::mat44<float> cam_to_scene;
        if (v.isActivelyMoving()) {

            sm::vec mv_camframe = v.getMovementVector (opts.test(eye3d::options::keep_moving));
            std::cout << "Movement vector is " << mv_camframe << " in the camspace model frame\n";
            sm::mat44<float> mv_camframe_mat;
            mv_camframe_mat.translate (mv_camframe);
            // If constrained by land, determine here if t will move it into a new triangle on the land
            if (land) {

                // 0. Find vertices of triangle, given its indices. These are in the land model frame
                sm::vec<sm::vec<>, 3> tv_landframe = land->triangle_vertices (ti0);
                std::cout << "ti0 " << ti0[0] << "," << ti0[1] << "," << ti0[2]
                          << " has vertices (landframe) at " << tv_landframe
                          << " and upcoming movement (camframe) of " << mv_camframe << std::endl;

                // 1. Find component of t that is in the current triangle plane. t is in camera frame.
                cam_to_scene = mplot::compoundray::getCameraSpace (scene);
                // cam_to_scene transforms camera frame to scene frame
                // scene_to_land * cam_to_scene transforms camera frame to model frame
                sm::mat44<float> cam_to_land = scene_to_land * cam_to_scene;
                // To use later:
                sm::mat44<float> mv_landframe_mat = cam_to_land * mv_camframe_mat;
                //std::cout << "Camera location from cam_to_scene is camloc = " << (cam_to_land * sm::vec<>{}) << std::endl;
                sm::vec<> camloc_landframe = (cam_to_land * sm::vec<>{}).less_one_dim();
                //std::cout << "camloc_landframe = " << camloc_landframe << std::endl;
                sm::vec<> mv_landframe = (cam_to_land * mv_camframe).less_one_dim() - camloc_landframe;
                float udn = mv_landframe.dot (tn0_land); // tn0_land is in landframe
                sm::vec<> mv_orthog = tn0_land * (udn / (tn0_land.dot(tn0_land))); // landframe
                sm::vec<> mv_inplane = mv_landframe - mv_orthog;                   // landframe
                std::cout << "mv_inplane = " << mv_inplane << std::endl;

                // 2. Determine if that component will cross any edge of the triangle
                // 3. Work out what the new edge is and find a location on that triangle
                // 4. Find the 'hover location' over that new location
                //
                sm::vec<> hovlocn = {};
                const sm::vec<>& t0 = tv_landframe[0];
                const sm::vec<>& t1 = tv_landframe[1];
                const sm::vec<>& t2 = tv_landframe[2];
                svp_t0->setViewTranslation (land_to_scene * t0);
                svp_t1->setViewTranslation (land_to_scene * t1);
                svp_t2->setViewTranslation (land_to_scene * t2);
                std::cout << "Current hover triangle is " << t0 << ", " << t1 << ", " << t2 << std::endl;
                bool isect = sm::algo::ray_tri_intersection<float> (t0, t1, t2, camloc_landframe, -tn0_land, hovlocn);
                std::cout << "hovlocn: " << hovlocn << std::endl; // hovlocn is in landframe

                sm::vec<> mv_part = {}; // The part-way movement to the edge (landframe)
                sm::vec<> tri_edge = {};
                if (isect) {
                    // For each edge in triangle, compute distance to edge for h and (h + mv_inplane)
                    sm::vec<> p = hovlocn + mv_inplane;
                    uint32_t common_a = 0;
                    uint32_t common_b = 0;

                    sm::vec<> edge = t1 - t0;
                    sm::vec<> ptoe = p - t0;
                    bool inside01 = (tn0_land.dot (edge.cross (ptoe)) >= 0);
                    if (!inside01) {
                        common_a = ti0[0]; common_b = ti0[1];
                        mv_part = subr_compute_mv_part (t0, t1, mv_inplane, tn0_land, hovlocn);
                        tri_edge = edge;
                    }

                    edge = t2 - t1; ptoe = p - t1;
                    bool inside21 = (tn0_land.dot (edge.cross (ptoe)) >= 0);
                    if (!inside21) {
                        common_a = ti0[2]; common_b = ti0[1];
                        mv_part = subr_compute_mv_part (t1, t2, mv_inplane, tn0_land, hovlocn);
                        tri_edge = edge;
                    }

                    edge = t0 - t2; ptoe = p - t2;
                    bool inside02 = (tn0_land.dot (edge.cross (ptoe)) >= 0);
                    if (!inside02) {
                        common_a = ti0[0]; common_b = ti0[2];
                        mv_part = subr_compute_mv_part (t2, t0, mv_inplane, tn0_land, hovlocn);
                        tri_edge = edge;
                    }

                    if (!inside01 || !inside21 || !inside02) {
                        std::cout << "Crossed over " << (inside01 ? " " : "0-1") <<  (inside21 ? " " : "2-1") <<  (inside02 ? " " : "0-2") << std::endl;
                        // Can work out new triangle here
                        auto [_ti, _tn] = land->find_other_triangle_containing (common_a, common_b, ti0);
                        if (_ti[0] != std::numeric_limits<uint32_t>::max()) {
                            // Re-orient onto the new triangle
                            sm::vec<sm::vec<>, 3> newtv_landframe = land->triangle_vertices (_ti); // debug only
                            std::cout << "Re-orient to new triangle " << _ti[0] << "," << _ti[1] << "," << _ti[2]
                                      << "[ " << newtv_landframe << " ] with normal " << _tn << "\n";

                            std::cout << " mv_inplane (landframe): " << mv_inplane
                                      << " length " << mv_inplane.length() << " angle wrt z "
                                      << mv_inplane.angle (sm::vec<>::uz())<< std::endl;

                            std::cout << " mv_part =               " << mv_part
                                      << " length " << mv_part.length()
                                      << " angle wrt z " << mv_part.angle (sm::vec<>::uz())<< std::endl;

                            float d_rest = mv_inplane.length() - mv_part.length();
                            std::cout << "  additional distance = " << d_rest << std::endl;

                            // The reorientation transform is to take us from the previous
                            // hover-surface location to the new hover-surface location.

                            sm::mat44<float> reorient_rotn;
                            sm::mat44<float> reorient_land; // reorientation transformation in landframe

                            reorient_land.pretranslate (mv_part); // now we're ON the triangle boundary

                            //reorient_land.pretranslate (-hovlocn); // not quite. Want the matrix that defines hovlocn+orientation. is that cam_to_scene?

                            // Rotate by the angle between the normals. I think this is constrained to be <= pi
                            float rotn_angle = tn0_land.angle (_tn);

                            // Use the *edge* as the rotation axis. Need to translate for the rotate though.
                            std::cout << "Rotate about edge " << tri_edge << " by angle " << rotn_angle << std::endl;
                            reorient_land.rotate (tri_edge, rotn_angle);
                            reorient_rotn.rotate (tri_edge, rotn_angle); // Just the rotn

                            // Untranslate for edge?
                            //reorient_land.translate (hovlocn); // not quite

                            sm::vec<> mv_rest = (mv_inplane - mv_part); // FIXME: What if mv_rest sails past the next triangle and on to ANOTHER one?
                            std::cout << "mv_rest (unrotated) in land frame = " << mv_rest << std::endl;

                            // This is translating in unrotated direction
                            reorient_land.translate (reorient_rotn * mv_rest); // reorients points in the land model frame to change the coordinate axes

                            std::cout << "reorient_land:\n" << reorient_land << std::endl;
                            //std::cout << "mv_part + mv_rest = " << (mv_part + mv_rest) << std::endl;

                            // reorient_land should now move the projection of the coordinate frame
                            // onto the land into the right location
                            sm::vec<> new_hovlocn = (reorient_land * hovlocn).less_one_dim();
                            std::cout << "Old hovlocn = " << hovlocn << " and new_hovlocn = reorient_land * hovlocn = " << new_hovlocn << std::endl;

                            // a) Get sphere locn into land model frame. Note: *starts* with sphere location
                            // b) apply reorient_land
                            // c) Return sphere lcon into scene coordinates (or bunch it all together):
                            // d) Update svp->viewmatrix
                            // All in one line:
                            std::cout << "! starting from the sphere's current origin in scene frame: " <<  svp->get_viewmatrix_origin() << std::endl;
                            std::cout << "! to land " << scene_to_land * svp->get_viewmatrix_origin() << "\n";
                            std::cout << "! re-orients " << reorient_land * scene_to_land * svp->get_viewmatrix_origin() << "\n";
                            std::cout << "! to scene " << land_to_scene * reorient_land * scene_to_land * svp->get_viewmatrix_origin() << std::endl;

                            svp->setViewTranslation (land_to_scene * reorient_land * scene_to_land * svp->get_viewmatrix_origin());

                            // we have cam_to_scene = mplot::compoundray::getCameraSpace (scene);
                            // Take cam_to_scene.
                            // Apply scene_to_land
                            // sink to the surface
                            // Apply reorient_land
                            // Raise from surface
                            // Apply land_to_scene
                            sm::mat44<float> sink;
                            //std::cout << "Will sink in dirn " << -tn0_land << " len " << tn0_land.length() << std::endl;
                            sink.translate (-tn0_land * hoverheight); // assumes we normalized tn0
                            sm::mat44<float> unsink;
                            //std::cout << "Will unsink in dirn " << _tn << " len " << _tn.length() << std::endl;
                            unsink.translate (_tn * hoverheight); // assumes we normalized _tn

                            std::cout << "===================\n";
                            std::cout << "!! starting camera in scene: " << cam_to_scene * sm::vec<>{} << std::endl;
                            std::cout << "!! to land: " << scene_to_land * cam_to_scene * sm::vec<>{} << std::endl;
                            std::cout << "!! sinks: " << sink * scene_to_land * cam_to_scene * sm::vec<>{} << std::endl;
                            std::cout << "!! re-orients " << reorient_land * sink * scene_to_land * cam_to_scene * sm::vec<>{} << std::endl;
                            std::cout << "!! unsink " << unsink * reorient_land * sink * scene_to_land * cam_to_scene * sm::vec<>{} << std::endl;
                            std::cout << "!! to scene " << land_to_scene * unsink * reorient_land * sink * scene_to_land * cam_to_scene * sm::vec<>{} << std::endl;
                            sm::mat44<float> cam_transform = land_to_scene * unsink * reorient_land * sink * scene_to_land * cam_to_scene;
                            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_transform));

                            ti0 = _ti;
                            tn0_land = _tn;
                        }
                    } else {
                        std::cout << "No crossings " << (inside01 ? " " : "!!0-1") <<  (inside21 ? " " : "!!2-1") <<  (inside02 ? " " : "!!0-2") << std::endl;

                        translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());

                        // Whats the movement of the camera in the scene frame?
                        sm::vec<float, 4> mv_sceneframe = cam_to_scene * mv_camframe;
                        sm::vec<float, 4> orig_sceneframe = cam_to_scene * sm::vec<>{};
                        std::cout << "cam movement is " << mv_camframe << " in its own frame or from "
                                  << orig_sceneframe << " to " << mv_sceneframe << " = " << (mv_sceneframe - orig_sceneframe) << " in the scene frame\n";
                        svp->addViewTranslation ((mv_sceneframe - orig_sceneframe));
                    }
                } else {
                    std::cout << "No intersection with triangle t1t2t3\n";
                    translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());
                    //svp->addViewTranslation (cam_to_scene * mv_camframe);
                    svp->setViewTranslation (land_to_scene * mv_landframe_mat * scene_to_land * svp->get_viewmatrix_origin());
                }
            } else {
                std::cout << "No land, translate cameras only.\n";
                translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());
            }

            // Up-down (pitch) is rotation about local camera frame axis x
            rotateCamerasLocallyAround (v.getVerticalRotationAngle (opts.test(eye3d::options::keep_moving)), 1.0f, 0.0f, 0.0f);
            // Left-and-right (yaw) is rotation about local camera frame axis y
            rotateCamerasLocallyAround (v.getHorizontalRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 1.0f, 0.0f);
            // Roll
            rotateCamerasLocallyAround (v.getRollRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 0.0f, 1.0f);

            cam_to_scene = mplot::compoundray::getCameraSpace (scene);

            std::cout << std::endl; // debug

        } else { // not actively moving
            // Get the camera space and update our eye and camera-frame models
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        }

        // reset to initial camera space if requested
        if (v.vstate.test (eye3dvisual::state::campose_reset_request) == true) {
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (initial_camera_space));
            v.stop(); // cancel any active movements
            cam_to_scene = initial_camera_space;
            v.vstate.reset (eye3dvisual::state::campose_reset_request);
        }

        // Update the view matrix of eye and eye localspace axes
        eyevm_ptr->setViewMatrix (cam_to_scene);
        cam_cs_ptr->setViewMatrix (cam_to_scene);
    };

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
