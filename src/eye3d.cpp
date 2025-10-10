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
#include <mplot/PolygonVisual.h> // debug

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

    /*
     * The partial movement that takes us to the crossing point, specified as movement + endpoint
     * (rather than startpoint + movement)
     */
    struct partial_movement
    {
        // The movement vector
        sm::vec<float> mv = {};
        // The end coordinate of the movement
        sm::vec<float> end = {};
    };

    /*
     * Find the part of mv_inplane that gets us to the triangle boundary defined by edge_s and
     * edge_e
     *
     * IS IS ASSUMED that mv_s is in the triangle plane and that a movement of mv_inplane would cross
     * the edge if it were long enough.
     *
     * The triangle is part of a model, which has its own coordinate frame. All vectors and
     * coordinates here are in the model coordinate frame.
     *
     * \param edge_s Starting coordinate of the edge
     * \param edge_e End coordinate of the edge
     * \param t_norm The triangle normal vector
     * \param mv_s The movement starting point
     * \param mv_inplane The planned movement, starting from hovlocn
     *
     * \return a struct containing the partial movement vector and the end of the movement as a
     * coordinate. If mv_inplane does not cross the edge, then the return object contains the vector
     * mv_inplane itself, and the coordinate that this movement ends at.
     */
    eye3d::partial_movement find_edge_crossing (const sm::vec<float>& edge_s,
                                                const sm::vec<float>& edge_e,
                                                const sm::vec<float>& t_norm,
                                                const sm::vec<float>& mv_s,
                                                const sm::vec<float>& mv_inplane)
    {
        constexpr bool debug = false;

        eye3d::partial_movement pm;

        sm::vec<float> edge = edge_e - edge_s;

        sm::vec<float> u_y = edge;
        u_y.renormalize();
        sm::vec<float> u_z = t_norm;
        u_z.renormalize();
        sm::vec<float> u_x = u_y.cross (u_z);
        if constexpr (debug) {
            std::cout << "edge = " << edge << std::endl;
            std::cout << "Basis: " << u_x << " " << u_y << " " << u_z << std::endl;
        }

        // Create a matrix to convert from mdl frame movements to the triangle frame of ref.
        sm::mat44<float> from_triangle_frame = sm::mat44<float>::frombasis (u_x, u_y, u_z);

        sm::mat44<float> to_triangle_frame = from_triangle_frame.inverse();

        // Use Edge as our 'y' and the orthogonal as our 'x', then express mv_inplane in terms
        // of these two unit vectors. We also have our 'z' which is the triangle normal.
        sm::vec<float, 4> mv_inplane4d = to_triangle_frame * mv_inplane;
        sm::vec<float, 2> mv_inplane2d = { mv_inplane4d[0], mv_inplane4d[1] };

        sm::vec<float, 4> h_4d = to_triangle_frame * mv_s;
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

        // Can now apply algo to find crossing point
        if constexpr (debug) {
            std::cout << "intersection test for lines: " << orig_2d << " --> " << edge_2d
                      << " and " << h_2d << " --> " << (h_2d + mv_inplane2d) << "\n";
        }

        std::bitset<2> si = sm::algo::segments_intersect<float> (orig_2d + edge_s_2d, edge_2d + edge_s_2d, h_2d, h_2d + mv_inplane2d);
        if (si.test(1)) {
            throw std::runtime_error ("Deal with colinear movement and triangle edge!\n");
        } else {
            if (si.test(0)) {
                // Intersects as expected
                sm::vec<float, 2> cross_point_2d = sm::algo::crossing_point<float> (orig_2d + edge_s_2d, edge_2d + edge_s_2d, h_2d, h_2d + mv_inplane2d);
                if constexpr (debug) { std::cout << "Cross point (2d) is " << cross_point_2d << std::endl; }
                // Now go from cross point 2d to a point in model coordinates?
                pm.end = (from_triangle_frame * cross_point_2d.plus_one_dim(edge_s_4d[2])).less_one_dim();
                if constexpr (debug) { std::cout << "Cross point in mdl frame: " << pm.end << std::endl; }
                pm.mv = pm.end - mv_s;
            } else {
                if constexpr (debug) {
                    std::cout << "Huh?!? Got no intersection across edge for:\n";
                    std::cout << (orig_2d + edge_s_2d) << " -- " << (edge_2d + edge_s_2d) << " AND "
                              << h_2d << " -- " << (h_2d + mv_inplane2d) << std::endl;
                    // Don't expect a lack of intersection, but for debug mode:
                    pm.mv = mv_inplane;
                    pm.end = mv_s + mv_inplane;
                } else {
                    throw std::runtime_error ("Huh?!? Got no intersection across edge?");
                }
            }
        }

        return pm;
    }

    /*
     * After testing up to all three edges of a triangle, we return information about the crossing
     * location and the indices of the triangle that form the crossed edge in this structure.
     */
    struct crossing_data
    {
        // Did we get an intersection of the movement mv_inplane through a triangle edge?
        bool crossed = false;
        // If we crossed a specific vertex, say which one here
        uint32_t crossed_vertex = std::numeric_limits<uint32_t>::max();
        // edge_idx_a/b are the indices of the triangle vertices on the crossed edge
        uint32_t edge_idx_a = 0;
        uint32_t edge_idx_b = 0;
        // The crossed edge as a vector
        sm::vec<float> tri_edge = {};
        // The partial movement. pm.mv is the movement, pm.end is the crossing point
        eye3d::partial_movement pm = {};
    };

    /*
     * Find the location at which a movement from mv_s in the direction mv_inplane crosses one of
     * the edges of the triangle specified by the three vertices in t_verts/t_indices.
     *
     * IT IS ASSUMED that the triangle normal passing through mv_s WILL intersect the
     * triangle. (Test beforehand with sm::algo::ray_tri_intersection)
     *
     * All coordinates are in the frame of the model that contains this triangle.
     *
     * \param t_verts *Ordered* vertices of the triangle. Vertices should be in anti-clockwise order
     * when viewed with the triangle normal vector coming 'out of the page'
     *
     * \param t_indices The *Ordered* indices of the vertices in t_verts. Used to return the crossed
     * edge specified as two common indices. See t_verts for correct order of triangle vertices.
     *
     * \param mv_s The start of the planned movement
     *
     * \param mv_inplane The planned movement
     *
     * \param t_norm The triangle normal. While this could be computed from t_verts, it has already
     * been computed by the program and so I'm passing it in here.
     */
    eye3d::crossing_data compute_crossing_location (const sm::vec<sm::vec<float>, 3>& t_verts,
                                                    const std::array<uint32_t, 3>& t_indices,
                                                    const sm::vec<float>& mv_s,
                                                    const sm::vec<float>& mv_inplane,
                                                    const sm::vec<float>& t_norm)
    {
        eye3d::crossing_data cd;

        const sm::vec<float>& t0 = t_verts[0];
        const sm::vec<float>& t1 = t_verts[1];
        const sm::vec<float>& t2 = t_verts[2];

        sm::vec<float> p = mv_s + mv_inplane;
        sm::vec<float> edge = t1 - t0;
        sm::vec<float> ptoe = p - t0;
        bool inside01 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside01) {
            cd.edge_idx_a = t_indices[0]; cd.edge_idx_b = t_indices[1];
            cd.pm = eye3d::find_edge_crossing (t0, t1, t_norm, mv_s, mv_inplane);
            cd.tri_edge = edge;
        }

        edge = t2 - t1; ptoe = p - t1;
        bool inside21 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside21) {
            cd.edge_idx_a = t_indices[2]; cd.edge_idx_b = t_indices[1];
            cd.pm = eye3d::find_edge_crossing (t1, t2, t_norm, mv_s, mv_inplane);
            cd.tri_edge = edge;
        }

        edge = t0 - t2; ptoe = p - t2;
        bool inside02 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside02) {
            cd.edge_idx_a = t_indices[0]; cd.edge_idx_b = t_indices[2];
            cd.pm = eye3d::find_edge_crossing (t2, t0, t_norm, mv_s, mv_inplane);
            cd.tri_edge = edge;
        }
        if (!inside01 || !inside21 || !inside02) {
            std::cout << "Crossed over " << (inside01 ? " " : "0-1") << (inside21 ? " " : "2-1") <<  (inside02 ? " " : "0-2") << std::endl;
            cd.crossed = true;
            if (!inside01 && !inside21) {
                // crossed vertex 1
                cd.crossed_vertex = 1;
            } else if (!inside01 && !inside02) {
                // crossed vertex 0
                cd.crossed_vertex = 0;
            } else if (!inside21 && !inside02) {
                // crossed vertex 2
                cd.crossed_vertex = 2;
            } // else crossed one edge
        } else {
            std::cout << "No crossings " << (inside01 ? " " : "!!0-1") << (inside21 ? " " : "!!2-1") <<  (inside02 ? " " : "!!0-2") << std::endl;
        }

        return cd;
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
    v.coordArrowsInScene (true);
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
            if (vmp->name == "Cube.002" && land == nullptr) { land = vmp; }
            else if (vmp->name == "Cube.001" && land == nullptr) { land = vmp; }
            else if (vmp->name == "Landscape.003" && land == nullptr) { land = vmp; }
            else if (vmp->name == "Rock.Landscape.Style_2.Mesh.003" && land == nullptr) { land = vmp; }
            else { std::cout << "Model name " << vmp->name << std::endl; }
        }
    }
    sm::vec<> hp_scene = {};

    mplot::PolygonVisual<>* svp = nullptr;

    mplot::SphereVisual<>* svp2 = nullptr;
    mplot::SphereVisual<>* svp4 = nullptr;

    mplot::SphereVisual<>* svp_t0 = nullptr;
    mplot::SphereVisual<>* svp_t1 = nullptr;
    mplot::SphereVisual<>* svp_t2 = nullptr;

    mplot::RodVisual<>* rvp1 = nullptr;
    mplot::RodVisual<>* rvp2 = nullptr;
    mplot::RodVisual<>* rvp3 = nullptr;

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

        land_to_scene = land->getViewMatrix();
        scene_to_land = land_to_scene.inverse();
        sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);
        auto camloc = camspace * sm::vec<>{0,10,0};
        auto camloc_landframe = (scene_to_land * camloc).less_one_dim();
        auto [hit, ti, tn] = land->find_triangle_crossing (camloc_landframe);
        std::cout << "find_triangle_crossing hit: " << hit << ", ti[0] = " << ti[0] << "\n";
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

        sm::vec<float, 3> posn = {0, 0, -0.0025};
        sm::vec<float, 3> dirvertex = {-1, 0, 0};
        auto pv = std::make_unique<mplot::PolygonVisual<>>(hp_scene, posn, dirvertex, 0.005, 0.005, mplot::colour::goldenrod3, 4);
        v.bindmodel (pv);
        pv->finalize();
        svp = v.addVisualModel (pv);

        auto sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.005, mplot::colour::magenta3);
        v.bindmodel (sv);
        sv->finalize();
        svp2 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.0025, mplot::colour::black);
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

        auto rv = std::make_unique<mplot::RodVisual<>>(land->get_viewmatrix_origin(), sm::vec<>{}, sm::vec<>{-2,2,2}, 0.001f, mplot::colour::crimson);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp2 = v.addVisualModel (rv);

        rv = std::make_unique<mplot::RodVisual<>>(land->get_viewmatrix_origin(), sm::vec<>{}, sm::vec<>{-2,2,2}, 0.001f, mplot::colour::blue);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp1 = v.addVisualModel (rv);

        rv = std::make_unique<mplot::RodVisual<>>(land->get_viewmatrix_origin(), sm::vec<>{}, sm::vec<>{-2,1,2}, 0.0011f, mplot::colour::springgreen);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp3 = v.addVisualModel (rv);

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
                                 opts, land, &svp, &svp2, &svp4, &svp_t0, &svp_t1, &svp_t2, &rvp1, &rvp2, &rvp3, land_to_scene, scene_to_land, &tn0_land, &ti0, hoverheight]()
    {
        cam_cs_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));

        sm::mat44<float> cam_to_scene;
        if (v.isActivelyMoving()) {

            sm::vec mv_camframe = v.getMovementVector (opts.test(eye3d::options::keep_moving));
            sm::mat44<float> mv_camframe_mat;
            mv_camframe_mat.translate (mv_camframe);
            // If constrained by land, determine here if t will move it into a new triangle on the land
            if (land) {

                // Compute transform amtrices that we'll be needing:
                cam_to_scene = mplot::compoundray::getCameraSpace (scene);
                // cam_to_scene transforms camera frame to scene frame
                // scene_to_land * cam_to_scene transforms camera frame to model frame
                sm::mat44<float> cam_to_land = scene_to_land * cam_to_scene;
                // Movement in the land frame
                sm::mat44<float> mv_landframe_mat = cam_to_land * mv_camframe_mat;
                // Camera location in the land frame
                sm::vec<> camloc_landframe = (cam_to_land * sm::vec<>{}).less_one_dim();

                // 0. Find vertices of triangle, given its indices. These are in the land model frame
                sm::vec<sm::vec<>, 3> tv_landframe = land->triangle_vertices (ti0);
                std::cout << "ti0 " << ti0[0] << "," << ti0[1] << "," << ti0[2]
                          << " has vertices (landframe) at " << tv_landframe
                          << " and upcoming movement (camframe) of " << mv_camframe << std::endl;

                // 1. Find component of t that is in the current triangle plane. t is in camera frame.
                sm::vec<> mv_landframe = (cam_to_land * mv_camframe).less_one_dim() - camloc_landframe;
                sm::vec<> mv_orthog = tn0_land * (mv_landframe.dot (tn0_land) / (tn0_land.dot(tn0_land)));
                sm::vec<> mv_inplane = mv_landframe - mv_orthog; // landframe

                // 2. Determine if that component will cross any edge of the triangle
                // 3. Work out what the new edge is and find a location on that triangle
                // 4. Find the 'hover location' over that new location

                // Debug/vis
                //std::cout << "Current hover triangle is " << tv_landframe;
                svp_t0->setViewTranslation (land_to_scene * tv_landframe[0]);
                svp_t1->setViewTranslation (land_to_scene * tv_landframe[1]);
                svp_t2->setViewTranslation (land_to_scene * tv_landframe[2]);
                //std::cout << " with normal " << tn0_land << std::endl;

                // Does camloc_landframe in dirn tn0_land intersect the tv_landframe triangle?
                auto [ isect, hovlocn ] = sm::algo::ray_tri_intersection<float> (tv_landframe[0], tv_landframe[1], tv_landframe[2], camloc_landframe, -tn0_land);

                // Debug/vis
                //std::cout << "hovlocn: " << hovlocn << std::endl;   // hovlocn is in landframe

                // Ideally, want to keep the orientation from cam_to_land, but move it to hovlocn.
                // std::cout << "cam_to_land = \n" << cam_to_land << " and hovlocn is " << hovlocn << std::endl;

                sm::vec<> cam_displacement  = cam_to_land.translation() - hovlocn;
                sm::mat44<float> cam_to_surface = cam_to_land;
                cam_to_surface.translate (-cam_displacement); // This is our init pose, placed on the surface

                //svp2->setViewTranslation (land_to_scene * hovlocn); // last hover locn is magenta
                svp2->setViewMatrix (land_to_scene * cam_to_surface); // last hover locn is magenta
                svp->setViewMatrix (land_to_scene * cam_to_surface);

                rvp2->update (hovlocn, hovlocn + mv_inplane);

                if (isect) {

                    bool done = false;
                    //bool done2 = false;
                    bool simple = false;

                    sm::mat44<float> sink;
                    sm::mat44<float> unsink;

                    // Want initial pose matrix here.
                    sm::mat44<float> reorient_final = cam_to_surface;
                    std::cout << "Initial sphere pose:\n" << cam_to_surface << std::endl;

                    sm::mat44<float> reorient_cam_final;

                    while (!done) {

                        // For each edge in triangle, compute distance to edge for hovlocn and (hovlocn + mv_inplane)
                        eye3d::crossing_data cd = eye3d::compute_crossing_location (tv_landframe, ti0, hovlocn, mv_inplane, tn0_land);

                        // Debug/vis
                        svp4->setViewTranslation (land_to_scene * cd.pm.end); // cross point is black sphere

                        if (cd.crossed) {

                            // Can work out new triangle here
                            std::cout << "find_other_triangle_containing ("
                                      << cd.edge_idx_a << ", " <<  cd.edge_idx_b
                                      << ", [" <<  ti0[0] << "," << ti0[1] << "," << ti0[2] << "])" << std::endl;
                            auto [_ti, _tn] = land->find_other_triangle_containing (cd.edge_idx_a, cd.edge_idx_b, ti0);

                            if (_ti[0] != std::numeric_limits<uint32_t>::max()) {

                                // Re-orient onto the new triangle

                                sm::vec<sm::vec<>, 3> newtv_landframe = land->triangle_vertices (_ti);
                                std::cout << "Re-orient to new triangle " << _ti[0] << "," << _ti[1] << "," << _ti[2]
                                          << "[ " << newtv_landframe << " ] with normal " << _tn << "\n";

                                sm::mat44<float> reorient_land; // reorientation transformation in landframe
                                sm::vec<float, 4> mv_rest;

                                sm::mat44<float> r_t1;
                                sm::mat44<float> r_t_to;
                                sm::mat44<float> r_r;
                                sm::mat44<float> r_t_fro;
                                sm::mat44<float> r_t2;
                                {
                                    // Initial rotational state of the land!

                                    // Rotate by the angle between the normals. I think this is constrained to be <= pi
                                    float rotn_angle = tn0_land.angle (_tn, cd.tri_edge);
                                    std::cout << "rotn edge is " << cd.tri_edge << " and angle is " << rotn_angle << "\n";
                                    // Use the *edge* as the rotation axis.
                                    cd.tri_edge.renormalize(); // !!! although in future this won't be necessary (maths main has quaternion that auto-renormalizes)
                                    r_r.rotate (cd.tri_edge, rotn_angle);
                                    // Before doing any additional work, apply this rotation to mv_rest
                                    mv_rest = r_r * (mv_inplane - cd.pm.mv);
                                    // The edge may not already be a coordinate axis, so pre- and post-translate by hovlocn
                                    r_t_to.translate (-(hovlocn + cd.pm.mv));
                                    r_t_fro.translate (hovlocn + cd.pm.mv);
                                    // Apply pre- and post-translations.
                                    r_t1.translate (cd.pm.mv);
                                    r_t2.translate (mv_rest);

                                    reorient_land = r_t2 * r_t_fro * r_r * r_t_to * r_t1;
                                }

                                // Show mv_rest as green tube, which should enclose the blue tube of rvp1
                                rvp3->update (hovlocn + cd.pm.mv, hovlocn + cd.pm.mv + mv_rest.less_one_dim());

                                // Angle between cm.pm.mv and mv_rest?
                                std::cout << "Angle between pre-rotate and post- moves? "
                                          << sm::mathconst<float>::rad2deg * cd.pm.mv.angle (mv_rest.less_one_dim()) << std::endl;

                                // At this point, can test to see if the end point of the movement
                                // lands in the adjacent triangle. If so, we're done, if not, time
                                // for another loop.
                                sm::vec<float> endmv = (reorient_land * reorient_final * sm::vec<>{}).less_one_dim();
                                // Is endmv in newtv_landframe/_ti?
                                bool isect2 = false;
                                sm::vec<> isectpoint2 = {};

                                std::tie (isect2, isectpoint2) = sm::algo::ray_tri_intersection<float> (newtv_landframe[0], newtv_landframe[1], newtv_landframe[2],
                                                                                                        endmv + (_tn / 2.0f), -_tn);

                                std::cout << "endmv = " << endmv << " DOES " << (isect2 ? "" : "NOT ") << "land in next tri\n";

                                if (isect2 /* || done2*/) {
                                    // Complete, exit loop
                                    reorient_final = reorient_land * reorient_final;

                                    // Show remaining movement in blue
                                    rvp1->update (hovlocn + cd.pm.mv, isectpoint2);

                                    done = true;
                                } else {
                                    // Incomplete.
                                    // We've sailed past newtv_landframe
                                    // We need to set an end-point that is on newtv_landframe, update hovlocn, then recurse.
                                    // also recompute the movement encoded in reorient_land
                                    reorient_land.translate (-mv_rest);
                                    reorient_final = reorient_land * reorient_final;

                                    hovlocn = cd.pm.end; // crossing data planned movement end

                                    // Also update planned move, which is now shorter and in a new direction
                                    tv_landframe = newtv_landframe;
                                    mv_inplane = mv_rest.less_one_dim();
                                }

                                // Set sink/unsink and apply to camera transform
                                sink.setToIdentity();
                                sink.translate (-tn0_land * hoverheight); // assumes we normalized tn0

                                unsink.setToIdentity();
                                unsink.translate (_tn * hoverheight); // assumes we normalized _tn
                                reorient_cam_final = unsink * reorient_final;

                                ti0 = _ti;
                                tn0_land = _tn;
                            }

                        } else {
                            translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());
                            // Whats the movement of the camera in the scene frame?
                            sm::vec<float, 4> mv_sceneframe = cam_to_scene * mv_camframe;
                            sm::vec<float, 4> orig_sceneframe = cam_to_scene * sm::vec<>{};
                            svp->addViewTranslation ((mv_sceneframe - orig_sceneframe));
                            done = true;
                            simple = true; // to avoid a further transformation with reorient_final
                        }

                    } // end while

                    if (simple) {
                        // Already moved camera
                    } else {
                        // a) Get sphere locn into land model frame. Note: *starts* with sphere location
                        // b) apply reorient_land
                        // c) Return sphere lcon into scene coordinates (or bunch it all together):
                        // d) Update svp->viewmatrix
                        // All in one line to update the sphere indicator's location
                        svp->setViewMatrix (land_to_scene * reorient_final);

                        setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (land_to_scene * reorient_cam_final));
                    }

                } else {
                    std::cout << "No intersection with triangle t1t2t3\n";
                    translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());
                    svp->setViewTranslation (land_to_scene * mv_landframe_mat * scene_to_land * svp->get_viewmatrix_origin());
                }

            } else { // no landscape object to follow
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
