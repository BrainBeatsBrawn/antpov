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
#include <mplot/SphereVisual.h>  // for debug
#include <mplot/RodVisual.h>     // also for debug
#include <mplot/PolygonVisual.h> // debug
#include <mplot/NormalsVisual.h>

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
        can_exit,         // Can exit the program
        show_normals      // show normal vectors on the land
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
            } else if (arg == "-n") {
                opts |= eye3d::options::show_normals;
            } else if (arg == "-k") {
                opts |= eye3d::options::keep_moving;
            }
        }
        if (path.empty()) {
            eye3d::printHelp();
            opts |= eye3d::options::can_exit;
        }
        return path;
    }

    // Flags class
    enum class pm_fl : uint32_t
    {
        no_cross_point, // Means 'there was no crossing'
        colinear        // Means the movement was colinear with an edge
    };
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
        // boolean state
        sm::flags<eye3d::pm_fl> flags;
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
            std::cout << "fec: mv_inplane = " << mv_inplane << std::endl;
            std::cout << "fec: edge = " << edge << std::endl;
            std::cout << "fec: Basis: " << u_x << " " << u_y << " " << u_z << std::endl;
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

        // Can now apply algo to find crossing point
        if constexpr (debug) {
            std::cout << "fec: intersection test for lines: " << edge_s_2d << " --> " << (edge_2d + edge_s_2d)
                      << " and " << h_2d << " --> " << (h_2d + mv_inplane2d) << "\n";
        }

        std::bitset<2> si = sm::algo::segments_intersect<float> (edge_s_2d, edge_s_2d + edge_2d, h_2d, h_2d + mv_inplane2d);
        if (si.test(1)) {
            if constexpr (debug) { std::cout << "fec: Colinear with triangle edge!\n"; }
            pm.flags.set (eye3d::pm_fl::colinear, true);
            // Identify the vertex that we're moving towards. edge_4d is the triangle edge.
            // so: mv_inplane4d.dot (edge_4d) should be positive if edge_e is the vertex
            sm::vec<float> mv_inplane3d = mv_inplane4d.less_one_dim();
            sm::vec<float> edge_e_3d = (to_triangle_frame * edge_e).less_one_dim();
            sm::vec<float> edge_s_3d = edge_s_4d.less_one_dim();

            if constexpr (debug) {
                std::cout << "mv_inplane: " << mv_inplane3d << ", edge_e: " << edge_e_3d << ", edge_s: " << edge_s_3d << std::endl;
                std::cout << "mv_inplane.dot (edge_e): " << mv_inplane3d.dot (edge_e_3d) << std::endl;
                std::cout << "mv_inplane.dot (edge_s): " << mv_inplane3d.dot (edge_s_3d) << std::endl;
            }
            sm::vec<float> to_v = {};
            if (mv_inplane3d.dot (edge_e_3d) > mv_inplane3d.dot (edge_s_3d)) {
                to_v = edge_e_3d - (h_4d).less_one_dim();
            } else {
                to_v = edge_s_3d - (h_4d).less_one_dim();
            }

            if (to_v.length() <= mv_inplane3d.length()) {
                if constexpr (debug) { std::cout << "fec: partial colinear move to vertex\n"; }
                pm.flags.set (eye3d::pm_fl::no_cross_point, false);
                pm.mv = (from_triangle_frame * to_v).less_one_dim(); // need to know if we were to go over a vertex
                pm.end = (from_triangle_frame * edge_e_3d).less_one_dim();
            } else {
                if constexpr (debug) { std::cout << "fec: partial colinear along/within edge\n"; }
                pm.flags.set (eye3d::pm_fl::no_cross_point, true);
                // Compute end from mv_inplane4d
                pm.mv = (from_triangle_frame * mv_inplane4d).less_one_dim();
                pm.end = (from_triangle_frame * (h_4d + mv_inplane4d)).less_one_dim();
            }

        } else {
            if (si.test(0)) {
                // Intersects as expected
                sm::vec<float, 2> cp2d = sm::algo::crossing_point<float> (edge_s_2d, edge_s_2d + edge_2d, h_2d, h_2d + mv_inplane2d);
                // Now go from cross point 2d to a point in model coordinates?
                pm.end = (from_triangle_frame * cp2d.plus_one_dim(edge_s_4d[2])).less_one_dim();
                if constexpr (debug) { std::cout << "fec: Cross point in mdl frame: " << pm.end << std::endl; }
                pm.mv = pm.end - mv_s;

            } else {
                // 'No intersection' can occur when: the movement goes over/close to the end of the edge.
                // Or when: the move starts ON the edge of a triangle and then moves *away* from the tri.
                if constexpr (debug) {
                    std::cout <<  "fec: No intersection across edge for: "
                              << (edge_s_2d) << " -- " << (edge_2d + edge_s_2d) << " and "
                              << h_2d << " -- " << (h_2d + mv_inplane2d) << std::endl;
                }
                // Mark that there was no intersection
                pm.flags.set (eye3d::pm_fl::no_cross_point, true);
                pm.mv = sm::vec<>{};
                pm.end = mv_s;
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
     * triangle (this may include an edge or vertex intersection). (Test beforehand with sm::algo::ray_tri_intersection)
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
        constexpr bool debug = false;
        eye3d::crossing_data cd;
        cd.pm.flags.set (eye3d::pm_fl::no_cross_point, true);

        const sm::vec<float>& t0 = t_verts[0];
        const sm::vec<float>& t1 = t_verts[1];
        const sm::vec<float>& t2 = t_verts[2];

        sm::vec<float> p = mv_s + mv_inplane;
        sm::vec<float> edge = t1 - t0;
        sm::vec<float> ptoe = p - t0;
        bool inside01 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside01) {
            eye3d::partial_movement pm = eye3d::find_edge_crossing (t0, t1, t_norm, mv_s, mv_inplane);
            if constexpr (debug) {
                if (pm.flags.test (eye3d::pm_fl::colinear)) {
                    std::cout << "ccl: fec returned pm.colinear true for t0t1\n";
                }
            }
            if (pm.flags.test (eye3d::pm_fl::no_cross_point)
                && pm.flags.test (eye3d::pm_fl::colinear) == false) {
                inside01 = true;
                if constexpr (debug) {
                    std::cout << "ccl: No intersection for edge t0t1 " << t0 << " -- " << t1
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
            } else {
                if constexpr (debug) {
                    if (pm.flags.test (eye3d::pm_fl::colinear)) { std::cout << "ccl: colinear t0t1\n"; }
                    std::cout << "ccl: Intersection for edge t0t1 " <<  t0 << " -- " << t1
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
                cd.pm = pm;
                cd.tri_edge = edge;
                cd.edge_idx_a = t_indices[0];
                cd.edge_idx_b = t_indices[1];
            }
        }

        edge = t2 - t1; ptoe = p - t1;
        bool inside21 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside21) {
            eye3d::partial_movement pm = eye3d::find_edge_crossing (t1, t2, t_norm, mv_s, mv_inplane);
            if constexpr (debug) {
                if (pm.flags.test (eye3d::pm_fl::colinear)) {
                    std::cout << "ccl: fec returned pm.colinear true for t1t2\n";
                }
            }
            if (pm.flags.test (eye3d::pm_fl::no_cross_point)
                && pm.flags.test (eye3d::pm_fl::colinear) == false) {
                inside21 = true;
                if constexpr (debug) {
                    std::cout << "ccl: No intersection for edge t1t2 " << t1 << " -- " << t2
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
            } else {
                if constexpr (debug) {
                    if (pm.flags.test (eye3d::pm_fl::colinear)) { std::cout << "ccl: colinear t1t2\n"; }
                    std::cout << "ccl: Intersection for edge t1t2 " <<  t1 << " -- " << t2
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
                cd.pm = pm;
                cd.tri_edge = edge;
                cd.edge_idx_a = t_indices[2];
                cd.edge_idx_b = t_indices[1];
            }
        }

        edge = t0 - t2; ptoe = p - t2;
        bool inside02 = (t_norm.dot (edge.cross (ptoe)) >= 0);
        if (!inside02) {
            eye3d::partial_movement pm = eye3d::find_edge_crossing (t2, t0, t_norm, mv_s, mv_inplane);
            if constexpr (debug) {
                if (pm.flags.test (eye3d::pm_fl::colinear)) {
                    std::cout << "ccl: fec returned pm.colinear true for t2t0\n";
                }
            }
            if (pm.flags.test (eye3d::pm_fl::no_cross_point)
                && pm.flags.test (eye3d::pm_fl::colinear) == false) {
                inside02 = true;
                if constexpr (debug) {
                    std::cout << "ccl: No intersection for edge t2t0 " << t2 << " -- " << t0
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
            } else {
                if constexpr (debug) {
                    if (pm.flags.test (eye3d::pm_fl::colinear)) { std::cout << "ccl: colinear t2t0\n"; }
                    std::cout << "ccl: Intersection for edge t2t0 " <<  t2 << " -- " << t0
                              << " and move " << mv_s << " -- " << (mv_s + mv_inplane) << std::endl;
                }
                cd.pm = pm;
                cd.tri_edge = edge;
                cd.edge_idx_a = t_indices[0];
                cd.edge_idx_b = t_indices[2];
            }
        }

        // We've now tested edge crossing for three edges in the triangle.
        //
        if constexpr (debug) {
            if (cd.pm.flags.test (eye3d::pm_fl::no_cross_point) == false) {
                std::cout << "ccl: Crossed over" << (inside01 ? " " : " 0-1")
                          << (inside21 ? " " : " 2-1") <<  (inside02 ? " " : " 0-2") << std::endl;
                // could test pairs of inside01 etc to detect crossing a vertex
            } else if (cd.pm.flags.test (eye3d::pm_fl::colinear) == true) {
                // Movement was colinear. Set Crossed vertex?
                std::cout << "ccl: movement was colinear!\n";
                if (cd.pm.flags.test (eye3d::pm_fl::no_cross_point)) {
                    std::cout << "ccl: Colinear along edge" << std::endl;
                } else {
                    std::cout << "ccl: Colinear to vertex" << std::endl;
                }
                // cd.pm.no_cross_point will tell if there's a cross point or not
            } else {
                // We have NO crossing, which can occur for a variety of reasons
                std::cout << "ccl: No crossings " << (inside01 ? " " : "!!0-1")
                          << (inside21 ? " " : "!!2-1") <<  (inside02 ? " " : "!!0-2") << std::endl;
            }
        }

        return cd;
    }

    // Find the land location. land: obvious; loc1: location to start from in land model frame.
    std::tuple<sm::vec<>, sm::vec<>, std::array<uint32_t, 3>>
    find_land (mplot::VisualModel<>* land, const sm::vec<>& loc1)
    {
        sm::mat44<float> land_to_scene = land->getViewMatrix();
        sm::mat44<float> scene_to_land = land_to_scene.inverse();
        sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);
        sm::vec<float> camstart = {}; // use camera location in gltf to start from, then find land surface.
        sm::vec<float, 4> camloc = camspace * camstart;
        sm::vec<float> camloc_landframe = (scene_to_land * camloc).less_one_dim();
        std::array<uint32_t, 3> ti0;
        sm::vec<float> tn0_land = {};
        sm::vec<float> hit = {};
        std::tie (hit, ti0, tn0_land) = land->find_triangle_crossing (camloc_landframe);
        // Can I make hit the centre of the triangle?
        constexpr bool hit_tri_centre = false;
        if constexpr (hit_tri_centre) {
            sm::vec<sm::vec<>, 3> tv_landframe = land->triangle_vertices (ti0);
            hit = tv_landframe.mean();
        }
        sm::vec<> hp_scene = (land_to_scene * hit).less_one_dim();
        return { hp_scene, tn0_land, ti0 };
    }

    // Using data about the land location for the camera found with eye3d::find_land, set the camera posn
    void set_landlocked_camera (const sm::vec<>& hp_scene, const sm::mat44<float>& land_to_scene,
                                mplot::VisualModel<>* land,
                                const sm::vec<>& tn0_land, const std::array<uint32_t, 3>& ti0,
                                mplot::PolygonVisual<>* pvp, const float hoverheight, bool randomize_dir = true)
    {
        // Let's 'draw' the camera towards the land and then arrange its normal upwards wrt to the normal of the land.
        if (ti0[0] == std::numeric_limits<uint32_t>::max()) {
            std::cout << "set_landlocked_camera: No hit\n";
            return;
        }

        // Place the camera on the land, and orient it randomly in the 'land plane'
        // Turn the hit point into a translation matrix (scene frame)
        sm::mat44<float> hp_scene_mat;
        hp_scene_mat.translate (hp_scene);

        // The camera frame always has y up. Choose a random vector in the plane for 'x'
        // and then set z from this random x and the triangle norm (y).
        sm::mat44<float> coord_rotn;
        if (randomize_dir) {
            // First determine rotation wrt the 'land' model
            sm::vec<> rand_vec;
            rand_vec.randomize();
            sm::vec<> _x = rand_vec.cross (tn0_land);
            _x.renormalize();
            sm::vec<> _z = _x.cross (tn0_land);
            coord_rotn = sm::mat44<float>::frombasis (_x, tn0_land, _z); // rotn from model frame to triangle
        } else {
            throw std::runtime_error ("handle this case");
#if 0
            // THIS IS DEBUG code to get one kind of camera oriented exactly on an edge
            // Get current camera orientation, extract rotation, use that?
            // otherwise, just use identity rotation (this will be wrong)
            sm::vec<> _x = {0,1,-1};
            _x.renormalize();
            sm::vec<> _z = {0,-1,-1};
            _z.renormalize();
            std::cout << "call frombasis ("<<_x<<", "<<tn0_land<<", "<<_z<<std::endl;
            coord_rotn = sm::mat44<float>::frombasis (_x, tn0_land, _z); // rotn from model frame to triangle
#endif
        }

        // Get the rotation from scene frame to model
        coord_rotn = land_to_scene.rotation_mat44() * coord_rotn;
        pvp->setViewMatrix (hp_scene_mat * coord_rotn); // reposition sphere
        // Want to place camera just 'above' hp.
        coord_rotn.pretranslate (hoverheight * tn0_land);
        setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (hp_scene_mat * coord_rotn));
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
    v.vstate.flip (eye3dvisual::state::show_camframe);

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
    std::cout << "Initial camera space for reset:\n" << initial_camera_space << std::endl;

    // Plot the visual models
    mplot::compoundray::scene_to_visualmodels (scene, &v);

    // Create an EyeVisual 'eye' in our mathplot scene, v.
    mplot::compoundray::EyeVisual<>* eyevm_ptr = nullptr;
    auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<>> (sm::vec<>{}, &ommatidiaData, ommatidia);
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

    mplot::PolygonVisual<>* pvp1 = nullptr;

    mplot::SphereVisual<>* svp1 = nullptr;
    mplot::SphereVisual<>* svp2 = nullptr;

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

        if (opts.test(eye3d::options::show_normals)) {
            // Create NormalsVisual
            auto nv = std::make_unique<mplot::NormalsVisual<>>(land);
            v.bindmodel (nv);
            nv->finalize();
            v.addVisualModel (nv);
        }

        auto loc1 = sm::vec<>{8.9f, -1.0f, 0.0f};

        // I have an issue where this may be a *scaling* transformation. This becomes awkward.
        land_to_scene = land->getViewMatrix();   // could be passed to find_land
        scene_to_land = land_to_scene.inverse();

        std::tie(hp_scene, tn0_land, ti0) = eye3d::find_land (land, loc1);

        sm::vec<float, 3> posn = {0, 0, -0.0025};
        sm::vec<float, 3> dirvertex = {-1, 0, 0};
        auto pv = std::make_unique<mplot::PolygonVisual<>>(hp_scene, posn, dirvertex, 0.0025, 0.0025, mplot::colour::goldenrod3, 4);
        v.bindmodel (pv);
        pv->finalize();
        pvp1 = v.addVisualModel (pv);

        auto sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.001, mplot::colour::magenta3);
        v.bindmodel (sv);
        sv->finalize();
        svp1 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.00105, mplot::colour::black);
        v.bindmodel (sv);
        sv->finalize();
        svp2 = v.addVisualModel (sv);

        auto rv = std::make_unique<mplot::RodVisual<>>(sm::vec<>{}, sm::vec<>{}, sm::vec<>{-2,2,1}, 0.0005f, mplot::colour::blue);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp1 = v.addVisualModel (rv);

        rv = std::make_unique<mplot::RodVisual<>>(sm::vec<>{}, sm::vec<>{}, sm::vec<>{-2,2,2}, 0.0005f, mplot::colour::crimson);
        v.bindmodel (rv);
        rv->use_oriented_tube = false;
        rv->finalize();
        rvp2 = v.addVisualModel (rv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.004, mplot::colour::orangered1);
        v.bindmodel (sv);
        sv->setAlpha (0.8f);
        sv->finalize();
        svp_t0 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.004, mplot::colour::darkgreen);
        v.bindmodel (sv);
        sv->setAlpha (0.8f);
        sv->finalize();
        svp_t1 = v.addVisualModel (sv);

        sv = std::make_unique<mplot::SphereVisual<>>(sm::vec<>{}, 0.004, mplot::colour::blue2);
        v.bindmodel (sv);
        sv->setAlpha (0.8f);
        sv->finalize();
        svp_t2 = v.addVisualModel (sv);

        // Set up our camera using the data obtained from find_land()
        eye3d::set_landlocked_camera (hp_scene, land_to_scene, land, tn0_land, ti0, pvp1, hoverheight, true);
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
     *
     * Lets separate into unconstrained and landscape-constrained versions
     */
    auto subr_key_move_camera = [&v, &eyevm_ptr, &cam_cs_ptr, &initial_camera_space, opts]()
    {
        cam_cs_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));
        sm::mat44<float> cam_to_scene;
        if (v.isActivelyMoving()) {
            sm::vec mv_camframe = v.getMovementVector (opts.test(eye3d::options::keep_moving));
            sm::mat44<float> mv_camframe_mat;
            mv_camframe_mat.translate (mv_camframe);
            translateCamerasLocally (mv_camframe.x(), mv_camframe.y(), mv_camframe.z());
            // Up-down (pitch) is rotation about local camera frame axis x
            rotateCamerasLocallyAround (v.getVerticalRotationAngle (opts.test(eye3d::options::keep_moving)), 1.0f, 0.0f, 0.0f);
            // Left-and-right (yaw) is rotation about local camera frame axis y
            rotateCamerasLocallyAround (v.getHorizontalRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 1.0f, 0.0f);
            // Roll
            rotateCamerasLocallyAround (v.getRollRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 0.0f, 1.0f);
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);
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

    auto subr_key_move_over_land = [&v, &eyevm_ptr, &cam_cs_ptr, &initial_camera_space,
                                    opts, land, &pvp1, &svp1, &svp2, &svp_t0, &svp_t1, &svp_t2,
                                    &rvp1, &rvp2,
                                    land_to_scene, scene_to_land, &tn0_land, &ti0, hoverheight]()
    {
        constexpr bool debug_move = false;
        cam_cs_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));
        sm::mat44<float> cam_to_scene;
        sm::mat44<float> cam_to_land;

        if (v.isActivelyRotating()) {
            // Only permit rotation around one axis:
            rotateCamerasLocallyAround (v.getHorizontalRotationAngle (opts.test(eye3d::options::keep_moving)), 0.0f, 1.0f, 0.0f);

        } else if (v.isActivelyMoving()) { // translating

            // Obtain the commanded movement vector and turn this into a translation matrix
            sm::vec mv_camframe = v.getMovementVector (opts.test(eye3d::options::keep_moving));
            sm::mat44<float> mv_camframe_mat;
            mv_camframe_mat.translate (mv_camframe); // has no rotation

            // Compute transform matrices between camera, land model and scene/world frames
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);
            cam_to_land = scene_to_land * cam_to_scene;
            // Camera location in the land frame
            sm::vec<> camloc_landframe = (cam_to_land * sm::vec<>{}).less_one_dim();

            // Convert indices to vertices for triangle ti0. These are in the land model frame
            sm::vec<sm::vec<>, 3> tv_landframe = land->triangle_vertices (ti0);
            if constexpr (debug_move) {
                std::cout << "ti0 " << ti0[0] << "," << ti0[1] << "," << ti0[2]
                          << " has vertices (landframe) at " << tv_landframe
                          << " and upcoming movement (camframe) of " << mv_camframe << std::endl;
            }

            // Does camloc_landframe in dirn tn0_land intersect the tv_landframe triangle? This
            // returns true if camloc_landframe is on the edge of the triangle or on a
            // vertex. Assumes we're above the model and within the length of tn0_land of the
            // surface.
            //
            // IF we're on an edge, then this intersection algo may disagree with
            // compute_crossing_location, which currently looks for crossing each of the three
            // boundaries and so requires that the start point is *within* the boundary.
            //
            auto [ isect, hov_land ] = sm::algo::ray_tri_intersection<float> (tv_landframe[0], tv_landframe[1], tv_landframe[2], camloc_landframe, -tn0_land);

            // Use the detected location, hov_land to compute the surface location of the camera - its 'hover location'
            sm::vec<> cam_displacement  = cam_to_land.translation() - hov_land;
            sm::mat44<float> cam_to_surface = cam_to_land;
            cam_to_surface.pretranslate (-cam_displacement); // This is our init pose, placed on the surface

            if (isect == false) {

                if constexpr (debug_move) {
                    std::cout << "No intersection (at start) with triangle "
                              << ti0[0] << "," << ti0[1] << "," << ti0[2]
                              << " from coord " << camloc_landframe << " and dirn " << -tn0_land
                              << ", so correct ti0 and tn0_land (if we can)" << std::endl;
                }

                // When very close to the boundary, ray_tri_intersection may fail. This triggers a
                // search for a neighbouring triangle which the camera may instead be hovering over
                // (this can occur when moving along an edge)
                for (uint32_t i = 0u; i < 3u; i++) {
                    uint32_t i1 = i;
                    uint32_t i2 = (i+1) % 3u;
                    auto [_ti, _tn] = land->find_other_triangle_containing (ti0[i1], ti0[i2], ti0);
                    if (_ti[0] != std::numeric_limits<uint32_t>::max()) {
                        // Test to see if start location was inside a neighbour
                        sm::vec<sm::vec<>, 3> tv_lf = land->triangle_vertices (_ti);
                        auto [ is, h ] = sm::algo::ray_tri_intersection<float> (tv_lf[0], tv_lf[1], tv_lf[2], camloc_landframe, -_tn);
                        if constexpr (debug_move) {
                            std::cout << "Start of move " << (is ? "IS" : "is NOT") << " in " << _ti[0] << "," << _ti[1] << "," << _ti[2] << std::endl;
                        }
                        if (is) {
                            if constexpr (debug_move) { std::cout << "*** Correcting!\n"; }
                            // We're in this neighbour, so update ti0/tn0_land and mark isect true
                            ti0 = _ti;
                            tn0_land = _tn;
                            isect = true;
                            // This requires a number of matrix recomputations:
                            hov_land = h;
                            cam_displacement = cam_to_land.translation() - hov_land;
                            cam_to_surface = cam_to_land;
                            cam_to_surface.pretranslate (-cam_displacement); // This is our init pose, placed on the surface
                            break;
                        }
                    }
                }
            }

            if (isect == false) {
                std::cout << "No intersection (at start) with triangle "
                          << ti0[0] << "," << ti0[1] << "," << ti0[2]
                          << " OR neighbours, from coord " << camloc_landframe << " and dirn " << -tn0_land
                          << ", so stop/freeze" << std::endl;
                v.stop();
                v.freeze (true);

            } else {

                if constexpr (debug_move) {
                    std::cout << "Start of move is IN triangle "
                              << ti0[0] << "," << ti0[1] << "," << ti0[2]
                              << " from coord " << camloc_landframe << " and dirn " << -tn0_land << std::endl;
                }

                // Find component that is in the current triangle plane, in the land model frame of reference
                sm::vec<> mv_landframe = (cam_to_land * mv_camframe).less_one_dim() - camloc_landframe;
                sm::vec<> mv_orthog = tn0_land * (mv_landframe.dot (tn0_land) / (tn0_land.dot(tn0_land)));
                sm::vec<> mv_inplane = mv_landframe - mv_orthog;

                // State for our loop
                sm::mat44<float> cam_final;
                bool done = false;
                bool detected_crossing = false;
                sm::vec<uint32_t, 2> detected_edge;
                sm::vec<> detected_edgevec = {};

                // Now loop while our path may traverse one or more triangles
                while (!done) {

                    // mv_inplane is in land frame but relative to current location
                    if (mv_inplane.length() == 0) {
                        throw std::runtime_error ("Zero length mv_inplane so stop/freeze/crash");
                    } // i.e. don't try to compute a movement

                    // For each edge in triangle, compute distance to edge for hov_land and (hov_land + mv_inplane)
                    eye3d::crossing_data cd = eye3d::compute_crossing_location (tv_landframe, ti0, hov_land, mv_inplane, tn0_land);

                    if (cd.pm.flags.test (eye3d::pm_fl::no_cross_point) == false || detected_crossing) {

                        if (detected_crossing) {
                            if constexpr (debug_move) {
                                std::cout << "This is a detected crossing; changing edge_idx_a/b to " << detected_edge << std::endl;
                            }
                            // We have to update our crossing data, as we detected a crossing over
                            // an edge (probably while moving along that edge)
                            cd.edge_idx_a = detected_edge[0];
                            cd.edge_idx_b = detected_edge[1];
                            cd.tri_edge = detected_edgevec;
                            cd.pm.mv = mv_inplane;
                            cd.pm.end = hov_land + mv_inplane;
                        }

                        // Can work out new triangle here
                        if constexpr (debug_move) {
                            std::cout << "find_other_triangle_containing ("
                                      << cd.edge_idx_a << ", " <<  cd.edge_idx_b
                                      << ", [" <<  ti0[0] << "," << ti0[1] << "," << ti0[2] << "])" << std::endl;
                        }

                        auto [_ti, _tn] = land->find_other_triangle_containing (cd.edge_idx_a, cd.edge_idx_b, ti0);

                        if (_ti[0] != std::numeric_limits<uint32_t>::max()) {

                            // Re-orient onto the new triangle
                            sm::vec<sm::vec<>, 3> newtv_landframe = land->triangle_vertices (_ti);

                            if constexpr (debug_move) {
                                std::cout << "Re-orient to new triangle " << _ti[0] << "," << _ti[1] << "," << _ti[2]
                                          << "[ " << newtv_landframe << " ] with normal " << _tn << "\n";
                            }

                            sm::mat44<float> reorient_land; // reorientation transformation in landframe
                            sm::vec<float, 3> mv_rest;
                            // Compute the reorientation due to the requested movement.
                            // Rotate by the angle between the normals. I think this is constrained to be <= pi
                            float rotn_angle = tn0_land.angle (_tn, cd.tri_edge);
                            reorient_land.rotate (cd.tri_edge, rotn_angle);
                            mv_rest = (reorient_land * (mv_inplane - cd.pm.mv)).less_one_dim();
                            reorient_land.pretranslate (hov_land + cd.pm.mv + mv_rest);
                            reorient_land.translate (-hov_land); // r_t_to + r_t1 = -(hov_land + cd.pm.mv) + cd.pm.mv = -hov_land

                            if (mv_rest.length() == 0) {
                                // The first movement to edge completed the movement. We actually landed ON the edge.
                                cam_to_surface = reorient_land * cam_to_surface;
                                done = true;
                            } else {
                                // There's additional movement to complete.
                                // At this point, can test to see if the end point of the movement
                                // lands in the adjacent triangle. If so, we're done, if not, time
                                // for another loop.
                                sm::vec<float> endmv = (reorient_land * cam_to_surface * sm::vec<>{}).less_one_dim();
                                // Is endmv in newtv_landframe/_ti?
                                bool isect2 = false;
                                sm::vec<> isectpoint2 = {};
                                std::tie (isect2, isectpoint2) = sm::algo::ray_tri_intersection<float> (newtv_landframe[0], newtv_landframe[1], newtv_landframe[2],
                                                                                                        endmv + (_tn / 2.0f), -_tn);
                                if constexpr (debug_move) {
                                    std::cout << "endmv = " << endmv << " DOES" << (isect2 ? "" : " NOT") << " land in next tri\n";
                                }
                                if (isect2) {
                                    // We DID land in the neighbouring triangle. We are done.
                                    cam_to_surface = reorient_land * cam_to_surface;
                                    done = true;
                                } else {
                                    // Incomplete; We've sailed past newtv_landframe.  We need to
                                    // set an end-point that is on newtv_landframe, update hov_land,
                                    // then recurse.  also recompute the movement encoded in
                                    // reorient_land
                                    reorient_land.pretranslate (-mv_rest);
                                    cam_to_surface = reorient_land * cam_to_surface;
                                    hov_land = cd.pm.end; // crossing data planned movement end
                                    // Also update planned move, which is now shorter and in a new direction
                                    tv_landframe = newtv_landframe;
                                    mv_inplane = mv_rest;
                                }
                            }

                            ti0 = _ti;
                            tn0_land = _tn;

                        } else { throw std::runtime_error ("other triangle not found?!"); }

                    } else { // no triangle edge crossing was detected with eye3d::compute_crossing_location

                        // We had intersection in ti0, but no apparent crossing over its edges.
                        // We may have moved entirely within the starting triangle or colinear with an edge. Test for these cases.

                        // Check if it was a colinear movement
                        bool single_movement = false;
                        if (cd.pm.flags.test (eye3d::pm_fl::colinear)) {
                            if (cd.pm.flags.test (eye3d::pm_fl::no_cross_point) == true) {
                                single_movement = true;
                            } else { // We've moved to a vertex, should have captured this case
                                throw std::runtime_error ("We've moved to a vertex, should have captured this case");
                            }
                        } else {
                            // Test if it was movement-within; the simplest case
                            if constexpr (debug_move) {
                                std::cout << "No cross point and not colinear.\n  Testing if "
                                          << (hov_land + mv_inplane) << " is inside tv_landframe (" << tv_landframe << ") dirn "
                                          << -tn0_land << "...\n";
                            }
                            sm::vec<> he = {};
                            std::tie (single_movement, he) = sm::algo::ray_tri_intersection<float> (tv_landframe[0], tv_landframe[1], tv_landframe[2], hov_land + mv_inplane + (tn0_land / 2.0f), -tn0_land);
                        }

                        if (single_movement) {
                            if constexpr (debug_move) { std::cout << "End of movement is *still* in ti0, so move mv_inplane/mv_camframe\n"; }
                            // Perform simplest movement, which is just to translate by mv_inplane
                            cam_to_surface.pretranslate (mv_inplane);
                            done = true;

                        } else {
                            if constexpr (debug_move) {
                                std::cout << "End of movement is NOT in " << ti0[0] << "," << ti0[1] << "," << ti0[2] << ". Look for start neighbours\n";
                            }
                            // Test 3 neighbours across the edges to find any for which the start location is also within-boundary
                            std::array<uint32_t, 3> _ti_2n = { std::numeric_limits<uint32_t>::max() };
                            sm::vec<>_tn_2n = {};
                            for (uint32_t i = 0u; i < 3u; i++) {
                                uint32_t i1 = i;
                                uint32_t i2 = (i+1) % 3u;
                                auto [_ti, _tn] = land->find_other_triangle_containing (ti0[i1], ti0[i2], ti0);
                                if (_ti[0] != std::numeric_limits<uint32_t>::max()) {
                                    // Test to see if start location was inside a neighbour
                                    sm::vec<sm::vec<>, 3> tv_nb = land->triangle_vertices (_ti);
                                    auto [ is, h ] = sm::algo::ray_tri_intersection<float> (tv_nb[0], tv_nb[1], tv_nb[2], hov_land, -_tn);
                                    sm::vec<> mv_orthog_nb = _tn * (mv_landframe.dot (_tn) / (_tn.dot(_tn)));
                                    sm::vec<> mv_inplane_nb = mv_landframe - mv_orthog_nb;
                                    auto [ endis, endh ] = sm::algo::ray_tri_intersection<float> (tv_nb[0], tv_nb[1], tv_nb[2], hov_land + mv_inplane_nb, -_tn);
                                    if constexpr (debug_move) {
                                        std::cout << "Start of move " << (is ? "IS" : "is NOT")
                                                  << " in " << _ti[0] << "," << _ti[1] << "," << _ti[2] << std::endl;
                                        std::cout << "End of move " << (endis ? "IS" : "is NOT")
                                                  << " in " << _ti[0] << "," << _ti[1] << "," << _ti[2] << std::endl;
                                    }

                                    // Here, start is in original, end may not be in original. This
                                    // is an 'intersection detected crossing' of a triangle edge
                                    // which wasn't picked up with eye3d::compute_crossing_location
                                    if (endis) {
                                        // End is in neighbour so this is a detected crossing
                                        if constexpr (debug_move) { std::cout << "DETECTED crossing! Pass on to next loop!\n"; }
                                        detected_crossing = true;
                                        detected_edge = { ti0[i1], ti0[i2] };
                                        detected_edgevec = tv_nb[i2] - tv_nb[i1];
                                        break; // out of for
                                    } else { // end not in neighbour
                                        if (is) { // start is in original tri
                                            _ti_2n = _ti;
                                            _tn_2n = _tn;
                                            break; // out of for
                                        } // else end is not in original, and neither is start. Huh? Will get runtime error in the next stanza
                                    }
                                }
                            }

                            if (_ti_2n[0] != std::numeric_limits<uint32_t>::max()) {
                                // Now we know an alternative start triangle for the movement. Re-orient to this and re-loop
                                ti0 = _ti_2n;
                                tn0_land = _tn_2n;
                                // recompute mv_inplane for this neighbour triangle
                                mv_orthog = tn0_land * (mv_landframe.dot (tn0_land) / (tn0_land.dot(tn0_land)));
                                mv_inplane = mv_landframe - mv_orthog; // landframe
                            } else if (detected_crossing == true) {
                                // We didn't find an alternative start triangle, but we did detect an edge crossing by intersection, so continue.
                            } else {
                                throw std::runtime_error ("Should have detected crossing just now\n");
                            }

                        } // single movement if/else

                    } // compute_crossing_location if/else

                } // triangle traversing while loop

                cam_final = cam_to_surface;
                auto _tn_scaled = land_to_scene.scaling_mat33().inverse() * tn0_land;
                cam_final.pretranslate (_tn_scaled * hoverheight);
                setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (land_to_scene * cam_final));

            } // (second) intersection if/else

        } // else not activly moving or rotating

        // Get the camera space and update our eye and camera-frame models
        cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        // reset to initial camera space if requested
        if (v.vstate.test (eye3dvisual::state::campose_reset_request) == true) {
            v.stop(); // cancel any active movements
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (initial_camera_space));
            cam_to_land = scene_to_land * initial_camera_space;
            sm::vec<> camloc_landframe = (cam_to_land * sm::vec<>{}).less_one_dim();
            sm::vec<> hp_scene;
            std::tie(hp_scene, tn0_land, ti0) = eye3d::find_land (land, camloc_landframe); // sets tn0_land and ti0
            eye3d::set_landlocked_camera (hp_scene, land_to_scene, land, tn0_land, ti0, pvp1, hoverheight);
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);
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
        if (land) {
            subr_key_move_over_land();
        } else {
            subr_key_move_camera();
        }
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
