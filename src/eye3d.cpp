#include <iostream>
#include <vector>
#include <array>
#include <deque>
#include <chrono>

#include <sm/flags>
#include <sm/vvec>
#include <sm/grid>
#include <sm/hdfdata>

#include <sampleConfig.h>

#include "MulticamScene.h"
#include "libEyeRenderer.h"

#include <mplot/gl/version.h>
constexpr int glver = mplot::gl::version_4_3;

#include "eye3dvisual.h"
#include "AntVisual.h"
#include <mplotext/fpsprofiler.h>
#include <mplot/compoundray/interop.h> // mathplot <--> compoundray interoperability

#include <mplot/compoundray/EyeVisual.h>
#include <mplot/CoordArrows.h>
#include <mplot/GridVisual.h>
#include <mplot/RodVisual.h>
#include <mplot/InstancedScatterVisual.h>

#include "spline.hpp" // tkspline plus wrapper in sm::algo space

#include <oces/reader>

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

// When the program starts, how many samples per ommatidium/element do you want?
constexpr int samples_per_omm_default = 64;

namespace mplot
{
    // The data we pass to NavMesh::compute_mesh_movement, so I can save, say the last 100 movements
    // in a deque and then save/load and replay
    struct NavMeshMovementData
    {
        sm::vec<float> mv_camframe = {};
        sm::mat44<float> cam_to_scene = {};
        sm::mat44<float> model_to_scene = {};
        std::array<uint32_t, 4> ti0 = {};
        float hoverheight = 0.0f;

        bool operator== (const NavMeshMovementData& rhs) const noexcept
        {
            return (mv_camframe == rhs.mv_camframe
                    && cam_to_scene == rhs.cam_to_scene
                    && model_to_scene == rhs.model_to_scene
                    && ti0 == rhs.ti0
                    && hoverheight == rhs.hoverheight);
        }

        bool operator!= (const NavMeshMovementData& rhs) const noexcept { return !(*this == rhs); }

        // Save data into the already-open hdfdata object
        void save (sm::hdfdata& hd, const uint32_t data_index)
        {
            std::stringstream pcom;
            pcom << "/navmeshmv_" << data_index;
            std::string s = pcom.str() + std::string("/mv_camframe");
            hd.add_contained_vals (s.c_str(), mv_camframe);
            s = pcom.str() + std::string("/cam_to_scene");
            hd.add_contained_vals (s.c_str(), cam_to_scene.mat);
            s = pcom.str() + std::string("/model_to_scene");
            hd.add_contained_vals (s.c_str(), model_to_scene.mat);
            s = pcom.str() + std::string("/ti0");
            hd.add_contained_vals (s.c_str(), ti0);
            s = pcom.str() + std::string("/hoverheight");
            hd.add_val (s.c_str(), hoverheight);
        }

        // Load data
        void load (sm::hdfdata& hd, const uint32_t data_index)
        {
            std::stringstream pcom;
            pcom << "/navmeshmv_" << data_index;
            std::string s = pcom.str() + std::string("/mv_camframe");
            hd.read_contained_vals (s.c_str(), mv_camframe);
            s = pcom.str() + std::string("/cam_to_scene");
            hd.read_contained_vals (s.c_str(), cam_to_scene.mat);
            s = pcom.str() + std::string("/model_to_scene");
            hd.read_contained_vals (s.c_str(), model_to_scene.mat);
            s = pcom.str() + std::string("/ti0");
            hd.read_contained_vals (s.c_str(), ti0);
            s = pcom.str() + std::string("/hoverheight");
            hd.read_val (s.c_str(), hoverheight);
        }
    };
} // mplot

namespace eye3d
{
    // Your application-specific help message
    void printHelp()
    {
        std::cout << "USAGE:\neye3d -f <path to gltf scene>\n\n"
                  << "\t-h\tDisplay this help information.\n"
                  << "\t-f\tPath to a gltf scene file (absolute or relative to current "
                  << "working directory, e.g. './data/axis_coloured_blocks.gltf').\n";
    }
    // Helper to plot coords
    mplot::CoordArrows<glver>* plot_axes (mplot::Visual<glver>* thevisual, const float len = 1.0f)
    {
        auto cavm = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<>{});
        thevisual->bindmodel (cavm);
        cavm->em = 0.0f; // labels don't work so well
        cavm->lengths = { len, len, len };
        cavm->finalize();
        return thevisual->addVisualModel (cavm);
    }

    void add_tube_vm (mplot::Visual<glver>* v,
                      const sm::vec<float>& v0, const sm::vec<float>& v1,
                      const std::array<float, 3>& clr)
    {
        float r = (v1 - v0).length() / 20.0f;
        auto tvm = std::make_unique<mplot::RodVisual<glver>> (sm::vec<>{}, v0, v1, r, clr);
        v->bindmodel (tvm);
        tvm->finalize();
        v->addVisualModel (tvm);
    }


    // Flags class
    enum class options : uint8_t
    {
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
        max_fps,          // If true, poll, instead of fps
        playback,         // Play back saved sequence of poses from a crash data file (.h5 format)
        path_from_csv,    // Move the ant from a sequence of 2D coordinates that give it a path
        can_exit
    };
    // Parse cmd line to find the path and set options
    std::tuple<std::string, std::string, std::string> parse_inputs (int argc, char* argv[], sm::flags<eye3d::options>& opts)
    {
        std::string path = "";
        std::string csvpath = "";
        std::string hovh = "";
        for (int i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-h") {
                eye3d::printHelp();
                opts |= eye3d::options::can_exit;
            } else if (arg == "-f") {
                i++;
                path = std::string(argv[i]);
            } else if (arg == "-H") {
                i++;
                hovh = std::string(argv[i]);
            } else if (arg == "-b") {
                opts |= eye3d::options::blender_axes;
            } else if (arg == "-x") {
                opts |= eye3d::options::max_fps;
            } else if (arg == "-p") {
                opts |= eye3d::options::playback;
            } else if (arg == "-c") {
                opts |= eye3d::options::path_from_csv;
                i++;
                csvpath = std::string(argv[i]);
            }
        }
        if (path.empty()) {
            eye3d::printHelp();
            opts |= eye3d::options::can_exit;
        }
        return {path, hovh, csvpath};
    }

    // Make a randomized path to follow
    template <typename T>
    struct random_outbound
    {
        random_outbound (const uint32_t _n_steps, const uint32_t _a_tau)
        {
            this->n_steps = _n_steps;
            this->a_tau = _a_tau;
            this->init();
        }

        random_outbound (const uint32_t _n_steps, const uint32_t _a_tau, const T& _kappa)
        {
            this->n_steps = _n_steps;
            this->a_tau = _a_tau;
            this->kappa = _kappa;
            this->init();
        }

        void init()
        {
            this->rVM = std::make_unique<sm::rand_vonmises<T>> (T{0}, kappa);
            this->a.resize (this->n_steps / this->a_tau);
            this->a.randomize();
            this->a *= (amm.span());
            this->a += amm.min;
            // pass this to cubic_spline to make a
            sm::algo::cubic_spline (this->a, this->a_tau);
        }

        // Reset state
        void reset()
        {
            this->t = 0;
            this->theta = T{0};
            this->omega = T{0};
            this->velocity = {};
            this->speed = T{0};
        }

        // Advance the route generation by one timestep
        void step()
        {
            // This is the model as stated in the paper and it should be equivalent to lfilter
            // function.
            T epsilon = this->rVM->get(); // Angular acceleration
            this->omega = this->lambda * this->omega + epsilon;
            this->theta += this->omega;

            T accel = T{0};
            if (t < this->a.size()) { accel = this->a[this->t]; }

            sm::vec<T, 2> thrust = { accel * std::sin (theta), accel * std::cos (theta) };
            this->velocity = (this->velocity + thrust) * one_minus_FD;
            this->speed = (this->speed + accel) * one_minus_FD;

            ++this->t;
            if (this->t > this->n_steps) { this->reset(); }
        }

        // Number of steps total.
        uint32_t n_steps = 0;
        // Current time step
        uint32_t t = 0;

        // State
        T theta = T{0};              // Heading/theta
        T omega = T{0};              // Angular velocity
        sm::vec<T, 2> velocity = {}; // Cartesian velocity (or can use speed):
        T speed = T{0};              // Linear speed

        // Parameters
        const T lambda = T{0.4};
        T kappa = T{100};                   // Von Mises concentration parameter
        sm::vvec<T> a = {};                 // Acceleration values
        // Uniform RNG range outbound [0, 0.05]
        const sm::range<T> amm = { T{0}, T{0.005} };
        // how often does the acceleration change?
        uint32_t a_tau = 50;

        // FD is the drag coefficient
        static constexpr T FD = T{0.15};
        static constexpr T one_minus_FD = (T{1} - FD);

        // Random number generation
        std::unique_ptr<sm::rand_vonmises<T>> rVM;
    };

    // Read a simple csv with 2D coordinates.
    bool read_csv (const std::string& path, sm::vvec<sm::vec<float, 2>>& positions)
    {
        std::ifstream f (path.c_str(), std::ios::in);
        if (f.is_open() == false) { return false; }
        std::string line;
        while (std::getline (f, line)) {
            sm::vec<float, 2> twodpos;
            twodpos.set_from_str (line, ",");
            positions.push_back (twodpos);
        }
        return true;
    }

} // namespace eye3d

int main (int argc, char* argv[])
{
    using mc = sm::mathconst<float>;

    double waittime = 0.0167; // for debug, so I can make playback slow in a simple way

    // Program options and boolean state
    sm::flags<eye3d::options> opts;
    auto[path, hovh, csv_path] = eye3d::parse_inputs (argc, argv, opts);
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
    eye3dvisual v (2000, 2000, "Scene (mathplot graphics)", opts.test(eye3d::options::blender_axes));
    // Choose how fast the camera should move for key press and mouse events
    v.speed = 0.5f; // 0.5 m/s max speed for our Cataglyphis Velox
    v.angularSpeed = 2.0f * mc::two_pi / 360.0f;
    v.lightingEffects (true);
    // Use a non-default zFar as we use large environments
    v.zFar = 2400;
    // Rotate about the nearest VisualModel
    v.rotateAboutNearest (true);
    // Rotate about a scene vertical axis? true for landscapes, false for cubes/objects (Ctrl-k changes I think, at runtime)
    v.rotateAboutVertical (true);
    if (opts.test(eye3d::options::blender_axes)) {
        v.switch_scene_vertical_axis(); // to uz up
    }
    v.vstate.flip (eye3dvisual::state::show_camframe);

    // We start rotated into a drone view initial orientation for taking pictures of the world
    sm::quaternion<float> def_q (sm::vec<float>::ux(), mc::pi_over_2); // non-blender only
    v.setSceneRotation (def_q);

    // A window for the eye view
    mplot::Visual<glver> veye (512, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{0}, float{0}, float{-4.1} });
    veye.setSceneRotation (sm::quaternion<float>{ float{0.658107}, float{0.752674}, float{0.0157197}, float{0.0114156} });

    // Use a FPS profiling with a text object on screen
    mplotext::fps::profiler fps_profiler;
    mplot::VisualTextModel<glver>* fps_label;
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

    // Get the visual models from the scene
    mplot::compoundray::scene_to_visualmodels<glver> (scene, &v, false); // true for 'make_navmeshes'

    // Use oces_reader to read in our eye data, esp. for the head
    std::string oces_path = efpath;
    mplot::tools::stripFileSuffix (oces_path);
    oces_path += ".gltf";
    // Now try to open oces_path
    std::cout << "Attempt to load OCES file " << oces_path << "\n";
    oces::reader oces_reader (oces_path);
    if (oces_reader.read_success == false) {
        std::cout << "No associated OCES file for a head\n";
    } else {
        // Read the head and make a VisualModel
        oces_reader.head_mesh.single_colour = {0.345f, 0.122f, 0.082f};
    }

    // Create an EyeVisual 'eye' in our mathplot scene, v.
    mplot::compoundray::EyeVisual<glver>* ep1 = nullptr;
    auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &ommatidiaData,
                                                                         reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia),
                                                                         oces_reader.read_success ? reinterpret_cast<mplot::meshgroup*>(&oces_reader.head_mesh) : nullptr);
    v.bindmodel (eyevm);
    eyevm->setViewMatrix (initial_camera_space);
    eyevm->name = "EyeVisual";
    eyevm->finalize();
    ep1 = v.addVisualModel (eyevm);

    // We follow the eyevisual as it moves
    v.options.set (mplot::visual_options::viewFollowsVMTranslations);

    //v.options.set (mplot::visual_options::viewFollowsVMRotations);
    v.setFollowedVM (ep1);

    // A second eye goes in the 'eye only' window
    auto ptype = mplot::compoundray::EyeVisual<glver>::projection_type::mercator;
    mplot::compoundray::EyeVisual<glver>* ep2 = nullptr;
    auto eyevm2 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &ommatidiaData,
                                                                          reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia),
                                                                          oces_reader.read_success ? reinterpret_cast<mplot::meshgroup*>(&oces_reader.head_mesh) : nullptr);
    veye.bindmodel (eyevm2);
    eyevm2->name = "Big Eye";
    // First eye of eye pair (one spherical projection)
    uint32_t sz = 1024;
    float ps_rad = 0.0001f;                  // projection sphere radius
    sm::vec<> centre = { -0.00002f, 0, 0 };  // projection sphere centre

    if (oces_reader.read_success == true) {
        sz = oces_reader.position.size();
        ps_rad = 0.0002f;
        centre = { -0.00056, 0, -0.00005 };
    }

    sm::mat44<float> twod_tr;                // twod projection transformation
    float twod_scale = 14.0f;                // twod projection scaling
    sm::vec<> twod_offset = { 0.0001f, 0.0f, 0.0f }; // twod projection translation to move to centre
    sm::vec<> twod_offset2 = { -0.0004f, 0.0007f, 0.0f }; // post scale/rotate translation
    float rotn = -sm::mathconst<float>::pi_over_8;
    if (oces_reader.read_success == true) {
        ptype = mplot::compoundray::EyeVisual<glver>::projection_type::equirectangular;
        twod_tr.scale (sm::vec<>{4, 1, 1});
    } else {
        twod_tr.translate (twod_offset2);
        twod_tr.scale (twod_scale);
        twod_tr.rotate (sm::vec<>::uy(), rotn);
        twod_tr.translate (twod_offset);
    }
    eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, 0, sz/2);

    // Second of eye pair (another spherical projection)
    if (oces_reader.read_success == true) {
        if (oces_reader.mirrors.empty() == false) {
            centre = (oces_reader.mirrors[0] * centre).less_one_dim();
            eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, sz/2, sz);
        }
    } else {
        centre[0] = -centre[0];
        twod_tr.setToIdentity();
        twod_offset[0] = -twod_offset[0];
        twod_offset2[0] = -twod_offset2[0];
        twod_tr.translate (twod_offset2);
        twod_tr.scale (twod_scale);
        twod_tr.rotate (sm::vec<>::uy(), -rotn);
        twod_tr.translate (twod_offset);
        eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, sz/2, sz);
    }

    // Visualization options
    eyevm2->show_3d = true;
    eyevm2->show_sphere = false;
    eyevm2->show_rays = false;
    eyevm2->finalize();
    ep2 = veye.addVisualModel (eyevm2);
    // Scale this model up, so it's not tiny like the one in the scene
    ep2->scaleViewMatrix (1000);

    // The ant body
    auto av = std::make_unique<biosim::AntVisual<glver>>();
    v.bindmodel (av);
    av->finalize();
    auto ant_ptr = v.addVisualModel (av);
    ant_ptr->name = "ant";
    ant_ptr->setViewMatrix (initial_camera_space);

    // Breadcrumb trail
    uint64_t move_counter = 0u;
    uint64_t max_bc = 8000;
    sm::vvec<sm::vec<float, 3>> breadcrumb_coords = {};
    sm::vvec<float> breadcrumb_data = {};

    auto isv = std::make_unique<mplot::InstancedScatterVisual<glver>> (sm::vec<>{});
    v.bindmodel (isv);
    isv->max_instances = max_bc;
    isv->radiusFixed = 0.004f;
    isv->finalize();
    mplot::InstancedScatterVisual<glver>* isvp = v.addVisualModel (isv);

    // Make CoordArrows axes to show our camera's localspace (and to help find our tiny ant)
    auto antca = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<>{});
    v.bindmodel (antca);
    antca->em = 0.0f; // labels don't work so well
    float len = 2.0f;
    antca->lengths = { len, len, len };
    antca->thickness = 1.0f;
    antca->endsphere_size = 1.2f;
    antca->finalize();
    auto antca_ptr = v.addVisualModel (antca);
    antca_ptr->name = "ant";
    antca_ptr->setViewMatrix (initial_camera_space);

    // Get access to the landscape VisualModel by searching for a selection of model names
    mplot::VisualModel<glver>* land = nullptr;
    {
        mplot::VisualModel<glver>* vmp = nullptr;
        v.init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
        while ((vmp = v.get_next_vm_accessor()) != nullptr) {
            // The 'land' is a cube for now
            if (vmp->name == "Cube.002" && land == nullptr) { land = vmp; land->make_navmesh(); }
            else if (vmp->name == "Cube.001" && land == nullptr) { land = vmp; land->make_navmesh(); }
            else if (vmp->name == "Landscape.003" /* && land == nullptr */) { land = vmp; land->make_navmesh(); } // land trumps other objects
            else if (vmp->name == "Rock.Landscape.Style_2.Mesh.003" && land == nullptr) { land = vmp; land->make_navmesh(); }
            else if (vmp->name == "ground_inner_high_res" && land == nullptr) { land = vmp; land->make_navmesh(); }
            else { std::cout << "Model name " << vmp->name << std::endl; }
        }
    }

    // Load data from h5 or csv file for pre-defined paths
    constexpr uint32_t qlen = 20;
    std::deque<mplot::NavMeshMovementData> mdq;
    uint32_t di = 0; // data index (for playback)
    // A file to be read from csv with 2D coordinates
    sm::vvec<sm::vec<float, 2>> csv_positions;

    if (opts.test (eye3d::options::playback)) {
        // populate mdq from file
        try {
            // Make this a cmd line arg, and open either .h5 or .csv
            sm::hdfdata hd ("./navmesh_data.h5", std::ios::in);
            for (uint32_t i = 0; i < qlen; ++i) {
                mplot::NavMeshMovementData nmd;
                nmd.load (hd, i);
                mdq.push_back (nmd);
            }
            std::cout << "Loaded navigation data from navmesh_data.h5 (hardcoded)\n";
        } catch (const std::exception& e) {
            /* No file to open */
            std::cout << "Playback mode, but there's no file to open\n";
        }
    } else if (opts.test (eye3d::options::path_from_csv)) {
        //waittime = 0.25; // make it slow
        if (eye3d::read_csv (csv_path, csv_positions) == false) {
            throw std::runtime_error ("Failed to read CSV file");
        } else {
            std::cout << "Read " << csv_positions.size() << " ant positions from CSV\n";
        }
    }

    sm::mat44<float> land_to_scene;  // land's viewmatrix. converts land model to scene

    float hoverheight = 0.01f;
    if (!hovh.empty()) { hoverheight = std::atof (hovh.c_str()); }

    if (land) {
        std::cout << "Landscape name: " << land->name << " was found [" << (land->vpos_size() / 3) << " vertices]\n";

        land_to_scene = land->getViewMatrix();

        sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);

        if (opts.test (eye3d::options::path_from_csv) && !csv_positions.empty()) {
            // Initial position from first entry in the csv
            std::cout << "Set initial position from csv\n";
            sm::vec<float> nextloc = { csv_positions[0][0], 0.0f, csv_positions[0][1] };
            std::cout << "Initial position is " << nextloc << std::endl;
            // Change camspace based on nextloc. nextloc in landscape coords, so cam_nextloc = landscape.location + nextloc;
            sm::vec<float> ltstr = land_to_scene.translation();
            sm::vec<float> cam_nextloc = nextloc;
            cam_nextloc[0] += ltstr[0];
            cam_nextloc[2] += ltstr[2]; // update only x and z
            std::cout << "cam_nextloc = land locn (" << ltstr << ") + nextloc [xz ONLY] (" << nextloc << ") = " << cam_nextloc << std::endl;
            std::cout << "cf from-gltf camera location: " << camspace.translation() << std::endl;

            sm::mat44<float> cnl;
            cnl.translate (cam_nextloc);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cnl));

            ++move_counter;
        }

        auto[hp_scene, _tn0, _ti0] = land->navmesh->find_triangle_hit (camspace, land_to_scene, 100.0f);
        if (_ti0[0] != std::numeric_limits<uint32_t>::max()) {
            // Set up our camera using the data obtained from find_triangle_hit()
            sm::mat44<float> cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight);
            if (cam_to_scene != sm::mat44<float>::identity()) {
                std::cout << "Set camera pose matrix from\n" << cam_to_scene << std::endl;
                setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            } else {
                std::cout << "cam_to_scene is identity??\n";
            }
        } else {
            std::cout << "Failed to find the landscape; Camera position unchanged from glTF\n";
        }

        sm::mat44<float> _cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        std::cout << "Got camera pose matrix from scene:\n" << _cam_to_scene << std::endl;
        sm::vec<float> _lastloc = _cam_to_scene.translation();
        std::cout << "lastloc = " << _lastloc << " [this is cam_to_scene.translation()]" << std::endl;
    }
    std::cout << "*****\n";

    // Random route generation
    eye3d::random_outbound<float> rrg(1500, 150, 100);

    // We keep a track of the eye size. Used in subr_detect_camera_changes
    size_t last_eye_size = 0u;

    uint32_t render_counter = 0u;
    auto subr_detect_camera_changes = [&v, &ommatidia, &ommatidiaData, &ommatidiaPositions,
                                       &last_eye_size, &ep1, &ep2, &render_counter, opts] ()
    {
        size_t curr_eye_size = last_eye_size;
        // Detect changes in the camera and update eye model as necessary
        if (ommatidiaData.size() == 0) {
            if (isCompoundEyeActive()) { getCameraData (ommatidiaData); }
        } // else no need to re-get data

        // Change showing the 'cones' of the compound eye visual model?
        if (ep1->show_cones != v.vstate.test(eye3dvisual::state::show_cones)) {
            ep1->show_cones = v.vstate.test(eye3dvisual::state::show_cones);
            ep1->reinit();
            ep2->show_cones = v.vstate.test(eye3dvisual::state::show_cones);
            ep2->reinit();
        }
        // Change the length of the cones?
        if (ep1->get_cone_length() != v.manual_cone_length) {
            std::cout << "Cone length " << ep1->get_cone_length() << " != requested: " << v.manual_cone_length << std::endl;
            ep1->set_cone_length (v.manual_cone_length);
            ep2->set_cone_length (v.manual_cone_length);
        }
        // Update eyevm model (or just update colours)
        ep1->ommatidia = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);
        ep2->ommatidia = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);

        static constexpr uint32_t render_every = 1u; // set to 1 for max update, 60 to reduce compute
        if (ommatidia != nullptr) {
            curr_eye_size = ommatidia->size();
            if (curr_eye_size != last_eye_size) {
                if (render_counter % render_every == 0u) { ep1->reinit(); }
                ep2->reinit();
                last_eye_size = curr_eye_size;
            } else {
                if (render_counter % render_every == 0u) { ep1->reinitColours(); }
                ep2->reinitColours(); // 4x faster to just reinitColours
            }
            ++render_counter;
        }
    };

    auto subr_key_move_over_land = [&v, &ep1, &ant_ptr, &antca_ptr, &initial_camera_space, &rrg,
                                    &opts, &move_counter, max_bc, &breadcrumb_coords, &breadcrumb_data,
                                    &isvp, &mdq, csv_positions, &di, land, land_to_scene, &hoverheight](const float fps)
    {
        antca_ptr->setHide (!v.vstate.test(eye3dvisual::state::show_camframe));

        sm::mat44<float> cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        mplot::ColourMap cm (mplot::ColourMapType::Plasma);
        //sm::vvec<std::array<float, 3>> bc_clr = { cm.convert(0.0f), cm.convert(0.3f), cm.convert(0.6f), cm.convert(0.9f) };
        sm::vvec<std::array<float, 3>> bc_clr = { cm.convert(0.9f), cm.convert(0.9f), cm.convert(0.9f), cm.convert(0.9f) };
        sm::vvec<float> bc_alpha = { 1, 1, 1, 1 };
        sm::vvec<float> bc_scale = { 4, 6, 8, 10 };

        // A random walk mode
        if (v.vstate.test (eye3dvisual::state::walk)) {
            // set rotation and step length according to the Stone paper
            rrg.step();
            // rrg.omega is the angular speed rrg.speed is the linear speed
            //std::cout << "rotating in this step by " << rrg.omega << " and moving forward by " << rrg.speed << std::endl;
            rotateCamerasLocallyAround (rrg.omega, 0.0f, 1.0f, 0.0f);
            cam_to_scene = mplot::compoundray::getCameraSpace (scene);
            sm::vec<float> mv_camframe = { 0, 0, rrg.speed };
            // saves
            sm::mat44<float> cam_to_scene_sv = cam_to_scene;
            //std::array<uint32_t, 4> ti0_sv = ti0;
            try {
                try {
                    // Note that even if the last mesh movement would land on a triangle, a further
                    // rotation might mean that we get a 'no triangle intersection' exception (esp. if
                    // we are on the edge of a landscape)
                    mdq.push_back (mplot::NavMeshMovementData { mv_camframe, cam_to_scene, land_to_scene, land->navmesh->ti0, hoverheight });
                    cam_to_scene = land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, land_to_scene, /*ti0,*/ hoverheight);
                    ++move_counter;
                    if (mdq.size() > qlen) { mdq.pop_front(); }

                    if (breadcrumb_coords.size() < max_bc) {
                        breadcrumb_coords.push_back (cam_to_scene_sv.translation());
                        breadcrumb_data.push_back (0.0f); // dummy for now
                    } else {
                        breadcrumb_coords[move_counter % max_bc] = cam_to_scene_sv.translation();
                        // breadcrumb_data.push_back (0.0f); // dummy for now
                    }
                    isvp->set_instance_data (breadcrumb_coords);

                } catch (mplot::NavException& e) {
                    if (e.m_type == mplot::NavException::type::off_edge) {
                        // After movement we'd be near the edge, so cancel movement
                        cam_to_scene = cam_to_scene_sv;
                        //ti0 = ti0_sv;
                        mdq.pop_back();
                    } else {
                        throw e;
                    }
                }
            } catch (mplot::NavException& e) {

                std::cout << "Exception navigating mesh at movement count " << move_counter << ": " << e.what() << std::endl;

                // save data
                {
                    sm::hdfdata hd("./navmesh_data.h5", std::ios::out | std::ios::trunc);
                    for (uint32_t i = 0; i < mdq.size(); ++i) { mdq[i].save (hd, i); }
                }

                opts.set (eye3d::options::max_fps, false); // don't burn electricity after exception
                // Draw triangle tubes
                bool first = true;
                for (auto t : e.tris) {
                    if (first) {
                        std::cout << t[0] << "-" << t[1] << "-" << t[2] << " has t[3] = " << t[3] << std::endl;
                    }
                    auto tv = land->navmesh->triangle_vertices (t, land_to_scene);
                    eye3d::add_tube_vm (&v, tv[0], tv[1], first ? mplot::colour::black : mplot::colour::maroon2);
                    eye3d::add_tube_vm (&v, tv[1], tv[2], first ? mplot::colour::black : mplot::colour::maroon2);
                    eye3d::add_tube_vm (&v, tv[2], tv[0], first ? mplot::colour::black : mplot::colour::maroon2);

                    // JSON line to view with triangle_intersect
                    sm::vec<float> tn0 = land->navmesh->triangle_normal (tv);
                    std::cout << std::endl;
                    std::cout << "{ \"t0\" : " << tv[0].str_mat() << ", "
                              << "\"t1\" : " << tv[1].str_mat() << ", "
                              << "\"t2\" : " << tv[2].str_mat() << ", "
                              << "\"l0\" : " << cam_to_scene.translation().str_mat() << ", "
                              << "\"l\" : " << (-tn0).str_mat() << " }" << std::endl;

                    first = false;
                }

                v.vstate.set (eye3dvisual::state::walk, false);
            }
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));

        } else if (opts.test (eye3d::options::playback)) { // play back deque of movements
            try {
                // Note that even if the last mesh movement would land on a triangle, a further
                // rotation might mean that we get a 'no triangle intersection' exception (esp. if
                // we are on the edge of a landscape)
                land->navmesh->ti0 = mdq[di].ti0;
                //std::array<uint32_t, 4> ti0_sv = ti0;
                std::cout << "Playback of saved movement index = " << di << std::endl;
                sm::mat44<float> cam_to_scene_sv = cam_to_scene;
                cam_to_scene = land->navmesh->compute_mesh_movement (mdq[di].mv_camframe, mdq[di].cam_to_scene, mdq[di].model_to_scene, /*ti0,*/ mdq[di].hoverheight);
                di++;
                if (land->navmesh->ti0[3] == 1) {
                    // After movement we'd be on the edge, so cancel movement
                    std::cout << "(Playback) Would be on edge, cancel movement\n";
                    cam_to_scene = cam_to_scene_sv;
                    //ti0 = ti0_sv;
                }
            } catch (mplot::NavException& e) {
                std::cout << "Exception navigating mesh at movement count " << move_counter << ": " << e.what() << std::endl;
                opts.set (eye3d::options::max_fps, false); // don't burn electricity after exception
                opts.set (eye3d::options::playback, false);
            }
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));

        } else if (opts.test (eye3d::options::path_from_csv)) { // Construct path from csv file of 2D ant locations

            if (csv_positions.size() > move_counter) {

                /*
                 * With a csv path, teleport between each location (and then estimate the heading of
                 * the ant). CSV positions are relative to the landscape model.
                 */
                sm::vec<float> lastcamloc = cam_to_scene.translation();

                sm::vec<float> nextloc = { csv_positions[move_counter][0], 0, csv_positions[move_counter][1] };
                sm::vec<float> lastloc = { csv_positions[move_counter - 1][0], 0, csv_positions[move_counter - 1][1] };

                sm::vec<float> ltstr = land_to_scene.translation(); // always the same
                sm::vec<float> cam_nextloc = nextloc;
                cam_nextloc[0] += ltstr[0];
                cam_nextloc[2] += ltstr[2]; // update only x and z

                sm::mat44<float> cnl;
                cnl.translate (cam_nextloc);
                setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cnl));
                cam_to_scene = mplot::compoundray::getCameraSpace (scene);

                // Find triangle hits using the scene's 'up' direction.
                sm::vec<float> camloc_mf = (land_to_scene.inverse() * cam_to_scene).translation();
                sm::vec<float> vnrm = v.scene_up;
                vnrm *= 4.0f;
                auto[hp_scene, _tn0, _ti0] = land->navmesh->find_triangle_hit (land_to_scene, camloc_mf + (vnrm / 2.0f), -2.0f * vnrm);

                if (_ti0[0] != std::numeric_limits<uint32_t>::max()) {
                    sm::vec<float> fwds = nextloc - lastloc;
                    // Set up our camera using the data obtained from find_triangle_hit()
                    cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight, fwds);
                    if (cam_to_scene != sm::mat44<float>::identity()) {
                        setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
                    } // else what to do if cam_to_scene is identity?
                } else {
                    throw std::runtime_error ("Failed to find the landscape so can't teleport to that location!?!");
                }

                move_counter++;
                if (breadcrumb_coords.size() < max_bc) {
                    breadcrumb_coords.push_back (lastcamloc);
                } else {
                    breadcrumb_coords[move_counter % max_bc] = lastcamloc;
                }
                isvp->set_instance_data (breadcrumb_coords, bc_clr, bc_alpha, bc_scale);

            } // else no more movements

        } else {

            if (v.isActivelyRotating()) {
                // Up-down (pitch) is rotation about local camera frame axis x
                rotateCamerasLocallyAround (v.getVerticalRotationAngle(), 1.0f, 0.0f, 0.0f);
                // Left-and-right (yaw) is rotation about local camera frame axis y
                rotateCamerasLocallyAround (v.getHorizontalRotationAngle(), 0.0f, 1.0f, 0.0f);
                // Roll
                rotateCamerasLocallyAround (v.getRollRotationAngle(), 0.0f, 0.0f, 1.0f);

                cam_to_scene = mplot::compoundray::getCameraSpace (scene); // update

            }

            if (v.isActivelyTranslating()) { // translating

                if (v.move_state.test (eye3dvisual::move_sense::up)) {
                    hoverheight += 0.001f;
                } else if (v.move_state.test (eye3dvisual::move_sense::down)) {
                    hoverheight -= 0.001f;
                    if (hoverheight < 0.0f) { hoverheight = 0.0f; }
                }

                // Obtain the commanded movement vector and turn this into a translation matrix
                sm::vec<float> mv_camframe = v.getMovementVector (fps);
                sm::vec<float> lastloc = cam_to_scene.translation();
                cam_to_scene = land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, land_to_scene, hoverheight);
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
        }

        // reset to initial camera space if requested
        if (v.vstate.test (eye3dvisual::state::campose_reset_request) == true) {
            v.stop(); // cancel any active movements
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (initial_camera_space));
            sm::mat44<float> camspace = mplot::compoundray::getCameraSpace (scene);
            auto[hp_scene, _tn0, _ti0] = land->navmesh->find_triangle_hit (camspace, land_to_scene);
            cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            v.vstate.reset (eye3dvisual::state::campose_reset_request);
        }

        // Update the view matrix of eye and eye localspace axes
        ep1->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    /**
     * The main program loop
     */
    v.render();
    std::string m_count_str = {};
    while (!v.readyToFinish()) {

        // Tell the fps_profiler that we're at the start of a loop
        fps_profiler.at_begin (getCurrentEyeSamplesPerOmmatidium());
        // The current camera may have changed, this subroutine deals with any changes
        subr_detect_camera_changes();
        // Now render the mathplot window
        v.render();
        // Change label after render (it needs v's context, not veye's)
        if (move_counter % 1000 == 0) {
            m_count_str = std::to_string (move_counter);
            fps_label->setupText (fps_profiler.fps_txt + std::string(" ") + m_count_str);
        }
        // Save some electricity while developing - limit to 60 FPS. For max speed use v.poll() (-x)
        if (opts.test (eye3d::options::max_fps)) { v.poll(); } else { v.wait (waittime); }
        // Render the eye-only window
        veye.render();
        // Deal with any movements commanded by key press events (including reset)

        v.setContext(); // right now key move over land needs v's context
        subr_key_move_over_land (fps_profiler.fps_mean);

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
