#include <iostream>
#include <cstdint>
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
constexpr int32_t glver = mplot::gl::version_4_3;

#include "antpovvisual.h"
#include "AntBodyVisual.h"
#include <mplot/fps/profiler.h>
#include <mplot/compoundray/interop.h> // mathplot <--> compoundray interoperability

#include <mplot/compoundray/EyeVisual.h>
#include <mplot/CoordArrows.h>
#include <mplot/GridVisual.h>
#include <mplot/RodVisual.h>
#include <mplot/VectorVisual.h>
#include <mplot/InstancedScatterVisual.h>
#include <mplot/NormalsVisual.h>

#include "spline.hpp" // tkspline plus wrapper in sm::algo space

#include <oces/reader>

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

// When the program starts, how many samples per ommatidium/element do you want?
constexpr int32_t samples_per_omm_default = 64;

namespace antpov
{
    // Your application-specific help message
    void printHelp()
    {
        std::cout << "USAGE:\nantpov -f <path to gltf scene>\n\n"
                  << "\t-h\tDisplay this help information.\n"
                  << "\t-f\tPath to a gltf scene file (absolute or relative to current "
                  << "working directory, e.g. './data/axis_coloured_blocks.gltf').\n";
    }

    template<typename T>
    sm::vec<T, 3> get_cam_movement (sm::mat<T, 4>& current, const sm::mat<T, 4>& target, sm::vec<T, 3>& vel, T time_90, T dt)
    {
        const T c0 = dt * T{3.75} / time_90;
        if (c0 >= T{1}) { // here, constant is too small, spring too stiff.
            throw std::runtime_error ("spring too stiff");
        }
        const sm::vec<T, 3> delta = target.translation() - current.translation();
        const sm::vec<T, 3> force = delta - (vel * T{2});
        sm::vec<T, 3> pos_shift = vel * c0;
        vel += force * c0;
        return pos_shift;
    }

    // Flags class
    enum class options : uint8_t
    {
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
        max_fps,          // If true, poll, instead of fps
        path_from_csv,    // Move the ant from a sequence of 2D coordinates that give it a path
        save_hdf5,        // If true, then save output data (path_from_csv mode only at present)
        hidehead,         // If true, hide the 3D head/eye view in the Eye-only window
        debug_mv,         // Open debug h5 file and run compute_mesh_movement once for debug
        can_exit
    };
    // Parse cmd line to find the path and set options
    std::tuple<std::string, std::string, std::string> parse_inputs (int argc, char* argv[], sm::flags<antpov::options>& opts)
    {
        std::string path = "";
        std::string csvpath = "";
        std::string hovh = "";
        for (int i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-h") {
                antpov::printHelp();
                opts |= antpov::options::can_exit;
            } else if (arg == "-f") {
                i++;
                path = std::string(argv[i]);
            } else if (arg == "-H") {
                i++;
                hovh = std::string(argv[i]);
            } else if (arg == "-b") {
                opts |= antpov::options::blender_axes;
            } else if (arg == "-x") {
                opts |= antpov::options::max_fps;
            } else if (arg == "-c") {
                opts |= antpov::options::path_from_csv;
                i++;
                csvpath = std::string(argv[i]);
            } else if (arg == "-d") {
                opts |= antpov::options::save_hdf5;
            } else if (arg == "-g") {
                opts |= antpov::options::debug_mv;
            } else if (arg == "-i") {
                opts |= antpov::options::hidehead;
            }
        }
        if (path.empty()) {
            antpov::printHelp();
            opts |= antpov::options::can_exit;
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

        void about_turn() { this->theta += sm::mathconst<float>::pi; }

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

    // Read a simple csv with 2D coordinates. Should also read flags.
    bool read_csv (const std::string& path, sm::vvec<sm::vec<float, 2>>& positions, sm::vvec<uint32_t>& antflags)
    {
        std::ifstream f (path.c_str(), std::ios::in);
        if (f.is_open() == false) { return false; }
        std::string line;
        std::vector<std::string> tokens;
        while (std::getline (f, line)) {
            sm::vec<float, 2> twodpos;
            // Tokenize line into the coordinates and the flags
            twodpos.set_from_str (line, ",");
            positions.push_back (twodpos);
            // Get flags from third entry
            tokens.clear();
            tokens = mplot::tools::stringToVector (line, ",");
            if (tokens.size() > 2) {
                uint32_t fl = std::stoi (tokens[2]);
                antflags.push_back (fl);
            } else {
                antflags.push_back (0u);
            }
        }
        return true;
    }

    // For a given samples per omm, return a sensible number of loops over which to average fps, so
    // that fps takes around 1 sec to stabilize.
    static constexpr uint32_t best_n_samples (int32_t samples_per_omm)
    {
        uint32_t best_n = 0;
        switch (samples_per_omm) {
        case 1:
        case 2:
        {
            best_n = 1024; // about a seconds worth
            break;
        }
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        {
            best_n = 512;
            break;
        }
        case 128:
        case 256:
        {
            best_n = 256;
            break;
        }
        case 512:
        {
            best_n = 128;
            break;
        }
        case 1024:
        case 2048:
        {
            best_n = 64;
            break;
        }
        default:
        {
            best_n = 32;
        }
        }
        return best_n;
    }

    // The flags recorded by the experimenters
    enum class antflags : uint8_t
    {
        bush,
        cookie,
        shadow,
        visibility
    };

} // namespace antpov

int32_t main (int32_t argc, char* argv[])
{
    using mc = sm::mathconst<float>;

    double waittime = 0.0167; // for debug, so I can make playback slow in a simple way

    // Program options and boolean state
    sm::flags<antpov::options> opts;
    auto[path, hovh, csv_path] = antpov::parse_inputs (argc, argv, opts);
    std::string h5_path = csv_path;
    mplot::tools::stripFileSuffix (h5_path);
    if (h5_path.empty()) { h5_path = "trail"; }
    h5_path += ".h5";
    if (opts.test (antpov::options::can_exit)) { return 1; }

    // Boilerplate memory alloc for compound-ray
    multicamAlloc();

    std::vector<std::array<float, 3>> ommatidiaData;
    std::vector<Ommatidium>* ommatidia = nullptr;

    // Turn off verbose logging
    setVerbosity (false);
    // Load the file
    std::cout << "Loading glTF file \"" << path << "\"..." << std::endl;
    std::string basepath = path;
    mplot::tools::stripUnixFile (basepath);
    std::cout << "glTF dir: " << basepath << std::endl;
    loadGlTFscene (path.c_str(), (opts.test(antpov::options::blender_axes)
                                  ? mplot::compoundray::blender_transform() : sutil::Matrix4x4::identity()));

    // Create a mathplot window to render the eye/sensor
    antpovvisual v (2000, 2000, "Scene (mathplot graphics)", opts.test(antpov::options::blender_axes));
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
    if (opts.test(antpov::options::blender_axes)) {
        v.switch_scene_vertical_axis(); // to uz up
    }
    v.vstate.flip (antpovvisual::state::show_camframe);

    // We start rotated into a drone view initial orientation for taking pictures of the world
    sm::quaternion<float> def_q (sm::vec<float>::ux(), mc::pi_over_2); // non-blender only
    v.setSceneRotation (def_q);
    v.bgcolour = {static_cast<float>(0x4c)/0xff, static_cast<float>(0x69)/0xff, static_cast<float>(0x93)/0xff, 1.0f};

    // A window for the 2D eye view projection
    mplot::Visual<glver> veye (920, 512, "Eye view");
    veye.setSceneTrans (sm::vec<float,3>{ float{0}, float{0}, float{-4.1} });
    veye.setSceneTrans (sm::vec<float,3>{ float{-0.00859182}, float{-0.616208}, float{-0.972577} });

    // A window for the Ant body view
    mplot::Visual<glver> vant (920, 920, "Ant view");
    vant.setSceneTrans (sm::vec<float,3>{ float{0.113123}, float{0.0217872}, float{-3.7961} });
    vant.setSceneRotation (sm::quaternion<float>{ float{0.937372}, float{0.106131}, float{0.330499}, float{0.0289824} });

    // Use a FPS profiling with a text object on screen
    mplot::fps::profiler fps_profiler;
    mplot::VisualTextModel<glver>* fps_label;
    v.addLabel ("0 FPS", {0.63f, -0.43f, 0.0f}, fps_label);

    // We get the eye data path from the glTF file
    std::string efpath("");
    int32_t ncam = static_cast<int32_t>(getCameraCount());
    int32_t num_compound_cameras = 0;
    int32_t my_compound_camera = -1;
    for (int32_t ci = 0; ci < ncam; ++ci) {
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
        int32_t csamp = getCurrentEyeSamplesPerOmmatidium();
        std::cout << "Current eye samples per ommatidium is " << csamp << std::endl;
        if (csamp < 32000) { changeCurrentEyeSamplesPerOmmatidiumBy (samples_per_omm_default - csamp); }
    }

    // We get the initial camera localspace. This also serves to reset the camera pose. This is set
    // in the GLTF file and note that it may be a LEFT HANDED coordinate system!
    sm::mat<float, 4> ics = mplot::compoundray::getCameraSpace (scene);
    sm::mat<float, 4> initial_camera_space;
    initial_camera_space.translate (ics.translation()); // Right handed

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
    mplot::compoundray::EyeVisual<glver>* ep0 = nullptr;
    auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &ommatidiaData,
                                                                         reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia),
                                                                         oces_reader.read_success ? reinterpret_cast<mplot::meshgroup*>(&oces_reader.head_mesh) : nullptr);
    v.bindmodel (eyevm);
    eyevm->setViewMatrix (initial_camera_space);
    eyevm->name = "EyeVisual";
    eyevm->finalize();
    ep0 = v.addVisualModel (eyevm);

    // We follow the eyevisual as it moves by default
    v.options.set (mplot::visual_options::viewFollowsVMTranslations);
    // or in a future version:
    // v.options.set (mplot::visual_options::viewFollowsVMBehind);

    v.setFollowedVM (ep0);

    // A second eye goes in the 'eye only' window
    auto ptype = mplot::compoundray::EyeVisual<glver>::projection_type::mercator;
    mplot::compoundray::EyeVisual<glver>* ep1 = nullptr;
    mplot::compoundray::EyeVisual<glver>* ep2 = nullptr;

    auto eyevm1 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &ommatidiaData,
                                                                          reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia),
                                                                          oces_reader.read_success ? reinterpret_cast<mplot::meshgroup*>(&oces_reader.head_mesh) : nullptr);
    vant.bindmodel (eyevm1);

    auto eyevm2 = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &ommatidiaData,
                                                                          reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia), nullptr);
    veye.bindmodel (eyevm2);
    eyevm2->name = "Big Eye";
    // First eye of eye pair (one spherical projection)
    uint32_t sz = 1024;
    float ps_rad = 0.0001f;                  // projection sphere radius
    sm::vec<> centre = { -0.00002f, 0, 0 };  // projection sphere centre

    if (oces_reader.read_success == true) {
        sz = oces_reader.position.size();
        ps_rad = 0.0002f;
        centre = { -0.00056, 0.00005, -0.00005 };
    }

    sm::mat<float, 4> twod_tr;                            // twod projection transformation
    float twod_scale = 1.0f;                              // twod projection scaling
    sm::vec<> twod_offset = { 0.0001f, 0.0f, 0.0f };      // twod projection translation to move to centre
    sm::vec<> twod_offset2 = { -0.0004f, 0.0007f, 0.0f }; // post scale/rotate translation
    sm::vec<> twod_shift = {0,0.0006,0};
    float rotn = -sm::mathconst<float>::pi_over_8;
    if (oces_reader.read_success == true) {
        std::cout << "Read from oces file!!\n";
        ptype = mplot::compoundray::EyeVisual<glver>::projection_type::equirectangular;
        twod_tr.translate (twod_shift);
    } else {
        twod_tr.translate (twod_offset2);
        twod_tr.scale (twod_scale);
        twod_tr.rotate (sm::vec<>::uy(), rotn);
        twod_tr.translate (twod_offset);
    }

    // Projection sphere rotation about x axis by 0.2 radians. Numbers determined using oces_viewer
    sm::quaternion<float> psrotn (sm::vec<>::ux(), 0.2f);

    eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn, 0, sz/2);

    // Second of eye pair (another spherical projection)
    if (oces_reader.read_success == true) {
        if (oces_reader.mirrors.empty() == false) {
            centre = (oces_reader.mirrors[0] * centre).less_one_dim();
            sm::vec<> twod_shift_left = twod_shift;
            twod_shift_left[0] *= -1.0f;
            twod_tr.set_identity();
            twod_tr.translate (twod_shift_left);
            eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn.invert(), sz/2, sz);
        }
    }

    // Visualization options ant body
    eyevm1->show_3d = true;
    eyevm1->finalize();
    ep1 = vant.addVisualModel (eyevm1);
    // Scale this model up, so it's not tiny like the one in the scene
    ep1->scaleViewMatrix (1000);
    // The ant body itself
    auto av1 = std::make_unique<biosim::AntBodyVisual<glver>>();
    vant.bindmodel (av1);
    av1->finalize();
    auto ant_ptr1 = vant.addVisualModel (av1);
    ant_ptr1->name = "ant";
    ant_ptr1->scaleViewMatrix (1000);

    // Visualization options - 2D one
    eyevm2->show_3d = false;
    eyevm2->twodimensional (true);
    eyevm2->show_sphere = false;
    eyevm2->show_rays = false;
    auto flip = sm::quaternion<float>{0, 0, 1, 0}; // In 2D, flip the model
    sm::mat<float, 4> mflip;
    mflip.rotate (flip);
    eyevm2->setViewMatrix (mflip);
    eyevm2->finalize();
    ep2 = veye.addVisualModel (eyevm2);
    ep2->scaleViewMatrix (1000);

    // The ant body
    auto av = std::make_unique<biosim::AntBodyVisual<glver>>();
    v.bindmodel (av);
    av->finalize();
    auto ant_ptr = v.addVisualModel (av);
    ant_ptr->name = "ant";
    ant_ptr->setViewMatrix (initial_camera_space);

    // Breadcrumb trail
    uint64_t move_counter = 0u;
    uint64_t max_bc = 32000;//00; // 32000
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

#if 0
    // A follower coordinate arrow, to debug camera following.
    auto folca = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<>{});
    v.bindmodel (folca);
    folca->em = 0.0f;
    len *= 0.5f;
    folca->lengths = { len, len, len };
    folca->thickness = 0.8f;
    folca->endsphere_size = 1.1f;
    folca->finalize();
    auto folca_ptr = v.addVisualModel (folca);
    folca_ptr->name = "fol";
    folca_ptr->setViewMatrix (initial_camera_space);
#endif

    // Get access to the landscape VisualModel by searching for a selection of model names
    mplot::VisualModel<glver>* land = nullptr;
    {
        mplot::VisualModel<glver>* vmp = nullptr;
        v.init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
        while ((vmp = v.get_next_vm_accessor()) != nullptr) {
            if (vmp->name == "Landscape.003" || vmp->name == "ground_inner_high_res") {
                land = vmp;
                land->make_navmesh (basepath);
                // normals for debug
                auto nrm = std::make_unique<mplot::NormalsVisual<glver>> (land);
                v.bindmodel (nrm);
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
    sm::vvec<uint32_t> csv_antflags;
    // When reproducing csv paths, it's useful to keep a record of the last triangle, because the
    // most likely next triangle is the last triangle.
    uint32_t last_ti = std::numeric_limits<uint32_t>::max();

    if (opts.test (antpov::options::path_from_csv)) {
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
    for (uint32_t i = 0; i < csv_antflags.size(); ++i) {
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

        if (opts.test (antpov::options::path_from_csv) && !csv_positions.empty()) {
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
        if (_ti0 != std::numeric_limits<uint32_t>::max()) {
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
    antpov::random_outbound<float> rrg(1500, 150, 100);

    // We keep a track of the eye size. Used in subr_detect_camera_changes
    size_t last_eye_size = 0u;

    // For debug saving:
    sm::mat<float, 4> tm1_cam_to_scene;
    sm::vec<float> tm1_mv_camframe = {};
    uint32_t tm1_ti0 = 0u;

    uint32_t render_counter = 0u;
    auto subr_detect_camera_changes = [&v, &ommatidia, &ommatidiaData,
                                       &last_eye_size, &ep0, &ep1, &ep2, &render_counter, opts] ()
    {
        size_t curr_eye_size = last_eye_size;
        // Detect changes in the camera and update eye model as necessary
        if (ommatidiaData.size() == 0) {
            if (isCompoundEyeActive()) { getCameraData (ommatidiaData); }
        } // else no need to re-get data

        // Update eyevm model (or just update colours)
        ep0->ommatidia = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);
        ep1->ommatidia = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);
        ep2->ommatidia = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);

        static constexpr uint32_t render_every = 1u; // set to 1 for max update, 60 to reduce compute
        if (ommatidia != nullptr) {
            curr_eye_size = ommatidia->size();
            if (curr_eye_size != last_eye_size) {
                if (render_counter % render_every == 0u) { ep0->reinit(); }
                ep1->reinit();
                ep2->reinit();
                last_eye_size = curr_eye_size;
            } else {
                if (render_counter % render_every == 0u) { ep0->reinitColours(); }
                ep1->reinitColours(); // 4x faster to just reinitColours
                ep2->reinitColours();
            }
            ++render_counter;
        }
    };

    // Helper subroutine used by all the movement subroutines
    auto subr_reset_camspace = [&v, &initial_camera_space, &hoverheight, land, land_to_scene] (sm::mat<float, 4>& cam_to_scene)
    {
        // reset to initial camera space if requested
        if (v.vstate.test (antpovvisual::state::campose_reset_request) == true) {
            v.stop(); // cancel any active movements
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (initial_camera_space));
            sm::mat<float, 4> camspace = mplot::compoundray::getCameraSpace (scene);
            auto[hp_scene, _ti0] = land->navmesh->find_triangle_hit (camspace, land_to_scene);
            cam_to_scene = land->navmesh->position_camera (hp_scene, land_to_scene, hoverheight);
            setCameraPoseMatrix (mplot::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
            v.vstate.reset (antpovvisual::state::campose_reset_request);
        }
    };

    auto subr_key_move_over_land = [&v, &ep0, &ant_ptr, &antca_ptr, &initial_camera_space, &move_counter,
                                    &breadcrumb_coords, &breadcrumb_data, &isvp, &hoverheight, &rrg,
                                    max_bc, land, land_to_scene, subr_reset_camspace,
                                    &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0](const float fps)
    {
        antca_ptr->setHide (!v.vstate.test(antpovvisual::state::show_camframe));
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
            if (v.move_state.test (antpovvisual::move_sense::up)) {
                hoverheight += 0.0001f;
            } else if (v.move_state.test (antpovvisual::move_sense::down)) {
                hoverheight -= 0.0001f;
                if (hoverheight < 0.0f) { hoverheight = 0.0f; }
            }
            // Obtain the commanded movement vector and turn this into a translation matrix
            rrg.step();
            //sm::vec<float> mv_camframe = v.getMovementVector (1.0f / rrg.speed); // confusing, can have -ve speed
            sm::vec<float> mv_camframe = v.getMovementVector (60);
            sm::vec<float> lastloc = cam_to_scene.translation();
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            uint32_t ti0_sv = land->navmesh->ti0;
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
        ep0->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    auto subr_walk_over_land = [&v, &ep0, &ant_ptr, &antca_ptr, &initial_camera_space, &rrg,
                                &opts, &move_counter, max_bc, &breadcrumb_coords, &breadcrumb_data,
                                &isvp, land, land_to_scene,
                                &hoverheight, subr_reset_camspace,
                                &tm1_cam_to_scene, &tm1_mv_camframe, &tm1_ti0](const float fps)
    {
        antca_ptr->setHide (!v.vstate.test(antpovvisual::state::show_camframe));
        sm::mat<float, 4> cam_to_scene = mplot::compoundray::getCameraSpace (scene);

        // A random walk mode
        if (v.vstate.test (antpovvisual::state::walk) == false) { return; }

        // set rotation and step length according to the Stone paper
        rrg.step();
        // rrg.omega is the angular speed rrg.speed is the linear speed
        //std::cout << "rotating in this step by " << rrg.omega << " and moving forward by " << rrg.speed << std::endl;
        rotateCamerasLocallyAround (rrg.omega, 0.0f, 1.0f, 0.0f);
        cam_to_scene = mplot::compoundray::getCameraSpace (scene);
        sm::vec<float> mv_camframe = { 0, 0, rrg.speed };
        sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
        uint32_t ti0_sv = land->navmesh->ti0;
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
                opts.set (antpov::options::max_fps, false); // don't burn electricity after exception
                v.vstate.set (antpovvisual::state::walk, false);
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
        ep0->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    auto subr_csv_playback = [&v, &ep0, &ant_ptr, &antca_ptr, &initial_camera_space,
                              &move_counter, &breadcrumb_coords, &breadcrumb_data, &isvp, &hoverheight, &opts,
                              max_bc, csv_positions, bc_clr, bc_alpha, bc_scale,
                              land, land_to_scene, subr_reset_camspace]
    (const float fps, uint32_t& _last_ti)
    {
        antca_ptr->setHide (!v.vstate.test(antpovvisual::state::show_camframe));
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

            if (_ti0 != std::numeric_limits<uint32_t>::max()) {
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
            opts.set (antpov::options::path_from_csv, false);
        }

        subr_reset_camspace (cam_to_scene); // if requested
        // Update the view matrix of eye and eye localspace axes
        ep0->setViewMatrix (cam_to_scene);
        ant_ptr->setViewMatrix (cam_to_scene);
        antca_ptr->setViewMatrix (cam_to_scene);
    };

    if (opts.test (antpov::options::debug_mv)) {

        std::cout << "Loading compute_mesh_movement data from crash file\n";

        sm::mat<float, 4> _cam_to_scene = {{}};
        sm::mat<float, 4> _land_to_scene = {{}};
        sm::vec<float> _mv_camframe = {};
        float _hoverheight = 0.0f;
        uint32_t _ti0 = 0u;

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
            ep0->setViewMatrix (tm1_cam_to_scene);
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
            ep0->setViewMatrix (_cam_to_scene);
            ant_ptr->setViewMatrix (_cam_to_scene);
            antca_ptr->setViewMatrix (_cam_to_scene);

        } catch (const std::exception& e) {
            std::cout << "Exception moving: " << e.what() << std::endl;
            while (!v.readyToFinish()) {
                antca_ptr->setHide (!v.vstate.test(antpovvisual::state::show_camframe));
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
    std::string m_count_str = {};

    mplot::fps::profiler move_fps;
    mplot::fps::profiler mplot_fps;
    mplot::fps::profiler cray_fps;
    mplot::fps::profiler detect_fps;

#if 0
    // Follower velocity
    sm::vec<float> fvel = {};
    // Follower rotation
    sm::quaternion<float> cam_rotn;
#endif

    sm::hdfdata record (h5_path, std::ios::out | std::ios::trunc);
    while (!v.readyToFinish()) {

        // Tell the fps_profiler that we're at the start of a loop
        fps_profiler.at_begin (antpov::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));

        detect_fps.at_begin (antpov::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));
        // The current camera may have changed, this subroutine deals with any changes
        subr_detect_camera_changes();
        detect_fps.at_end();

        mplot_fps.at_begin (antpov::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));
        // Now render the mathplot window
        v.render();
        // Change label after render (it needs v's context, not veye's)
        if (move_counter % 1000 == 0) {
            m_count_str = std::to_string (move_counter);
            fps_label->setupText (fps_profiler.fps_txt + std::string(" ") + m_count_str);
#if 0
            std::cout << "mplot FPS: " << mplot_fps.fps_txt << std::endl;
            std::cout << "cray FPS: " << cray_fps.fps_txt << std::endl;
            std::cout << "move FPS: " << move_fps.fps_txt << std::endl;
            std::cout << "detect FPS: " << detect_fps.fps_txt << std::endl;
            std::cout << "Overall FPS: " << fps_profiler.fps_txt << std::endl;
#endif
        }
        // Save some electricity while developing - limit to 60 FPS. For max speed use v.poll() (-x)
        if (opts.test (antpov::options::max_fps)) { v.poll(); } else { v.wait (waittime); }
        // Render the eye-only window
        vant.render();
        veye.render();
        // Deal with any movements commanded by key press events (including reset)

        v.setContext(); // right now key move over land needs v's context
        mplot_fps.at_end();

        // tmp profile
        move_fps.at_begin (antpov::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));
        if (v.vstate.test (antpovvisual::state::paused) == false) {
            if (v.vstate.test (antpovvisual::state::walk)) {
                subr_walk_over_land (fps_profiler.fps_mean);
            } else if (opts.test (antpov::options::path_from_csv)) { // Construct path from csv file of 2D ant locations
                subr_csv_playback (fps_profiler.fps_mean, last_ti);
            } else {
                subr_key_move_over_land (fps_profiler.fps_mean);
            }
        }
        // tmp profile
        move_fps.at_end();

#if 0
        // Follower coordinate arrows
        {
            constexpr float time_90 = 1.0f; // s
            sm::mat<float, 4> fol_targ = mplot::compoundray::getCameraSpace (scene);
            fol_targ.translate (sm::vec<float>{0, 0.4f, -0.5f}); // Always follow behind the camera
            sm::mat<float, 4> cur_ = folca_ptr->getViewMatrix();
            sm::mat<float, 4> fol_cur;
            fol_cur.translate (cur_.translation());
            cur_.translate (-cur_.translation());
            sm::quaternion<float> r_cur0 = cur_.rotation();
            sm::vec<float> pos_shift = antpov::get_cam_movement<float> (fol_cur, fol_targ, fvel, time_90,
                                                                        static_cast<float>(waittime));
            sm::mat<float, 4> fol_targ0 = fol_targ;
            fol_targ0.translate (-fol_targ.translation());
            sm::mat<float, 4> fol_cur0 = fol_cur;
            fol_cur0.translate (-fol_cur.translation());
            sm::quaternion<float> r_targ0 = fol_targ0.rotation();
            r_targ0.renormalize();
            r_cur0.renormalize();
            cam_rotn = r_cur0.slerp (r_targ0, 0.05f);
            // set the translation/rotation into fol_cur
            fol_cur.pretranslate (pos_shift); // Aaaaaargh pretranslate, not translate!!!
            fol_cur.rotate (cam_rotn);
            folca_ptr->setViewMatrix (fol_cur);
            folca_ptr->setHide (v.options.test (mplot::visual_options::viewFollowsVMBehind));
        }
#endif
        cray_fps.at_begin (antpov::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));
        // Do the compound-ray ray casting to recompute the scene
        renderFrame();
        // Access data so that a brain model could be fed
        if (isCompoundEyeActive()) {
            getCameraData (ommatidiaData);
            ommatidia = &scene->m_ommVecs[scene->getCameraIndex()];

            // if csv mode, then save the data
            if (opts.test (antpov::options::path_from_csv) && opts.test (antpov::options::save_hdf5)) {
                std::cout << "Saving frame...\n";
                std::string ommframe = "/ommatidiaData/frame_" + std::to_string (move_counter);
                try {
                    record.add_contained_vals (ommframe.c_str(), ommatidiaData);
                } catch (const std::exception& e) {
                    // Probably didn't move this time.
                }
            }
        }
        cray_fps.at_end();

        // Scale size of breadcrumbs based on distance
        float iscl = 2.0f * std::log (1.0f + v.get_d_to_rotation_centre());
        //if (move_counter % 50 == 0) { std::cout << "iscl = " << iscl << std::endl; }
        isvp->set_instance_scale (iscl);

        // Mark that we got to the end of the loop
        fps_profiler.at_end();
    }

    if (opts.test (antpov::options::path_from_csv)) {
        // convert std::vector<Ommatidium>* ommatidia into vvecs that can be h5 saved
        auto ommat = reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);
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

    stop(); // stop compound-ray from running
    multicamDealloc(); // De-allocate compound-ray memory

    return 0;
}
