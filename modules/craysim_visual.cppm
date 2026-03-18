module;

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>
#include <array>

#include <sampleConfig.h>
#include <MulticamScene.h>
#include <libEyeRenderer.h> // getCurrentEyeSamplesPerOmmatidium

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

#include <mplot/keys.h>

export module craysim.visual;

import sm.mathconst;
import sm.vvec;
import sm.quaternion;
import sm.mat;

import mplot.tools;
import mplot.compoundray.interop; // mathplot <--> compoundray interoperability
import mplot.compoundray.ommatidium; // The mplot::Ommatidium structure
import mplot.compoundray.eyevisual;

export import mplot.gl.version;
export import mplot.visual;
export import mplot.fps.profiler;
export import oces.reader;

// Reproduce controller functions for the mplot window for ease of use
export namespace craysim
{
    void printHelp (const char* progname)
    {
        std::cout << "USAGE:\n" << progname << " -f <path to gltf scene>\n\n"
                  << "\t-h\tDisplay this help information.\n"
                  << "\t-f\tPath to a gltf scene file (absolute or relative to current "
                  << "working directory, e.g. './data/axis_coloured_blocks.gltf').\n";
    }

    // Flags class
    enum class options : std::uint32_t
    {
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
        max_fps,          // If true, poll, instead of fps
        path_from_csv,    // Move the agent from a pre-defined sequence of 2D coordinates that give it a path
        save_hdf5,        // If true, then save any output data in HDF5 (active in 'path_from_csv' mode)
        debug_mv,         // Open a debug h5 file and run compute_mesh_movement once for debug of NavMesh
        can_exit          // If set, program can exit now
    };

    // Parse cmd line to find the path and set options. Return filepath of main scene gltf file and any csv path
    std::tuple<std::string, std::string, std::string> parse_inputs (std::int32_t argc, char* argv[], sm::flags<craysim::options>& opts)
    {
        std::string path = "";
        std::string csvpath = "";
        std::string hovh = "";
        for (std::int32_t i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-h") {
                craysim::printHelp (argv[0]);
                opts |= craysim::options::can_exit;
            } else if (arg == "-f") {
                i++;
                path = std::string(argv[i]);
            } else if (arg == "-b") {
                opts |= craysim::options::blender_axes;
            } else if (arg == "-x") {
                opts |= craysim::options::max_fps;
            } else if (arg == "-c") {
                opts |= craysim::options::path_from_csv;
                i++;
                csvpath = std::string(argv[i]);
            } else if (arg == "-d") {
                opts |= craysim::options::save_hdf5;
            } else if (arg == "-g") {
                opts |= craysim::options::debug_mv;
            }
        }
        if (path.empty()) {
            craysim::printHelp (argv[0]);
            opts |= craysim::options::can_exit;
        }

        std::string h5_path = csvpath;
        mplot::tools::stripFileSuffix (h5_path);
        if (h5_path.empty()) { h5_path = "trail"; }
        h5_path += ".h5";

        return {path, csvpath, h5_path};
    }

    // For a given samples per omm, return a sensible number of loops over which to average fps, so
    // that fps takes around 1 sec to stabilize.
    constexpr std::uint32_t best_n_samples (std::int32_t samples_per_omm)
    {
        std::uint32_t best_n = 0;
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

    // Read a simple csv with 2D coordinates, using first two entries on each line
    bool read_csv (const std::string& path, sm::vvec<sm::vec<float, 2>>& positions)
    {
        std::ifstream f (path.c_str(), std::ios::in);
        if (f.is_open() == false) { return false; }
        std::string line;
        std::vector<std::string> tokens;
        while (std::getline (f, line)) {
            sm::vec<float, 2> twodpos;
            // Tokenize line into the coordinates
            twodpos.set_from_str (line, ",");
            positions.push_back (twodpos);
        }
        return true;
    }

    template <int glver>
    struct visual final : public mplot::Visual<glver>
    {
        using mc = sm::mathconst<float>;

        // When the program starts, how many samples per ommatidium/element do you want?
        static constexpr std::int32_t samples_per_omm_default = 64;

        visual (int width, int height, const std::string& title, const std::string& gltfpath, sm::flags<craysim::options>& opts)
            : mplot::Visual<glver> (width, height, title)
        {
            // Boilerplate memory alloc for compound-ray and turn off verbose logging.
            multicamAlloc(); setVerbosity (false);

            this->speed = 0.5f; // 0.5 m/s max speed for our agent
            this->angularSpeed = 2.0f * mc::two_pi / 360.0f;
            this->lightingEffects (true);
            // Use a non-default zFar as we are likely to use large environments
            this->zFar = 2400;
            // Rotate about the nearest VisualModel
            this->rotateAboutNearest (true);
            // Rotate about a scene vertical axis? true for landscapes, false for cubes/objects (Ctrl-k changes I think, at runtime)
            this->rotateAboutVertical (true);
            // A blue sky background colour by default (client code can change this)
            this->bgcolour = { 0.298f, 0.412f,  0.576f };
            // State defaults
            //this->vstate |= state::show_camframe;
            if (opts.test(craysim::options::blender_axes)) {
                this->switch_scene_vertical_axis(); // to uz up
                this->updateCoordLabels ("X", "Y", "Z(up)");
            } else {
                this->updateCoordLabels ("X", "Y(up)", "Z");
                // We start rotated into a drone view initial orientation for taking pictures of the world.
                // Into craysim::visual (with the blender_axes==true equivalent)...
                sm::quaternion<float> def_q (sm::vec<float>::ux(), mc::pi_over_2); // non-blender only
                this->setSceneRotation (def_q);
            }

            // We follow the agent as it moves by default.
            this->options.set (mplot::visual_options::viewFollowsVMTranslations);

            this->load (gltfpath, opts);

            // Use a FPS profiling with a text object on screen
            this->addLabel ("0 FPS", {0.63f, -0.43f, 0.0f}, this->fps_label);

            this->setup_camera();

            this->setup_oces();

            this->setup_eyevisual();
        }

        ~visual()
        {
            stop(); // stop compound-ray from running
            multicamDealloc(); // De-allocate compound-ray memory
        }

        void setup_camera()
        {
            // We get the eye data path from the glTF file
            std::int32_t ncam = static_cast<std::int32_t>(getCameraCount());
            std::int32_t num_compound_cameras = 0;
            std::int32_t my_compound_camera = -1;
            for (std::int32_t ci = 0; ci < ncam; ++ci) {
                gotoCamera (ci);
                this->efpath = getEyeDataPath();
                if (!this->efpath.empty()) {
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
                std::int32_t csamp = getCurrentEyeSamplesPerOmmatidium();
                std::cout << "Current eye samples per ommatidium is " << csamp << std::endl;
                if (csamp < 32000) { changeCurrentEyeSamplesPerOmmatidiumBy (samples_per_omm_default - csamp); }
            }
        }

        void setup_oces()
        {
            // Use oces_reader to read in our eye data, esp. for the head
            std::string oces_path = this->efpath;
            mplot::tools::stripFileSuffix (oces_path);
            oces_path += ".gltf";
            // Now try to open oces_path
            std::cout << "Attempt to load OCES file " << oces_path << "\n";
            this->oces_reader.read (oces_path);
            if (oces_reader.read_success == false) {
                std::cout << "No associated OCES file for a head\n";
            } else {
                // Read the head and make a VisualModel
                oces_reader.head_mesh.single_colour = {0.345f, 0.122f, 0.082f};
            }
        }

        void setup_eyevisual()
        {
            // We get the initial camera localspace. This also serves to reset the camera pose. This is set
            // in the GLTF file and note that it may be a LEFT HANDED coordinate system!
            sm::mat<float, 4> ics = mplot::compoundray::getCameraSpace (scene);
            this->initial_camera_space.translate (ics.translation()); // Right handed

            // Create an EyeVisual 'eye' in our scene
            auto eyevm = std::make_unique<mplot::compoundray::EyeVisual<glver>> (sm::vec<>{}, &this->ommatidiaData, this->get_ommatidia_ptr(), this->get_head_mesh());
            eyevm->set_parent (this->get_id());
            eyevm->setViewMatrix (this->initial_camera_space);
            eyevm->name = "EyeVisual";
            eyevm->finalize();
            this->eye = this->addVisualModel (eyevm);
            // This eye is the followed VM.
            this->setFollowedVM (this->eye);
        }

        void load (const std::string& gltfpath, sm::flags<craysim::options>& opts)
        {
            // Load the file
            this->path = gltfpath;
            this->basepath = this->path;
            std::cout << "Loading glTF file \"" << this->path << "\"..." << std::endl;
            mplot::tools::stripUnixFile (this->basepath);
            std::cout << "glTF dir: " << this->basepath << std::endl;
            loadGlTFscene (this->path.c_str(), (opts.test(craysim::options::blender_axes)
                                                ? mplot::compoundray::blender_transform() : sutil::Matrix4x4::identity()));
            // Get the visual models from the scene
            mplot::compoundray::scene_to_visualmodels<glver> (scene, this, false); // true for 'make_navmeshes'
        }

        void fps_label_update()
        {
            this->fps_label->setupText (this->fps_profiler.fps_txt + std::string(" "));
        }

        std::vector<mplot::compoundray::Ommatidium>* get_ommatidia_ptr()
        {
            return reinterpret_cast<std::vector<mplot::compoundray::Ommatidium>*>(ommatidia);
        }

        mplot::meshgroup* get_head_mesh()
        {
            return this->oces_reader.read_success ? reinterpret_cast<mplot::meshgroup*>(&this->oces_reader.head_mesh) : nullptr;
        }

        // A member fps_profiler
        mplot::fps::profiler fps_profiler;
        // The FPS label, accessible to client code
        mplot::VisualTextModel<glver>* fps_label;
        // Base path for glTF file
        std::string basepath = {};
        // Full path for glTF file
        std::string path = {};
        // The eye file path, obtained from OCES file
        std::string efpath = {};
        // Open Compound Eye Standard reader used to access an agent head mesh (compound-ray reads the ommatidia info)
        oces::reader oces_reader;

        // Required in every craysim, I think. craysim::state? member of craysim::visual?
        std::vector<std::array<float, 3>> ommatidiaData;
        std::vector<Ommatidium>* ommatidia = nullptr;

        // This is the start position of the camera as loaded from the gltf
        sm::mat<float, 4> initial_camera_space;

        // An mplot::VisualModel of the compound-ray eye
        mplot::compoundray::EyeVisual<glver>* eye = nullptr;

        // Movement state (class and bitset) (flags?)
        enum class move_sense : uint16_t { forward, backward, left, right, up, down, rotUp, rotDown, rotLeft, rotRight, rotRollLeft, rotRollRight, zoomIn, zoomOut };
        sm::flags<move_sense> move_state;

        // Speed of translations (in scene units per second). From this determine distance for one
        // movement step based on current FPS/seconds per frame
        float speed = 1.0f;
        // Speed of rotations
        float angularSpeed = mc::two_pi / 360.0f;
        // Parameter for EyeVisual. If focal offset is 0, then user has to choose how long the cones should be
        float manual_cone_length = 0.2f;

        enum class state : uint8_t {
            show_cones,            // Parameter for EyeVisual. Draw simple flared tubes in mathplot window
            campose_reset_request, // A request to reset the pose of the camera
            show_camframe,         // Show camera axes?
            paused,                // Pause sim (i.e. pause time)?
            stepfwd,               // If true and if paused is true, step forward one timestep in the camera input
            walk,                  // If true, do a random walk
            freeze                 // Freeze movement
        };
        sm::flags<state> vstate;

        void freeze (const bool val)
        {
            this->vstate.set (state::freeze, val);
            this->stop();
        }

        // Get the camera's movement vector to give speed in model world at the current FPS
        sm::vec<float, 3> getMovementVector (const float fps)
        {
            sm::vec<float, 3> output = {};
            if (this->move_state.test (move_sense::up)) { output += 0.1f * speed / fps * sm::vec<>::uy(); } // uy is up
            if (this->move_state.test (move_sense::down)) { output += 0.1f * speed / fps * -sm::vec<>::uy(); }
            if (this->move_state.test (move_sense::left)) { output += speed / fps * sm::vec<>::ux(); }
            if (this->move_state.test (move_sense::right)) { output += speed / fps * -sm::vec<>::ux(); }    // right is in -x dirn
            if (this->move_state.test (move_sense::forward)) { output += speed / fps * sm::vec<>::uz(); }   // fwd is in uz dirn
            if (this->move_state.test (move_sense::backward)) { output += speed / fps * -sm::vec<>::uz(); }
            return output;
        }

        // Get the camera's vertical rotation angle (pitch).
        float getVerticalRotationAngle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rotUp)) { out += angularSpeed; }
            if (this->move_state.test (move_sense::rotDown)) { out -= angularSpeed; }
            return out;
        }

        // Get the camera's horizontal rotation angle (yaw). Rightward is positive.
        float getHorizontalRotationAngle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rotLeft)) { out += angularSpeed; }
            if (this->move_state.test (move_sense::rotRight)) { out -= angularSpeed; }
            return out;
        }

        // Get the camera's roll
        float getRollRotationAngle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rotRollLeft)) { out -= angularSpeed; }
            if (this->move_state.test (move_sense::rotRollRight)) { out += angularSpeed; }
            return out;
        }

        bool isActivelyRotating()
        {
            return (this->move_state.test (move_sense::rotUp)
                    || this->move_state.test (move_sense::rotDown)
                    || this->move_state.test (move_sense::rotLeft)
                    || this->move_state.test (move_sense::rotRight)
                    || this->move_state.test (move_sense::rotRollLeft)
                    || this->move_state.test (move_sense::rotRollRight));
        }

        bool isActivelyTranslating()
        {
            return (this->move_state.test (move_sense::up)
                    || this->move_state.test (move_sense::down)
                    || this->move_state.test (move_sense::left)
                    || this->move_state.test (move_sense::right)
                    || this->move_state.test (move_sense::forward)
                    || this->move_state.test (move_sense::backward));
        }

        // Is the camera 'actively moving'?
        bool isActivelyMoving() { return this->move_state.any(); }

        // Cancel any movement. Also unpause
        void stop()
        {
            this->vstate.reset (state::paused);
            this->move_state.reset();
        }

    protected:

        static constexpr bool debug_callback_extra = false;
        void key_callback_extra (int key, int scancode, int action, int mods) override
        {
            if (this->vstate.test (state::freeze)) { return; } // Don't respond to movement keys

            // Process press/repeat key actions (none will work with Ctrl or Shift)
            if (action == mplot::keyaction::press && !(mods & mplot::keymod::shift)) {
                if (key == mplot::key::w) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::forward);
                } else if (key == mplot::key::a && !mods) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::left);
                } else if (key == mplot::key::d) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::right);
                } else if (key == mplot::key::s) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::backward);
                } else if (key == mplot::key::p) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::up);
                } else if (key == mplot::key::l) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::down);
                } else if (key == mplot::key::up) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotUp);
                } else if (key == mplot::key::down) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotDown);
                } else if (key == mplot::key::left) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotLeft);
                } else if (key == mplot::key::right) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotRight);
                } else if (key == mplot::key::comma) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotRollLeft);
                } else if (key == mplot::key::period) {
                    this->vstate.reset (state::paused);
                    this->move_state.set (move_sense::rotRollRight);
                } else if (key == mplot::key::end) {
                    this->speed = this->speed * 0.5f;
                    std::cout << "Speed reduced to " << this->speed  << "m/s" << std::endl;
                } else if (key == mplot::key::home) {
                    this->speed = this->speed * 2.0f;
                    std::cout << "Speed increased to " << this->speed  << "m/s" << std::endl;
                } else if (key == mplot::key::r) {
                    this->stop();
                    this->vstate.set (state::campose_reset_request);
                }

            } else if (action == mplot::keyaction::release && !(mods & mplot::keymod::shift)) {

                if (key == mplot::key::w) {
                    this->move_state.reset (move_sense::forward);
                } else if (key == mplot::key::a && !mods) {
                    this->move_state.reset (move_sense::left);
                } else if (key == mplot::key::d) {
                    this->move_state.reset (move_sense::right);
                } else if (key == mplot::key::s) {
                    this->move_state.reset (move_sense::backward);
                } else if (key == mplot::key::p) {
                    this->move_state.reset (move_sense::up);
                } else if (key == mplot::key::l) {
                    this->move_state.reset (move_sense::down);
                } else if (key == mplot::key::up) {
                    this->move_state.reset (move_sense::rotUp);
                } else if (key == mplot::key::down) {
                    this->move_state.reset (move_sense::rotDown);
                } else if (key == mplot::key::left) {
                    this->move_state.reset (move_sense::rotLeft);
                } else if (key == mplot::key::right) {
                    this->move_state.reset (move_sense::rotRight);
                } else if (key == mplot::key::comma) {
                    this->move_state.reset (move_sense::rotRollLeft);
                } else if (key == mplot::key::period) {
                    this->move_state.reset (move_sense::rotRollRight);
                }
            }

            if (action == mplot::keyaction::press) {
                if (key == mplot::key::t) {
                    // Toggle the morph view
                    this->vstate.flip (state::show_cones);
                } else if (key == mplot::key::w && (mods & mplot::keymod::control)) {
                    // walk
                    std::cout << "Flip walk\n";
                    this->vstate.flip (state::walk);
                } else if (key == mplot::key::c) {
                    this->vstate.flip (state::show_camframe);
                } else if (key == mplot::key::i) {
                    // Increase manual disc size
                    if (this->manual_cone_length < 0.0f) {
                        this->manual_cone_length = 0.001f;
                    } else {
                        this->manual_cone_length *= 2.0f;
                    }
                } else if (key == mplot::key::o) {
                    // Decrease manual disc sizne
                    if (this->manual_cone_length >= 0.0f) {
                        this->manual_cone_length *= 0.5f;
                    }

                } else if (key == mplot::key::escape) {
                    this->stop();

                } else if (key == mplot::key::f && this->vstate.test (state::paused)) {
                    this->vstate.set (state::stepfwd);

                } else if (key == mplot::key::space) {
                    this->vstate.flip (state::paused);

                } else if (key == mplot::key::page_up) {
                    int csamp = getCurrentEyeSamplesPerOmmatidium();
                    if (csamp < 32000) {
                        changeCurrentEyeSamplesPerOmmatidiumBy (csamp); // double
                    } else {
                        // else graphics memory use will get very large
                        std::cout << "max allowed samples\n";
                    }
                } else if (key == mplot::key::page_down) {
                    int csamp = getCurrentEyeSamplesPerOmmatidium();
                    changeCurrentEyeSamplesPerOmmatidiumBy (-(csamp/2)); // halve

                } else if (key == mplot::key::v) { // switch view
                    // cycle between:
                    this->switch_view_follows_mode();
                    // Don't show camframe when following
                    if (this->options.test (mplot::visual_options::viewFollowsVMBehind) == true) {
                        this->vstate.reset (state::show_camframe);
                    }
                }
            }
        }
    };
} // namespace
