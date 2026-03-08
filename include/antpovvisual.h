#pragma once

#include <string>
#include <iostream>
#include <sm/mathconst>

//import sm.vec;
//import sm.flags;

import mplot.visual;

// Reproduce controller functions for the mplot window for ease of use
struct antpovvisual final : public mplot::Visual<glver>
{
    using mc = sm::mathconst<float>;

    // Boilerplate constructor (just copy this):
    antpovvisual (int width, int height, const std::string& title, const bool blender_axes)
        : mplot::Visual<glver> (width, height, title)
    {
        // State defaults
        //this->vstate |= state::show_cones;
        this->vstate |= state::show_camframe;
        if (blender_axes) {
            this->updateCoordLabels ("X", "Y", "Z(up)");
        } else {
            this->updateCoordLabels ("X", "Y(up)", "Z");
        }
    }

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
        if (action == mplot::keyaction::press && !(mods & keymod::shift)) {
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

        } else if (action == mplot::keyaction::release && !(mods & keymod::shift)) {

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
            } else if (key == mplot::key::w && (mods & keymod::control)) {
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
                if (this->options.test (mplot::visual_options::viewFollowsVMBehind) == true
                    && this->options.test (mplot::visual_options::viewFollowsVMTranslations) == false) {
                    this->options.reset (mplot::visual_options::viewFollowsVMBehind);
                    this->options.set (mplot::visual_options::viewFollowsVMTranslations);
                    std::cout << "sceneview follows agent movements (drone view)\n";
                } else { // this->options.test (mplot::visual_options::viewFollowsVMTranslations) == true
                    this->options.set (mplot::visual_options::viewFollowsVMBehind);
                    this->options.reset (mplot::visual_options::viewFollowsVMTranslations);
                    std::cout << "sceneview follows behind agent (follower view)\n";
                    // Don't show camframe when following
                    this->vstate.reset (antpovvisual::state::show_camframe);
                }
            }
        }
    }
};
