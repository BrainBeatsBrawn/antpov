#pragma once

#include <string>
#include <iostream>
#include <sm/mathconst>
#include <sm/vec>
#include <sm/flags>
#include <mplot/Visual.h>

// Reproduce controller functions for the mplot window for ease of use
struct eye3dvisual final : public mplot::Visual<>
{
    using mc = sm::mathconst<float>;

    // Boilerplate constructor (just copy this):
    eye3dvisual (int width, int height, const std::string& title, const bool blender_axes)
        : mplot::Visual<> (width, height, title)
    {
        // State defaults
        this->vstate |= state::show_cones;
        this->vstate |= state::show_camframe;
        if (blender_axes) {
            this->updateCoordLabels ("X", "Y", "Z(up)");
        } else {
            this->updateCoordLabels ("X", "Y(up)", "Z");
        }
    }

    // Movement state (class and bitset) (flags?)
    enum class move_sense : uint32_t { forward, backward, left, right, up, down, rotUp, rotDown, rotLeft, rotRight, rotRollLeft, rotRollRight, zoomIn, zoomOut };
    sm::flags<move_sense> move_state;

    // unit vectors for movements in the compound-ray camera frame of reference (which is
    // left-handed)
    static constexpr sm::vec<float, 3> ux = { 1.0f, 0.0f, 0.0f }; // right
    static constexpr sm::vec<float, 3> uy = { 0.0f, 1.0f, 0.0f }; // up
    static constexpr sm::vec<float, 3> uz = { 0.0f, 0.0f, 1.0f }; // fwd
    // Speed of translations
    float speed = 0.04;
    // Speed of rotations
    float angularSpeed = mc::two_pi / 360.0f;
    // Parameter for EyeVisual. If focal offset is 0, then user has to choose how long the cones should be
    float manual_cone_length = 0.2f;

    enum class state : uint32_t {
        show_cones,            // Parameter for EyeVisual. Draw simple flared tubes in mathplot window
        campose_reset_request, // A request to reset the pose of the camera
        show_camframe,         // Show camera axes?
        paused,                // Pause sim (i.e. pause time)?
        stepfwd                // If true and if paused is true, step forward one timestep in the camera input
    };
    sm::flags<state> vstate;

    // Get the camera's movement vector.
    sm::vec<float, 3> getMovementVector (const bool retain_move_state = false)
    {
        sm::vec<float, 3> output = { 0.0f, 0.0f, 0.0f };
        if (this->move_state.test (move_sense::up)) {
            output += 0.2f * speed * uy; // up is in y dirn
            this->move_state.set(move_sense::up, retain_move_state);
        }
        if (this->move_state.test(move_sense::down)) {
            output += 0.2f * speed * -uy;
            this->move_state.set(move_sense::down, retain_move_state);
        }
        if (this->move_state.test(move_sense::left)) {
            output += speed * -ux;
            this->move_state.set(move_sense::left, retain_move_state);
        }
        if (this->move_state.test(move_sense::right)) {
            output += speed * ux; // right is in x dirn
            this->move_state.set(move_sense::right, retain_move_state);
        }
        if (this->move_state.test(move_sense::forward)) {
            output += speed * uz; // fwd is in uz dirn
            this->move_state.set(move_sense::forward, retain_move_state);
        }
        if (this->move_state.test(move_sense::backward)) {
            output += speed * -uz;
            this->move_state.set(move_sense::backward, retain_move_state);
        }
        return output;
    }

    // Get the camera's vertical rotation angle (pitch).
    float getVerticalRotationAngle (const bool retain_move_state = false)
    {
        float out = 0.0f;
        if (this->move_state.test(move_sense::rotUp)) {
            out += angularSpeed;
            this->move_state.set(move_sense::rotUp, retain_move_state);
        }
        if (this->move_state.test(move_sense::rotDown)) {
            out -= angularSpeed;
            this->move_state.set(move_sense::rotDown, retain_move_state);
        }
        return out;
    }

    // Get the camera's horizontal rotation angle (yaw). Rightward is positive.
    float getHorizontalRotationAngle (const bool retain_move_state = false)
    {
        float out = 0.0f;
        if (this->move_state.test(move_sense::rotLeft)) {
            out += angularSpeed;
            this->move_state.set(move_sense::rotLeft, retain_move_state);
        }
        if (this->move_state.test(move_sense::rotRight)) {
            out -= angularSpeed;
            this->move_state.set(move_sense::rotRight, retain_move_state);
        }
        return out;
    }

    // Get the camera's roll
    float getRollRotationAngle (const bool retain_move_state = false)
    {
        float out = 0.0f;
        if (this->move_state.test(move_sense::rotRollLeft)) {
            out -= angularSpeed;
            this->move_state.set(move_sense::rotRollLeft, retain_move_state);
        }
        if (this->move_state.test(move_sense::rotRollRight)) {
            out += angularSpeed;
            this->move_state.set(move_sense::rotRollRight, retain_move_state);
        }
        return out;
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
        // Process press/repeat key actions (none will work with Ctrl or Shift)
        if (action == mplot::keyaction::press || action == mplot::keyaction::repeat) {
            if (key == mplot::key::w) {
                this->stop();
                this->move_state.set (move_sense::forward);
            } else if (key == mplot::key::a && !mods) {
                this->stop();
                this->move_state.set (move_sense::left);
            } else if (key == mplot::key::d) {
                this->stop();
                this->move_state.set (move_sense::right);
            } else if (key == mplot::key::s) {
                this->stop();
                this->move_state.set (move_sense::backward);
            } else if (key == mplot::key::p) {
                this->stop();
                this->move_state.set (move_sense::up);
            } else if (key == mplot::key::l) {
                this->stop();
                this->move_state.set (move_sense::down);
            } else if (key == mplot::key::up) {
                this->stop();
                this->move_state.set (move_sense::rotUp);
            } else if (key == mplot::key::down) {
                this->stop();
                this->move_state.set (move_sense::rotDown);
            } else if (key == mplot::key::left) {
                this->stop();
                this->move_state.set (move_sense::rotLeft);
            } else if (key == mplot::key::right) {
                this->stop();
                this->move_state.set (move_sense::rotRight);
            } else if (key == mplot::key::comma) {
                this->stop();
                this->move_state.set (move_sense::rotRollLeft);
            } else if (key == mplot::key::period) {
                this->stop();
                this->move_state.set (move_sense::rotRollRight);
            } else if (key == mplot::key::end) {
                this->speed = this->speed * 0.5f;
                this->angularSpeed = this->angularSpeed * 0.5f;
                std::cout << "Speed reduced to " << this->speed << ", angularSpeed to "
                          << this->angularSpeed * mc::rad2deg  << "deg/s" << std::endl;
            } else if (key == mplot::key::home) {
                this->speed = this->speed * 2.0f;
                this->angularSpeed = this->angularSpeed * 2.0f;
                std::cout << "Speed increased to " << this->speed  << ", angularSpeed to "
                          << this->angularSpeed * mc::rad2deg << "deg/s" << std::endl;
            } else if (key == mplot::key::r) {
                this->stop();
                this->vstate.set (state::campose_reset_request);
            }
        }

        if (action == mplot::keyaction::press) {
            if (key == mplot::key::t) {
                // Toggle the morph view
                this->vstate.flip (state::show_cones);
            } else if (key == mplot::key::c) {
                this->vstate.flip (state::show_camframe);
            } else if (key == mplot::key::i) {
                // Increase manual disc size
                if (this->manual_cone_length < 0.0f) {
                    this->manual_cone_length = 0.05f;
                } else {
                    this->manual_cone_length *= 2.0f;
                }
            } else if (key == mplot::key::o) {
                // Decrease manual disc sizne
                if (this->manual_cone_length >= 0.0f) {
                    this->manual_cone_length *= 0.5f;
                }

            } else if (key == mplot::key::r) {
                this->stop();
                this->vstate.set (state::campose_reset_request);

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
            }
        }
    }
};
