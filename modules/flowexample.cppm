module;

#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <unistd.h>

export module flowexample;

import sm.grid;
import sm.vec;
import sm.vvec;
import sm.mathconst;

import mplot.loadpng;

import flowprocessordyn;
import flowprocessor;
import flowconfig;

import mplot.visual;
import mplot.gridvisual;
import mplot.colourmap;
import mplot.hsvwheelvisual;
import mplot.vectorvisual;
import mplot.keys;

export namespace flowexample
{
    constexpr int gl_version = mplot::gl::version_4_6;

    constexpr int img_w = 1920;
    constexpr int img_h = 1080;
    constexpr int img_sz = img_w * img_h;

    /**
     * \brief A struct for optic flow output, in visualizable form
     *
     */
    struct flow_output
    {
        sm::vvec<sm::vec<float>> fmain;
        sm::vvec<sm::vec<float>> pool1;
        sm::vvec<sm::vec<float>> pool2;
        sm::vvec<sm::vec<float>> pool3;

        static constexpr sm::vec<float, 3> nullvec = {0.0f, 0.0f, 0.0f};

        flow_output (const perception::FlowProcessorDyn<float>& flow_2d)
        {
            fmain.resize(flow_2d.img_sz, nullvec);
            pool1.resize(flow_2d.pool1_sz, nullvec);
            pool2.resize(flow_2d.pool2_sz, nullvec);
            pool3.resize(flow_2d.pool3_sz, nullvec);
        }

        flow_output()
        {
            fmain.resize(perception::FlowProcessor<img_w, img_h, float>::img_sz, nullvec);
            pool1.resize(perception::FlowProcessor<img_w, img_h, float>::pool1_sz, nullvec);
            pool2.resize(perception::FlowProcessor<img_w, img_h, float>::pool2_sz, nullvec);
            pool3.resize(perception::FlowProcessor<img_w, img_h, float>::pool3_sz, nullvec);
        }

        void copy_in (perception::FlowProcessorDyn<float>& flow_2d)
        {
            // fmain
            for (int i = 0; i < flow_2d.img_sz; ++i) {
                this->fmain[i][0] = flow_2d.out[i][0];
                this->fmain[i][1] = flow_2d.out[i][1];
            }
            // pool1
            for (int i = 0; i < flow_2d.pool1_sz; ++i) {
                this->pool1[i][0] = flow_2d.pool1[i][0];
                this->pool1[i][1] = flow_2d.pool1[i][1];
            }
            // pool2
            for (int i = 0; i < flow_2d.pool2_sz; ++i) {
                this->pool2[i][0] = flow_2d.pool2[i][0];
                this->pool2[i][1] = flow_2d.pool2[i][1];
            }
            // pool3
            for (int i = 0; i < flow_2d.pool3_sz; ++i) {
                this->pool3[i][0] = flow_2d.pool3[i][0];
                this->pool3[i][1] = flow_2d.pool3[i][1];
            }
        }

        // Helper to copy array of vecs to vvec of vecs (for viz)
        void copy_in (perception::FlowProcessor<img_w, img_h, float>& flow_2d)
        {
            // fmain
            for (size_t i = 0; i < flow_2d.img_sz; ++i) {
                this->fmain[i][0] = flow_2d.out[i][0];
                this->fmain[i][1] = flow_2d.out[i][1];
            }
            // pool1
            for (size_t i = 0; i < flow_2d.pool1_sz; ++i) {
                this->pool1[i][0] = flow_2d.pool1[i][0];
                this->pool1[i][1] = flow_2d.pool1[i][1];
            }
            // pool2
            for (size_t i = 0; i < flow_2d.pool2_sz; ++i) {
                this->pool2[i][0] = flow_2d.pool2[i][0];
                this->pool2[i][1] = flow_2d.pool2[i][1];
            }
            // pool3
            for (size_t i = 0; i < flow_2d.pool3_sz; ++i) {
                this->pool3[i][0] = flow_2d.pool3[i][0];
                this->pool3[i][1] = flow_2d.pool3[i][1];
            }
        }

        // Helper to copy array of vecs to vvec of vecs (for viz)
        void copy_in (perception::FlowProcessor<img_w, img_h, float>* flow_2d)
        {
            // fmain
            for (size_t i = 0; i < flow_2d->img_sz; ++i) {
                this->fmain[i][0] = flow_2d->out[i][0];
                this->fmain[i][1] = flow_2d->out[i][1];
            }
            // pool1
            for (size_t i = 0; i < flow_2d->pool1_sz; ++i) {
                this->pool1[i][0] = flow_2d->pool1[i][0];
                this->pool1[i][1] = flow_2d->pool1[i][1];
            }
            // pool2
            for (size_t i = 0; i < flow_2d->pool2_sz; ++i) {
                this->pool2[i][0] = flow_2d->pool2[i][0];
                this->pool2[i][1] = flow_2d->pool2[i][1];
            }
            // pool3
            for (size_t i = 0; i < flow_2d->pool3_sz; ++i) {
                this->pool3[i][0] = flow_2d->pool3[i][0];
                this->pool3[i][1] = flow_2d->pool3[i][1];
            }
        }
    };

    /**
     * \brief A struct containing pointers to Visual elements.
     * This is a return type for create_vis_set()
     *
     */
    struct vis_set
    {
        mplot::GridVisual<float, int, float, gl_version> * fmain; // main flow
        mplot::GridVisual<float, int, float, gl_version> * pool1; // pool1 flow
        mplot::GridVisual<float, int, float, gl_version> * pool2; // pool2 flow
        mplot::GridVisual<float, int, float, gl_version> * pool3; // pool3 flow
        mplot::VectorVisual<float, 3, gl_version> * globalflow;

        void update(flow_output & out)
        {
            // Update flow views
            this->fmain->updateData(&out.fmain);
            this->pool1->updateData(&out.pool1);
            this->pool2->updateData(&out.pool2);
            this->pool3->updateData(&out.pool3);
            this->globalflow->thevec = out.pool3.mean();
            //this->globalflow->thevec.renormalize();
            this->globalflow->thevec *= 10.0f;
            this->globalflow->reinit();
        }
    };

    /**
     * \brief A struct of Grids for this application, with set up in the constructor
     *
     */
    struct grid_set
    {
        std::unique_ptr<sm::grid<int>> cgbig;
        std::unique_ptr<sm::grid<int>> cgpool1;
        std::unique_ptr<sm::grid<int>> cgpool2;
        std::unique_ptr<sm::grid<int>> cgpool3;

        grid_set()
        {
            // Grid for flow vis
            sm::vec<float, 2> grid_spacing = {0.02f, 0.02f};
            sm::vec<float, 2> grid_offset = grid_spacing * -0.5f;
            this->cgbig = std::make_unique<sm::grid<int>>(
                img_w, img_h, grid_spacing, grid_offset,
                sm::griddomainwrap::horizontal);
            if (this->cgbig->n() != img_w * img_h) {throw std::runtime_error("Grid wrong size");}

            // Grid for Pool 1
            sm::vec<float,
                   2> grid_spacing_pool = grid_spacing * perception::FlowProcessor<img_w, img_h, float>::pooldiv;
            grid_offset = grid_spacing_pool * -0.5f;
            this->cgpool1 = std::make_unique<sm::grid<int>>(
                perception::FlowProcessor<img_w, img_h, float>::pool1_w,
                perception::FlowProcessor<img_w, img_h, float>::pool1_h,
                grid_spacing_pool, grid_offset);
            if (this->cgpool1->n() != perception::FlowProcessor<img_w, img_h, float>::pool1_sz) {
                throw std::runtime_error("Pool1 Grid wrong size");
            }
            // Pool 2
            grid_spacing_pool *= (perception::FlowProcessor<img_w, img_h, float>::pooldiv / 2.5f);
            grid_offset = grid_spacing_pool * -0.5f;
            this->cgpool2 = std::make_unique<sm::grid<int>>(
                perception::FlowProcessor<img_w, img_h, float>::pool2_w,
                perception::FlowProcessor<img_w, img_h, float>::pool2_h,
                grid_spacing_pool, grid_offset);
            if (this->cgpool2->n() != perception::FlowProcessor<img_w, img_h, float>::pool2_sz) {
                throw std::runtime_error("Pool2 Grid wrong size");
            }

            // Pool 3
            grid_spacing_pool *= perception::FlowProcessor<img_w, img_h, float>::pooldiv;
            grid_offset = grid_spacing_pool * -0.5f;
            this->cgpool3 = std::make_unique<sm::grid<int>>(
                perception::FlowProcessor<img_w, img_h, float>::pool3_w,
                perception::FlowProcessor<img_w, img_h, float>::pool3_h,
                grid_spacing_pool, grid_offset);
            if (this->cgpool3->n() != perception::FlowProcessor<img_w, img_h, float>::pool3_sz) {
                throw std::runtime_error("Pool3 Grid wrong size");
            }
        }
    };


    /**
     * \brief Helper to set up a set of visual elements, so I can have two columns for two flow processors and
     * compare them.
     *
     */
    vis_set create_vis_set(
        sm::vec<float> offset, mplot::Visual<gl_version> & v,
        grid_set & cgs, flow_output & output, int set_index,
        const std::string & description)
    {
        auto vismode = mplot::GridVisMode::Pixels;
        vis_set vs;
        auto flow2dfv1 = std::make_unique<mplot::GridVisual<float, int, float, gl_version>>(
            cgs.cgbig.get(), offset);
        flow2dfv1->set_parent (v.get_id());
        flow2dfv1->showborder(true);
        flow2dfv1->gridVisMode = vismode;
        flow2dfv1->cm.setType(mplot::ColourMapType::HSV);  // Using HSV wheel to show direction
        flow2dfv1->setVectorData(&output.fmain);  // init with luminance
        // Have to set two colour scales for HSV (and Duochrome as well)
        flow2dfv1->colourScale.set_params(1, 0.5);  // 10, 0.5?
        flow2dfv1->colourScale2.set_params(1, 0.5);  // 10, 0.5?
        //flow2dfv1->zScale.set_params (0.1, 0); // To add relief
        flow2dfv1->zScale.set_params(0, 0);
        flow2dfv1->addLabel(
            std::string("flow ") + std::to_string(set_index) + std::string(
                set_index == 2 ? " (edges)" : ""),
            sm::vec<float, 3>({0.0f, -0.13f, 0.0f}),
            mplot::TextFeatures(0.1f, 48));
        flow2dfv1->addLabel(
            description,
            sm::vec<float, 3>({0.0f, 1.4f, 0.0f}),
            mplot::TextFeatures(0.14f, 48));
        flow2dfv1->finalize();
        vs.fmain = v.addVisualModel(flow2dfv1);

        offset += sm::vec<float>({0.0f, -1.8f, 0.0f});
        auto pool1fv = std::make_unique<mplot::GridVisual<float, int, float, gl_version>>(
            cgs.cgpool1.get(), offset);
        pool1fv->set_parent (v.get_id());
        pool1fv->showborder(true);
        pool1fv->border_thickness_fixed(true);
        pool1fv->border_thickness = 0.01f;
        pool1fv->cm.setType(mplot::ColourMapType::HSV);  // Using HSV wheel to show direction
        pool1fv->setVectorData(&output.pool1);
        pool1fv->colourScale.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool1fv->colourScale2.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool1fv->zScale.set_params(0.0f, 0.0f);
        sm::vec<float> pixel_offset = cgs.cgpool1->get_dx().plus_one_dim();
        pool1fv->addLabel(
            std::string("flow pool1"), sm::vec<float, 3>(
                {0.0f, -0.13f,
                 0.0f}) - pixel_offset, mplot::TextFeatures(
                     0.1f));
        pool1fv->finalize();
        vs.pool1 = v.addVisualModel(pool1fv);

        offset += sm::vec<float>({0.0f, -0.75f, 0.0f});
        auto pool2fv = std::make_unique<mplot::GridVisual<float, int, float, gl_version>>(
            cgs.cgpool2.get(), offset);
        pool2fv->set_parent (v.get_id());
        pool2fv->showborder(true);
        pool2fv->border_thickness_fixed(true);
        pool2fv->border_thickness = 0.01f;
        pool2fv->cm.setType(mplot::ColourMapType::HSV);  // Using HSV wheel to show direction
        pool2fv->setVectorData(&output.pool2);
        pool2fv->colourScale.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool2fv->colourScale2.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool2fv->zScale.set_params(0.0f, 0.0f);
        pixel_offset = cgs.cgpool2->get_dx().plus_one_dim();
        pool2fv->addLabel(
            std::string("flow pool2"), sm::vec<float, 3>(
                {0.0f, -0.13f,
                 0.0f}) - pixel_offset, mplot::TextFeatures(
                     0.1f));
        pool2fv->finalize();
        vs.pool2 = v.addVisualModel(pool2fv);

        offset += sm::vec<float>({1.05f * cgs.cgpool2->width_of_pixels(), 0.0f, 0.0f});
        auto pool3fv = std::make_unique<mplot::GridVisual<float, int, float, gl_version>>(
            cgs.cgpool3.get(), offset);
        pool3fv->set_parent (v.get_id());
        pool3fv->showborder(true);
        pool3fv->border_thickness_fixed(true);
        pool3fv->border_thickness = 0.01f;
        pool3fv->cm.setType(mplot::ColourMapType::HSV);  // Using HSV wheel to show direction
        pool3fv->setVectorData(&output.pool3);
        pool3fv->colourScale.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool3fv->colourScale2.set_params(1.0f, 0.5f);  // 10, 0.5?
        pool3fv->zScale.set_params(0.0f, 0.0f);
        pixel_offset = cgs.cgpool3->get_dx().plus_one_dim();
        pool3fv->addLabel(
            std::string("flow pool3"), sm::vec<float, 3>(
                {0.0f, -0.13f,
                 0.0f}) - pixel_offset, mplot::TextFeatures(
                     0.1f));
        pool3fv->finalize();
        vs.pool3 = v.addVisualModel(pool3fv);

        // A global flow visual
        offset += sm::vec<float>({1.05f * cgs.cgpool3->width_of_pixels(), 0.2f, 0.0f});
        auto vvm = std::make_unique<mplot::VectorVisual<float, 3, gl_version>>(offset);
        vvm->set_parent (v.get_id());
        vvm->scale_factor = 0.6f;
        vvm->thickness = 0.04f;
        vvm->addLabel(
            std::string("global"), sm::vec<float, 3>({-0.4f, -0.31f, 0.0f}), mplot::TextFeatures(
                0.1f));

        vvm->finalize();
        vs.globalflow = v.addVisualModel(vvm);

        return vs;
    }

    /**
     * \brief Extend mplot::Visual with some pause/resume/step controls
     *
     */
    struct FlowVisual final : public mplot::Visual<gl_version>
    {
        /// Boilerplate constructor calls Visual constructor
        FlowVisual(int width, int height, const std::string & title) : mplot::Visual<gl_version>(width, height, title)
        {
            this->addLabel ("0", {0.0f, -0.46f, 0.0f}, this->framenum_label);
        }
        /// If true, pause the flow processing
        bool paused = false;
        /// Time direction
        bool stepfwd = true;
        /// If paused, step one frame
        bool stepone = true;
        /// In turbo mode, do more frames per second
        bool turbo = false;
        /// The frame number
        mplot::VisualTextModel<gl_version>* framenum_label;
        int current_frame = 0;
        void framenum_label_update()
        {
            this->framenum_label->setupText (std::to_string (this->current_frame));
        }

    protected:
        /// Add our key event handling in this extension of mplot::Visual::key_callback_extra
        void key_callback_extra (int key, int scancode, int action, int mods) override
        {
            // get round compiler errors for unused scancode/mods args
            if constexpr (false) {std::cout << "scancode: " << scancode << " mods " << mods << std::endl;}
            // space key means toggle the 'paused' attribute
            if (key == mplot::key::space && action == mplot::keyaction::press) {
                this->paused = this->paused ? false : true;
            }
            // T key means toggle turbo
            if (key == mplot::key::t && action == mplot::keyaction::press) {
                this->turbo = this->turbo ? false : true;
                if (this->turbo) { std::cout << "TURBO!" << std::endl; }
            }
            // Right arrow key to step forward
            if (key == mplot::key::right && action == mplot::keyaction::press) {
                this->stepfwd = true;
                this->stepone = true;
            }
            if (key == mplot::key::left && action == mplot::keyaction::press) {
                this->stepfwd = false;
                this->stepone = true;
            }
            // Add app-specific help output:
            if (key == mplot::key::h && action == mplot::keyaction::press) {
                std::cout << "FlowVisual-specific:\n";
                std::cout << "[space]: Toggle pause\n";
                std::cout << "[right]: If paused, step forward one timestep\n";
            }
        }
    };

} // namespace flowexample
