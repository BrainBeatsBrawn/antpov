/**
 * \file flow_vis.cpp
 * Process optic flow from example movie data and visualize it at 30 FPS (original movie pace)
 *
 * This program makes two sets of visualizations that can be compared side-by-side.
 *
 * Uses new FlowProcessor class
 *
 * \author Seb James
 * \date January 2024
 */

#include <sstream>
#include <vector>
#include <array>
#include <memory>
#include <iostream>
#include <format>

import flowprocessor;
import flowconfig;
import flowexample;

import sm.vec;
import sm.vvec;
import mplot.loadpng;
import mplot.visual;
import mplot.gridvisual;
import mplot.hexgridvisual;
import mplot.hsvwheelvisual;

int main(int argc, char ** argv)
{
    /*
     * Load Data
     */
    int first_png = 1;
    int last_png = 1000;

    if (argc > 1) {
        first_png = std::atoi (argv[1]);
        last_png = first_png + 1000;
    }
    if (argc > 2) {
        last_png = std::atoi (argv[2]);
    }
    int num_pngs = last_png - first_png + 1;

    // First load the images into memory. All of 'em should fit.
    std::cout << "Load images into memory..." << std::endl;
    std::vector<sm::vvec<sm::vec<float, 3>>> images(num_pngs);
    sm::vec<bool, 2> flip = {false, true};
    for (int idx = first_png; idx <= last_png; ++idx) {
        //std::string fpath = std::format ("./data/flow_vis/input/cropped/{:04d}.png", idx);
        std::string fpath = std::format ("./data/seville/Ant12R02_frames/{:05d}.png", idx);
        std::cout << "fpath: " << fpath << std::endl;
        images[idx - first_png].resize(flowexample::img_w * flowexample::img_h);
        mplot::loadpng<float, 3>(fpath, images[idx - first_png], flip);
    }

    // Now extract the channel that we'll present to the flow algorithm.
    std::vector<std::array<float, flowexample::img_sz>> flow_input(num_pngs);
    for (int i = 0; i < num_pngs; ++i) {
        for (int j = 0; j < flowexample::img_sz; ++j) {
            // Make the flow input the greyscale of the rgb from the image
            flow_input[i][j] = images[i][j].rgb_to_grey();
        }
    }

    /*
     * Set up flow processors
     */

    // A 2d flow object
    auto flow_2d_1 = std::make_unique<perception::FlowProcessor<flowexample::img_w, flowexample::img_h, float>>();

    // Two config objects. These are the default paths.
    std::string paramPath1 = "./data/flow_vis/FlowParameters.json";

    perception::FlowConfig<float> fc1(paramPath1, flow_2d_1.get());
    fc1.updateConfig();

    // Containers for flow output
    flowexample::flow_output out_1;

    // Pre-process the flow
    std::vector<flowexample::flow_output> all_flow (num_pngs);
    for (int idx = first_png; idx <= last_png; ++idx) {
        // Update the input for the flow algo and process one timestep
        if ((idx % 20) == 0) {
            std::cout << "Computing flow (" << idx << ")" << std::endl;
        }
        flow_2d_1->update_input (&flow_input[idx - first_png]);
        all_flow[idx - first_png].copy_in (flow_2d_1.get());
    }

    /*
     * Set up visualization with our FlowVisual
     */

    flowexample::FlowVisual v(1920, 1080, "Flow visuals");
    v.setSceneTrans(sm::vec<float, 3>({-0.122763f, -0.226266f, -12.3f}));
    // A vector for scene offsets
    sm::vec<float> offset = {0.0f, 0.0f, 0.0f};

    //mplot::Visual<flowexample::gl_version> v2(1920, 1080, "Camera");

    // Our set of grids for use in visualizations
    flowexample::grid_set cgs;

    // Visualize the flow
    auto vismode = mplot::GridVisMode::Triangles;
    auto flow2dfv1 = std::make_unique<mplot::GridVisual<float, int, float, flowexample::gl_version>>(cgs.cgbig.get(), offset);
    flow2dfv1->set_parent (v.get_id());
    flow2dfv1->showborder(true);
    flow2dfv1->border_thickness = 1.0f;
    flow2dfv1->gridVisMode = vismode;
    flow2dfv1->cm.setType(mplot::ColourMapType::HSV);  // Using HSV wheel to show direction
    flow2dfv1->setVectorData(&out_1.fmain);  // init with luminance
    // Have to set two colour scales for HSV (and Duochrome as well)
    flow2dfv1->colourScale.set_params(1, 0.5);  // 10, 0.5?
    flow2dfv1->colourScale2.set_params(1, 0.5);  // 10, 0.5?
    //flow2dfv1->zScale.set_params (0.1, 0); // To add relief
    flow2dfv1->zScale.set_params(0, 0);
    flow2dfv1->addLabel (std::string("flow"), sm::vec<float, 3>({0.0f, -0.13f, 0.0f}), mplot::TextFeatures(0.1f, 48));
    flow2dfv1->finalize();
    auto flowp = v.addVisualModel (flow2dfv1);

    // Visualise the original camera image
    offset = {-cgs.cgbig->width_of_pixels(), 0.0f, 0.0f};
    auto cgv =
    std::make_unique<mplot::GridVisual<float, int, float, flowexample::gl_version>>(cgs.cgbig.get(), offset);
    cgv->set_parent (v.get_id());
    cgv->gridVisMode = vismode;
    cgv->setVectorData(&images[0]);
    cgv->cm.setType(mplot::ColourMapType::RGB);
    cgv->zScale.set_params(0.0f, 0.0f);
    cgv->addLabel(std::string("Camera"), sm::vec<float, 3>({0.0f, -0.13f, 0.0f}), mplot::TextFeatures(0.1f, 48));
    cgv->finalize();
    auto cgvp = v.addVisualModel(cgv); // or v2

    constexpr bool show_dirn_wheel = false;
    if constexpr (show_dirn_wheel) {
        // Place a direction wheel key next to the original image
        sm::vec<float> woffset = {-4.0f - cgs.cgbig->width_of_pixels(), 2.6f, 0.0f};
        auto hsvw_vis = std::make_unique<mplot::HSVWheelVisual<float, flowexample::gl_version>>(woffset);
        hsvw_vis->set_parent (v.get_id());
        hsvw_vis->radius = 0.35f;
        hsvw_vis->labels = {"Flow up", "Left", "Flow down", "Right"};
        hsvw_vis->tf.fontsize = 0.1f;
        hsvw_vis->twodimensional (false);
        hsvw_vis->cm = flowp->cm;
        hsvw_vis->finalize();
        v.addVisualModel(hsvw_vis);
    }

    /*
     * Loop, refresh visualization
     */

    int idx = first_png;
    while (!v.readyToFinish()) {

        // Handle being paused:
        while (v.paused && !v.stepone && !v.readyToFinish()) {
            v.wait (v.turbo ? 0.00001 : 0.03);
        }
        v.stepone = false;
        v.wait (v.turbo ? 0.00001 : 0.03);

        v.current_frame = idx;
        v.framenum_label_update();

        // Update flow views
        flowp->vectorData = &(all_flow[idx - first_png].fmain);
        flowp->reinitColours();

        // Update camera view
        cgvp->vectorData = &images[idx - first_png];
        cgvp->reinitColours();

        v.render();

        if (v.stepfwd) {
            ++idx;
            if (idx > last_png) { idx = first_png; }
        } else {
            --idx;
            if (idx < first_png) { idx = last_png; }
        }
    }

    return 0;
}
