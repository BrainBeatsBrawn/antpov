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
#include <chrono>
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
    using namespace std::chrono;
    using sc = std::chrono::steady_clock;

    /*
     * Load Data
     */

    // First load the images into memory. All of 'em should fit.
    std::cout << "Load images into memory..." << std::endl;
    std::vector<sm::vvec<sm::vec<float, 3>>> images(flowexample::num_pngs);
    sm::vec<bool, 2> flip = {false, true};
    for (int idx = flowexample::first_png; idx <= flowexample::last_png; ++idx) {
        std::string fpath = std::format ("./data/flow_vis/input/cropped/{:04d}.png", idx);
        //std::string fpath = std::format ("./data/seville/Ant12R02_frames/{:05d}.png", idx);
        std::cout << "fpath: " << fpath << std::endl;
        images[idx - flowexample::first_png].resize(flowexample::img_w * flowexample::img_h);
        mplot::loadpng<float, 3>(fpath, images[idx - flowexample::first_png], flip);
    }

    // Now extract the channel that we'll present to the flow algorithm.
    std::vector<std::array<float, flowexample::img_sz>> flow_input(flowexample::num_pngs);
    for (int i = 0; i < flowexample::num_pngs; ++i) {
        for (int j = 0; j < flowexample::img_sz; ++j) {
            // Make the flow input the greyscale of the rgb from the image
            flow_input[i][j] = images[i][j].rgb_to_grey();
        }
    }

    /*
     * Set up flow processors
     */

    // Two 2d flow objects
    perception::FlowProcessor<flowexample::img_w, flowexample::img_h, float> flow_2d_1;
    perception::FlowProcessor<flowexample::img_w, flowexample::img_h, float> flow_2d_2;

    // Two config objects. These are the default paths.
    std::string paramPath1 = "./data/flow_vis/FlowParameters1.json";
    std::string paramPath2 = "./data/flow_vis/FlowParameters2.json";
    // Optionally use cmd line args to set paths
    if (argc > 2) {
        paramPath1 = std::string(argv[1]);
        paramPath2 = std::string(argv[2]);
    }
    perception::FlowConfig<float> fc1(paramPath1, &flow_2d_1);
    perception::FlowConfig<float> fc2(paramPath2, &flow_2d_2);
    fc1.updateConfig();
    fc2.updateConfig();

    // Containers for flow output
    flowexample::flow_output out_1;
    flowexample::flow_output out_2;

    /*
     * Set up visualization with our FlowVisual
     */

    flowexample::FlowVisual v(1860, 1080, "Flow visuals");
    v.setSceneTrans(sm::vec<float, 3>({-0.122763f, -0.226266f, -12.3f}));
    // A vector for scene offsets
    sm::vec<float> offset = {0.0f, 0.0f, 0.0f};

    // Our set of grids for use in visualizations
    flowexample::grid_set cgs;

    // A set of flow visualizations for 'flow 1'
    offset = {-5.2f, 0.0f, 0.0f};
    flowexample::vis_set vs1 = flowexample::create_vis_set(offset, v, cgs, out_1, 1, fc1.description);

    // A set of flow visualizations for 'flow 2'
    offset = {0.2f, 0.0f, 0.0f};
    flowexample::vis_set vs2 = flowexample::create_vis_set(offset, v, cgs, out_2, 2, fc2.description);

    // Visualise the original image (just above flow)
    offset = {-2.5f, 2.0f, 0.0f};
    auto cgv =
    std::make_unique<mplot::GridVisual<float, int, float, flowexample::gl_version>>(cgs.cgbig.get(), offset);
    cgv->set_parent (v.get_id());
    cgv->setVectorData(&images[0]);
    cgv->cm.setType(mplot::ColourMapType::RGB);
    cgv->zScale.set_params(0.0f, 0.0f);
    cgv->addLabel(std::string("Camera"), sm::vec<float, 3>({0.0f, -0.13f, 0.0f}), mplot::TextFeatures(0.1f, 48));
    cgv->finalize();
    auto cgvp = v.addVisualModel(cgv);

    // Place a direction wheel key next to the original image
    sm::vec<float> woffset = {-4.0f, 2.6f, 0.0f};
    auto hsvw_vis = std::make_unique<mplot::HSVWheelVisual<float, flowexample::gl_version>>(woffset);
    hsvw_vis->set_parent (v.get_id());
    hsvw_vis->radius = 0.35f;
    hsvw_vis->labels = {"Flow up", "Left", "Flow down", "Right"};
    hsvw_vis->tf.fontsize = 0.1f;
    hsvw_vis->twodimensional (false);
    hsvw_vis->cm = vs1.fmain->cm;
    hsvw_vis->finalize();
    v.addVisualModel(hsvw_vis);

    /*
     * Loop, update flow, refresh visualization
     */

    int frame = 0;
    while (!v.readyToFinish()) {

        sc::time_point t0 = sc::now();
        for (int idx = flowexample::first_png; idx <= flowexample::last_png && !v.readyToFinish(); ++idx) {

            // Handle being paused:
            while (v.paused && !v.stepfwd && !v.readyToFinish()) {v.wait(v.turbo ? 0.00001 : 0.03);}
            v.stepfwd = false;

            // Every 30 frames, update the configs
            if (idx % 30 == 0) {
                fc1.updateConfig();
                fc2.updateConfig();
            }

            // Update the input for the flow algo and process one timestep
            flow_2d_1.update_input(&flow_input[idx - flowexample::first_png]);
            flow_2d_2.update_input(&flow_input[idx - flowexample::first_png]);

            // Copy data into our Visual-friendly data structures
            out_1.copy_in(flow_2d_1);
            out_2.copy_in(flow_2d_2);

            // Update flow views
            vs1.update(out_1);
            vs2.update(out_2);

            // Update camera view
            cgvp->updateData(&images[idx - flowexample::first_png]);

            v.render();

            constexpr bool make_movie = false;
            if constexpr (make_movie) {
                std::stringstream ff1;
                ff1 << "flow_vis_" << std::setw(5) << std::setfill('0') << frame++ << ".png";
                v.saveImage(ff1.str());
                v.poll();
            } else {
                v.wait(v.turbo ? 0.00001 : 0.03);
            }
        }
        sc::time_point t1 = sc::now();
        if (!v.readyToFinish()) {
            sc::duration t_d = t1 - t0;
            auto us = duration_cast<microseconds>(t_d).count() / flowexample::num_pngs;
            double fps = 1.0 / (us / 1000000.0);
            std::cout << "(Warning: includes pauses!) Each frame processed in " << us << " us (" << fps <<
            " FPS)\n";
        }
    }

    return 0;
}
