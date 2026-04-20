/**
 * \file perception::FlowProcessor module
 *
 * The Opteran Optic Flow computation
 *
 * \author Seb James
 * \date Jan 2024, modularized Apr 2026
 *
 * Copyright Opteran
 */

module;

#include <cmath>
#include <algorithm>
#include <array>
#include <stdexcept>

export module flowprocessor;

export import sm.vec;
export import sm.mathconst;

import flowprocessorbase;

export namespace perception
{

    /**
     * \brief A class to implement a 2D flow algorithm
     *
     * This builds on flow.hpp and flow2d.hpp from Seb James' Opteran/research_code repository. This
     * is to become the reference Opteran flow implementation.
     *
     * This implementation acts on any rectangle of input, for example, this could be a cylindrical
     * projection to compute 2D flow in that band. It could also work directly with the Fourpi
     * projection (which is tiled with rectangular portions), or a hexagonal version could work with
     * a geodesic ployhedron:
     * https://stackoverflow.com/questions/46777626/mathematically-producing-sphere-shaped-hexagonal-grid
     *
     * One idea implemented in this class is the ability to average the flow between 2 *or*4 pairs
     * of detectors (which are pixels, here). On a Cartesian grid, we have the left-right pair, the
     * up-down pair, the upleft-downright pair and the downleft-upright pair. (On a hexagonal grid,
     * we would have the red axis pair, the green axis pair and the blue axis pair, but this class
     * is written for a Cartesian grid of pixels).
     *
     * \tparam img_w width of 'camera' input
     * \tparam img_h height of 'camera' input
     * \tparam F The floating point type for the data
     */
    template<int img_w = 256, int img_h = 64, typename F = float>
    struct FlowProcessor : public perception::FlowProcessorBase<F>
    {
        /// \name Compile time computations of image size etc
        ///@{
        static constexpr int img_sz = img_w * img_h;
        static constexpr int img_w_x2 = 2 * img_w;
        static constexpr int img_w_x3 = 3 * img_w;

        // What's our division factor (along one dimension)?
        static constexpr int pooldiv = 4;
        static constexpr int pooldiv_sq = pooldiv * pooldiv;
        // First pooling width
        static constexpr int pool1_w = img_w / pooldiv;
        static constexpr int pool1_h = img_h / pooldiv;
        static constexpr int pool1_sz = pool1_w * pool1_h;

        static constexpr int pool2_w = pool1_w / pooldiv;
        static constexpr int pool2_h = pool1_h / pooldiv;
        static constexpr int pool2_sz = pool2_w * pool2_h;

        static constexpr int pool3_w = pool2_w / pooldiv;
        static constexpr int pool3_h = pool2_h / pooldiv;
        static constexpr int pool3_sz = pool3_w * pool3_h;
        ///@}

        /**
         * \brief Current input (memory for this is external)
         *
         * Input to be externally managed memory, but a copy has to be kept if compression is applied
         */
        std::array<F, img_sz> * in = nullptr;

        /**
         * \brief Input from last timestep
         *
         * This array holds a copy of the input from the previous timestep.
         */
        std::array<F, img_sz> prev_in;

        /**
         * \brief Input from last-but-one timestep
         *
         * This array holds a copy of the input from two timesteps back in history.
         */
        std::array<F, img_sz> prev2_in;

        /**
         * \name smooth_input (adaptiveLIN) state variables
         */
        ///@{
        std::array<F, img_sz> si_a;
        std::array<F, img_sz> si_x;
        std::array<F, img_sz> si_xsq;
        ///@}

        /**
         * \brief Holds signal after compression/squashing
         *
         * To receive signal after compression/squashing. Input will be either FlowProcessor#in
         * (which is external) or FlowProcessor#si_a (which is internal), but neither should be
         * altered by the compression function and hence we have to store compression output in this
         * variable.
         */
        std::array<F, img_sz> cmpr;

        /**
         * \name Edges for the 4 detector pairs
         *
         * Each detector pair detects edges for the angle orthogonal to the pair
         */
        ///@{
        std::array<F, img_sz> edges_1;
        std::array<F, img_sz> edges_2;
        std::array<F, img_sz> edges_3;
        std::array<F, img_sz> edges_4;
        ///@}

        /**
         * \name Photoreceptor state variables.
         *
         * There are two leaky integrators for each pixel/ommatidium. With different time constants,
         * these act as movement detectors. Their difference, which is fed into the nodelay
         * population, is a kind of movement based edge detector. The choice of values should match
         * the rate of movement that the agent makes.
         */
        ///@{
        std::array<F, img_sz> pr_a;
        std::array<F, img_sz> pr_b;
        ///@}

        /**
         * \name Delay filter state variables
         *
         * In edge mode, we need additional dly_* states, hence dly_f2, dly_s2, etc.
         */
        ///@{
        std::array<F, img_sz> dly_f;
        std::array<F, img_sz> dly_s;
        std::array<F, img_sz> dly_f2;
        std::array<F, img_sz> dly_s2;
        std::array<F, img_sz> dly_f3;
        std::array<F, img_sz> dly_s3;
        std::array<F, img_sz> dly_f4;
        std::array<F, img_sz> dly_s4;
        ///@}

        /**
         * \name The 'nodelay' shiftdown output (not state; computed on each loop)
         *
         * As for dly_f/s, we need additional instances when computing in edge mode.
         */
        ///@{
        std::array<F, img_sz> nodelay;
        std::array<F, img_sz> nodelay2;
        std::array<F, img_sz> nodelay3;
        std::array<F, img_sz> nodelay4;
        ///@}

        /**
         * \brief Flow Components
         *
         * fast_prog_N/slow_prog_N are the 'flow components'
         *
         * It makes performance sense to work with these as individual array<F, img_sz>, rather than
         * a single array<FlowCmpts, img_sz> (with FlowCmpts containing 16 Fs) because each one is
         * computed in a separate loop and this ensures that as much of the data will make it into
         * low level cache as possible.
         *
         * These can be considered as outputs. fast/slow_prog receives the result of multiplying
         * shft_dly_f/s signal with nodelay. (not state; computed on each loop)
         */
        ///@{
        std::array<F, img_sz> fast_prog_1;
        std::array<F, img_sz> slow_prog_1;
        std::array<F, img_sz> fast_reg_1;
        std::array<F, img_sz> slow_reg_1;

        std::array<F, img_sz> fast_prog_2;
        std::array<F, img_sz> slow_prog_2;
        std::array<F, img_sz> fast_reg_2;
        std::array<F, img_sz> slow_reg_2;

        std::array<F, img_sz> fast_prog_3;
        std::array<F, img_sz> slow_prog_3;
        std::array<F, img_sz> fast_reg_3;
        std::array<F, img_sz> slow_reg_3;

        std::array<F, img_sz> fast_prog_4;
        std::array<F, img_sz> slow_prog_4;
        std::array<F, img_sz> fast_reg_4;
        std::array<F, img_sz> slow_reg_4;
        ///@}

        /**
         * \name Output attributes.
         *
         * Flow is output as vec objects, which are mathematical vectors. 2 dimensions in this
         * case. This highest resolution flow can also be averaged down, or 'pooled' to give coarser
         * estimates of the flow.
         */
        ///@{

        /**
         * \brief Main flow output (per-pixel flow vector(s)).
         *
         * An std::array of sm::vec is beautiful because it has meaning (an array of mathematical 2D
         * vectors)
         *
         * This memory could be/will be a pointer to external memory.
         */
        std::array<sm::vec<F, 2>, img_sz> out;

        /**
         * \brief Output, first pooling (4 pixels->1)
         *
         * A design suggestion was to output something like std::array<perception::Flow<F>, img_sz>,
         * where Flow<F> would contain flow at several lengthscales. I think a better design is to
         * have many arrays of vecs rather than a single array of things that contain vecs (or just
         * vector components). The reason for this is that during a loop that accesses just one
         * element of the output, there would be long jumps in memory between elements. This tends to
         * result in slow code.
         *
         * Thus we might have here:
         * \code
         * std::array<sm::vec<F, 2>, img_sz> out_lengthscale2; // optionally computed from detectors 2 pixels apart, not 1
         * int lengthscale3 = 4; // lengthscale could potentially be runtime configured
         * std::array<sm::vec<F, 2>, img_sz> out_lengthscale3; // computed from detectors that are N pixels apart, not 1
         * \endcode
         *
         * These could be optional, and if disabled, the FlowProcessor would not be slowed down at all.
         *
         * I already have the 'pooled' output, which gives lower resolution flow derived from the
         * finest lengthscale output:
         */
        std::array<sm::vec<F, 2>, pool1_sz> pool1;

        /**
         * \brief Second pooling (4x4 pixels -> 1)
         */
        std::array<sm::vec<F, 2>, pool2_sz> pool2;
        /**
         * \brief Third pooling (4x4x4 pixels -> 1)
         */
        std::array<sm::vec<F, 2>, pool3_sz> pool3;

        /**
         * \name subtractions
         */
        ///@{
        std::array<sm::vec<F, 2>, img_sz> out_minus_pool1;
        std::array<sm::vec<F, 2>, img_sz> out_minus_pool2;
        std::array<sm::vec<F, 2>, img_sz> out_minus_pool3;
        ///@}

        ///@}

        /**
         * \brief A default constructor with nothing special to do
         */
        FlowProcessor() {}

        /**
         * \name Disable copy and move
         *
         * I disable copy and move for this class as it's probably not necessary or desirable to
         * copy/move your FlowProcessor
         */
        ///@{
        /// copy constructor is deleted
        FlowProcessor(const FlowProcessor<img_w, img_h, F> &)            = delete;
        /// copy assignment operator is deleted
        FlowProcessor & operator=(const FlowProcessor<img_w, img_h, F> &) = delete;
        /// move constructor is deleted
        FlowProcessor(FlowProcessor<img_w, img_h, F> &&)                 = delete;
        /// move assignment operator is deleted
        FlowProcessor & operator=(FlowProcessor<img_w, img_h, F> &&)      = delete;
        ///@}

        /**
         * \brief Update input and have it processed
         *
         * This to be the standard mechanism for adding input. Input data is managed
         * externally. FlowProcessor updates the output based on the input. The input is
         * pixels. External code will decide whether these are luminance pixels, or red or green or
         * whatever.
         */
        void update_input(std::array<float, img_sz> * _pixels_ptr)
        {
            // Update previous input states
            std::copy (this->prev_in.begin(), this->prev_in.end(), this->prev2_in.begin());
            if (this->in != nullptr) {
                std::copy (this->in->begin(), this->in->end(), this->prev_in.begin());
            }
            // Update the input pointer
            this->in = _pixels_ptr;
            // Process the new data
            this->process();
        }

        /**
         * \brief Run one timestep of the flow processing
         */
        void process()
        {
            if (this->in == nullptr) {
                throw std::runtime_error ("perception::FlowProcessor::process: input is nullptr");
            }

            // Smoothing and input compression will determine which array<> contains the signal that
            // is passed onto edge convolution/photoreceptors
            std::array<F, img_sz>* signal = nullptr;

            if (this->parameters.apply_smoothing == true) {
                this->smooth_input();
                if (this->parameters.apply_input_compression == true) {
                    this->compress_input (&this->si_a, &this->cmpr);
                    signal = &this->cmpr;
                } else {
                    signal = &this->si_a;
                }
            } else {
                if (this->parameters.apply_input_compression == true) {
                    this->compress_input (this->in, &this->cmpr);
                    signal = &this->cmpr;
                } else {
                    signal = this->in;
                }
            }

            // Zero the output variable before recomputing it again
            for (auto& el : this->out) { el = sm::vec<F, 2>({0, 0}); }

            // Switching between 'edge mode' and 'non edge mode' is a little awkward, as there are
            // many more data structure required in edge mode, as each detector orientation requires
            // a separate convolution. The resulting edges are then passed through four populations
            // of photoreceptors and delay filters, etc etc.
            if (this->parameters.apply_edge_detection == true) {

                // edges_1 are horizontal edges obtained from sensor pair 1
                if (this->parameters.sensors.test(0)) {
                    this->convolve_for_edges_sensor1 (signal);
                    if (this->parameters.apply_edge_compression == true) {
                        this->apply_edge_compression (&this->edges_1, signal);
                    }
                    this->photoreceptor_and_delayfilt (&this->edges_1, this->dly_s, this->dly_f, this->nodelay);
                    this->shift_for_edges1();
                    this->multiply_and_divide (fast_prog_1, slow_prog_1, fast_reg_1, slow_reg_1,
                                               this->dly_s, this->dly_f, this->nodelay, sm::mathconst<F>::pi_over_2);
                }
                // edges_2 are diagonal edges obtained from sensor pair 2
                if (this->parameters.sensors.test(1)) {
                    this->convolve_for_edges_sensor2 (signal);
                    if (this->parameters.apply_edge_compression == true) {
                        this->apply_edge_compression (&this->edges_2, signal);
                    }
                    this->photoreceptor_and_delayfilt (&this->edges_2, this->dly_s2, this->dly_f2, this->nodelay2);
                    this->shift_for_edges2();
                    this->multiply_and_divide (fast_prog_2, slow_prog_2, fast_reg_2, slow_reg_2,
                                               this->dly_s2, this->dly_f2, this->nodelay2, sm::mathconst<F>::three_pi_over_4);
                }
                // edges_3 are vertical edges obtained from sensor pair 3. Cyclic algorithm for this detector pair
                if (this->parameters.sensors.test(2)) {
                    this->convolve_for_edges_sensor3 (signal);
                    if (this->parameters.apply_edge_compression == true) {
                        this->apply_edge_compression (&this->edges_3, signal);
                    }
                    this->photoreceptor_and_delayfilt (&this->edges_3, this->dly_s3, this->dly_f3, this->nodelay3);
                    this->shift_for_edges3();
                    this->multiply_and_divide (fast_prog_3, slow_prog_3, fast_reg_3, slow_reg_3,
                                               this->dly_s3, this->dly_f3, this->nodelay3, sm::mathconst<F>::pi);
                }
                // edges_4 are diagonal edges obtained from sensor pair 4
                if (this->parameters.sensors.test(3)) {
                    this->convolve_for_edges_sensor4 (signal);
                    if (this->parameters.apply_edge_compression == true) {
                        this->apply_edge_compression (&this->edges_4, signal);
                    }
                    this->photoreceptor_and_delayfilt (&this->edges_4, this->dly_s4, this->dly_f4, this->nodelay4);
                    this->shift_for_edges4();
                    this->multiply_and_divide (fast_prog_4, slow_prog_4, fast_reg_4, slow_reg_4,
                                               this->dly_s4, this->dly_f4, this->nodelay4, sm::mathconst<F>::five_pi_over_4);
                }

            } else {
                this->photoreceptor_and_delayfilt (signal, this->dly_s, this->dly_f, this->nodelay);
                // All four shifts use the same dly_s/dly_f/nodelay data
                if (this->parameters.sensors.test(0)) { this->shift_for_pair1(); }
                if (this->parameters.sensors.test(1)) { this->shift_for_pair2(); }
                if (this->parameters.sensors.test(2)) { this->shift_for_pair3(); }
                if (this->parameters.sensors.test(3)) { this->shift_for_pair4(); }

                if (this->parameters.sensors.test(0)) {
                    this->multiply_and_divide (fast_prog_1, slow_prog_1, fast_reg_1, slow_reg_1,
                                               this->dly_s, this->dly_f, this->nodelay, sm::mathconst<F>::pi_over_2);
                }
                if (this->parameters.sensors.test(1)) {
                    this->multiply_and_divide (fast_prog_2, slow_prog_2, fast_reg_2, slow_reg_2,
                                               this->dly_s, this->dly_f, this->nodelay, sm::mathconst<F>::three_pi_over_4);
                }
                if (this->parameters.sensors.test(2)) {
                    this->multiply_and_divide (fast_prog_3, slow_prog_3, fast_reg_3, slow_reg_3,
                                               this->dly_s, this->dly_f, this->nodelay, sm::mathconst<F>::pi);
                }
                if (this->parameters.sensors.test(3)) {
                    this->multiply_and_divide (fast_prog_4, slow_prog_4, fast_reg_4, slow_reg_4,
                                               this->dly_s, this->dly_f, this->nodelay, sm::mathconst<F>::five_pi_over_4);
                }
            }

            if (this->parameters.apply_pooling == true) { this->pool(); }
        }

        /**
         * \brief Apply input compression
         *
         * Apply input compression to the given input, writing to out_ptr. in_ptr and out_ptr may be
         * the same.
         */
        void compress_input(std::array<F, img_sz> * in_ptr, std::array<F, img_sz> * out_ptr)
        {
            // *could* use a logistic function:
            // this->in.logistic_inplace (12, 0.3);
            // ...but tanh has good properties with a single parameter:
            std::transform (in_ptr->begin(), in_ptr->end(), out_ptr->begin(),
                            [this](F x){ return std::tanh(this->parameters.ic_tanhm * x);});
        }

        /**
         * \brief Input smoothing
         *
         * Apply the variance-modulated temporal smoothing to the input. The output of the smoothing
         * is held in FlowProcessor#si_a
         *
         * In SpineML, see 'smooth_input' population (adaptiveLIN component)
         */
        void smooth_input()
        {
            for (unsigned int i = 0; i < img_sz; ++i) {
                // Variance computation
                F freq = ( (((this->prev_in[i] - (*this->in)[i]) > F{0}) - F{0.5}) * (((this->prev2_in[i] - this->prev_in[i]) > F{0}) - F{0.5}) < F{0} )
                * (this->prev_in[i] - (*this->in)[i] != F{0} && this->prev2_in[i] - this->prev_in[i] != F{0});
                F sd = (this->si_xsq[i] - this->si_x[i] * this->si_x[i]) * this->parameters.si_sd_scale;
                // Update si_a, the output, which is a leaky integrator, modulated by input variance
                // (sd). Lots of (temporal) variance => add to si_a more slowly.
                // This handles camera joins (si_a[i] not changed if in[i] is exactly 0)
                this->si_a[i] += (*this->in)[i] ? ((*this->in)[i] - this->si_a[i]) / (F{1}+sd) : F{0};
                // alternative: this->si_a[i] += (in[i]-si_a[i]) / (F{1} + sd); // which does not handle camera joins
                // Update the state variables associated with variance
                this->si_x[i] += (freq - this->si_x[i]) * this->parameters.si_inv_tau;
                this->si_xsq[i] += (freq * freq - this->si_xsq[i]) * this->parameters.si_inv_tau;
            }
        }

        /// It's fiddly to compute edges around the edge. Can exclude them:
        static constexpr bool include_edge_pixels = false;

        /**
         * \brief Convolve to find horizontal edges from sensor pair 1
         * output is stored in array edges_1
         */
        void convolve_for_edges_sensor1(std::array<F, img_sz> * cnv_in)
        {
            // Majority of data:
            for (int i = 2 * img_w; i < (img_sz - 2 * img_w); ++i) {
                this->edges_1[i] = -(*cnv_in)[i - img_w_x2] - (*cnv_in)[i - img_w] + (*cnv_in)[i + img_w] + (*cnv_in)[i + img_w_x2];
            }

            if constexpr (include_edge_pixels) {
                // Bottom rows
                for (int c = 0; c < img_w; ++c) {
                    this->edges_1[c]         = (              + (*cnv_in)[c + img_w]    + (*cnv_in)[c + img_w_x2]) * F{0.5}; // Bottom
                    this->edges_1[c + img_w] = (-(*cnv_in)[c] + (*cnv_in)[c + img_w_x2] + (*cnv_in)[c + img_w_x3]) * F{0.75}; // Next up
                }
                // Top rows
                int r0 = img_w * (img_h-2);
                for (int c = 0; c < img_w; ++c) {
                    this->edges_1[r0 + c]         = (- (*cnv_in)[r0 + c - img_w_x2] - (*cnv_in)[r0 + c - img_w] + (*cnv_in)[c + img_w]) * F{0.75};
                    this->edges_1[r0 + c + img_w] = (- (*cnv_in)[r0 + c - img_w]    - (*cnv_in)[r0 + c]) * F{0.5};
                }
            }
        }

        /**
         * \brief Convolve to find edges for sensor pair 2
         *
         * edges_2 are diagonal edges obtained from sensor pair 2 (combination of 1 and 3)
         */
        void convolve_for_edges_sensor2(std::array<F, img_sz> * cnv_in)
        {
            // Main block of data (ignore 2 pixel border)
            for (int i = 2 * img_w; i < (img_sz - 2 * img_w); ++i) {
                this->edges_2[i] = -(*cnv_in)[i - img_w_x2 - 2] - (*cnv_in)[i - img_w - 1] + (*cnv_in)[i + img_w + 1] + (*cnv_in)[i + img_w_x2 + 2];
            }
            // Note: The two top and bottom rows are ignored
        }

        /**
         * \brief Convolve to find edges for sensor pair 3
         *
         * edges_3 are vertical edges obtained from sensor pair 3. Cyclic algorithm for this
         * detector pair.
         */
        void convolve_for_edges_sensor3(std::array<F, img_sz> * cnv_in)
        {
            if constexpr (include_edge_pixels) {
                // First two pixels
                this->edges_3[0]  = -(*cnv_in)[img_w-2] - (*cnv_in)[img_w-1] + (*cnv_in)[1] + (*cnv_in)[2];
                this->edges_3[1]  = -(*cnv_in)[img_w-1] - (*cnv_in)[0]       + (*cnv_in)[2] + (*cnv_in)[3];
            }
            // The main body of the data
            for (int i = 2; i < img_sz-2; ++i) {
                this->edges_3[i] = -(*cnv_in)[i-2] -(*cnv_in)[i-1] + (*cnv_in)[i+1] + (*cnv_in)[i+2];
            }
            if constexpr (include_edge_pixels) {
                // last two pixels
                this->edges_3[img_sz-1] = -(*cnv_in)[img_sz-3] - (*cnv_in)[img_sz-2] + (*cnv_in)[img_sz-img_w] + (*cnv_in)[img_sz-img_w+1];
                this->edges_3[img_sz-2] = -(*cnv_in)[img_sz-4] - (*cnv_in)[img_sz-3] + (*cnv_in)[img_sz-1]     + (*cnv_in)[img_sz-img_w];
            }
        }

        /**
         * \brief Convolve to find edges for sensor pair 4
         *
         * edges_4 are diagonal edges obtained from sensor pair 4
         */
        void convolve_for_edges_sensor4(std::array<F, img_sz> * cnv_in)
        {
            // Main block of data, ignoring 2 pixel border
            for (int i = 2 * img_w; i < (img_sz - 2 * img_w); ++i) {
                this->edges_4[i] = -(*cnv_in)[i - img_w_x2 + 2] - (*cnv_in)[i - img_w + 1] + (*cnv_in)[i + img_w - 1] + (*cnv_in)[i + img_w_x2 - 2];
            }
            // Note: The two top and bottom rows are ignored
        }

        /**
         * \brief Edge compression
         *
         * Compression of edges is part of 'edges mode', which is enabled if
         * FlowParameters#apply_edge_detection is true in FlowProcessor#parameters
         */
        void apply_edge_compression (std::array<F, img_sz> * edges_ptr, std::array<F, img_sz> * signal_ptr)
        {
            for (unsigned int i = 0; i < img_sz; ++i) {
                F tmp = this->parameters.eic_value * ((*edges_ptr)[i] > this->parameters.eic_threshold) - this->parameters.eic_value * ((*edges_ptr)[i] < -this->parameters.eic_threshold);
                (*edges_ptr)[i] = tmp + (*signal_ptr)[i] * this->parameters.eic_signal_feedin; // param is 0.1
            }
        }

        /**
         * \brief Photoreceptor and delay filter computation
         *
         * The photoreceptor neurons and the delay filtering are bundled up in a single for loop for
         * performance. Inputs and outputs via pointers
         */
        void photoreceptor_and_delayfilt (std::array<F, img_sz> * pr_in, std::array<F, img_sz> & _dly_s,
                                          std::array<F, img_sz> & _dly_f, std::array<F, img_sz> & _nodelay)
        {
            for (unsigned int i = 0; i < img_sz; ++i) {

                // Photoreceptor population
                this->pr_a[i] += ((*pr_in)[i] - this->pr_a[i]) * this->parameters.pr_a_inv_tau;
                this->pr_b[i] += ((*pr_in)[i] - this->pr_b[i]) * this->parameters.pr_b_inv_tau;

                // Delay filt
                F pr_out = pr_a[i] - pr_b[i];
                if (this->parameters.apply_pr_squashing) {
                    pr_out = (F{1} / (F{1} + std::exp (-this->parameters.pr_squash_m*pr_out))) - F{0.5};
                }

                _dly_f[i] += (pr_out - _dly_f[i]) * this->parameters.dly_f_inv_tau;
                _dly_s[i] += (pr_out - _dly_s[i]) * this->parameters.dly_s_inv_tau;

                // Carries out the 'on condition' f*s<0 in the integration.
                _dly_s[i] = _dly_s[i] * (_dly_s[i] * _dly_f[i] >= F{0});

                _nodelay[i] = pr_out * F{0.5};
            }
        }

        /**
         * \brief Signal shift for edges_1
         */
        void shift_for_edges1()
        {
            // Shift for detector pair 1 is shift up
            for (int r = 0; r < img_h - this->parameters.sensor_distance; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti1 = c_start + c + (this->parameters.sensor_distance * img_w);
                    // slow_prog_1 is really 'shft_dly_s_1' at this point in algo, but placing the
                    // shifted-delayed signal in slow_prog_1 avoids an additional state variable.
                    this->slow_prog_1[shfti1] = this->dly_s[c_start + c];
                    // Similar for fast_prog_1.
                    this->fast_prog_1[shfti1] = this->dly_f[c_start + c];
                    // This would otherwise be an additional state variable called 'shft_1_nodelay'.
                    this->slow_reg_1[shfti1] = this->nodelay[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for edges_2
         */
        void shift_for_edges2()
        {
            // Shift for detector pair 2 (up and left)
            for (int r = 0; r < img_h - this->parameters.sensor_distance; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti2 = c_start + ((this->parameters.sensor_distance * img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w) + img_w;
                    this->slow_prog_2[shfti2] = this->dly_s2[c_start + c];
                    this->fast_prog_2[shfti2] = this->dly_f2[c_start + c];
                    this->slow_reg_2[shfti2] = this->nodelay2[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for edges_3
         */
        void shift_for_edges3()
        {
            // Shift for detector pair 3 (this is shift-to-the-left)
            for (int r = 0; r < img_h; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti3 = c_start + ((img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w);
                    this->slow_prog_3[shfti3] = this->dly_s3[c_start + c];
                    this->fast_prog_3[shfti3] = this->dly_f3[c_start + c];
                    this->slow_reg_3[shfti3] = this->nodelay3[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for edges_4
         */
        void shift_for_edges4()
        {
            // Shift for detector pair 4 (down and left)
            for (int r = 1; r < img_h; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti4 = c_start + ((this->parameters.sensor_distance * img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w) - img_w;
                    this->slow_prog_4[shfti4] = this->dly_s4[c_start + c];
                    this->fast_prog_4[shfti4] = this->dly_f4[c_start + c];
                    this->slow_reg_4[shfti4] = this->nodelay4[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for non-edge signal for pair 1
         *
         * NB: Although this function writes into slow_prog_1, fast_prog_1 and slow_reg_1, be aware
         * that these will actually contain the shifted-dly_f, shifted-dly_s and shifted-nodelay
         * signals. Use of slow_prog_1, fast_prog_1 and slow_reg_1 avoids having additional state
         * variable arrays.
         */
        void shift_for_pair1()
        {
            // Shift for detector pair 1 is shift up
            for (int r = 0; r < img_h - this->parameters.sensor_distance; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti1 = c_start + c + (this->parameters.sensor_distance * img_w);
                    // slow_prog_1 is really 'shft_dly_s_1' at this point in algo, but placing the
                    // shifted-delayed signal in slow_prog_1 avoids an additional state variable.
                    this->slow_prog_1[shfti1] = this->dly_s[c_start + c];
                    // Similar for fast_prog_1.
                    this->fast_prog_1[shfti1] = this->dly_f[c_start + c];
                    // This would otherwise be an additional state variable called 'shft_1_nodelay'.
                    this->slow_reg_1[shfti1] = this->nodelay[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for non-edge signal for pair 2
         *
         * NB: Although this function writes into slow_prog_2, fast_prog_2 and slow_reg_2, be aware
         * that these will actually contain the shifted-dly_f, shifted-dly_s and shifted-nodelay
         * signals. Use of slow_prog_2, fast_prog_2 and slow_reg_2 avoids having additional state
         * variable arrays.
         */
        void shift_for_pair2()
        {
            // Shift for detector pair 2 (up and left)
            for (int r = 0; r < img_h - this->parameters.sensor_distance; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti2 = c_start + ((this->parameters.sensor_distance * img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w) + img_w;
                    this->slow_prog_2[shfti2] = this->dly_s[c_start + c];
                    this->fast_prog_2[shfti2] = this->dly_f[c_start + c];
                    this->slow_reg_2[shfti2] = this->nodelay[c_start + c];
                }
            }
        }

        /**
         * \brief Signal shift for non-edge signal for pair 3
         *
         * See also the notes for FlowProcessor#shift_for_pair1 and FlowProcessor#shift_for_pair2.
         */
        void shift_for_pair3()
        {
            // Shift for detector pair 3 (this is shift-to-the-left)
            for (int r = 0; r < img_h; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti3 = c_start + ((img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w);
                    this->slow_prog_3[shfti3] = this->dly_s[c_start+c];
                    this->fast_prog_3[shfti3] = this->dly_f[c_start+c];
                    this->slow_reg_3[shfti3] = this->nodelay[c_start+c];
                }
            }
        }

        /**
         * \brief Signal shift for non-edge signal for pair 4
         *
         * See also the notes for FlowProcessor#shift_for_pair1 and FlowProcessor#shift_for_pair2.
         */
        void shift_for_pair4()
        {
            // Shift for detector pair 4 (down and left)
            for (int r = this->parameters.sensor_distance; r < img_h; ++r) {
                int c_start = r * img_w;
                for (int c = 0; c < img_w; ++c) {
                    int shfti4 = c_start + ((this->parameters.sensor_distance * img_w + ((c - this->parameters.sensor_distance) % img_w)) % img_w) - img_w;
                    this->slow_prog_4[shfti4] = this->dly_s[c_start + c];
                    this->fast_prog_4[shfti4] = this->dly_f[c_start + c];
                    this->slow_reg_4[shfti4] = this->nodelay[c_start + c];
                }
            }
        }

        /**
         * \brief The multiply stage of the flow algorithm
         *
         * This computes the multiplications, and also applies RHD gating, if enabled.
         *
         * The final flow output is computed in this function and written into the member variable
         * FlowProcessor#out.
         *
         * \param f_prog Shifted-input and also Intermediate output. The fast progressive signal is
         * computed from the shifted dly_f, which is presented to this function as the initial
         * content of f_prog, and non-shifted no-delay signals in the argument _nodelay.
         *
         * \param s_prog Shifted-input and also intermediate output. The slow progressive signal is
         * computed from the shifted dly_s, presented as initial content of s_prog and from
         * _nodelay.
         *
         * \param f_reg Intermediate output. Computed from un-shifted _dly_f signal and the shifted
         * nodelay signal, which is presented to this function in the argument s_reg.

         * \param s_reg Shifted nodelay input and intermediate output. s_reg is computed from the
         * un-shifted _dly_s signal and the shifted nodelay signal.
         *
         * \param _dly_s Input. Delayed signal (slow filter)
         *
         * \param _dly_f Input. Delayed signal (fast filter)
         *
         * \param _nodelay Input. Un-delayed signal
         *
         * \param detector_angle Input. The angle of the detector pair used for this multiply-and-divide,
         * in radians. \see FlowParamters#sensors for definitions of angles for sensor pairs.
         */
        void multiply_and_divide (std::array<F, img_sz> & f_prog, std::array<F, img_sz> & s_prog,
                                  std::array<F, img_sz> & f_reg, std::array<F, img_sz> & s_reg,
                                  std::array<F, img_sz> & _dly_s, std::array<F, img_sz> & _dly_f,
                                  std::array<F, img_sz> & _nodelay, float detector_angle)
        {
            // Compute the unit vector for the detector angle
            sm::vec<F, 2> unit = { std::cos (detector_angle), std::sin (detector_angle) };

            F prog_div = F{0};
            F reg_div = F{0};

            // Note: Simply ignoring pixel 0 and the last pixel for now.
            for (unsigned int i = 1; i < img_sz-1; ++i) {
                // multiply (Note: all the extra 'same sign stuff' adds 0.05s for 10000). Irrelevant with no edges?
                f_prog[i] = this->parameters.mult_scale * f_prog[i] * _nodelay[i] * (f_prog[i] * _nodelay[i] > F{0});
                s_prog[i] = this->parameters.mult_scale * s_prog[i] * _nodelay[i] * (s_prog[i] * _nodelay[i] > F{0});
                // f_reg ends up containing the *same* value as f_prog. s_reg is 'shift-nodelay'
                // signal at start. This line computing f_reg MUST PRECEDE the line that computes
                // s_reg!
                f_reg[i] =  this->parameters.mult_scale * _dly_f[i] * s_reg[i] * (_dly_f[i] * s_reg[i] > F{0});
                // Important: s_reg line MUST FOLLOW f_reg line!
                s_reg[i] =  this->parameters.mult_scale * _dly_s[i] * s_reg[i] * (_dly_s[i] * s_reg[i] > F{0});

                if (this->parameters.apply_rhd_gating == true) {
                    // RHD filter: gating the progressive/regressive signal

                    // condition_prog is false if (logic from rhd_filter component of SpineML):
                    //bool condition_prog = (f_prog[i]
                    //                      - (f_reg[i]  * this->parameters.rhd_deadzone)
                    //                      - (f_reg[i+1] * this->parameters.rhd_deadzone)
                    //                      + s_prog[i]
                    //                      - (s_reg[i]  * this->parameters.rhd_deadzone)) > F{0};
                    // Equivalent:
                    bool condition_prog = (f_prog[i] + s_prog[i] - ((f_reg[i] + f_reg[i+1] + s_reg[i]) * this->parameters.rhd_deadzone)) > F{0};
                    // Which means 'true' if the sum of the progressive signals is greater than the
                    // sum of the regressive signals (including the fast one from the next location)
                    // times the deadzone (typically value 1).

                    bool condition_reg =  (f_reg[i] + s_reg[i] - ((f_prog[i] + f_prog[i-1] + s_prog[i]) * this->parameters.rhd_deadzone)) > F{0};

                    // prog_on is only true if fast progressive is gtr than the rhd_filt_thresh AND the slow progressive signal is ALSO greater than the rhd_filt_thresh
                    bool prog_on = f_prog[i] > this->parameters.rhd_filt_thresh && s_prog[i] > this->parameters.rhd_filt_thresh;
                    bool reg_on  = f_reg[i]  > this->parameters.rhd_filt_thresh && s_reg[i]  > this->parameters.rhd_filt_thresh;

                    // In gating mode we get prog/reg output only if 'sums of prog/reg are > sums of
                    // reg/prog' AND only if f_prog/reg AND s_prog/reg are greater than a threshold.
                    F _f_prog_out = f_prog[i] * condition_prog * prog_on;
                    F _s_prog_out = s_prog[i] * condition_prog * prog_on;
                    F _f_reg_out = f_reg[i] * condition_reg * reg_on;
                    F _s_reg_out = s_reg[i] * condition_reg * reg_on;

                    prog_div = _f_prog_out / (_s_prog_out * (_s_prog_out>F{0}) + this->parameters.div_offset) * (_s_prog_out > F{0} && _f_prog_out > F{0});
                    reg_div = _f_reg_out / (_s_reg_out * (_s_reg_out>F{0}) + this->parameters.div_offset) * (_s_reg_out > F{0} && _f_reg_out > F{0});
                } else {
                    prog_div = f_prog[i] / (s_prog[i] * (s_prog[i]>F{0}) + this->parameters.div_offset) * (s_prog[i] > F{0} && f_prog[i] > F{0});
                    reg_div = f_reg[i] / (s_reg[i] * (s_reg[i]>F{0}) + this->parameters.div_offset) * (s_reg[i] > F{0} && f_reg[i] > F{0});
                }

                // Add a component to the output vector
                this->out[i] += unit * (prog_div - reg_div) * this->parameters.div_scale;
            }
        }

        /**
         * \brief Reset all the pools to 0
         */
        void zero_pools()
        {
            for (auto& el : this->pool1) { el = sm::vec<F, 2>({0, 0}); }
            for (auto& el : this->pool2) { el = sm::vec<F, 2>({0, 0}); }
            for (auto& el : this->pool3) { el = sm::vec<F, 2>({0, 0}); }
            if constexpr (compute_out_minus_pool) {
                for (auto& el : this->out_minus_pool1) { el = sm::vec<F, 2>({0, 0}); }
                for (auto& el : this->out_minus_pool2) { el = sm::vec<F, 2>({0, 0}); }
                for (auto& el : this->out_minus_pool3) { el = sm::vec<F, 2>({0, 0}); }
            }
        }

        /**
         * \brief Compile-time choice of whether to compute out-minus-pools
         */
        static constexpr bool compute_out_minus_pool = false;

        /**
         * \brief This function computes the pools from the output
         */
        void pool()
        {
            this->zero_pools();

            // pool this->out into this->pool1
            for (int i = 0; i < img_sz; ++i) {
                int pc = (i % img_w) / pooldiv;  //
                int pr = (i / img_w) / pooldiv;  // convert img_sz iterator i to a pool1 sized p1i
                pr = pr >= pool1_h ? pool1_h - 1 : pr;
                int p1i = pc + pool1_w * pr;
                this->pool1[p1i] += this->out[i] / pooldiv_sq;
            }

            // Compute pool2 (from pool1)
            for (int i = 0; i < pool1_sz; ++i) {
                int pc = (i % pool1_w) / pooldiv; //
                int pr = (i / pool1_w) / pooldiv; // convert pool1_sz iterator i to a pool2 sized p2i
                pr = pr >= pool2_h ? pool2_h - 1 : pr;
                int p2i = pc + pool2_w * pr;
                this->pool2[p2i] += this->pool1[i] / pooldiv_sq;
            }

            // Compute pool3 (from pool2)
            for (int i = 0; i < pool2_sz; ++i) {
                int pc = (i % pool2_w) / pooldiv; //
                int pr = (i / pool2_w) / pooldiv; // convert pool2_sz iterator i to a pool3 sized p3i
                pr = pr >= pool3_h ? pool3_h - 1 : pr;
                int p3i = pc + pool3_w * pr;
                this->pool3[p3i] += this->pool2[i] / pooldiv_sq;
            }

            // Compute out-pool1 now.
            for (int i = 0; i < img_sz; ++i) {
                int pc = (i % img_w) / pooldiv; //
                int pr = (i / img_w) / pooldiv; // convert img_sz iterator i to pool1 sized p1i
                pr = pr >= pool1_h ? pool1_h - 1 : pr;
                [[maybe_unused]] int p1i = pc + pool1_w * pr;
                if constexpr (compute_out_minus_pool) {
                    this->out_minus_pool1[i] = this->out[i] - this->pool1[p1i];
                }
            }

            // Compute out-pool2
            for (int i = 0; i < img_sz; ++i) {
                int pc = (i % img_w) / pooldiv; // This needs to convert img_sz iterator i to pool2 sized p2i. That's 2 steps
                int pr = (i / img_w) / pooldiv;
                pr = pr >= pool1_h ? pool1_h - 1 : pr;
                int p1i = pc + pool1_w * pr;

                int pc2 = (p1i % pool1_w) / pooldiv; //
                int pr2 = (p1i / pool1_w) / pooldiv; // convert pool1_sz iterator i to a pool2 sized p2i
                pr2 = pr2 >= pool2_h ? pool2_h - 1 : pr2;
                [[maybe_unused]] int p2i = pc2 + pool2_w * pr2;
                if constexpr (compute_out_minus_pool) {
                    this->out_minus_pool2[i] = this->out[i] - this->pool2[p2i];
                }
            }

            // Compute out-pool3
            for (int i = 0; i < img_sz; ++i) {
                int pc = (i % img_w) / pooldiv; // This needs to convert img_sz iterator i to pool2 sized p2i. That's 2 steps
                int pr = (i / img_w) / pooldiv;
                pr = pr >= pool1_h ? pool1_h - 1 : pr;
                int p1i = pc + pool1_w * pr;

                int pc2 = (p1i % pool1_w) / pooldiv; //
                int pr2 = (p1i / pool1_w) / pooldiv; // convert pool1_sz iterator p1i to a pool2 sized p2i
                pr2 = pr2 >= pool2_h ? pool2_h - 1 : pr2;
                int p2i = pc2 + pool2_w * pr2;

                int pc3 = (p2i % pool2_w) / pooldiv; //
                int pr3 = (p2i / pool2_w) / pooldiv; // convert pool2_sz iterator p2i to a pool3 sized p3i
                pr3 = pr3 >= pool3_h ? pool3_h - 1 : pr3;
                [[maybe_unused]] int p3i = pc3 + pool3_w * pr3;
                if constexpr (compute_out_minus_pool) {
                    this->out_minus_pool3[i] = this->out[i] - this->pool3[p3i];
                }
            }
        }

        /**
         * \brief Return the global flow, computed from FlowProcessor#pool3
         *
         * \return Global flow vector.
         */
        sm::vec<F, 2> global_flow()
        {
            sm::vec<F, 2> pool3_mean = this->pool3[0];
            for (int i = 1; i < pool3_sz; ++i) {
                pool3_mean += this->pool3[i];
            }
            pool3_mean /= static_cast<float>(pool3_sz);
            return pool3_mean;
        }
    };

} // namespace perception
