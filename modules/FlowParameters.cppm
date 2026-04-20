/**
 * perception::FlowParameters implementation
 *
 * The Opteran Optic Flow computation parameter class
 *
 * \author Seb James
 * \date Jan 2024
 *
 * Copyright Opteran
 */

module;

#include <bitset>
#include <string>
#include <iostream>

export module flowparameters;

export namespace perception
{

/**
 * \brief The parameters for the perception#FlowProcessor
 *
 * This class simply holds a set of boolean flags and some real valued parameters to define the
 * functionality of the flow algorithm implemented by perception#FlowProcessor
 */
template<typename F = float>
struct FlowParameters
{
  /**
   * \brief Output parameters in JSON
   *
   * Output the parameters in structured JSON format to stdout. If the JSON is written into a file
   * then the file can be loaded with perception#FlowConfig.
   *
   * \param desc A mandatory string description for the parameter set, which is placed in the
   * "description" JSON attribute. Although there is no "description" in a perception#FlowParameters
   * object, this tag can be useful to distinguish different parameter sets in side-by-side
   * visualizations.
   */
  void output_json(const std::string & desc)
  {
    std::cout << "{\n  \"description\" : \"" << desc << "\"\n\n";

    std::cout << "  \"apply_input_compression\" : " <<
      (apply_input_compression ? "true" : "false") << ",\n";
    std::cout << "  \"apply_smoothing\" : " << (apply_smoothing ? "true" : "false") << ",\n";
    std::cout << "  \"apply_edge_detection\" : " << (apply_edge_detection ? "true" : "false") <<
      ",\n";
    std::cout << "  \"apply_edge_compression\" : " << (apply_edge_compression ? "true" : "false") <<
      ",\n";
    std::cout << "  \"apply_pr_squashing\" : " << (apply_pr_squashing ? "true" : "false") <<
      ",\n\n";
    std::cout << "  \"apply_rhd_gating\" : " << (apply_rhd_gating ? "true" : "false") << ",\n";
    std::cout << "  \"apply_pooling\" : " << (apply_pooling ? "true" : "false") << ",\n";

    std::cout << "  \"pair1\" : " << (this->sensors.test(0) ? "true" : "false") << ",\n";
    std::cout << "  \"pair2\" : " << (this->sensors.test(1) ? "true" : "false") << ",\n";
    std::cout << "  \"pair3\" : " << (this->sensors.test(2) ? "true" : "false") << ",\n";
    std::cout << "  \"pair4\" : " << (this->sensors.test(3) ? "true" : "false") << ",\n";
    std::cout << "  \"sensor_distance\" : " << this->sensor_distance << ",\n\n";

    std::cout << "  \"ic_tanhm\" : " << this->ic_tanhm << ",\n\n";

    std::cout << "  \"si_sd_scale\" : " << this->si_sd_scale << ",\n";
    std::cout << "  \"si_tau\" : " << this->get_si_tau() << ",\n\n";

    std::cout << "  \"eic_value\" : " << this->eic_value << ",\n";
    std::cout << "  \"eic_threshold\" : " << this->eic_threshold << ",\n";
    std::cout << "  \"eic_signal_feedin\" : " << this->eic_signal_feedin << ",\n\n";

    std::cout << "  \"pr_a_tau\" : " << this->get_pr_a_tau() << ",\n";
    std::cout << "  \"pr_b_tau\" : " << this->get_pr_b_tau() << ",\n";
    std::cout << "  \"pr_squash_m\" : " << this->pr_squash_m << ",\n\n";

    std::cout << "  \"dly_f_tau\" : " << this->get_dly_f_tau() << ",\n";
    std::cout << "  \"dly_s_tau\" : " << this->get_dly_s_tau() << ",\n\n";

    std::cout << "  \"rhd_filt_thresh\" : " << this->rhd_filt_thresh << ",\n\n";

    std::cout << "  \"div_scale\" : " << this->div_scale << ",\n";
    std::cout << "  \"div_offset\" : " << this->div_offset << "\n";

    std::cout << "}\n";
  }

  /**
   * \name Smooth input setters and getters
   *
   * \see FlowProcessor#smooth_input
   */
  ///@{
  /**
   * \brief Setter for FlowParameters#si_inv_tau
   *
   * This takes the time constant tau as an argument and sets the attribute, which is the
   * inverse time constant.
   *
   * \param tau The time constant
   */
  void set_si_tau(F tau) {this->si_inv_tau = F{1} / tau;}

  /**
   * \brief Getter for FlowParameters#si_inv_tau
   *
   * This retrieves the time constant tau, rather than the inverse time constant as it is
   * stored in the attribute.
   *
   * \return The time constant
   */
  F get_si_tau() const {return F{1} / this->si_inv_tau;}
  ///@}

  /**
   * \name Photoreceptor getters and setters
   *
   * \see FlowProcessor#photoreceptor_and_delayfilt
   */
  ///@{
  /**
   * \brief Setter for FlowParameters#pr_a_inv_tau
   *
   * This takes the time constant tau as an argument and sets the attribute, which is the
   * inverse time constant.
   *
   * \param tau The time constant
   */
  void set_pr_a_tau(F tau) {this->pr_a_inv_tau = F{1} / tau;}

  /**
   * \brief Getter for FlowParameters#pr_a_inv_tau
   *
   * This retrieves the time constant tau, rather than the inverse time constant as it is
   * stored in the attribute.
   *
   * \return The time constant
   */
  F get_pr_a_tau() const {return F{1} / this->pr_a_inv_tau;}

  /**
   * \brief Setter for FlowParameters#pr_b_inv_tau
   *
   * This takes the time constant tau as an argument and sets the attribute, which is the
   * inverse time constant.
   *
   * \param tau The time constant
   */
  void set_pr_b_tau(F tau) {this->pr_b_inv_tau = F{1} / tau;}

  /**
   * \brief Getter for FlowParameters#pr_b_inv_tau
   *
   * This retrieves the time constant tau, rather than the inverse time constant as it is
   * stored in the attribute.
   *
   * \return The time constant
   */
  F get_pr_b_tau() const {return F{1} / this->pr_b_inv_tau;}
  ///@}

  /**
   * \name Delay filter setters and getters
   *
   * \see FlowProcessor#photoreceptor_and_delayfilt
   */
  ///@{
  /**
   * \brief Setter for FlowParameters#dly_f_inv_tau
   *
   * This takes the time constant tau as an argument and sets the attribute, which is the
   * inverse time constant.
   *
   * \param tau The time constant
   */
  void set_dly_f_tau(F tau) {this->dly_f_inv_tau = F{1} / tau;}

  /**
   * \brief Getter for FlowParameters#dly_f_inv_tau
   *
   * This retrieves the time constant tau, rather than the inverse time constant as it is
   * stored in the attribute.
   *
   * \return The time constant
   */
  F get_dly_f_tau() const {return F{1} / this->dly_f_inv_tau;}

  /**
   * \brief Setter for FlowParameters#dly_s_inv_tau
   *
   * This takes the time constant tau as an argument and sets the attribute, which is the
   * inverse time constant.
   *
   * \param tau The time constant
   */
  void set_dly_s_tau(F tau) {this->dly_s_inv_tau = F{1} / tau;}

  /**
   * \brief Getter for FlowParameters#dly_f_inv_tau
   *
   * This retrieves the time constant tau, rather than the inverse time constant as it is
   * stored in the attribute.
   *
   * \return The time constant
   */
  F get_dly_s_tau() const {return F{1} / this->dly_s_inv_tau;}
  ///@}

  /**
   * \name Feature choices
   * Boolean parameters that turn flow algorithm features on/off
   */
  ///@{

  /**
   * This bitset flags which sensor pairs are to be included in the flow computation.
   *
   * In a Cartesian grid of sensors I think of pairs of pixels as forming 'detector pairs':
   *
   * \verbatim
   *--------------------------------------
   *       o   o
   *           |  pair one (detector angle pi/2)
   *       o   O
   *--------------------------------------
   *       o   o
   *         \    pair two (detector angle 3pi/4)
   *       o   O
   *--------------------------------------
   *       o   o
   *              pair three (detector angle pi)
   *       o---O
   *--------------------------------------
   *       o   o
   *
   *       o   O  pair four (detector angle 5pi/4)
   *         /
   *       o
   *--------------------------------------
   * \endverbatim
   *
   * Bit 0 of this bitset flags that pair 1 should be included in the flow computation. bit
   * 1 flags whether pair 2 should be included, bit 2 flags pair 3 and bit 3 determines if pair
   * four should be included.
   *
   * sensors is initializated with a string here. Note that the std::bitset has the
   * lowest bit (Bit 0) as the right-most 'digit', as if it were being interpreted as
   * the binary representation of an integer. The default of "0101" which is set here
   * therefore means that pair one is enabled and pair three is enabled.
   */
  std::bitset<4> sensors = std::bitset<4>{"0101"};

  /**
   * \brief Sensor spacing
   *
   * How many pixels apart are our detector pairs. 1 means that they are adjacent pixels.
   */
  int sensor_distance = 1;

  /**
   * \brief Turn on input compression?
   *
   * If true, then the input to the FlowProcessor will be pass through a tanh squashing
   * function before further processing.
   *
   * \see FlowProcessor#compress_input
   */
  bool apply_input_compression = true;

  /**
   * \brief Turn on the input smoothing?
   *
   * If true, then FlowProcessor#smooth_input is used to temporally smooth the input data.
   */
  bool apply_smoothing = true;

  /**
   * \brief Apply edge detection mode?
   *
   * If true, then the flow computation should work in 'edge mode' (by applying FlowProcessor#convolve_for_edges_sensor1 and friends). If false, no edge
   * convolution should be applied.
   *
   *  \see FlowProcessor#convolve_for_edges_sensor1
   *  \see FlowProcessor#convolve_for_edges_sensor2
   *  \see FlowProcessor#convolve_for_edges_sensor3
   *  \see FlowProcessor#convolve_for_edges_sensor4
   */
  bool apply_edge_detection = false;

  /**
   * \brief Compress edges before passing to photoreceptors?
   *
   * If this is true (and we are in edge mode) we apply a compression to the edges. Any edge
   * which is greater in magnitude than the parameter FlowParameters#eic_threshold is replaced with the value
   * +/- FlowParameters#eic_value. Thus, all edges above a threshold have the same value (though their
   * direction/sign is preserved).
   *
   * Edge compression is applied in FlowProcessor#apply_edge_compression.
   *
   * Only relevant if apply_edge_detection is true.
   *
   */
  bool apply_edge_compression = true;

  /**
   * \brief Apply a squashing function after the photoreceptors?
   *
   * If this is set true, then in FlowProcessor#photoreceptor_and_delayfilt the photoreceptor
   * output (which is pr_a[i] - pr_b[i]) is passed through a sigmoid squashing function with
   * parameter FlowParamters#pr_squash_m.
   */
  bool apply_pr_squashing = true;

  /**
   * \brief Do the progressive/regressive flow signal gating?
   *
   * In RHD gating mode we pass on a progressive signal only if 'sums of progressive signals
   * (f,s,f[i+1] are > sums of regressive signals' AND only if fast progressive AND slow
   * progressive signals are greater than a threshold.
   *
   * In RHD gating mode we pass on a regressive output only if 'sums of regressive signals are
   * > sums of progressive signals AND only if fast regressive AND slow regressive signals are
   * greater than a threshold.
   *
   * \see FlowProcessor#multiply_and_divide
   */
  bool apply_rhd_gating = false;

  /**
   * \brief Apply the output pooling?
   *
   * If true, then the highest resolution flow output data in FlowProcessor#out is averaged into coarser
   * representations in FlowProcessor#pool1, FlowProcessor#pool2 and FlowProcessor#pool3.
   */
  bool apply_pooling = false;
  ///@}

  /**
   * \name Algorithm parameters
   *
   * Real valued parameters that control the behaviour of the flow algorithm
   */
  ///@{

  /**
   * \name Input compression attributes
   */
  ///@{
  /**
   * \brief Input compression tanh parameter
   *
   * Input compression is applied in FlowProcessor#compress_input. This is the single
   * parameter required to set the steepness of the tanh squashing function.
   */
  F ic_tanhm = F{6};
  ///@}

  /**
   * \name Smooth input attributes
   */
  ///@{
  /**
   * \brief smooth_input scale parameter
   *
   * In FlowProcessor#smooth_input, the variance of the signal is computed. This parameter
   * scales this variance before it is used to modulate the input.
   */
  F si_sd_scale = F{20};

  /**
   * \brief smooth_input 1/tau parameter
   *
   * This is the inverse time constant used in FlowProcessor#smooth_input to maintain a
   * first-order smoothed estimate of the variance of the input signal.
   */
  F si_inv_tau = F{0.01};
  ///@}

  /**
   * \name Photoreceptor attributes
   *
   * PR attributes
   */
  ///@{
  /**
   * \brief Photoreceptor A 1/tau parameter
   *
   * This is the inverse time constant used in FlowProcessor#photoreceptor_and_delayfilt for
   * the pr_a leaky integrator.
   */
  F pr_a_inv_tau = F{0.8};

  /**
   * \brief Photoreceptor B 1/tau parameter
   *
   * This is the inverse time constant used in FlowProcessor#photoreceptor_and_delayfilt for
   * the pr_b leaky integrator.
   */
  F pr_b_inv_tau = F{0.4};

  /**
   * Steepness parameter for the optional sigmoid squashing function for the photoreceptor
   * output
   *
   * \see FlowParamters#apply_pr_squashing
   */
  F pr_squash_m = F{3};
  ///@}

  /**
   * \name Delay filter parameters
   *
   * Delay filter params. How fast is 'fast_prog/reg' and how slow is 'slow_prog/reg'?
   */
  ///@{
  /**
   * \brief Fast delay 1/tau parameter
   *
   * This is the inverse time constant used in FlowProcessor#photoreceptor_and_delayfilt for
   * the fast delay leaky integrator
   */
  F dly_f_inv_tau = F{0.2};

  /**
   * \brief Slow delay 1/tau parameter
   *
   * This is the inverse time constant used in FlowProcessor#photoreceptor_and_delayfilt for
   * the slow delay leaky integrator
   */
  F dly_s_inv_tau = F{0.067};

  /**
   * \brief Multiplication param
   *
   * A (currently unity) gain for the delayed signal * nodelay signal multiplication
   */
  static constexpr F mult_scale = F{1};
  ///@}

  /**
   * \name RHD gating parameters
   *
   * This allows small progressive or regressive flow signals to be ignored. It adds some
   * computation.
   */
  ///@{
  /**
   * \brief RHD deadzone parameter
   *
   * This could be used to weight the contribution of opposing flow in the RHD gating, though
   * it is currently set to unity at compile time.
   */
  static constexpr F rhd_deadzone = F{1};

  /**
   * \brief RHD gating threshold parameter.
   */
  F rhd_filt_thresh = F{0.004};
  ///@}

  /**
   * \name Division parameters
   *
   * \see FlowProcessor#multiply_and_divide
   */
  ///@{
  /**
   * \brief Effectively a gain on the final output
   *
   * \see \see FlowProcessor#multiply_and_divide
   */
  F div_scale = F{1};

  /**
   * A small offset to avoid divide-by-zero in FlowProcessor#multiply_and_divide
   */
  F div_offset = F{0.001};
  ///@}

  /**
   * \name Edge input compression parameters
   */
  ///@{
  /**
   * \brief eic_value is the output signal size
   *
   * This value determines the output edge magnitude for threshold-triggering input edges.
   */
  F eic_value = F{1};
  /**
   * \brief Edge input compression threshold parameter
   *
   * This is the magnitude required of an input edge to trigger a +/- eic_value-sized edge
   * output.
   */
  F eic_threshold = F{0.05};
  /**
   * \brief Edge input compression signal feed-in parameter
   *
   * Determines how much of the non-edge signal is fed into the photoreceptors alongside the
   * edges
   */
  F eic_signal_feedin = F{0.1};
  ///@}

  ///@}
};

} // namespace perception
