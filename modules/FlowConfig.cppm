/**
 * \file perception::FlowConfig implementation
 *
 * A scheme to save/load FlowParameters to a JSON config file, with the help of sm::config
 *
 * \author Seb James
 * \date Jan 2024
 *
 * Copyright Opteran
 */

module;

#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

export module flowconfig;

import sm.config;
import flowparameters;
import flowprocessorbase;

export namespace perception
{
    /**
     * \brief Companion class to perception#FlowParameters
     *
     * A class to read parameters for the perception#FlowProcessorBase processor from a JSON config file and then
     * update the perception#FlowParameters values held in the FlowProcessorBase instance.
     *
     * This class is separate from perception#FlowParameters so that morph#Config is not baked into
     * Opteran Flow. In an unconstrained world, I would consider incorporating this code into
     * perception#FlowParameters
     */
    template<typename F>
    struct FlowConfig
    {
        /**
         * \brief FlowConfig constructor
         *
         * FlowConfig is constructed with two arguments; one to set the JSON configuration file that will be used
         * to read parameter values and one for the compansion perception#FlowProcessorBase instance to which the
         * parameter values will be written.
         *
         * \param fpath The path to the JSON configuration file, which must already exist and be valid JSON.
         *
         * \param proc A pointer to a perception#FlowProcessorBase instance, which contains a
         * perception#FlowParameters object whose values can be set using this FlowConfig.
         */
        FlowConfig(const std::string& fpath, perception::FlowProcessorBase<F>* proc)
        {
            this->configFilepath = fpath;
            this->myProcessor = proc;
        }

        /**
         * \brief Update the parameters in myProcessor.
         *
         * This function reads the configuration parameters from the morph#Config wrapped JSON file
         * at this->configFilepath. It then changes the values in the linked
         * perception#FlowProcessorBase object (this->myProcessor.parameters).
         */
        void updateConfig()
        {
            try {
                sm::config conf(this->configFilepath);

                if (!conf.ready) {
                    std::stringstream ee;
                    ee << "FlowConfig error: The sm::config for " << this->configFilepath
                       << " is not ready (file exists?)";
                    throw std::runtime_error(ee.str());
                }

                // Instantiate a FlowParameters object to get the defaults
                perception::FlowParameters dp;

                // Read parameters from JSON into myProcessor->parameters
                if (this->myProcessor != nullptr) {
                    this->myProcessor->parameters.apply_input_compression = conf.get<bool>("apply_input_compression", dp.apply_input_compression);
                    this->myProcessor->parameters.apply_smoothing = conf.get<bool>("apply_smoothing", dp.apply_smoothing);
                    this->myProcessor->parameters.apply_pooling = conf.get<bool>("apply_pooling", dp.apply_pooling);
                    this->myProcessor->parameters.apply_rhd_gating = conf.get<bool>("apply_rhd_gating", dp.apply_rhd_gating);
                    this->myProcessor->parameters.apply_edge_detection = conf.get<bool>("apply_edge_detection", dp.apply_edge_detection);
                    this->myProcessor->parameters.apply_edge_compression = conf.get<bool>("apply_edge_compression", dp.apply_edge_compression);
                    this->myProcessor->parameters.apply_pr_squashing = conf.get<bool>("apply_pr_squashing", dp.apply_pr_squashing);

                    this->myProcessor->parameters.sensors.set(0, conf.get<bool>("pair1", true));
                    this->myProcessor->parameters.sensors.set(1, conf.get<bool>("pair2", true));
                    this->myProcessor->parameters.sensors.set(2, conf.get<bool>("pair3", true));
                    this->myProcessor->parameters.sensors.set(3, conf.get<bool>("pair4", false));

                    this->myProcessor->parameters.sensor_distance = conf.get<int>("sensor_distance", dp.sensor_distance);

                    this->myProcessor->parameters.ic_tanhm = conf.get<F>("ic_tanhm", dp.ic_tanhm);

                    this->myProcessor->parameters.si_sd_scale = conf.get<F>("si_sd_scale", dp.si_sd_scale);
                    this->myProcessor->parameters.si_inv_tau = F{1} / conf.get<F>("si_tau", dp.get_si_tau());

                    this->myProcessor->parameters.eic_value = conf.get<F>("eic_value", dp.eic_value);
                    this->myProcessor->parameters.eic_threshold = conf.get<F>("eic_threshold", dp.eic_threshold);
                    this->myProcessor->parameters.eic_signal_feedin = conf.get<F>("eic_signal_feedin", dp.eic_signal_feedin);

                    this->myProcessor->parameters.pr_a_inv_tau = F{1} / conf.get<F>("pr_a_tau", dp.get_pr_a_tau());
                    this->myProcessor->parameters.pr_b_inv_tau = F{1} / conf.get<F>("pr_b_tau", dp.get_pr_b_tau());

                    this->myProcessor->parameters.dly_f_inv_tau = F{1} / conf.get<F>("dly_f_tau", dp.get_dly_f_tau());
                    this->myProcessor->parameters.dly_s_inv_tau = F{1} / conf.get<F>("dly_s_tau", dp.get_dly_s_tau());

                    this->myProcessor->parameters.rhd_filt_thresh = conf.get<F>("rhd_filt_thresh", dp.rhd_filt_thresh);

                    this->myProcessor->parameters.div_scale = conf.get<F>("div_scale", dp.div_scale);
                    this->myProcessor->parameters.div_offset = conf.get<F>("div_offset", dp.div_offset);

                    this->myProcessor->parameters.pr_squash_m = conf.get<F>("pr_squash_m", dp.pr_squash_m);

                    this->flow_component_scale_gradient = conf.get<F>("flow_component_scale_gradient", 1.0f);

                    this->description = conf.get<std::string>("description", "unset");

                } else {
                    throw std::runtime_error("FlowConfig::updateConfig: FlowParameters* myProcessor is null");
                }

            } catch (const std::exception & e) {
                std::cerr << "JSON file could not be parsed with error: " << e.what() << std::endl;
            }
        }

        /**
         * A non-owning pointer to the perception#FlowProcessorBase that this FlowConfig relates to.
         */
        perception::FlowProcessorBase<F>* myProcessor = nullptr;

        /**
         * The path to our JSON configuration file.
         */
        std::string configFilepath = "unset";

        /**
         * A description for this config (there is no description in a FlowParameters object)
         */
        std::string description = "unset";

        /**
         * The scaling gradient for the flow components
         */
        F flow_component_scale_gradient = 1.0f;
    };
}
