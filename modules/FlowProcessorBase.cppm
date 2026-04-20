export module flowprocessorbase;

import flowparameters;

export namespace perception
{
    /**
     * \brief An abstract base class for optic flow, holding the common 'parameters'
     * attribute.
     *
     * \tparam F The floating point type for the data
     */
    template<typename F>
    struct FlowProcessorBase
    {
        /**
         * \brief A parameters object
         *
         * The parameters for the any Optic Flow computation are stored in an object of
         * type perception#FlowParameters.
         */
        perception::FlowParameters<F> parameters;

        /**
         * To be implemented as a function that processes the optic flow.
         */
        virtual void process() = 0;
    };
}
