#pragma once

#include <sm/config>

#include <mplot/colour.h>
#include <mplot/VisualModel.h>

namespace biosim
{
    // Parameters for our ant model
    const char* ant_json =
    "{\n"
    "\"head_loc\" : [0, -0.0001, 0.0001],\n"
    "\"head_abc\" : [0.0005, 0.00045, 0.00065],\n"
    "\"head_rotn_angle\" : 0.7,\n"
    "\"thor_s\" : [0, -0.0002, -0.0003],\n"
    "\"thor_e\" : [0, -0.0003, -0.004],\n"
    "\"t1_loc\" : [0, 0.0001, -0.0009],\n"
    "\"t1_abc\" : [0.0003, 0.00025, 0.0008],\n"
    "\"t1_rotn_angle\" : 0.1,\n"
    "\"t2_loc\" : [0, -0.00022, -0.0019],\n"
    "\"t2_abc\" : [0.0002, 0.0002, 0.0005],\n"
    "\"t2_rotn_angle\" : -0.9,\n"
    "\"t3_loc\" : [0, -0.0004, -0.0025],\n"
    "\"t3_abc\" : [0.0002, 0.0002, 0.0008],\n"
    "\"t3_rotn_angle\" : -0.3,\n"
    "\"abdomen_loc\" : [0, -0.0003, -0.004],\n"
    "\"abdomen_abc\" : [0.0007, 0.0007, 0.001],\n"
    "\"abdomen_rotn_angle\" : -0.2\n"
    "}\n";

    template <int glver = mplot::gl::version_4_1>
    class AntVisual : public mplot::VisualModel<glver>
    {
    public:
        AntVisual() {}

        void initializeVertices()
        {
            // Try opening ant.json in the current working directory
            sm::config conf ("./ant.json");
            // If that fails, parse the built-in json
            if (!conf.ready) { conf.parse (biosim::ant_json); }

            constexpr int nseg = 16;
            constexpr int nring = 12;

            // Head
            sm::mat44<float> head_tr;
            head_tr.rotate (sm::vec<>::ux(), conf.get<float>("head_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("head_loc"),
                                    mplot::colour::firebrick4,
                                    mplot::colour::sepia,
                                    conf.getvec<float, 3>("head_abc"), nring, nseg, head_tr);

            // Three ellipsoids for thorax
            sm::mat44<float> t1_tr;
            t1_tr.rotate (sm::vec<>::ux(), conf.get<float>("t1_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("t1_loc"),
                                    mplot::colour::sepia,
                                    mplot::colour::firebrick4,
                                    conf.getvec<float, 3>("t1_abc"), nring, nseg, t1_tr);

            sm::mat44<float> t2_tr;
            t2_tr.rotate (sm::vec<>::ux(), conf.get<float>("t2_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("t2_loc"),
                                    mplot::colour::sepia,
                                    mplot::colour::firebrick4,
                                    conf.getvec<float, 3>("t2_abc"), nring, nseg, t2_tr);

            sm::mat44<float> t3_tr;
            t3_tr.rotate (sm::vec<>::ux(), conf.get<float>("t3_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("t3_loc"),
                                    mplot::colour::sepia,
                                    mplot::colour::firebrick4,
                                    conf.getvec<float, 3>("t3_abc"), nring, nseg, t3_tr);

            // Lastly, the abdomen
            sm::mat44<float> abdomen_tr;
            abdomen_tr.rotate (sm::vec<>::ux(), conf.get<float>("abdomen_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("abdomen_loc"),
                                    mplot::colour::ivoryblack,
                                    mplot::colour::sepia,
                                    conf.getvec<float, 3>("abdomen_abc"), nring, nseg, abdomen_tr);
        }
    };
}
