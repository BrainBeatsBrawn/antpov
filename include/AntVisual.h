#pragma once

#include <sm/config>

#include <mplot/colour.h>
#include <mplot/VisualModel.h>

namespace biosim
{
    template <int glver = mplot::gl::version_4_1>
    class AntVisual : public mplot::VisualModel<glver>
    {
    public:
        AntVisual() {}

        void initializeVertices()
        {
            sm::config conf ("./ant.json");
            if (!conf.ready) { conf.parse ("{}"); }

            // Head
            sm::mat44<float> head_tr;
            head_tr.rotate (sm::vec<>::ux(), conf.get<float>("head_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("head_loc"),
                                    mplot::colour::firebrick4,
                                    mplot::colour::sepia,
                                    conf.getvec<float, 3>("head_abc"), 30, 30, head_tr);

            this->computeCone (conf.getvec<float, 3>("thor_s"),
                               conf.getvec<float, 3>("thor_e"),
                               0.0001f,
                               mplot::colour::firebrick4,
                               conf.get<float>("head_radius", 0.0005f)/3, 18);

            sm::mat44<float> abdomen_tr;
            abdomen_tr.rotate (sm::vec<>::ux(), conf.get<float>("abdomen_rotn_angle", 0.0f));
            this->computeEllipsoid (conf.getvec<float, 3>("abdomen_loc"),
                                    mplot::colour::brown4,
                                    mplot::colour::sepia,
                                    conf.getvec<float, 3>("abdomen_abc"), 30, 30, abdomen_tr);
        }
    };
}
