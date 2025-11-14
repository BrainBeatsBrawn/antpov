#pragma once

#include <mplot/colour.h>
#include <mplot/VisualModel.h>

namespace biosim
{
    template <int glver = mplot::gl::version_4_1>
    class AntVisual : public mplot::VisualModel<glver>
    {
    public:
        AntVisual() {}


        // Head 1 mm
        static constexpr float head_radius = 0.0005f;
        static constexpr sm::vec<> head_loc = {0, 0, -0.0001f};

        static constexpr sm::vec<> thor_s = {0, -0.0002f, -0.0005f};
        static constexpr sm::vec<> thor_e = {0, -0.0003f, -0.002f};
        void initializeVertices()
        {
            // Head
            this->computeSphere (head_loc, mplot::colour::firebrick4, head_radius);

            // thorax, etc
            sm::vec<> thorax_uy = {1,0,0};
            sm::vec<> thorax_uz = {0,1,0};

/*
  this->computeTube (thor_s, thor_e,
  thorax_uy, thorax_uz,
  mplot::colour::firebrick4, mplot::colour::firebrick4,
  head_radius/3, 18);
*/
            this->computeCone (thor_s, thor_e, 0.0001f,
                               mplot::colour::firebrick4,
                               head_radius/3, 18);
        }
    };
}
