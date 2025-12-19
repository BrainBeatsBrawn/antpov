/*
 * Make a scene of fiducials
 */

#include <sm/vec>
#include <mplot/Visual.h>
#include <mplot/RhomboVisual.h>
#include <mplot/colour.h>

void fiducial (mplot::Visual<>& v, const sm::vec<float>& pos, const std::array<float, 3>& colour)
{
    // A pole
    constexpr sm::vec<float, 3> p1 = sm::vec<float>::ux() * 0.05f;
    constexpr sm::vec<float, 3> p2 = sm::vec<float>::uz() * 0.05f;
    constexpr sm::vec<float, 3> p3 = sm::vec<float>::uy() * 1.0f;
    // A block
    constexpr sm::vec<float, 3> b1 = sm::vec<float>::ux() * 0.2f;
    constexpr sm::vec<float, 3> b2 = sm::vec<float>::uz() * 0.2f;
    constexpr sm::vec<float, 3> b3 = sm::vec<float>::uy() * 0.3f;

    auto rv = std::make_unique<mplot::RhomboVisual<>> (pos, p1, p2, p3, mplot::colour::gold1);
    v.bindmodel (rv);
    rv->finalize();
    v.addVisualModel (rv);
    rv = std::make_unique<mplot::RhomboVisual<>> (pos, b1, b2, b3, colour);
    v.bindmodel (rv);
    rv->finalize();
    v.addVisualModel (rv);
}

int main()
{
    mplot::Visual v(2000, 2000, "Fiducial markers for Seville ant world");
    fiducial (v, sm::vec<>{ 8, 0, 8 }, mplot::colour::crimson);
    fiducial (v, sm::vec<>{ 8, 0,-8 }, mplot::colour::blue);
    fiducial (v, sm::vec<>{-8, 0, 8 }, mplot::colour::springgreen);
    v.keepOpen();
}
