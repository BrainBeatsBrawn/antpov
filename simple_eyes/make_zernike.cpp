/*
 * Make an eye which maps a hemi-spherical view onto a square suitable for Zernike
 * moment calculations
 *
 * \author Seb James
 * \date July 2025
 */

#include <cmath>
#include <iostream>

#include <sm/vec>
#include <sm/mat44>

#include <mplot/Visual.h>
#include <mplot/ScatterVisual.h>
#include <mplot/QuiverVisual.h>

// Application specific conversion from a cartesian grid to flat polar coordinates
sm::vec<float, 2> cart_to_polar (const sm::vec<int, 2>& xy, const int n)
{
    sm::vec<float, 2> polar = {}; // (r, theta)
    sm::vec<float, 2> _xy =  xy.as_float() * 2.0f - (n - 1);
    _xy.pow_inplace (2);
    polar[0] = std::sqrt (_xy.sum()) / n;
    polar[0] = polar[0] <= 1.0f ? polar[0] : 0.0f; // r

    float _x = 2 * xy[0] - (n - 1);
    float _y = (n - 1) - 2 * xy[1];
    polar[1] = std::atan2 (_y, _x); // theta
    return polar;
}

// Convert to spherical coordinates (maths sph coord convention with phi the azimuthal angle)
sm::vec<float, 3> polar_to_sph (const sm::vec<float, 2>& polar, const float r = 1.0f)
{
    // r is fixed/provided by user
    // phi is angle from 'up'. Comes from polar[0]
    // just copy theta from polar to sphere
    const float phi = std::asin (polar[0] / 1.0f); // Angle from up axis
    return sm::vec<float, 3>{r, polar[1], phi}; // r, theta, phi
}

int main()
{
    using mc = sm::mathconst<float>;
    constexpr int n = 208;
    constexpr float eye_r = 1.0f;
    constexpr bool no_stdout = false;
    mplot::Visual<> v (1024, 768, "Zernike debug", no_stdout);
    // Polar coords/quivs
    std::vector<sm::vec<float, 3>> coords(n * n);
    std::vector<sm::vec<float, 3>> quivs(n * n);
    // Sph coords/quivs
    std::vector<sm::vec<float, 3>> s_coords(n * n);
    std::vector<sm::vec<float, 3>> s_quivs(n * n);

    std::vector<float> clrs (n * n, 0.0f);

    float acceptance_angle = 0.01f;
    float focal_offset = 0.0f;

    sm::mat44<float> rotn;
    rotn.rotate (sm::vec<float>{1, 0, 0}, -mc::pi_over_2);

    for (int x = 0; x < n; ++x) {

        sm::vec<int, 2> xy = { x, 0 };

        for (int y = 0; y < n; ++y) {

            // From x, y on the square, compute position, direction on the sphere
            xy[1] = y;

            sm::vec<float, 2> polar = cart_to_polar (xy, n);

            // Debug polar with quiver visual
            coords[n * y + x] = {0,0,0};
            quivs[n * y + x] = { polar[0] * std::sin (polar[1]), polar[0] * std::cos (polar[1]), 0.0f};

            // Set colours from the flat polar positions
            clrs[n * y + x] = quivs[n * y + x].length();

            sm::vec<float, 3> sph = polar_to_sph (polar, eye_r);
            sm::vec<float, 3> xyz = sph.spherical_to_cartesian();

            // Rotate xyz about x axis
            xyz = (rotn * xyz).less_one_dim();

            s_coords[n * y + x] = xyz;
            if (!(xyz[0] == 0.0f)) {
                std::cout << xyz[0] << " " << xyz[1] << " " << xyz[2] << " ";
                xyz.renormalize();
                s_quivs[n * y + x] = xyz;
                std::cout << xyz[0] << " " << xyz[1] << " " << xyz[2] << " "
                      << acceptance_angle << " " << focal_offset << "\n";
            }
        }
    }

    sm::vec<float, 3> offset = {};
    offset[1] -= 1.25f * eye_r;
    auto vmp = std::make_unique<mplot::QuiverVisual<float>>(&coords, offset, &quivs, mplot::ColourMapType::Jet);
    v.bindmodel (vmp);
    vmp->setScalarData (&clrs);
    vmp->quiver_length_gain = 1.0f;
    vmp->fixed_quiver_thickness = 0.003f; // Also possible to request a fixed thickness
    vmp->shapesides = 4;
    vmp->finalize();
    v.addVisualModel (vmp);

    offset[1] += 2.5f * eye_r;
    vmp = std::make_unique<mplot::QuiverVisual<float>>(&s_coords, offset, &s_quivs, mplot::ColourMapType::Jet);
    v.bindmodel (vmp);
    vmp->setScalarData (&clrs);
    vmp->fixed_length = 0.25f;
    vmp->fixed_quiver_thickness = 0.003f; // Also possible to request a fixed thickness
    vmp->shapesides = 4;
    vmp->finalize();
    v.addVisualModel (vmp);

    v.keepOpen();

    return 0;
}
