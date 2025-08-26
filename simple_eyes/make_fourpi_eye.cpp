// Make a Fourpi eye
#include <sm/mathconst>
#include <sm/vec>
#include <cmath>
#include <iostream>

#include <mplotext/healpix/doublepix.h>

int main (int argc, char** argv)
{
    int nside = 32;
    if (argc > 1) { nside = std::atoi (argv[1]); }

    if (!hp::am::is_power_of_two (nside)) {
        std::cerr << nside << " is not a power of 2. Try again.\n";
        return -1;
    }

    // fourpi radius
    constexpr float fourpi_rad = 0.01f;

    constexpr float focal_offset = 0.0f;

    // For each pixel in a double healpix, generate the xyz/acceptance angle/direction
    double ddx = 0.5;
    double ddy = 0.5;
    double x = 0, y = 0, z = 0;

    // Find neighbour of index 0 to get the acceptance angle
    int64_t np = mplotext::double_get_neighbour(0, mplotext::neighbourDir_NE, nside);
    mplotext::double_to_xyz (0, nside, ddx, ddy, &x, &y, &z);
    sm::vec<float> zeropos = sm::vec<double>({x, y, z}).as_float();
    mplotext::double_to_xyz (np, nside, ddx, ddy, &x, &y, &z);
    sm::vec<float> neighpos = sm::vec<double>({x, y, z}).as_float();

    // Find angle between zeropos and neighpos to be the acceptance angle
    float acceptance_angle = zeropos.angle (neighpos);

    int64_t n_p = mplotext::double_num_pixels (nside);
    for (int64_t p = 0; p < n_p; ++p) {
        // Get location
        mplotext::double_to_xyz (p, nside, ddx, ddy, &x, &y, &z);
        // Convert it into a sm::vec
        sm::vec<float> vpf = sm::vec<double>({x, y, z}).as_float();
        vpf *= fourpi_rad;
        // direction same as position, but normalized
        sm::vec<float> dpf = vpf;
        dpf.renormalize();

        std::cout << vpf[0] << " " << vpf[1] << " " << vpf[2] << " "
                  << dpf[0] << " " << dpf[1] << " " << dpf[2] << " "
                  << acceptance_angle << " " << focal_offset << "\n";
    }

    return 0;
}
