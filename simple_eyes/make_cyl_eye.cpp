// Make a cylindrical eye for our ant
#include <cmath>
#include <iostream>

import sm.mathconst;

int main()
{
    using mc = sm::mathconst<float>;

    constexpr int n_per_ring = 360;
    constexpr int n_rings = 90;
    // ring radius in m
    constexpr float ring_rad = 0.001f;
    // Angle between elements in the ring is the acceptance angle
    constexpr float acceptance_angle = mc::two_pi / n_per_ring;
    // Compute to be same as ommatidium to ommatidium in the ring
    constexpr float ring_to_ring = (mc::pi * ring_rad * 2.0f) / static_cast<float>(n_per_ring);

    // A truly cylindrical array isn't much use as an eye. If you want that, set this to
    // 0.0f. Otherwise, give an angle in radians that is the vertical field of view for the
    // cyl. array
    constexpr float vertical_array_angle = mc::pi_over_2;
    constexpr float vertical_el_angle = vertical_array_angle / (n_rings - 1);
    constexpr float start_angle = -vertical_array_angle / 2.0f;

    // Position
    float xp = 0.0f;
    float yp = 0.0f;
    float zp = 0.0f;
    // Direction
    float xd = 0.0f;
    float yd = 0.0f;
    float zd = 0.0f;

    for (int r = 0; r < n_rings; ++r) {
        yp = ring_to_ring * r;
        float current_v_angle = start_angle + (r-1) * vertical_el_angle;
        yd = std::sin (current_v_angle);
        // Focal point always brings these 'ommatidia' to the centre of the cylinder
        float f = ring_rad / std::cos (current_v_angle);

        for (int el = 0; el < n_per_ring; ++el) {

            // z and x
            float theta = -el * acceptance_angle + sm::mathconst<float>::pi;
            zd = std::cos (theta);
            xd = std::sin (theta);

            zp = ring_rad * zd;
            xp = ring_rad * xd;


            std::cout << xp << " " << yp << " " << zp << " "
                      << xd << " " << yd << " " << zd << " "
                      << acceptance_angle << " " << f << "\n";
        }
    }

    return 0;
}
