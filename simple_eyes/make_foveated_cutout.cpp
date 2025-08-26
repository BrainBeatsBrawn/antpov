// Make a Fourpi eye
#include <sm/mathconst>
#include <sm/vec>
#include <sm/vvec>
#include <cmath>
#include <iostream>

#include <mplotext/healpix/doublepix.h>

sm::vec<float,2> polar_to_cartesian_2d (float ring_radius, float angle)
{
    sm::vec<float, 2> cart_loc = {0.0f, 0.0f};
    cart_loc[0] = ring_radius * std::cos(angle);
    cart_loc[1] = ring_radius * std::sin(angle);

    return cart_loc;
}

sm::vvec<sm::vec<float,3>> build_eye (int n_ang, int n_circ, float pixw)
{
    sm::vvec<float> angle_vector;
    // Make container for 2D cartesian position of cell and the receptive field radius of cell {x, y, radius}.
    sm::vvec<sm::vec<float,3>> sample_data;

    // Initialise eye model to the center of the supplied grid
    float angle_increment = sm::mathconst<float>::two_pi / n_ang;
    angle_vector.arange (0, sm::mathconst<float>::two_pi, sm::mathconst<float>::two_pi / (2 * n_ang));
    float b = 1.0f + (sm::mathconst<float>::pi / (std::sqrt(3.0f) * n_ang));
    float r_fovea = 0.1f * n_ang / sm::mathconst<float>::two_pi;
    sample_data.resize (n_ang * n_circ);
    float angle = 0.0f;
    float ring_radius = r_fovea;
    sm::vec<float, 2> point_loc_cart;
    float sample_radius;

    // Populate log-polar sampled region
    int i = 0;
    for (int c = 0; c < n_circ; c++){
        ring_radius = pixw *  r_fovea * std::pow(b, c);
        for (int a = 0; a < n_ang; a++){
            angle = a * angle_increment;
            if (c % 2 == 1){
                angle += angle_increment / 2;
            }
            // Get metric location of this polar coord point [radius, angle] in the cartesian [x,y] coordinates with respect to the center of the current sample circle
            point_loc_cart = polar_to_cartesian_2d (ring_radius, angle);
            // Compute the radius of the current sample circle
            sample_radius = (2.0f / 3.0f) * (ring_radius * sm::mathconst<float>::pi / n_ang);
            sm::vec<float,3> tmp_cs = {point_loc_cart[0], point_loc_cart[1], sample_radius};
            sample_data.at(i++) = tmp_cs;
        }
    }
    return sample_data;
}

sm::vec<float,3> mercator_projection (float x, float y, float r_sph)
{
    float longitude = x / r_sph;
    float latitude = 2.0f * std::atan (std::exp(y / r_sph)) - sm::mathconst<float>::pi_over_2;

    sm::vec<float,3> polar_coords_3d = {longitude, latitude, r_sph};

    return polar_coords_3d;
}

sm::vec<float, 3> polar_to_cartesian_3d (float longitude, float latitude, float radius)
{
    sm::vec<float,3> cart_3d;

    float z = radius * cos(latitude) * cos(longitude);
    float y = radius * cos(latitude) * sin(longitude);
    float x = radius * sin(latitude);

    cart_3d = {x, y, z};

    return cart_3d;
}

float acceptance_angle_from_sample_radius (float sample_rad, float eye_rad)
{
    return 2.0f * 2.0f * std::asin (sample_rad / (2.0f * eye_rad));
}


int main ()
{
    // Build all cells of the log polar eye
    int n_ang = 16;
    int n_circ = 60;
    int sim_img_w = 1418;
    float pixw = sm::mathconst<float>::pi_over_4 / sim_img_w;

    sm::vvec<sm::vec<float,3>> sample_data = build_eye (n_ang, n_circ, pixw);
    float eye_radius = 0.1;
    sm::vec<float> vpf = {0.0f, 0.0f, 0.0f};
    sm::vec<float,3> polar_3d;

    // Iterate through all cells
    for (const sm::vec<float,3>& sd : sample_data) {

        // Compute acceptance angle from sample radius
        float theta = acceptance_angle_from_sample_radius (sd[2], eye_radius);

        // std::cout << "X : " << sd[0] << " ,  Y : " << sd[1] << std::endl << std::endl;

        polar_3d = mercator_projection (sd[0], sd[1], eye_radius);

        // std::cout << "long : " << polar_3d[0] << " ,  Lattitude : " << polar_3d[1] << " ,  Radius : " << polar_3d[2] << std::endl << std::endl;

        vpf = polar_to_cartesian_3d (polar_3d[0], polar_3d[1], polar_3d[2]);

        // direction same as position, but normalized
        sm::vec<float> dpf = vpf;
        dpf.renormalize();

        if (vpf[0] > 0 && vpf[1] > 0) { continue; }

        std::cout << vpf[0] << " " << vpf[1] << " " << vpf[2] << " "
                  << dpf[0] << " " << dpf[1] << " " << dpf[2] << " "
                  << theta << " " << 0.0f << "\n";

    }
    return 0;
}
