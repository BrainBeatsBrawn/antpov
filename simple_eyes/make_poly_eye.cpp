#include <string>
#include <iostream>
#include <sm/geometry>

int main (int argc, char **argv)
{
    int iterations = 4; // Increase for finer subdivision

    sm::geometry::icosahedral_geodesic<double> geo = sm::geometry::make_icosahedral_geodesic<double> (iterations);

    constexpr float acceptance_angle = 0.15f;
    constexpr float focal_offset = 0.1f;
    constexpr float radius = 0.2f;

    for (unsigned int i = 0; i < geo.poly.vertices.size(); ++i) {
        std::string ntxt = geo.poly.vertices[i].str_comma_separated(' '); // normals (vertices were already normalized)
        std::string vtxt = (geo.poly.vertices[i] * radius).str_comma_separated(' ');
        std::cout << vtxt << " " << ntxt << " " << acceptance_angle << " " << focal_offset << std::endl;
    }

    return 0;
}
