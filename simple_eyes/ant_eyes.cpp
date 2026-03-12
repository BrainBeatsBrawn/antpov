/*
 * Make some toy ant eyes, using data from
 * Schwarz et al. - 2011 - The properties of the visual system in the Austral.pdf
 * (Species: Melophorus bagoti, an Australian desert ant)
 *
 * Eyes are about 1.5 mm apart and are about 0.3 mm in horizontal width and 0.25 mm in
 * vertical height. They have about 421-590 ommatidia which have an acceptance angle of
 * 2.9 degrees avg and an interommatidial angle of 3.7 deg, but the range is from 6 deg
 * at anterior. 5 lateral and 3 posterior.
 *
 * This program will output an eye file in metres, because our environment models will be specified
 * in metres.
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>

import sm.mathconst;
import sm.mat;
import sm.scale;
import sm.vec;
import sm.vvec;
import sm.hexgrid;
import sm.hexgrid.hdf;
import sm.hdfdata;

import mplot.visual;
import mplot.scattervisual;
import mplot.quivervisual;
import mplot.hexgridvisual;
import mplot.lengthscalevisual;

import antpov.doublehexgridvisual;
import antbodyvisual;

enum class spherical_projection
{
    mercator,
    equirectangular,
    cassini,
    splodge
};

// You can hide the RGB arrows and/or the second eye
constexpr bool two_eyes = true;
constexpr bool show_rgb = false;
constexpr bool show_scatter = false;
constexpr bool show_antoinette = true;

void buildModel (mplot::Visual<>& v, const sm::hexgrid& hg,
                 sm::vvec<sm::vec<float, 3>>& sphere_coords,
                 sm::vvec<sm::vec<float, 3>>& sphere_coords2,
                 sm::vvec<sm::vec<float, 3>>& eye_coords,
                 sm::vvec<sm::vec<float, 3>>& neighb_r,
                 sm::vvec<sm::vec<float, 3>>& neighb_g,
                 sm::vvec<sm::vec<float, 3>>& neighb_b,
                 sm::vvec<sm::vec<float, 3>>& neighb_r2,
                 sm::vvec<sm::vec<float, 3>>& neighb_g2,
                 sm::vvec<sm::vec<float, 3>>& neighb_b2)
{
    v.clear();

    // Positions for two 'eyes'. Actually, sepn is coded into hex vertex positions, so position of each eye model is the same.
    sm::vec<float, 3> eyepos = { 0.0f, 0.0f, 0.0f };

    sm::scale<float> clr_scale;
    clr_scale.set_params (1.0f, 0.0f);

    sm::vvec<float> data;
    data.linspace (0, 1, hg.num());
    sm::vvec<float> datatwice(data);
    datatwice.insert (datatwice.end(), data.begin(), data.end());

    // First eye
    constexpr float hex_d_prop = 0.2f;
    if constexpr (show_scatter) {
        auto sv = std::make_unique<mplot::ScatterVisual<float>> (eyepos);
        sv->set_parent (v.get_id());
        sv->setDataCoords (&sphere_coords);
        sv->setScalarData (&data);
        sv->radiusFixed = hg.getd() * hex_d_prop;
        sv->colourScale = clr_scale;
        sv->cm.setType (mplot::ColourMapType::Jet);
        sv->finalize();
        v.addVisualModel (sv);
    }

    // Add a DoubleHexGridVisual view of the eye pair (only works for two_eyes == true)
    if constexpr (two_eyes == true) {
        sm::vec<float, 3> offset = { 0.0f, 0.0f, 0.0f };
        auto hgv = std::make_unique<mplot::DoubleHexGridVisual<float,mplot::gl::version_4_1>>(&hg, eyepos+offset);
        hgv->set_parent (v.get_id());
        hgv->setDataCoords (&eye_coords); // pass combined coords
        hgv->setScalarData (&datatwice);          // pass combined data
        hgv->hexVisMode = mplot::HexVisMode::HexInterp; // HexInterp or mplot::HexVisMode::Triangles for a smoother surface plot
        hgv->cm.setType (mplot::ColourMapType::Jet);
        hgv->finalize();
        v.addVisualModel (hgv);
    }

    // Second eye
    if constexpr (two_eyes && show_scatter) {
        auto sv = std::make_unique<mplot::ScatterVisual<float>> (eyepos);
        sv->set_parent (v.get_id());
        sv->setDataCoords (&sphere_coords2);
        sv->setScalarData (&data);
        sv->radiusFixed = hg.getd() * hex_d_prop;
        sv->colourScale = clr_scale;
        sv->cm.setType (mplot::ColourMapType::Jet);
        sv->finalize();
        v.addVisualModel (sv);
    }

    if constexpr (show_rgb) {
        // Eye 1 RGB directions
        sm::vvec<float> clrs (neighb_r.size(), 0.0f); // red
        auto vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords, eyepos, &neighb_r,
                                                                mplot::ColourMapType::Rainbow);
        vmp->set_parent (v.get_id());
        vmp->scalarData = &clrs;
        vmp->colourScale.compute_scaling (0, 1);
        vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
        vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
        vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
        vmp->finalize();
        v.addVisualModel (vmp);

        clrs.set_from (0.33333f); // green
        vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords, eyepos, &neighb_g,
                                                           mplot::ColourMapType::Rainbow);
        vmp->set_parent (v.get_id());
        vmp->scalarData = &clrs;
        vmp->colourScale.compute_scaling (0, 1);
        vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
        vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
        vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
        vmp->finalize();
        v.addVisualModel (vmp);

        clrs.set_from (0.66667f); // blue
        vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords, eyepos, &neighb_b,
                                                           mplot::ColourMapType::Rainbow);
        vmp->set_parent (v.get_id());
        vmp->scalarData = &clrs;
        vmp->colourScale.compute_scaling (0, 1);
        vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
        vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
        vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
        vmp->finalize();
        v.addVisualModel (vmp);

        if constexpr (two_eyes) {
            // Eye 2 RGB directions
            clrs.set_from (0.0f); // red
            vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords2, eyepos, &neighb_r2,
                                                               mplot::ColourMapType::Rainbow);
            vmp->set_parent (v.get_id());
            vmp->scalarData = &clrs;
            vmp->colourScale.compute_scaling (0, 1);
            vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
            vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
            vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
            vmp->finalize();
            v.addVisualModel (vmp);

            clrs.set_from (0.33333f); // green
            vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords2, eyepos, &neighb_g2,
                                                               mplot::ColourMapType::Rainbow);
            vmp->set_parent (v.get_id());
            vmp->scalarData = &clrs;
            vmp->colourScale.compute_scaling (0, 1);
            vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
            vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
            vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
            vmp->finalize();
            v.addVisualModel (vmp);

            clrs.set_from (0.66667f); // blue
            vmp = std::make_unique<mplot::QuiverVisual<float>>(&sphere_coords2, eyepos, &neighb_b2,
                                                               mplot::ColourMapType::Rainbow);
            vmp->set_parent (v.get_id());
            vmp->scalarData = &clrs;
            vmp->colourScale.compute_scaling (0, 1);
            vmp->do_quiver_length_scaling = false; // Don't (auto)scale the lengths of the vectors
            vmp->quiver_length_gain = 0.5f;        // Apply a fixed gain to the length of the quivers on screen
            vmp->fixed_quiver_thickness = 0.01f/5; // Fixed quiver thickness
            vmp->finalize();
            v.addVisualModel (vmp);
        }
    }

    // This will be a 1 mm bar
    auto lsv = std::make_unique<mplot::LengthscaleVisual<>>(sm::vec<>{-0.5f, -2.0f, 0});
    lsv->set_parent (v.get_id());
    lsv->label = "1 mm";
    lsv->finalize();
    v.addVisualModel (lsv);

    if constexpr (show_antoinette) {
        auto av = std::make_unique<biosim::AntBodyVisual<>>();
        av->set_parent (v.get_id());
        av->finalize();
        auto avp = v.addVisualModel (av);
        avp->scaleViewMatrix (1000);
    }
}

int main (int argc, char** argv)
{
    constexpr spherical_projection proj =  spherical_projection::splodge;

    using mc = sm::mathconst<float>;

    // radius of sphere. Compound-ray eye files are in mm.
    constexpr float r_sph = 0.17f; // Meaning 170 um from "Eye radius Lateral" row, Table 1

    // A spacing between the eyes inserted into abs position so it shows in compound-ray
    float eye_x_loc = 3.0f * r_sph; // 3 is a guess

    // Make a HexGrid of width similar to sphere
    constexpr float hex_d = 0.0126; // 12.6 um mean
    constexpr float hex_span = mc::two_pi * r_sph;
    sm::hexgrid hg(hex_d, hex_span, 0.0f);
    // the argument is the circlular boundary radius (0.5 * pi * r_sph) should wrap up to half way round sphere
    if constexpr (proj == spherical_projection::splodge) {
        hg.setCircularBoundary (0.85f * r_sph);
    } else {
        hg.setCircularBoundary (0.5f * mc::pi * r_sph);
    }

    // hg has d_x and d_y. Can make up a new container of 3D locations for each hex.
    sm::vvec<sm::vec<float, 3>> sphere_coords(hg.num());
    sm::vvec<sm::vec<float, 3>> sphere_coords2(hg.num());
    for (unsigned int i = 0u; i < hg.num(); ++i) {
        // This is the inverse Mercator projection.
        // See https://stackoverflow.com/questions/12732590/how-map-2d-grid-points-x-y-onto-sphere-as-3d-points-x-y-z
        sm::vec<float, 2> xy = { hg.d_x[i], hg.d_y[i] };

        float longitude = 0.0f; // or lambda
        float latitude = 0.0f;  // or phi
        if constexpr (proj == spherical_projection::equirectangular) {
            float phi0 = 0.0f;
            float phi1 = 0.0f;
            float lambda0 = 0.0f;
            longitude = xy[0] / (r_sph * std::cos(phi1)) + lambda0;
            latitude = xy[1] / r_sph + phi0;
        } else if constexpr (proj == spherical_projection::cassini) {
            // Spherical Cassini
            float phi0 = 0.0f;
            float lambda0 = 0.0f;
            float D = xy[1] / r_sph + phi0;
            longitude = lambda0 + std::atan2 (std::tan(xy[0]/r_sph), std::cos(D));
            latitude = std::asin (std::sin(D) * std::cos(xy[0]/r_sph));

        } else if constexpr (proj == spherical_projection::splodge) {
            // It's all below
        } else { // Default Mercator projection
            longitude = xy[0] / r_sph;
            latitude = 2.0f * std::atan (std::exp(xy[1]/r_sph)) - mc::pi_over_2;
        }

        if constexpr (proj == spherical_projection::splodge) {

            // Instead of a *sphere* we want some sort of ellipsoid structure with a variable radius.

            // In the splodge projection we just 'throw' the 2D plane onto a sphere
            sphere_coords[i][1] = xy[0];
            sphere_coords[i][2] = xy[1];
            float z_sq = r_sph * r_sph - xy.sq().sum();
            float z_sph = 0.0f;
            if (z_sq >= 0.0f)  {
                z_sph = std::sqrt (z_sq);
            } else {
                z_sph = -std::sqrt (-z_sq); // Anything beyond the edge of r_sph
            }
            sm::vec<float> prerotate = { eye_x_loc + z_sph, xy[0], xy[1] };
            sm::mat<float, 4> m1;
            m1.rotate (sm::vec<float>{0, 1, 0}, mc::pi_over_6 + mc::pi);
            sphere_coords[i] = (m1 * prerotate).less_one_dim();

        } else { // it's a serious projection
            float coslat = std::cos (latitude);
            float sinlat = std::sin (latitude);
            float coslong = std::cos (longitude);
            float sinlong = std::sin (longitude);
            sphere_coords[i] =  { eye_x_loc + r_sph * coslat * coslong, r_sph * coslat * sinlong , r_sph * sinlat };
        }

        // Second eye just has x reversed:
        sphere_coords2[i] = sphere_coords[i] * sm::vec<float>{-1, 1, 1};
    }
    // All coordinates in a single memory structure
    sm::vvec<sm::vec<float, 3>> eye_coords (sphere_coords);
    eye_coords.insert (eye_coords.end(), sphere_coords2.begin(), sphere_coords2.end());

    // 'R' neighbours (neighbour east) on the sphere
    sm::vvec<sm::vec<float, 3>> neighb_r(hg.num(), sm::vec<float, 3>{0,0,0});
    sm::vvec<sm::vec<float, 3>> neighb_g(hg.num(), sm::vec<float, 3>{0,0,0});
    sm::vvec<sm::vec<float, 3>> neighb_b(hg.num(), sm::vec<float, 3>{0,0,0});
    for (unsigned int i = 0; i < hg.num(); ++i) {

        if (hg.d_ne[i] != -1) {
            neighb_r[i] = sphere_coords[hg.d_ne[i]] - sphere_coords[i];
        }
        if (hg.d_nne[i] != -1) {
            neighb_g[i] = sphere_coords[hg.d_nne[i]] - sphere_coords[i];
        }
        if (hg.d_nnw[i] != -1) {
            neighb_b[i] = sphere_coords[hg.d_nnw[i]] - sphere_coords[i];
        }
    }

    // Neighbour vectors for sphere 2
    sm::vvec<sm::vec<float, 3>> neighb_r2(hg.num(), sm::vec<float, 3>{0,0,0});
    sm::vvec<sm::vec<float, 3>> neighb_g2(hg.num(), sm::vec<float, 3>{0,0,0});
    sm::vvec<sm::vec<float, 3>> neighb_b2(hg.num(), sm::vec<float, 3>{0,0,0});
    for (unsigned int i = 0; i < hg.num(); ++i) {
        if (hg.d_ne[i] != -1) {
            neighb_r2[i] = sphere_coords2[hg.d_ne[i]] - sphere_coords2[i];
        }
        if (hg.d_nne[i] != -1) {
            neighb_g2[i] = sphere_coords2[hg.d_nne[i]] - sphere_coords2[i];
        }
        if (hg.d_nnw[i] != -1) {
            neighb_b2[i] = sphere_coords2[hg.d_nnw[i]] - sphere_coords2[i];
        }
    }

    // Save data out that will give the neighbour information in the geoflow program
    sm::hexgrid_save (hg, "ant_eyes_dhex_hexgrid.h5");
    {
        sm::hdfdata d("ant_eyes_dhex_3d_coords.h5", std::ios::out | std::ios::trunc);
        d.add_contained_vals ("/neighb_r", neighb_r); // Will I need to access the neigb vectors?
        d.add_contained_vals ("/neighb_g", neighb_g);
        d.add_contained_vals ("/neighb_b", neighb_b);
        d.add_contained_vals ("/neighb_r2", neighb_r2);
        d.add_contained_vals ("/neighb_g2", neighb_g2);
        d.add_contained_vals ("/neighb_b2", neighb_b2);
        d.add_contained_vals ("/eye_coords", eye_coords);
        d.add_contained_vals ("/sphere_coords", sphere_coords);
        d.add_contained_vals ("/sphere_coords2", sphere_coords2);
    }

    // Will call this fn once for each eye
    auto output_coords = [hg](const sm::vvec<sm::vec<float, 3>>& coords, sm::vec<float, 3> eyeoffset, std::ofstream& fout) {
        constexpr float focal_offset = r_sph;
        constexpr float radius = r_sph;
        constexpr float mm_to_metres = 0.001f;
        constexpr float acceptance_angle_multiplier = 1.1f;
        for (unsigned int i = 0; i < coords.size(); ++i) {
            auto norm = coords[i];
            norm.renormalize();
            float acceptance_angle = 1.0f;
            auto c1 = radius * (coords[i] - eyeoffset);
            if (hg.d_ne[i] != -1) {
                auto c2 = radius * (coords[hg.d_ne[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } else if (hg.d_nne[i] != -1) {
                auto c2 = radius * (coords[hg.d_nne[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } else if (hg.d_nnw[i] != -1) {
                auto c2 = radius * (coords[hg.d_nnw[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } else if (hg.d_nw[i] != -1) {
                auto c2 = radius * (coords[hg.d_nw[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } else if (hg.d_nsw[i] != -1) {
                auto c2 = radius * (coords[hg.d_nsw[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } else if (hg.d_nse[i] != -1) {
                auto c2 = radius * (coords[hg.d_nse[i]] - eyeoffset);
                acceptance_angle *= c1.angle(c2) * 2.0f * acceptance_angle_multiplier;
            } // else acceptange angle will be unchanged at 1.0f

            std::string ntxt = norm.str_comma_separated(' '); // normals (vertices were already normalized)
            std::string vtxt = (coords[i] * mm_to_metres).str_comma_separated(' ');
            fout << vtxt << " " << ntxt << " " << acceptance_angle << " " << focal_offset * mm_to_metres << std::endl;
        }
    };
    std::ofstream fout ("ant_eyes_dhex.eye", std::ios::out | std::ios::trunc);
    if (fout.is_open()) {
        output_coords (sphere_coords, sm::vec<float, 3>{-eye_x_loc, 0, 0}, fout);
        output_coords (sphere_coords2, sm::vec<float, 3>{eye_x_loc, 0, 0}, fout);
    }

    constexpr bool show_version_stdout = false;
    mplot::Visual v(1024, 768, "Desert Anty Eyes", show_version_stdout);
    v.setSceneTrans (sm::vec<float,3>{ float{-1.91057}, float{1.07071}, float{-23.0144} });
    v.setSceneRotation (sm::quaternion<float>{ float{0.743712}, float{-0.0381476}, float{-0.666242}, float{0.0394651} });

    v.userInfoStdout (false);
    v.showCoordArrows (true);
    v.coordArrowsInScene (false);
    v.lightingEffects();

    buildModel (v, hg, sphere_coords, sphere_coords2, eye_coords,
                neighb_r, neighb_g, neighb_b, neighb_r2, neighb_g2, neighb_b2);

    int fcount = 0;
    while (!v.readyToFinish()) {
        v.waitevents (0.018);
        if (fcount % 60 == 0) { buildModel (v, hg, sphere_coords, sphere_coords2, eye_coords,
                                            neighb_r, neighb_g, neighb_b, neighb_r2, neighb_g2, neighb_b2); }
        v.render();
    }

    return 0;
}
