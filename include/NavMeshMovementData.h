namespace mplot
{
    // The data we pass to NavMesh::compute_mesh_movement, so I can save, say the last 100 movements
    // in a deque and then save/load and replay
    struct NavMeshMovementData
    {
        sm::vec<float> mv_camframe = {};
        sm::mat<float, 4> cam_to_scene = {};
        sm::mat<float, 4> model_to_scene = {};
        uint32_t ti0 = {};
        float hoverheight = 0.0f;

        bool operator== (const NavMeshMovementData& rhs) const noexcept
        {
            return (mv_camframe == rhs.mv_camframe
                    && cam_to_scene == rhs.cam_to_scene
                    && model_to_scene == rhs.model_to_scene
                    && ti0 == rhs.ti0
                    && hoverheight == rhs.hoverheight);
        }

        bool operator!= (const NavMeshMovementData& rhs) const noexcept { return !(*this == rhs); }

        // Save data into the already-open hdfdata object
        void save (sm::hdfdata& hd, const uint32_t data_index)
        {
            std::stringstream pcom;
            pcom << "/navmeshmv_" << data_index;
            std::string s = pcom.str() + std::string("/mv_camframe");
            hd.add_contained_vals (s.c_str(), mv_camframe);
            s = pcom.str() + std::string("/cam_to_scene");
            hd.add_contained_vals (s.c_str(), cam_to_scene.arr);
            s = pcom.str() + std::string("/model_to_scene");
            hd.add_contained_vals (s.c_str(), model_to_scene.arr);
            s = pcom.str() + std::string("/ti0");
            hd.add_val (s.c_str(), ti0);
            s = pcom.str() + std::string("/hoverheight");
            hd.add_val (s.c_str(), hoverheight);
        }

        // Load data
        void load (sm::hdfdata& hd, const uint32_t data_index)
        {
            std::stringstream pcom;
            pcom << "/navmeshmv_" << data_index;
            std::string s = pcom.str() + std::string("/mv_camframe");
            hd.read_contained_vals (s.c_str(), mv_camframe);
            s = pcom.str() + std::string("/cam_to_scene");
            hd.read_contained_vals (s.c_str(), cam_to_scene.arr);
            s = pcom.str() + std::string("/model_to_scene");
            hd.read_contained_vals (s.c_str(), model_to_scene.arr);
            s = pcom.str() + std::string("/ti0");
            hd.read_val (s.c_str(), ti0);
            s = pcom.str() + std::string("/hoverheight");
            hd.read_val (s.c_str(), hoverheight);
        }
    };
} // mplot
