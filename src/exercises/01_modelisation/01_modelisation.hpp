#pragma once

#include "../../exercises/base_exercise/base_exercise.hpp"

#ifdef INF443_01_MODELISATION

// Stores some parameters that can be set from the GUI
struct gui_scene_structure
{
    bool wireframe = false;

    bool terrain   = true;
    bool texture = true;
    bool texture_terrain = true;
    bool tree = true;
    bool rock1 = true;
    bool rock2 = true;
    bool skybox = true;
    bool snowman = true;
    bool snow = true;

    float height = 0.6f;
    float scaling = 3.0f;
    int octave = 7;
    float persistency = 0.4f;
};



struct trajectory_structure
{
    std::vector<vcl::vec3> position;
    std::vector<float> time;
};



struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
};



struct scene_exercise : base_scene_exercise
{

    /** A part must define two functions that are called from the main function:
     * setup_data: called once to setup data before starting the animation loop
     * frame_draw: called at every displayed frame within the animation loop
     *
     * These two functions receive the following parameters
     * - shaders: A set of shaders.
     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
     * - data: The part-specific data structure defined previously
     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
    */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui(vcl::timer_event& timer2);



    // visual representation of a surface
    vcl::mesh_drawable terrain;
    GLuint texture_terrain;
    void display_terrain(std::map<std::string,GLuint>& shaders, scene_structure& scene);

    vcl::mesh_drawable trunc;
    vcl::mesh_drawable foliage;
    vcl::mesh_drawable foliage2;
    std::vector<vcl::vec3> position;
    void update_tree_position();
    void display_tree(std::map<std::string,GLuint>& shaders, scene_structure& scene);

    // billbard_rock
    vcl::mesh_drawable billboard_surface;
    vcl::mesh create_billboard_surface();
    GLuint texture_rock1_billboard;
    GLuint texture_rock2_billboard;
    std::vector<vcl::vec3> rock1_position;
    std::vector<vcl::vec3> rock2_position;
    void update_rock1_position();
    void update_rock2_position();
    void display_rock1(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void display_rock2(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    // skybox
    vcl::mesh_drawable skybox;
    GLuint texture_skybox;
    void display_skybox(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    // bonhomme de neige
    vcl::mesh_drawable_hierarchy snowman;
    std::vector<vcl::vec3> snowman_position;
    void update_snowman_position();
    void display_snowman(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    // teleski
    vcl::mesh_drawable_hierarchy hierarchy;    // skieur
    vcl::mesh_drawable_hierarchy teleski;
    vcl::mesh_drawable_hierarchy teleski_bout;
    vcl::mesh_drawable_hierarchy perch;
    void display_skieur_teleski(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    // trajectoires
    vcl::mesh_drawable surface_bis;                            // moving point
    vcl::mesh_drawable sphere;                             // keyframe samples
    vcl::segment_drawable_immediate_mode segment_drawer;   // used to draw segments between keyframe samples
    vcl::curve_dynamic_drawable trajectory;           // Draw the trajectory of the moving point as a curve
    vcl::curve_dynamic_drawable remontee;


    // Data (p_i,t_i)
    std::vector<vcl::vec3> keyframe_position; // Given positions
    std::vector<float> keyframe_time;         // Time at given positions

    // Data (p_iteleski,t_iteleski)
    std::vector<vcl::vec3> keyframe_position_tele; // Given positions
    int picked_object = -1;
    void update_trajectory();
    void display_trajectory(std::map<std::string,GLuint>& shaders, scene_structure& scene);

    // Called every time the mouse is clicked
    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);
    // Called every time the mouse is moved
    void mouse_move(scene_structure& scene, GLFWwindow* window);
    // visual representation of a surface


    // snow
    std::list<particle_structure> particles; // Storage of all currently active particles
    vcl::mesh_drawable sphere2; // Visual representation of a particle - a sphere
    vcl::timer_event timer2;    // Timer allowing to indicate periodic events
    void display_snow(std::map<std::string,GLuint>& shaders, scene_structure& scene);













    std::vector<vcl::vec3> position_bill;

    vcl::mesh_drawable surface;
    GLuint texture_id;
    GLuint texture_id_bill;




    vcl::mesh_drawable ground;



    // Store the index of a selected sphere
    //int picked_object;
    //float mu;





    gui_scene_structure gui_scene;

    vcl::timer_interval timer;
};

#endif


