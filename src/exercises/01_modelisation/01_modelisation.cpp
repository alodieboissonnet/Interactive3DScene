
#include "01_modelisation.hpp"
#include <random>
#include <math.h>


#ifdef MODELISATION

// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;

// Uniform distribution in [0,1]
std::uniform_real_distribution<float> distrib(0.0,1.0);
std::default_random_engine generator;


float evaluate_terrain_z(float u, float v);
vec3 evaluate_terrain(float u, float v);
mesh create_terrain();

mesh create_cylinder(float radius, float height);
mesh create_cone(float radius, float height, float z_offset);
mesh create_tree_foliage(float radius, float height, float z_offset,float z);

vcl::mesh create_skybox();

mesh_drawable_hierarchy create_snowman();

mesh_drawable_hierarchy create_skier();
mesh_drawable_hierarchy create_button_lift();
mesh_drawable_hierarchy create_button_lift_bout();
mesh_drawable_hierarchy create_perch();

size_t index_at_value(float t, const std::vector<float>& vt);
vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3);
vec3 cardinal_spline_interpolation(const trajectory_structure& trajectory, float t);
vec3 cardinal_spline_derivative_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3);
vec3 cardinal_spline_derivative_interpolation(const trajectory_structure& trajectory, float t);


/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    // Setup initial camera mode and position
    scene.camera.camera_type = camera_control_spherical_coordinates;
    scene.camera.scale = 10.0f;
    scene.camera.apply_rotation(0,0,0,1.2f);


    // Create visual terrain surface and trees
    terrain = create_terrain();
    terrain.uniform_parameter.color = {1.0f,1.0f,1.0f};
    terrain.uniform_parameter.shading.specular = 0.0f; // non-specular terrain material
    terrain.uniform_parameter.shading.ambiant = 1.0f;
    terrain.uniform_parameter.shading.diffuse = 0.0f;

    trunc = create_cylinder(0.3f,4.0f);
    trunc.uniform_parameter.color = {0.7f,0.5f,0.5f};

    foliage = create_tree_foliage(1.5f,2.0f,0.4f,2.0f);
    foliage.uniform_parameter.color = {0.6f,0.85f,0.5f};

    foliage2 = create_tree_foliage(1.5f,2.0f,0.4f,2.0f);
    foliage2.uniform_parameter.color = {0.98f,0.98f,0.98f};
    foliage2.uniform_parameter.shading.specular = 0.0f;
    foliage2.uniform_parameter.shading.ambiant = 1.0f;
    foliage2.uniform_parameter.shading.diffuse = 0.0f;
    update_tree_position();

    // snowman
    snowman = create_snowman();
    update_snowman_position();


    // billboard_rock
    billboard_surface = create_billboard_surface();
    billboard_surface.uniform_parameter.shading  = {1,0,0};
    texture_rock1_billboard = texture_gpu(image_load_png("data/billboard_rock1.png"));
    texture_rock2_billboard = texture_gpu(image_load_png("data/billboard_rock2.png"));
    update_rock1_position();
    update_rock2_position();

    // Create a surface with (u,v)-texture coordinates
    mesh surface_cpu;
    surface_cpu.position     = {{-1,-1,0}, { 1,-1,0}, { 1, 1,0}, {-1, 1,0}};
    surface_cpu.connectivity = {{0,1,2}, {0,2,3}};
    surface_cpu.texture_uv = {{0,1}, {1,1}, {1,0}, {0,0}};

    surface = surface_cpu;

    // Load a texture image on GPU and stores its ID
    texture_id = texture_gpu( image_load_png("data/snow2.png") );


    // skybox
    skybox = create_skybox();
    skybox.uniform_parameter.shading = {1,0,0};
    skybox.uniform_parameter.rotation = rotation_from_axis_angle_mat3({1,0,0},-3.014f/2.0f);
    texture_skybox = texture_gpu(image_load_png("data/skybox.png"));


    // Initial Keyframe data (button_lift and skier)
    keyframe_position = {{-4,-9,evaluate_terrain_z(-4.f/40+0.5f,-9.f/40+0.5f)}, {-2,-5,evaluate_terrain_z(-2.f/40+0.5f,-5.f/40+0.5f)}, {0,-7,evaluate_terrain_z(0.f/40+0.5f,-7.f/40+0.5f)}, {4,-4,evaluate_terrain_z(4.f/40+0.5f,-4.f/40+0.5f)}, {8,-5,evaluate_terrain_z(8.f/40+0.5f,-5.f/40+0.5f)}, {12,-9,evaluate_terrain_z(12.f/40+0.5f,-9.f/40+0.5f)}, {8,-9,evaluate_terrain_z(8.f/40+0.5f,-9.f/40+0.5f)}, {4,-9,evaluate_terrain_z(4.f/40+0.5f,-9.f/40+0.5f)}, {0,-9,evaluate_terrain_z(0.f/40+0.5f,-9.f/40+0.5f)}, {-4,-9,evaluate_terrain_z(-4.f/40+0.5f,-9.f/40+0.5f)}, {-2,-5,evaluate_terrain_z(-2.f/40+0.5f,-5.f/40+0.5f)}, {0,-7,evaluate_terrain_z(0.f/40+0.5f,-7.f/40+0.5f)}};
    keyframe_time = {0,1,2,3,4,5,6,7,8,9,10,11};
    keyframe_position_tele = {{-4,-9,evaluate_terrain_z(-4.f/40+0.5f,-9.f/40+0.5f)}, {0,-9,evaluate_terrain_z(0.f/40+0.5f,-9.f/40+0.5f)}, {4,-9,evaluate_terrain_z(4.f/40+0.5f,-9.f/40+0.5f)}, {8,-9,evaluate_terrain_z(8.f/40+0.5f,-9.f/40+0.5f)}, {12,-9,evaluate_terrain_z(12.f/40+0.5f,-9.f/40+0.5f)}, {12,-9,evaluate_terrain_z(12.f/40+0.5f,-9.f/40+0.5f)}, {8,-9,evaluate_terrain_z(8.f/40+0.5f,-9.f/40+0.5f)}, {4,-9,evaluate_terrain_z(4.f/40+0.5f,-9.f/40+0.5f)}, {0,-9,evaluate_terrain_z(0.f/40+0.5f,-9.f/40+0.5f)}, {-4,-9,evaluate_terrain_z(-4.f/40+0.5f,-9.f/40+0.5f)}, {0,-9,evaluate_terrain_z(0.f/40+0.5f,-9.f/40+0.5f)}, {4,-9,evaluate_terrain_z(4.f/40+0.5f,-9.f/40+0.5f)}};


    // Set timer bounds
    //  To ease spline interpolation of a closed curve time \in [t_1,t_{N-2}]
    timer.t_min = keyframe_time[1];
    timer.t_max = keyframe_time[keyframe_time.size()-2];
    timer.t = timer.t_min;
    update_trajectory();

    // Prepare the visual elements
    surface_bis = mesh_primitive_sphere();
    surface_bis.uniform_parameter.color   = {0,0,1};
    surface_bis.uniform_parameter.scaling = 0.08f;

    sphere = mesh_primitive_sphere();
    sphere.uniform_parameter.color = {1,1,1};
    sphere.uniform_parameter.scaling = 0.05f;

    segment_drawer.init();



    // skier and button lift
    hierarchy = create_skier();
    perch = create_perch();
    button_lift = create_button_lift();
    button_lift_bout = create_button_lift_bout();


    // snow
    // Create mesh for particles represented as spheres
    const float r = 0.08f; // radius of spheres
    sphere2 = mesh_primitive_sphere(r);
    sphere2.uniform_parameter.color = {1.0f, 1.0f, 1.0f};

    // Delay between emission of a new particles
    timer2.periodic_event_time_step = 0.005f;

    timer.scale = 0.5f;
}



/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    set_gui(timer2);
    glEnable( GL_POLYGON_OFFSET_FILL ); // avoids z-fighting when displaying wireframe


    // Before displaying a textured surface: bind the associated texture id
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);


    display_terrain(shaders, scene);
    display_tree(shaders,scene);

    // skybox
    display_skybox(shaders, scene);

    // snowman
    display_snowman(shaders, scene);

    // billboard_rock
    display_rock1(shaders, scene);
    display_rock2(shaders, scene);

    // skier and button lift
    display_skier_button_lift(shaders,scene);
    display_trajectory(shaders,scene);

    // snow
    display_snow(shaders,scene);


    set_gui(timer2);

    // After the surface is displayed it is safe to set the texture id to a white image
    //  Avoids to use the previous texture for another object
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);


}


void scene_exercise::display_terrain(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(!gui_scene.terrain)
        return ;

    glPolygonOffset( 1.0, 1.0 );
    if(gui_scene.texture_terrain)
        glBindTexture(GL_TEXTURE_2D, texture_id);
    terrain.draw(shaders["mesh"], scene.camera);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);



    if( gui_scene.wireframe ){ // wireframe if asked from the GUI
        glPolygonOffset( 1.0, 1.0 );
        terrain.draw(shaders["wireframe"], scene.camera);
    }
}


void scene_exercise::display_tree(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(!gui_scene.tree)
        return ;

    int N=10;
    for (int i=0;i<N;i++){
        trunc.uniform_parameter.translation=position[i];
        trunc.draw(shaders["mesh"], scene.camera);
        foliage.uniform_parameter.translation=position[i];
        foliage.draw(shaders["mesh"], scene.camera);
        foliage2.uniform_parameter.translation=position[i];
        foliage2.uniform_parameter.translation.z += 0.1f;
        foliage2.draw(shaders["mesh"], scene.camera);
    }

    if( gui_scene.wireframe ){
        int N=10;
        for (int i=0;i<N;i++){
            trunc.uniform_parameter.translation=position[i];
            trunc.draw(shaders["wireframe"], scene.camera);
            foliage.uniform_parameter.translation=position[i];
            foliage.draw(shaders["wireframe"], scene.camera);
            foliage2.uniform_parameter.translation=position[i];
            foliage2.uniform_parameter.translation.z += 0.1f;
            foliage2.draw(shaders["wireframe"], scene.camera);
        }
    }
}


void scene_exercise::display_skybox(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(gui_scene.skybox)
    {
        if(gui_scene.texture)
            glBindTexture(GL_TEXTURE_2D,texture_skybox);
        skybox.uniform_parameter.scaling = 150.0f;
        skybox.uniform_parameter.translation = scene.camera.camera_position() + vec3(0,0,-50.0f);
        skybox.draw(shaders["mesh"], scene.camera);
        glBindTexture(GL_TEXTURE_2D,scene.texture_white);
    }
}


void scene_exercise::display_snowman(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(!gui_scene.snowman)
        return ;

    const vec3 offset_ground = vec3{0,0,-0.025f};
    const size_t N_snowman = snowman_position.size();
    for(size_t k=0; k<N_snowman; ++k)
    {
        const vec3& p = snowman_position[k];
        snowman.translation("bas") = p + offset_ground;
        snowman.rotation("bas") = rotation_from_axis_angle_mat3({0,0,1}, 3.14);

        glPolygonOffset( 1.0, 1.0 );
        snowman.draw(shaders["mesh"], scene.camera);
    }

    if( gui_scene.wireframe ){
        for(size_t k=0; k<N_snowman; ++k)
        {
            const vec3& p = snowman_position[k];
            snowman.translation("bas") = p + offset_ground;

            glPolygonOffset( 1.0, 1.0 );
            snowman.draw(shaders["wireframe"], scene.camera);
        }
    }
}


void scene_exercise::display_rock1(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(!gui_scene.rock1)
        return;

    glEnable(GL_BLEND);
    glDepthMask(false);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const size_t N_rock1 = rock1_position.size();
    if(gui_scene.texture)
        glBindTexture(GL_TEXTURE_2D, texture_rock1_billboard);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    billboard_surface.uniform_parameter.rotation = scene.camera.orientation;
    billboard_surface.uniform_parameter.scaling = 3.0f;

    for(size_t k=0; k<N_rock1; ++k)
    {
        const vec3& p = rock1_position[k];
        billboard_surface.uniform_parameter.translation = p;

        glPolygonOffset( 1.0, 1.0 );
        billboard_surface.draw(shaders["mesh"], scene.camera);
    }
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glDepthMask(true);


    if( gui_scene.wireframe ){
        for(size_t k=0; k<N_rock1; ++k)
        {
            const vec3& p = rock1_position[k];
            billboard_surface.uniform_parameter.translation = p;

            glPolygonOffset( 1.0, 1.0 );
            billboard_surface.draw(shaders["wireframe"], scene.camera);
        }
    }
}


void scene_exercise::display_rock2(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(!gui_scene.rock2)
        return;

    glEnable(GL_BLEND);
    glDepthMask(false);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const size_t N_rock2 = rock2_position.size();
    if(gui_scene.texture)
        glBindTexture(GL_TEXTURE_2D, texture_rock2_billboard);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    billboard_surface.uniform_parameter.rotation = scene.camera.orientation;
    billboard_surface.uniform_parameter.scaling = 4.5f;

    for(size_t k=0; k<N_rock2; ++k)
    {
        const vec3& p = rock2_position[k];
        billboard_surface.uniform_parameter.translation = p;

        glPolygonOffset( 1.0, 1.0 );
        billboard_surface.draw(shaders["mesh"], scene.camera);
    }
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glDepthMask(true);


    if( gui_scene.wireframe ){
        for(size_t k=0; k<N_rock2; ++k)
        {
            const vec3& p = rock2_position[k];
            billboard_surface.uniform_parameter.translation = p;

            glPolygonOffset( 1.0, 1.0 );
            billboard_surface.draw(shaders["wireframe"], scene.camera);
        }
    }
}


void scene_exercise::display_skier_button_lift(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    timer.update();
    const float t = timer.t;
    const size_t idx = index_at_value(t, keyframe_time);

    // Assume a closed curve trajectory
    const size_t N_bis = keyframe_time.size();



    // Linear interpolation
    const float t0 = keyframe_time[idx-1];
    const float t1 = keyframe_time[idx];
    const float t2 = keyframe_time[idx+1];
    const float t3 = keyframe_time[idx+2];

    const vec3& p0 = keyframe_position[idx-1];
    const vec3& p1 = keyframe_position[idx];
    const vec3& p2 = keyframe_position[idx+1];
    const vec3& p3 = keyframe_position[idx+2];


    const vec3 p = cardinal_spline_interpolation(t,t0,t1,t2,t3,p0,p1,p2,p3);
    trajectory.add_point(p);


    const vec3& p0_tele = keyframe_position_tele[idx-1];
    const vec3& p1_tele = keyframe_position_tele[idx];
    const vec3& p2_tele = keyframe_position_tele[idx+1];
    const vec3& p3_tele = keyframe_position_tele[idx+2];


    const vec3 p_tele = cardinal_spline_interpolation(t,t0,t1,t2,t3,p0_tele,p1_tele,p2_tele,p3_tele);
    remontee.add_point(p);

    const float long_poteau = 4.0f;
    if (keyframe_time[idx]>4 && keyframe_time[idx]<9){
        hierarchy.rotation("body")=rotation_from_axis_angle_mat3({0,0,1},3.14);
        perch.translation("perche")={p_tele[0],p_tele[1]+3*long_poteau/10,p_tele[2]};
        perch.rotation("perche")=rotation_from_axis_angle_mat3({0,0,0},0);
        //hierarchy.draw(shaders["mesh"],scene.camera);
     }
     else {
        hierarchy.rotation("body")=rotation_from_axis_angle_mat3({0,0,0},0);
        perch.translation("perche")={p_tele[0],p_tele[1]-3*long_poteau/10,p_tele[2]};
        perch.rotation("perche")=rotation_from_axis_angle_mat3({0,0,1},3.14);
        //hierarchy.draw(shaders["mesh"],scene.camera);
     }

     if (keyframe_time[idx]==4) {
        perch.rotation("perche")=perch.rotation("perche")*rotation_from_axis_angle_mat3({0,0,1},(t-t1)/(t2-t1)*3.14);
        perch.translation("perche")={perch.translation("perche")[0],perch.translation("perche")[1]+(t-t1)/(t2-t1)*3*long_poteau/5,perch.translation("perche")[2]};
        hierarchy.rotation("perche")=hierarchy.rotation("perche")*rotation_from_axis_angle_mat3({0,0,-1},(t-t1)/(t2-t1)*3.14);
     }

     if (keyframe_time[idx]==8 && t>t1 + (t2-t1)/2) {
        perch.rotation("perche")=perch.rotation("perche")*rotation_from_axis_angle_mat3({0,0,1},2*(t-t1-(t2-t1)/2)/(t2-t1)*3.14);
        perch.translation("perche")={perch.translation("perche")[0],perch.translation("perche")[1]-2*(t-t1-(t2-t1)/2)/(t2-t1)*3*long_poteau/5,perch.translation("perche")[2]};
        hierarchy.rotation("perche")=hierarchy.rotation("perche")*rotation_from_axis_angle_mat3({0,0,-1},2*(t-t1-(t2-t1)/2)/(t2-t1)*3.14);
     }

     hierarchy.translation("body")= {p[0],p[1]+3*long_poteau/10,p[2]};    // Draw current position



    // Draw sphere at each keyframe position
    for(size_t k=0; k<N_bis; ++k)
    {
        const vec3& p_keyframe = keyframe_position[k];
        sphere.uniform_parameter.translation = p_keyframe;
        sphere.draw(shaders["mesh"],scene.camera);
    }

    const vec3& p_keyframe_tele_h = keyframe_position_tele[0];
    button_lift_bout.translation("poteau") = p_keyframe_tele_h;
    button_lift_bout.rotation("poteau")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
    button_lift_bout.rotation("poteau")=button_lift_bout.rotation("poteau")*rotation_from_axis_angle_mat3({0,-1,0},3.14/2);
    button_lift_bout.draw(shaders["mesh"],scene.camera);

    for(size_t k=0; k<N_bis/2-2; ++k)
    {
        const vec3& p_keyframe_tele = keyframe_position_tele[k];
        button_lift.translation("poteau") = p_keyframe_tele;
        button_lift.rotation("poteau")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
        button_lift.rotation("cable")=rotation_from_axis_angle_mat3({1,0,0}, atan((keyframe_position_tele[k].z-keyframe_position_tele[k+1].z)/4.0f));
        button_lift.rotation("poteau")=button_lift.rotation("poteau")*rotation_from_axis_angle_mat3({0,1,0},3.14/2);
        button_lift.draw(shaders["mesh"],scene.camera);
    }

    const vec3& p_keyframe_tele_b = keyframe_position_tele[N_bis/2-2];
    button_lift_bout.translation("poteau") = p_keyframe_tele_b;
    button_lift_bout.rotation("poteau")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
    button_lift_bout.rotation("poteau")=button_lift_bout.rotation("poteau")*rotation_from_axis_angle_mat3({0,1,0},3.14/2);
    button_lift_bout.draw(shaders["mesh"],scene.camera);

    for(size_t k=N_bis/2-1; k<N_bis-3; ++k)
    {
        const vec3& p_keyframe_tele = keyframe_position_tele[k];
        button_lift.translation("poteau") = p_keyframe_tele;
        button_lift.rotation("poteau")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
        button_lift.rotation("cable")=rotation_from_axis_angle_mat3({1,0,0}, atan((keyframe_position_tele[k].z-keyframe_position_tele[k+1].z)/4.0f));
        button_lift.rotation("poteau")=button_lift.rotation("poteau")*rotation_from_axis_angle_mat3({0,-1,0},3.14/2);
        button_lift.draw(shaders["mesh"],scene.camera);
    }


    // animation dans la figure

    const float l_corpse=0.6f;
    const float l_leg = 0.3f;
    //hierarchy.translation("body") = {hierarchy.translation("body")[0],hierarchy.translation("body")[1],hierarchy.translation("body")[2]+0.2f*(1+std::sin(2*3.14f*t))};
    hierarchy.translation("body") = {hierarchy.translation("body")[0],hierarchy.translation("body")[1], l_corpse+2*l_leg + evaluate_terrain_z(hierarchy.translation("body")[0]/40+0.5f,hierarchy.translation("body")[1]/40+0.5f)};
    hierarchy.rotation("body") = hierarchy.rotation("body")*rotation_from_axis_angle_mat3({1,0,0},3.14*1/2)*rotation_from_axis_angle_mat3({0,1,0},3.14/2);

    hierarchy.rotation("hat") = rotation_from_axis_angle_mat3({0,0,1}, 0.3f*std::sin(2*3.14*(t-0.4f)));
    hierarchy.rotation("hat_bottom")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
    hierarchy.rotation("hat_bis_bottom_2")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
    hierarchy.rotation("hat_bis_bottom_1")=rotation_from_axis_angle_mat3({1,0,0},3.14/2);
    hierarchy.rotation("arm_top_left") = rotation_from_axis_angle_mat3({0,1,0}, std::sin(2*3.14f*(t-0.4f)) );
    hierarchy.rotation("arm_bottom_left") = rotation_from_axis_angle_mat3({0,1,0}, std::sin(2*3.14f*(t-0.6f)) );

    hierarchy.rotation("arm_top_right") = rotation_from_axis_angle_mat3({0,-1,0}, std::sin(2*3.14f*(t-0.4f)) );
    hierarchy.rotation("arm_bottom_right") = rotation_from_axis_angle_mat3({0,-1,0}, std::sin(2*3.14f*(t-0.6f)) );
    hierarchy.draw(shaders["mesh"], scene.camera);
    if(gui_scene.wireframe)
        hierarchy.draw(shaders["wireframe"], scene.camera);

    perch.translation("perche") = {perch.translation("perche")[0],perch.translation("perche")[1],perch.translation("perche")[2]+2*l_leg};
    perch.rotation("perche")=perch.rotation("perche")*rotation_from_axis_angle_mat3({0,0,-1},3.14/2)*rotation_from_axis_angle_mat3({1,0,0},3.14/8);
    perch.draw(shaders["mesh"], scene.camera);
    if(gui_scene.wireframe)
        perch.draw(shaders["wireframe"], scene.camera);

}


void scene_exercise::display_trajectory(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    // Draw segments between each keyframe
    for(size_t k=0; k<keyframe_position.size()-1; ++k)
    {
        const vec3& pa = keyframe_position[k];
        const vec3& pb = keyframe_position[k+1];

        segment_drawer.uniform_parameter.p1 = pa;
        segment_drawer.uniform_parameter.p2 = pb;
        segment_drawer.draw(shaders["segment_im"], scene.camera);

        const vec3& pa_tele = keyframe_position_tele[k];
        const vec3& pb_tele = keyframe_position_tele[k+1];

        segment_drawer.uniform_parameter.p1 = pa_tele;
        segment_drawer.uniform_parameter.p2 = pb_tele;
        segment_drawer.draw(shaders["segment_im"], scene.camera);
    }

    // Draw moving point trajectory
    trajectory.draw(shaders["curve"], scene.camera);

    // Draw selected sphere in red
    if( picked_object!=-1 )
    {
        const vec3& p_keyframe = keyframe_position[picked_object];
        sphere.uniform_parameter.color = vec3(1,0,0);
        sphere.uniform_parameter.scaling = 0.06f;
        sphere.uniform_parameter.translation = p_keyframe;
        sphere.draw(shaders["mesh"],scene.camera);
        sphere.uniform_parameter.color = vec3(1,1,1);
        sphere.uniform_parameter.scaling = 0.05f;
    }

}


void scene_exercise::display_snow(std::map<std::string,GLuint>& shaders, scene_structure& scene){

    if(!gui_scene.snow)
        return;

    const float dt = timer2.update(); // dt: Elapsed time between last frame
    set_gui(timer2);


    // Emission of new particle if needed
    const bool is_new_particle = timer2.event;
    if( is_new_particle )
    {
        particle_structure new_particle;
        const vec3 p0 = {-20+40*distrib(generator),-20+40*distrib(generator),30};
        const vec3 v0 = vec3(0,0,0);

        particles.push_back({p0,v0});
    }


    // Evolve position of particles
    const vec3 g = {0.0f,0.0f,-9.81f};
    for(particle_structure& particle : particles)
    {
        const float m = 0.01f; // particle mass

        vec3& p = particle.p;
        vec3& v = particle.v;

        const vec3 F = m*g - 0.02*v;

        // Numerical integration
        v = v + dt*F/m;
        p = p + dt*v;
    }


    // Remove particles that are too low
    for(auto it = particles.begin(); it!=particles.end(); ++it)
        if( it->p.z < evaluate_terrain_z(it->p.x/40+0.5f, it->p.y/40+0.5f))
            it = particles.erase(it);


    // Display particles
    for(particle_structure& particle : particles)
    {
        sphere2.uniform_parameter.translation = particle.p;
        sphere2.draw(shaders["mesh"], scene.camera);
    }
}


void scene_exercise::update_tree_position(){
    int N = 10;
    position.resize(N);
    for (int i=0;i<N;i++){
        float u = distrib(generator);
        float v = distrib(generator);
        vec3 p = evaluate_terrain(u,v);
        position[i]=p;
    }
}


void scene_exercise::update_snowman_position(){
    int N = 15;
    snowman_position.resize(N);
    for (int i=0;i<N;i++){
        float u = distrib(generator);
        float v = distrib(generator);
        vec3 p = evaluate_terrain(u,v);
        snowman_position[i]=p;
    }
}


void scene_exercise::update_rock1_position()
{
    const size_t N_rock1 = 20;

    for(size_t k=0; k<N_rock1; ++k)
    {
        const float u = 0.025f+0.95f*distrib(generator);
        const float v = 0.025f+0.95f*distrib(generator);
        const vec3 p = evaluate_terrain(u,v);

        rock1_position.push_back(p);
    }
}


void scene_exercise::update_rock2_position()
{
    const size_t N_rock2 = 30;

    for(size_t k=0; k<N_rock2; ++k)
    {
        const float u = 0.025f+0.95f*distrib(generator);
        const float v = 0.025f+0.95f*distrib(generator);
        const vec3 p = evaluate_terrain(u,v);

        rock2_position.push_back(p);
    }
}


// Evaluate height of the terrain for any (u,v) \in [0,1]
float evaluate_terrain_z(float u, float v)
{
    float scaling = 3.0f;
    int octave = 7;
    float persistency = 0.4f;

    // Evaluate Perlin noise
    const float noise = perlin(scaling*u, scaling*v, octave, persistency);

    const std::vector<vec2> u0 = {{0.0f, 0.0f},{0.5f, 0.5f},{0.2f, 0.7f},{0.8f,0.7f},{0.5f, 1.0f},{0.2f,0.7f},{0.2f,0.3f}};
    const std::vector<float> h = {2.0f,-1.5f,5.0f,3.0f,7.0f,-1.8f};
    const std::vector<float> sigma = {0.1f,0.15f,0.2f,0.2f,0.5f,0.26f};
    float d = 0;
    float z = 0;
    for (int i=0;i<5;i++) {
       d = norm(vec2(u,v)-u0[i])/sigma[i];
       z += h[i]*std::exp(-d*d);
    }

    return z+noise;
}


// Evaluate 3D position of the terrain for any (u,v) \in [0,1]
vec3 evaluate_terrain(float u, float v)
{
    const float x = 40*(u-0.5f);
    const float y = 40*(v-0.5f);
    const float z = evaluate_terrain_z(u,v);

    return {x,y,z};
}


// Generate terrain mesh
mesh create_terrain()
{
    // Number of samples of the terrain is N x N
    const size_t N = 100;

    mesh terrain; // temporary terrain storage (CPU only)
    terrain.position.resize(N*N);
    terrain.texture_uv.resize(N*N);

    // Fill terrain geometry
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);

            // Compute coordinates
            terrain.position[kv+N*ku] = evaluate_terrain(u,v);
            terrain.texture_uv[kv+N*ku] = {u,v};
        }
    }

    // Generate triangle organization
    // Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    const unsigned int Ns = N;
    for(unsigned int ku=0; ku<Ns-1; ++ku)
    {
        for(unsigned int kv=0; kv<Ns-1; ++kv)
        {
            const unsigned int idx = kv + N*ku; // current vertex offset

            const index3 triangle_1 = {idx, idx+1+Ns, idx+1};
            const index3 triangle_2 = {idx, idx+Ns, idx+1+Ns};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    return terrain;
}


mesh create_cylinder(float radius, float height)
{
    mesh m;

    // Number of samples
    const size_t N = 20;

    // Geometry
    for(size_t k=0; k<N; ++k)
    {
        const float u = k/float(N);
        const vec3 p = {radius*std::cos(2*3.14f*u), radius*std::sin(2*3.14f*u), 0.0f};
        m.position.push_back( p );
        m.position.push_back( p+vec3(0,0,height) );
    }

    // Connectivity
    for(size_t k=0; k<N; ++k)
    {
        const unsigned int u00 = 2*k;
        const unsigned int u01 = (2*k+1)%(2*N);
        const unsigned int u10 = (2*(k+1))%(2*N);
        const unsigned int u11 = (2*(k+1)+1) % (2*N);

        const index3 t1 = {u00, u10, u11};
        const index3 t2 = {u00, u11, u01};
        m.connectivity.push_back(t1);
        m.connectivity.push_back(t2);
    }

    return m;
}


mesh create_cone(float radius, float height, float z_offset)
{
    mesh m;

    // conical structure
    // *************************** //

    const size_t N = 20;

    // geometry
    for(size_t k=0; k<N; ++k)
    {
        const float u = k/float(N);
        const vec3 p = {radius*std::cos(2*3.14f*u), radius*std::sin(2*3.14f*u), 0.0f};
        m.position.push_back( p+vec3{0,0,z_offset} );
    }
    // apex
    m.position.push_back({0,0,height+z_offset});

    // connectivity
    const unsigned int Ns = N;
    for(unsigned int k=0; k<Ns; ++k) {
        m.connectivity.push_back( {k , (k+1)%N, Ns} );
    }

    // close the part in the bottom of the cone
    // *************************** //

    // Geometry
    for(size_t k=0; k<N; ++k)
    {
        const float u = k/float(N);
        const vec3 p = {radius*std::cos(2*3.14f*u), radius*std::sin(2*3.14f*u), 0.0f};
        m.position.push_back( p+vec3{0,0,z_offset} );
    }
    // central position
    m.position.push_back( {0,0,z_offset} );

    // connectivity
    for(unsigned int k=0; k<Ns; ++k)
        m.connectivity.push_back( {k+Ns+1, (k+1)%Ns+N+1, 2*Ns+1} );

    return m;
}


mesh create_tree_foliage(float radius, float height, float z_offset,float z)
{
    mesh m = create_cone(radius, height, z);
    m.push_back( create_cone(radius, height, z+z_offset) );
    m.push_back( create_cone(radius, height, z+2*z_offset) );

    return m;
}


vcl::mesh scene_exercise::create_billboard_surface()
{
    mesh billboard;
    billboard.position = {{-0.1f,0,0}, {0.1f,0,0}, {0.1f,0.2f,0}, {-0.1f,0.2f,0}};
    billboard.texture_uv = {{0,1}, {1,1}, {1,0}, {0,0}};
    billboard.connectivity = {{0,1,2}, {0,2,3}};

    return billboard;
}


vcl::mesh create_skybox()
{
    const vec3 p000 = {-1,-1,-1};
    const vec3 p001 = {-1,-1, 1};
    const vec3 p010 = {-1, 1,-1};
    const vec3 p011 = {-1, 1, 1};
    const vec3 p100 = { 1,-1,-1};
    const vec3 p101 = { 1,-1, 1};
    const vec3 p110 = { 1, 1,-1};
    const vec3 p111 = { 1, 1, 1};

    mesh skybox;

    skybox.position = {
        p000, p100, p110, p010,
        p010, p110, p111, p011,
        p100, p110, p111, p101,
        p000, p001, p010, p011,
        p001, p101, p111, p011,
        p000, p100, p101, p001
    };


    skybox.connectivity = {
        {0,1,2}, {0,2,3}, {4,5,6}, {4,6,7},
        {8,11,10}, {8,10,9}, {17,16,19}, {17,19,18},
        {23,22,21}, {23,21,20}, {13,12,14}, {13,14,15}
    };

    const float e = 1e-3f;
    const float u0 = 0.0f;
    const float u1 = 0.25f+e;
    const float u2 = 0.5f-e;
    const float u3 = 0.75f-e;
    const float u4 = 1.0f;
    const float v0 = 0.0f;
    const float v1 = 1.0f/3.0f+e;
    const float v2 = 2.0f/3.0f-e;
    const float v3 = 1.0f;
    skybox.texture_uv = {
        {u1,v1}, {u2,v1}, {u2,v2}, {u1,v2},
        {u1,v2}, {u2,v2}, {u2,v3}, {u1,v3},
        {u2,v1}, {u2,v2}, {u3,v2}, {u3,v1},
        {u1,v1}, {u0,v1}, {u1,v2}, {u0,v2},
        {u4,v1}, {u3,v1}, {u3,v2}, {u4,v2},
        {u1,v1}, {u2,v1}, {u2,v0}, {u1,v0}
    };


    return skybox;

}


mesh_drawable_hierarchy create_snowman(){

    mesh_drawable_hierarchy snowman;

    const float r_bas = 0.25f;
    const float r_milieu = 0.22f;
    const float r_haut = 0.15f;

    mesh_drawable bas = mesh_primitive_sphere(r_bas,{0,0,0},40,40);
    mesh_drawable milieu = mesh_primitive_sphere(r_milieu,{0,0,0},40,40);
    mesh_drawable haut = mesh_primitive_sphere(r_haut,{0,0,0},40,40);

    bas.uniform_parameter.color = {1,1,1};
    bas.uniform_parameter.shading.diffuse = 0.8;
    bas.uniform_parameter.shading.specular = 0;

    milieu.uniform_parameter.color = {1,1,1};
    milieu.uniform_parameter.shading.diffuse = 0.8;
    milieu.uniform_parameter.shading.specular = 0;

    haut.uniform_parameter.color = {1,1,1};
    haut.uniform_parameter.shading.diffuse = 0.8;
    haut.uniform_parameter.shading.specular = 0;

    mesh_drawable eye = mesh_primitive_sphere(0.02f,{0,0,0},20,20);
    eye.uniform_parameter.color = {0.3f,0.3f,0.3f};

    mesh_drawable nez = mesh_primitive_cone(0.02f, {0,0,0}, {0,0.2f,0});
    nez.uniform_parameter.color = {1.0f, 0.6f, 0.0f};

    mesh_drawable dent1 = mesh_primitive_sphere(0.01f,{0,0,0},20,20);
    dent1.uniform_parameter.color = {0.3f, 0.3f, 0.3f};
    mesh_drawable dent2 = mesh_primitive_sphere(0.01f,{0,0,0},20,20);
    dent2.uniform_parameter.color = {0.3f, 0.3f, 0.3f};
    mesh_drawable dent3 = mesh_primitive_sphere(0.01f,{0,0,0},20,20);
    dent3.uniform_parameter.color = {0.3f, 0.3f, 0.3f};
    mesh_drawable dent4 = mesh_primitive_sphere(0.01f,{0,0,0},20,20);
    dent4.uniform_parameter.color = {0.3f, 0.3f, 0.3f};
    mesh_drawable dent5 = mesh_primitive_sphere(0.01f,{0,0,0},20,20);
    dent5.uniform_parameter.color = {0.3f, 0.3f, 0.3f};


    snowman.add_element(bas, "bas", "root");
    snowman.add_element(milieu, "milieu", "bas", {0.0f, 0.0f, r_bas*1.5f});
    snowman.add_element(haut, "haut", "milieu", {0.0f, 0.0f, r_milieu*1.5f});
    snowman.add_element(eye, "eye_left", "haut",{r_haut/1.6f,r_haut/1.6f,r_haut/2.0f});
    snowman.add_element(eye, "eye_right", "haut",{-r_haut/1.6f,r_haut/1.6f,r_haut/2.0f});
    snowman.add_element(nez, "nez", "haut", {0, r_haut/1.5f, r_haut/4.0f});
    snowman.add_element(dent1, "dent1", "haut", {0,r_haut/1.1f,-r_haut/3.0f});
    snowman.add_element(dent2, "dent2", "haut", {r_haut/8.0f,r_haut/1.1f,-r_haut/3.3f});
    snowman.add_element(dent3, "dent3", "haut", {-r_haut/8.0f,r_haut/1.1f,-r_haut/3.3f});
    snowman.add_element(dent4, "dent4", "haut", {r_haut/4.0f,r_haut/1.1f,-r_haut/4.0f});
    snowman.add_element(dent5, "dent5", "haut", {-r_haut/4.0f,r_haut/1.1f,-r_haut/4.0f});

    return snowman;
}


mesh_drawable_hierarchy create_skier(){

    mesh_drawable_hierarchy hierarchy;

    const float r_body = 0.25f;
    const float r_cylinder = 0.05f;
    const float l_arm = 0.2f;
    const float l_corpse=0.6f;
    const float r_corpse=0.15f;
    const float r_hat = 0.1f;
    const float l_hat = 0.3f;
    const float l_leg = 0.3f;
    const float r_leg = 0.06f;
    const float fold = 0.05f; //inclinaison des genoux
    const float long_ski = 1.0f;
    const float larg_ski = 0.05f;
    const float epai_ski = 0.01f;
    mesh body = mesh_primitive_sphere(r_body,{0,0,0},40,40);

    mesh_drawable eye = mesh_primitive_sphere(0.05f,{0,0,0},20,20);
    eye.uniform_parameter.color = {0,0,0};

    mesh_drawable arm_top_left = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {-l_arm,0,0});
    arm_top_left.uniform_parameter.color = {1.0f, 0.25f, 0.5f};
    mesh_drawable arm_bottom_left = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {-l_arm/1.5f,-l_arm/1.0f,0});
    arm_bottom_left.uniform_parameter.color = {1.0f, 0.25f, 0.5f};
    mesh_drawable arm_top_right = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {l_arm,0,0});
    arm_top_right.uniform_parameter.color = {1.0f, 0.25f, 0.5f};
    mesh_drawable arm_bottom_right = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {l_arm/1.5f,-l_arm/1.0f,0});
    arm_bottom_right.uniform_parameter.color = {1.0f, 0.25f, 0.5f};
    mesh_drawable corpse = mesh_primitive_cylinder(r_corpse, {0,0,0}, {0,-l_corpse,0});
    corpse.uniform_parameter.color = {1.0f, 0.25f, 0.5f};
    mesh_drawable leg_top_left = mesh_primitive_cylinder(r_leg, {0,0,0}, {0,-l_leg,fold});
    leg_top_left.uniform_parameter.color = {0.0f, 0.9f, 0.46f};
    mesh_drawable leg_bottom_left = mesh_primitive_cylinder(r_leg, {0,0,0}, {0,-l_leg,-fold});
    leg_bottom_left.uniform_parameter.color = {0.0f, 0.9f, 0.46f};
    mesh_drawable leg_top_right = mesh_primitive_cylinder(r_leg, {0,0,0}, {0,-l_leg,fold});
    leg_top_right.uniform_parameter.color = {0.0f, 0.9f, 0.46f};
    mesh_drawable leg_bottom_right = mesh_primitive_cylinder(r_leg, {0,0,0}, {0,-l_leg,-fold});
    leg_bottom_right.uniform_parameter.color = {0.0f, 0.9f, 0.46f};
    mesh_drawable hat = mesh_primitive_cylinder(r_hat, {0,0,0}, {0,l_hat,0});
    hat.uniform_parameter.color = {0.f, 0.f, 0.f};
    mesh_drawable hat_bis = mesh_primitive_cylinder(r_hat*1.5f, {0,0,0}, {0,0.003f,0});
    hat_bis.uniform_parameter.color = {0.f, 0.f, 0.f};
    mesh_drawable hat_bottom = mesh_primitive_disc(r_hat);
    hat_bottom.uniform_parameter.color = {0.f, 0.f, 0.f};
    mesh_drawable hat_bis_bottom_1 = mesh_primitive_disc(r_hat*1.5f);
    hat_bis_bottom_1.uniform_parameter.color = {0.f, 0.f, 0.f};
    mesh_drawable hat_bis_bottom_2 = mesh_primitive_disc(r_hat*1.5f);
    hat_bis_bottom_2.uniform_parameter.color = {0.f, 0.f, 0.f};
    mesh_drawable ski_left = mesh_primitive_parallelepiped({-3*larg_ski/10,-epai_ski/2,-long_ski/2},{larg_ski,0,0},{0,epai_ski,0},{0,0,long_ski});
    ski_left.uniform_parameter.color = {1.0f, 0.86f, 0.21f};
    mesh_drawable ski_right = mesh_primitive_parallelepiped({-3*larg_ski/10,-epai_ski/2,-long_ski/2},{larg_ski,0,0},{0,epai_ski,0},{0,0,long_ski});
    ski_right.uniform_parameter.color = {1.0f, 0.86f, 0.21f};

    mesh_drawable shoulder = mesh_primitive_sphere(0.055f);
    shoulder.uniform_parameter.color = {1.0f, 0.25f, 0.5f};

    mesh_drawable knee = mesh_primitive_sphere(0.07f);
    knee.uniform_parameter.color = {0.0f, 0.9f, 0.46f};

    hierarchy.add_element(body, "body", "root");
    hierarchy.add_element(eye, "eye_left", "body",{r_body/3,r_body/2,r_body/1.5f});
    hierarchy.add_element(eye, "eye_right", "body",{-r_body/3,r_body/2,r_body/1.5f});

    hierarchy.add_element(hat,"hat","body", {0,r_body-0.02f,0});
    hierarchy.add_element(hat_bis, "hat_bis","hat",{0,0,0});
    hierarchy.add_element(hat_bottom,"hat_bottom","hat",{0,l_hat,0});
    hierarchy.add_element(hat_bis_bottom_1,"hat_bis_bottom_1","hat_bis",{0,0,0});
    hierarchy.add_element(hat_bis_bottom_2,"hat_bis_bottom_2","hat_bis",{0,0.003f,0});

    hierarchy.add_element(corpse, "corpse", "body", {0,-r_body+0.1f,0});
    hierarchy.add_element(arm_top_left, "arm_top_left", "corpse",{-r_corpse+0.05f,-l_corpse/4,0});
    hierarchy.add_element(arm_bottom_left, "arm_bottom_left", "arm_top_left",{-l_arm,0,0});

    hierarchy.add_element(arm_top_right, "arm_top_right", "corpse",{r_corpse-0.05f,-l_corpse/4,0});
    hierarchy.add_element(arm_bottom_right, "arm_bottom_right", "arm_top_right",{l_arm,0,0});

    hierarchy.add_element(shoulder, "shoulder_left", "arm_bottom_left");
    hierarchy.add_element(shoulder, "shoulder_right", "arm_bottom_right");

    hierarchy.add_element(leg_top_left,"leg_top_left","body",{r_corpse/2,-l_corpse,0});
    hierarchy.add_element(leg_bottom_left,"leg_bottom_left","leg_top_left",{0,-l_leg,fold});

    hierarchy.add_element(leg_top_right,"leg_top_right","body",{-r_corpse/2,-l_corpse,0});
    hierarchy.add_element(leg_bottom_right,"leg_bottom_right","leg_top_right",{0,-l_leg,fold});

    hierarchy.add_element(knee, "knee_left", "leg_bottom_left");
    hierarchy.add_element(knee, "knee_right", "leg_bottom_right");

    hierarchy.add_element(ski_left,"ski_left","leg_bottom_left",{0,-l_leg,0});
    hierarchy.add_element(ski_right,"ski_right","leg_bottom_right",{0,-l_leg,0});


    return hierarchy;
}


mesh_drawable_hierarchy create_button_lift(){

    mesh_drawable_hierarchy button_lift;

    const float l_poteau = 4.0f;
    const float r_poteau = 0.4f;
    const float long_poteau = 4.0f;
    const float epai_poteau = 0.1f;
    const float r_poulie = 0.2f;
    const float l_poulie = 0.2f;
    const float r_cable = 0.05f;
    const float distance = 4.0f;

    mesh_drawable poteau = mesh_primitive_cylinder(r_poteau, {0,0,0}, {0,l_poteau,0});
    mesh_drawable poteau_haut = mesh_primitive_parallelepiped({-long_poteau/2,0,-r_poteau/2},{long_poteau,0,0},{0,epai_poteau,0},{0,0,r_poteau});
    mesh_drawable poulie = mesh_primitive_cylinder(r_poulie, {0,0,0}, {l_poulie,0,0});
    mesh_drawable cable = mesh_primitive_cylinder(r_cable, {0,0,0}, {0,0,distance});

    poteau.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    poteau_haut.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    poulie.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    cable.uniform_parameter.color = {0.f, 0.f, 0.f};


    button_lift.add_element(poteau, "poteau", "root");
    button_lift.add_element(poteau_haut,"poteau_haut","poteau",{0,l_poteau,0});
    button_lift.add_element(poulie,"poulie","poteau_haut",{3*long_poteau/10,0,0});
    button_lift.add_element(cable,"cable","poulie",{l_poulie/2,r_poulie,0});

    return button_lift;
}


mesh_drawable_hierarchy create_button_lift_bout(){

    mesh_drawable_hierarchy button_lift_bout;

    const float l_poteau = 4.0f;
    const float r_poteau = 0.4f;
    const float long_poteau = 4.0f;
    const float epai_poteau = 0.1f;
    const float r_poulie = 0.2f;
    const float l_poulie = 0.2f;

    mesh_drawable poteau = mesh_primitive_cylinder(r_poteau, {0,0,0}, {0,l_poteau,0});
    mesh_drawable poteau_haut = mesh_primitive_parallelepiped({-long_poteau/2,0,-3*r_poteau},{long_poteau,0,0},{0,2*epai_poteau,0},{0,0,7*r_poteau});
    mesh_drawable poulie = mesh_primitive_cylinder(r_poulie, {0,0,0}, {l_poulie,0,0});

    poteau.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    poteau_haut.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    poulie.uniform_parameter.color = {0.6f, 0.6f, 0.6f};

    button_lift_bout.add_element(poteau, "poteau", "root");
    button_lift_bout.add_element(poteau_haut,"poteau_haut","poteau",{0,l_poteau,0});
    button_lift_bout.add_element(poulie,"poulie","poteau_haut",{3*long_poteau/10,0,0});

    return button_lift_bout;
}


mesh_drawable_hierarchy create_perch(){

    mesh_drawable_hierarchy perche;

    const float r_perche = 0.04f;
    const float pench = 3.5f;

    mesh_drawable per = mesh_primitive_cylinder(r_perche, {0,0,0}, {0,0,pench});
    per.uniform_parameter.color = {0.6f, 0.6f, 0.6f};
    perche.add_element(per,"perche","root");

    return perche;
}


void scene_exercise::update_trajectory()
{
    trajectory = curve_dynamic_drawable(100); // number of steps stroed in the trajectory
    trajectory.uniform_parameter.color = {0,0,1};
    remontee = curve_dynamic_drawable(100); // number of steps stroed in the trajectory
    remontee.uniform_parameter.color = {0,0,1};

    picked_object=-1;
}


size_t index_at_value(float t, const std::vector<float>& vt)
{
    const size_t N = vt.size();
    assert(vt.size()>=2);
    assert(t>=vt[0]);
    assert(t<vt[N-1]);

    size_t k=0;
    while( vt[k+1]<t )
        ++k;
    return k;
}


vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3)
{
    const float sigma = t2-t1;

    const vec3 d1 = (p2-p0)/(t2-t0) * sigma;
    const vec3 d2 = (p3-p1)/(t3-t1) * sigma;

    const float s = (t-t1)/sigma;
    const float s2 = s*s;
    const float s3 = s2*s;

    const vec3 p = (2*s3-3*s2+1)*p1 + (s3-2*s2+s)*d1 + (-2*s3+3*s2)*p2 + (s3-s2)*d2;

    return p;
}


vec3 cardinal_spline_derivative_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3)
{
    const float sigma = t2-t1;

    const vec3 d1 = (p2-p0)/(t2-t0) * sigma;
    const vec3 d2 = (p3-p1)/(t3-t1) * sigma;

    const float s = (t-t1)/sigma;
    const float s2 = s*s;

    const vec3 p = (6*s2-6*s)*p1 + (3*s2-4*s+1)*d1 + (-6*s2+6*s)*p2 + (3*s2-2*s)*d2;

    return p;
}


vec3 cardinal_spline_interpolation(const trajectory_structure& trajectory, float t)
{
    const size_t idx = index_at_value(t, trajectory.time);

    const float t0 = trajectory.time[idx-1];
    const float t1 = trajectory.time[idx];
    const float t2 = trajectory.time[idx+1];
    const float t3 = trajectory.time[idx+2];

    const vec3& p0 = trajectory.position[idx-1];
    const vec3& p1 = trajectory.position[idx];
    const vec3& p2 = trajectory.position[idx+1];
    const vec3& p3 = trajectory.position[idx+2];

    //const vec3 p = linear_interpolation(t,t1,t2,p1,p2);
    const vec3 p = cardinal_spline_interpolation(t,t0,t1,t2,t3,p0,p1,p2,p3);
    return p;
}


vec3 cardinal_spline_derivative_interpolation(const trajectory_structure& trajectory, float t)
{
    const size_t idx = index_at_value(t, trajectory.time);

    const float t0 = trajectory.time[idx-1];
    const float t1 = trajectory.time[idx];
    const float t2 = trajectory.time[idx+1];
    const float t3 = trajectory.time[idx+2];

    const vec3& p0 = trajectory.position[idx-1];
    const vec3& p1 = trajectory.position[idx];
    const vec3& p2 = trajectory.position[idx+1];
    const vec3& p3 = trajectory.position[idx+2];

    const vec3 p = cardinal_spline_derivative_interpolation(t,t0,t1,t2,t3,p0,p1,p2,p3);
    return p;
}


void scene_exercise::mouse_click(scene_structure& scene, GLFWwindow* window, int , int action, int )
{
    // Mouse click is used to select a position of the control polygon
    // ******************************************************************** //

    // Window size
    int w=0;
    int h=0;
    glfwGetWindowSize(window, &w, &h);

    // Current cursor position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Convert pixel coordinates to relative screen coordinates between [-1,1]
    const float x = 2*float(xpos)/float(w)-1;
    const float y = 1-2*float(ypos)/float(h);

    // Check if shift key is pressed
    const bool key_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT));

    if(action==GLFW_PRESS && key_shift)
    {
        // Create the 3D ray passing by the selected point on the screen
        const ray r = picking_ray(scene.camera, x,y);

        // Check if this ray intersects a position (represented by a sphere)
        //  Loop over all positions and get the intersected position (the closest one in case of multiple intersection)
        const size_t N = keyframe_position.size();
        picked_object = -1;
        float distance_min = 0.0f;
        for(size_t k=0; k<N; ++k)
        {
            const vec3 c = keyframe_position[k];
            const picking_info info = ray_intersect_sphere(r, c, 0.1f);

            if( info.picking_valid ) // the ray intersects a sphere
            {
                const float distance = norm(info.intersection-r.p); // get the closest intersection
                if( picked_object==-1 || distance<distance_min ){
                    picked_object = k;
                }
            }
        }
    }

}


void scene_exercise::mouse_move(scene_structure& scene, GLFWwindow* window)
{
    // Mouse move is used to translate a position once selected
    // ******************************************************************** //

    // Window size
    int w=0;
    int h=0;
    glfwGetWindowSize(window, &w, &h);

    // Current cursor position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Convert pixel coordinates to relative screen coordinates between [-1,1]
    const float x = 2*float(xpos)/float(w)-1;
    const float y = 1-2*float(ypos)/float(h);

    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left  = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT )==GLFW_PRESS);
    const bool key_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT));

    const size_t N = keyframe_position.size();
    if(mouse_click_left && key_shift && picked_object!=-1)
    {
        // Translate the selected object to the new pointed mouse position within the camera plane
        // ************************************************************************************** //

        // Get vector orthogonal to camera orientation
        const mat4 M = scene.camera.camera_matrix();
        const vec3 n = {M(0,2),M(1,2),M(2,2)};

        // Compute intersection between current ray and the plane orthogonal to the view direction and passing by the selected object
        const ray r = picking_ray(scene.camera, x,y);
        vec3& p0 = keyframe_position[picked_object];
        const picking_info info = ray_intersect_plane(r,n,p0);

        // translate the position
        p0 = info.intersection;

        // Make sure that duplicated positions are moved together
        int Ns = N;
        if(picked_object==0 || picked_object==Ns-3){
            keyframe_position[0] = info.intersection;
            keyframe_position[N-3] = info.intersection;
        }
        if(picked_object==1 || picked_object==Ns-2){
            keyframe_position[1] = info.intersection;
            keyframe_position[N-2] = info.intersection;
        }
        if(picked_object==2 || picked_object==Ns-1){
            keyframe_position[2] = info.intersection;
            keyframe_position[N-1] = info.intersection;
        }

    }
}


void scene_exercise::set_gui(timer_event& timer2)
{
    ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max);

    const float time_scale_min = 0.1f;
    const float time_scale_max = 3.0f;
    ImGui::SliderFloat("Time scale", &timer.scale, time_scale_min, time_scale_max);

    if( ImGui::Button("Print Keyframe") )
    {
        std::cout<<"keyframe_position={";
        for(size_t k=0; k<keyframe_position.size(); ++k)
        {
            const vec3& p = keyframe_position[k];
            std::cout<< "{"<<p.x<<"f,"<<p.y<<"f,"<<p.z<<"f}";
            if(k<keyframe_position.size()-1)
                std::cout<<", ";
        }
        std::cout<<"}"<<std::endl;
    }

    ImGui::Checkbox("Wireframe", &gui_scene.wireframe);
    ImGui::Checkbox("Skybox", &gui_scene.skybox);

    ImGui::Checkbox("Terrain", &gui_scene.terrain);
    ImGui::Checkbox("Tree", &gui_scene.tree);

    ImGui::Checkbox("Rock 1", &gui_scene.rock1);
    ImGui::Checkbox("Rock 2", &gui_scene.rock2);

    ImGui::Checkbox("Skybox", &gui_scene.skybox);
    ImGui::Checkbox("Snow", &gui_scene.snow);

    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer2.scale, &scale_min, &scale_max, "%.2f s");

    // Start and stop animatiosnown
    if (ImGui::Button("Stop snow"))
        timer2.stop();
    if (ImGui::Button("Start snow"))
        timer2.start();

}





#endif
