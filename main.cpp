#include <iostream>
#include <limits>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include "camera.h"
#include "shader.h"
#include "physys.h"
#include "mesh.h"
#include "plane.h"
#include "Integrator.h"
#include "debugger.h"
#include "object_collision.h"
#include "tetra_intersect.h"



#define CHECK_GL_ERRORS assert(glGetError() == GL_NO_ERROR)

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);


namespace {
    // window setting
    unsigned int WIDTH = 800;
    unsigned int HEIGHT = 600;

    // camera
    glm::vec3 viewPoint = glm::vec3(0, 10.0, 25.5);
    Camera camera(viewPoint);
    float lastX = WIDTH / 2.0f;
    float lastY = HEIGHT / 2.0f;
    bool firstMouse = true;

    //timing
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    float timestep = 1.0f / 300;
    int step_per_frame = 5;

    // fracture
    bool fracture = false;
    float roughness = 100.0f;

    //lighting
    glm::vec3 lightPos(-3.0f, 20.0f, 5.0f);


    //model
    glm::vec3 location_1 = glm::vec3(-3, 10, 1);
    glm::vec3 location_2 = glm::vec3(-3, 5, 1);
    float densities = 0.3;  // kg/m3
    float v = 0.2f;
    float E = 800.0f;
    float v_damping = 0.1f;
    float E_damping = 10.0f;
    bool use_gravity = true;

    //ground
    glm::vec3 ground_origin(-15.0f, 0.0f, 15.0f);
    glm::vec3 ground_side1(0.0f, 0.0f, -1.0f);
    glm::vec3 ground_side2(1.0f, 0.0f, 0.0f);
    float ground_distance = 30.0f;
    // wall1
    glm::vec3 wall_origin(-15.0f, 0.0f, 15.0f);
    glm::vec3 wall_side1(0.0f, 1.0f, 0.0f);
    glm::vec3 wall_side2(0.0f, 0.0f, -1.0f);
    float wall_distance = 30.0f;
    // wall2
    glm::vec3 wall2_origin(-15.0f, 0.0f, -15.0f);
    glm::vec3 wall2_side1(0.0f, 1.0f, 0.0f);
    glm::vec3 wall2_side2(1.0f, 0.0f, 0.0f);
    float wall2_distance = 30.0f;

}

int main() {
    // intiailize window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Apple setting
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "LearnOpenGl", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
//    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // load all openGL functions pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }


    // configure global opengl state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // model
//    MassSpringSystem cube1("model/cube_origin/cube.node",
//                          "model/cube_origin/cube.face",
//                          "model/cube_origin/cube.ele",
//                          densities,
//                          location_1,
//                          v, E,
//                          v_damping, E_damping,
//                          timestep,
//                          use_gravity,
//                          fracture,
//                          roughness,
//                          lightPos,
//                          camera.Position);
//    cube1.initShader();
//
//    MassSpringSystem cube2("model/cube_origin/cube.node",
//                          "model/cube_origin/cube.face",
//                          "model/cube_origin/cube.ele",
//                          densities,
//                          location_2,
//                          v, E,
//                          v_damping, E_damping,
//                          timestep,
//                          use_gravity,
//                          fracture,
//                          roughness);
//    cube2.initShader();


    MassSpringSystem mesh("model/cube/cube.1.node",
                          "model/cube/cube.1.face",
                          "model/cube/cube.1.ele",
                          densities,
                          location_1,
                          v, E,
                          v_damping, E_damping,
                          timestep,
                          use_gravity,
                          fracture,
                          roughness,
                          lightPos,
                          camera.Position);
    mesh.initShader();

//    MassSpringSystem rectangle("model/rectangle/rectangle.1.node",
//                               "model/rectangle/rectangle.1.face",
//                               "model/rectangle/rectangle.1.ele",
//                               densities,
//                               location_2,
//                               v, E,
//                               v_damping, E_damping,
//                               timestep,
//                               use_gravity,
//                               fracture,
//                               roughness);
//    rectangle.initShader();



    // ground
    Plane ground(ground_origin, ground_side1, ground_side2, ground_distance, lightPos, camera.Position);
    ground.initShaders();

    // wall1
    Plane wall(wall_origin, wall_side1, wall_side2, wall_distance, lightPos, camera.Position);
    wall.initShaders();

    // wall2
    Plane wall2(wall2_origin, wall2_side1, wall2_side2, wall2_distance, lightPos, camera.Position);
    wall2.initShaders();

    ForwardEulerIntegrator ForwardEuler;

    std::vector<MassSpringSystem> object_list = {mesh};

    //render loop
    while(!glfwWindowShouldClose(window)){
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        processInput(window);

        // render start 166, 202, 240
//        glClearColor(0.65f, 0.79f, 0.94f, 0.0f);
        glClearColor(0.05, 0.05, 0.05, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view         = glm::mat4(1.0f);
        view = camera.GetViewMatrix();
        glm::mat4 projection   = glm::perspective(glm::radians(camera.Zoom), (float)WIDTH/(float)HEIGHT, 0.1f, 100.0f);
        glm::mat4 transform = projection * view;
        glm::mat4 model = glm::mat4(1.0f);

        ground.setTransform(transform);
        ground.setVirePos(camera.Position);
        ground.renderPlane();
        wall.setTransform(transform);
        wall.setVirePos(camera.Position);
        wall.renderPlane();
        wall2.setTransform(transform);
        wall2.setVirePos(camera.Position);
        wall2.renderPlane();

//        cube1.set_transformation(transform);
//        cube1.set_viewpos(camera.Position);
//        cube1.render_system();
//
//        cube2.set_transformation(transform);
//        cube2.render_system();

        mesh.set_transformation(transform);
//        mesh.set_viewpos(camera.Position);
        mesh.render_system();

//        rectangle.set_transformation(transform);
//        rectangle.render_system();
        // draw geometry
        glfwSwapBuffers(window);
        glfwPollEvents();


        for (int i = 0; i < step_per_frame; ++i) {
            // clear obkject force first
            CollideQuery::clear_collision_response(object_list);
            // object collision detection
            std::vector<std::tuple<int, int, std::vector<std::pair<unsigned int, unsigned int>>>> collide_result_list = CollideQuery::collide_quert_list(object_list);
            // object collision response
            if(!collide_result_list.empty())
                CollideQuery::collision_response(collide_result_list, object_list);
            // collision with ground
            for (int j = 0; j < object_list.size(); ++j) {
                ground.processCollision(object_list[j]);
            }
            for (int j = 0; j < object_list.size(); ++j) {
                wall.processCollision(object_list[j]);
            }
            for (int j = 0; j < object_list.size(); ++j) {
                wall2.processCollision(object_list[j]);
            }
            // integration
            for (int k = 0; k < object_list.size(); ++k) {
                ForwardEuler.Integrate(&object_list[k], timestep);
            }
            // aabb update
            for(int i = 0; i < object_list.size(); ++i)
                object_list[i].update_aabb_tree();
        }

        // buffer data update
        for (int l = 0; l < object_list.size(); ++l) {
            object_list[l].update();
        }
//        cube1_de.update(object_list[0]);
//        cube2_de.update(object_list[1]);
    }
//    cube1.delete_shader();
//    cube2.delete_shader();
    mesh.delete_shader();
//    rectangle.delete_shader();
    ground.deleteBuffer();
    wall.deleteBuffer();
    wall2.deleteBuffer();

    return 0;
}

void processInput(GLFWwindow* window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}