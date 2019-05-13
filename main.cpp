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
    glm::vec3 viewPoint = glm::vec3(0, 3.0, 3.5);
    Camera camera(viewPoint);
    float lastX = WIDTH / 2.0f;
    float lastY = HEIGHT / 2.0f;
    bool firstMouse = true;

    //timing
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    float timestep = 1.0f / 50;

    //lighting


    //model
    glm::vec3 location = glm::vec3(-0.2, 3, -1);
    float densities = 900.0;  // kg/m3
    float v = 0.4f;
    float E = 50.0f;
    float v_damping = 0.2f;
    float E_damping = 40.0f;
    bool use_gravity = false;

    //ground
    glm::vec3 ground_origin(-30.7f, 0.0f, 30.5f);
    glm::vec3 ground_side1(0.0f, 0.0f, -1.0f);
    glm::vec3 ground_side2(1.0f, 0.0f, 0.0f);
    float ground_distance = 60.0f;

}


void init(){

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
    MassSpringSystem mesh("model/cube/cube.1.node",
                          "model/cube/cube.1.face",
                          "model/cube/cube.1.ele",
                          densities,
                          location,
                          v, E,
                          v_damping, E_damping,
                          timestep,
                          use_gravity);
    mesh.initShader();

    // ground
    Plane ground(ground_origin, ground_side1, ground_side2, ground_distance);
    ground.initShaders();

    ForwardEulerIntegrator ForwardEuler;

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    //render loop
    while(!glfwWindowShouldClose(window)){
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        processInput(window);

        // render start
        glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view         = glm::mat4(1.0f);
        view = camera.GetViewMatrix();
        glm::mat4 projection   = glm::perspective(glm::radians(camera.Zoom), (float)WIDTH/(float)HEIGHT, 0.1f, 100.0f);
        glm::mat4 transform = projection * view;

        ground.setTransform(transform);
        ground.renderPlane();

        mesh.set_transformation(transform);
        mesh.render_system();
        // draw geometry
        glfwSwapBuffers(window);
        glfwPollEvents();
//        // update position
//        std::vector<glm::vec3> before_position = mesh.getVertex();
//        std::vector<glm::vec3> before_vel = mesh.getVelocity();
//
//        ForwardEuler.Integrate(&mesh, timestep);
//
//        std::vector<glm::vec3> after_position = mesh.getVertex();
//        std::vector<glm::vec3> after_vel;
//
//        bool collide = false;
//        std::vector<bool> collision(mesh.GetNumPoint(), false);
//        float t = std::numeric_limits<float>::max();
//        for (int j = 0; j < mesh.GetNumPoint(); ++j) {
//            float dist = after_position[j].y;
//            if(dist < 0.0f) {
//                collision[j] = true;
//                float deltat = timestep * (before_position[j].y) / (before_position[j].y - after_position[j].y);
//                t = fminf(t, deltat);
//                collide = true;
//            }
//        }
//        if(collide){
//            int n = mesh.GetNumPoint();
//            mesh.SetPositions(before_position);
//            mesh.SetVelocities(before_vel);
//            before_position.clear();
//            before_vel.clear();
//            ForwardEuler.Integrate(&mesh, t);
//            mesh.GetVelocities(after_vel);
//            for (int j = 0; j < n; ++j) {
//                if(collision[j])
//                    after_vel[j].y = -after_vel[j].y;
//            }
//            mesh.SetVelocities(after_vel);
//            after_vel.clear();
//
//        }


//        glBindBuffer(GL_ARRAY_BUFFER, vbo);
//        glBufferData(GL_ARRAY_BUFFER, mesh.GetNumPoint()*sizeof(glm::vec3), &mesh.getVertex()[0], GL_STATIC_DRAW);
    }

    mesh.delete_shader();
    ground.deleteBuffer();

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