#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "camera.h"
#include "shader.h"
#include "sphere.h"
#include "mesh.h"

#define CHECK_GL_ERRORS assert(glGetError() == GL_NO_ERROR)

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

// window setting
unsigned int WIDTH = 800;
unsigned int HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0, 0.0, 3.0));
float lastX = WIDTH / 2.0f;
float lastY = HEIGHT / 2.0f;
bool firstMouse = true;

//timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

//lighting

// gravity
glm::vec3 gravity = glm::vec3(0, 9.82f, 0);

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
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // load all openGL functions pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    Shader groundShader("shader/ground.vs", "shader/ground.fs");
    Shader sp("shader/particle.vs", "shader/particle.fs");

    // configure global opengl state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    float ground[] = {
        -0.7f, -0.5f, 0.5f,
        -0.7f, -0.5f, -0.5f,
         0.7f, -0.5f, -0.5f,
         0.7f, -0.5f, 0.5f
    };

    unsigned int indices[] = {
         0, 1, 2,
         0, 2, 3
    };


    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(ground), ground, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    float particle[] = {
            0.0f, 0.0f, 0.0f,
            0.1f, 0.0f, 0.0f,
            0.0f, 0.1f, 0.0f,
            0.0f, 0.0f, 0.1f
    };

    GLuint indicesBall[] = {
            0, 1, 2,
            1, 0, 3,
            0, 2, 3,
            2, 1, 3
    };
    unsigned int vaoBall, vboBall, eboBall;
    glGenVertexArrays(1, &vaoBall);
    glGenBuffers(1, &vboBall);
    glGenBuffers(1, &eboBall);

    glBindVertexArray(vaoBall);
    glBindBuffer(GL_ARRAY_BUFFER, vboBall);
    glBufferData(GL_ARRAY_BUFFER, sizeof(particle), particle, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboBall);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicesBall), indicesBall, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    // sphere

    Mesh mesh("model/mymesh.1.node", "model/mymesh.1.face", "model/mymesh.1.ele");


    unsigned int vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, int(mesh.getVertex().size())*sizeof(float), &mesh.getVertex()[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, int(mesh.getIndex().size())*sizeof(float), &mesh.getIndex()[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    int l1 = mesh.getVertex().size();
    int l2 = mesh.getIndex().size();
    for(int i = 0; i < l1; ++i){
        std::cout << mesh.getVertex()[i] << " ";
        if((i+1)%3==0)
            std::cout << "\n";
    }

    for(int i = 0; i < l2; ++i){
        std::cout << mesh.getIndex()[i] << " ";
        if((i+1)%3==0)
            std::cout << "\n";
    }



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

        groundShader.use();

        glm::mat4 view         = glm::mat4(1.0f);
        view = camera.GetViewMatrix();
        glm::mat4 projection   = glm::perspective(glm::radians(camera.Zoom), (float)WIDTH/(float)HEIGHT, 0.1f, 100.0f);
        glm::mat4 transform = projection * view;

        unsigned int transLoc = glGetUniformLocation(groundShader.ID, "transform");
        glUniformMatrix4fv(transLoc, 1, GL_FALSE, glm::value_ptr(transform));
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);


        sp.use();
        glUniformMatrix4fv(glGetUniformLocation(sp.ID, "transform"), 1, GL_FALSE, glm::value_ptr(transform));
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, l2, GL_UNSIGNED_INT, 0);
//        glDrawArrays(GL_POINT, 0, 2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    glDeleteVertexArrays(1, &vaoBall);
    glDeleteBuffers(1, &vboBall);
    glDeleteBuffers(1, &eboBall);

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