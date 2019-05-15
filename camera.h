//
// Created by Xinming Zhang on 4/13/19.
//

#ifndef PHYSICSSIMULATION_CAMERA_H
#define PHYSICSSIMULATION_CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <math.h>

// define several possible options for camera movement
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// default camera values
const float YAW        = -90.0;
const float PITCH      = 0.0f;
const float SPEED      = 3.0f;
const float SENSITIVITY= 0.1f;
// fov
const float ZOOM       = 45.0f;

class Camera
{
public:
    //camera attribute
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    // Euler angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // constructor with vectors
    Camera(glm::vec3 position = glm::vec3(0.0, 0.0, 0.0), glm::vec3 up = glm::vec3(0.0, 1.0, 0.0), float yaw = YAW, float pitch = PITCH);
    // constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch);
    // return the view matrix calculated using euler angles and lookat matrix
    glm::mat4 GetViewMatrix();
    // process input received from any keyboard-like input system. accept input parameters in the form of camera defined ENUM
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);
    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset);
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);

private:
    // calculate the front vector from the camera's euler angles
    void updateCameraVectors();
};

Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch) : Front(glm::vec3(0.0, 0.0, -1.0)),
                                                                           MovementSpeed(SPEED),
                                                                           MouseSensitivity(SENSITIVITY),
                                                                           Zoom(ZOOM)
{
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}
Camera::Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(glm::vec3(0.0, 0.0, -1.0)),
                                                                                                              MovementSpeed(SPEED),
                                                                                                              MouseSensitivity(SENSITIVITY),
                                                                                                              Zoom(ZOOM)
{
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix()
{
    return glm::lookAt(Position, Position + Front, Up);
}
void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    float velocity = MovementSpeed * deltaTime;
    if(direction == FORWARD)
        Position += Front * velocity;
    if(direction == BACKWARD)
        Position -= Front * velocity;
    if(direction == LEFT)
        Position -= RIGHT * velocity;
    if(direction == RIGHT)
        Position += RIGHT * velocity;

}

void Camera::ProcessMouseScroll(float yoffset)
{
    if(Zoom >= 1.0f && Zoom <= 45.0f)
        Zoom -= yoffset;
    if(Zoom <= 1.0)
        Zoom = 1.0f;
    if(Zoom >= 45.0)
        Zoom = 45.0f;
}

void Camera::updateCameraVectors()
{
    // calculate the new front vector
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = cos(glm::radians(Pitch)) * sin(glm::radians(Yaw));
    Front = glm::normalize(front);
    // recalculate the right and up vector
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch)
{
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw  += xoffset;
    Pitch+= yoffset;

    //make sure that when pitch is out of bounds, screen doesn't get flipped
    if(constrainPitch)
    {
        if(Pitch > 89.0)
            Pitch = 89.0;
        if(Pitch < -89.0)
            Pitch = -89.0;
    }
    updateCameraVectors();
}

#endif //PHYSICSSIMULATION_CAMERA_H
