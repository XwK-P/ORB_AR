// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "GL_Rendering/controls.hpp"

glm::mat4 ModelMatrix;
glm::mat4 TranslateMatrix;
glm::mat4 ScalingMatrix;
glm::mat4 ProjectionMatrix;

glm::mat4 getModelMatrix(){
	return ModelMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}


float scale_factor = 1;
float rotation = 0;

// Initial Field of View
float initialFoV = 45.0f;

float speed = 3.0f; // 3 units / second
float rotate_speed = 90.0f;



void computeMatricesFromInputs(){

    ModelMatrix = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3( -1, 0, 0));

	// glfwGetTime is called only once, the first time this function is called
	static double lastTime = glfwGetTime();

	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);


	// Move forward
	if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
		scale_factor += deltaTime * speed;
	}
	// Move backward
	if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
		scale_factor -= deltaTime * speed;
	}
	// Strafe right
	if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
		rotation += deltaTime * rotate_speed;
	}
	// Strafe left
	if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
        rotation -= deltaTime * rotate_speed;
	}

	float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.

	// Projection matrix : 45� Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(61.5f), 4.0f / 3.0f, 0.1f, 100.0f);

	// Model matrix
    ModelMatrix = glm::rotate(ModelMatrix, glm::radians(rotation), glm::vec3( 0, 1, 0));
    TranslateMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(3.5,3,0));
    ScalingMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(scale_factor));
    ModelMatrix = TranslateMatrix * ModelMatrix * ScalingMatrix;

	// For the next frame, the "last time" will be "now"
	lastTime = currentTime;
}