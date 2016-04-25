// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

using namespace glm;
using namespace cv;

#include "GL_Rendering/shader.hpp"
#include "GL_Rendering/texture.hpp"
#include "GL_Rendering/controls.hpp"
#include "GL_Rendering/objloader.hpp"
//#include "track_chessboard/track_chessboard.hpp"
#include "orb_slam.h"
#include "planar_tracking.h"

VideoCapture cap_left(0);
VideoCapture cap_right(2);

int main( void )
{
    //VideoWriter out;
    //out.open("video.avi", VideoWriter::fourcc('S', 'V', 'Q', '3'), 30, Size(640*16/9-1, 480*16/9-1), true);

    cap_left.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap_left.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cap_right.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap_right.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


    // Initialise Tracking System
    bool success = initTracking("Remap.xml", "Extrinsics.xml");
    Mat K = getCameraMatrix();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("ORB_Vocabulary/ORBvoc.bin", "StereoCam.yaml", ORB_SLAM2::System::STEREO,false);

    if (!success)
        return 0;

    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 640, 480, "OpenGL Demo", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));

    // Initialize GLEW
    glewExperimental = GL_TRUE; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Set the mouse at the center of the screen
    //glfwPollEvents();
    //glfwSetCursorPos(window, 1024/2, 768/2);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "Shaders/TransformVertexShader.vertexshader", "Shaders/TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    GLint MatrixID = glGetUniformLocation(programID, "MVP");

    // Load the texture
    int width, height;
    GLuint Texture = png_texture_load("SpongeBob/spongebob.png", &width, &height);
    //GLuint Texture = loadDDS("uvmap.dds");
    //GLuint Texture1 = png_texture_load("1.png", &width, &height);
    //GLuint Texture1 = loadImg_opencv("1.png");
    GLuint Texture1;
    glGenTextures(1, &Texture1);


    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // Read our .obj file
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals; // Won't be used at the moment.
    loadOBJ("SpongeBob/spongebob.obj", vertices, uvs, normals);
    //bool res = loadOBJ("cube.obj", vertices, uvs, normals);

    // Load it into a VBO

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);


    static const GLfloat g_vertex_buffer_data[] = {
            1.0f, -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            -1.0f,  -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f,
            -1.0f,  -1.0f, 0.0f,
    };

    static const GLfloat g_uv_buffer_data[] = {
            1.0f, 0.0f,
            1.0f, 1.0f,
            0.0f,  0.0f,
            1.0f, 1.0f,
            0.0f,  1.0f,
            0.0f,  0.0f,
    };

    GLuint colorbuffer;
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    //glBufferData(GL_ARRAY_BUFFER, cube_uvs.size() * sizeof(glm::vec2), &cube_uvs[0], GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    GLuint cubebuffer;
    glGenBuffers(1, &cubebuffer);
    glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);


    Mat frame_left, frame_right, frame_left_rectified, frame_right_rectified;
    bool slamMode = 0;
    //loadIntrinsics("temp/Intrinsics.xml", "temp/Distortion.xml");

    Ptr<ORB> orb = ORB::create();
    orb->setScoreType(cv::ORB::FAST_SCORE);
    orb->setMaxFeatures(1000);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming(2)");
    Tracker orb_tracker(orb, matcher, K);
    //Tracker orb_tracker;
    orb_tracker.setFirstFrame("first_frame.jpg", "engineering_bb.xml");

    do{

        cap_left.grab();
        cap_right.grab();
        cap_left.retrieve(frame_left);
        cap_right.retrieve(frame_right);

        //cap_left >> frame_left;
        //cap_right >> frame_right;

        stereoRemap(frame_left, frame_right, frame_left_rectified, frame_right_rectified);

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);


        computeMatricesFromInputs();
        //MVP = ProjectionMatrix * ViewMatrix * ModelMatrix * ScalingMatrix;

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE1);
        Texture1 = loadframe_opencv(frame_left, Texture1);
        glBindTexture(GL_TEXTURE_2D, Texture1);


        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 1);
        //ModelMatrix= glm::translate(glm::mat4(1.0), glm::vec3(1.5f, 0.5f, 0.0f));
        //ScalingMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(0.5f));
        glm::mat4 MVP = glm::mat4(1.0);
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        glDisable(GL_DEPTH_TEST);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );
        glDrawArrays(GL_TRIANGLES, 0, 6 );


        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_SPACE ) == GLFW_PRESS)
            slamMode = true;


        //success = TryInitModelMatrix(frame_left, slamMode);
        success = orb_tracker.process(frame_left_rectified, slamMode);

        if (!success){
            glfwPollEvents();
            if (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
                glfwWindowShouldClose(window) == 0){
                glfwSwapBuffers(window);
                continue;
            } else
                break;
        }



        // Compute the MVP matrix from tracking result
        /*
        bool found_chessboard = track_chessboard(img);
        glfwPollEvents();

        if (!found_chessboard){
            if (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
                glfwWindowShouldClose(window) == 0){
                glfwSwapBuffers(window);
                continue;
            } else
                break;
        }
        */

        //std::cout << 1 << std::endl;

        glm::mat4 ViewMatrix;
        if (!slamMode)
            ViewMatrix = getViewMatrix(slamMode);
        else{
            Mat CameraPose = SLAM.TrackStereo(frame_left_rectified, frame_right_rectified, 1);
            trackStereo(CameraPose);
            ViewMatrix = getViewMatrix(slamMode);
        }



            //glm::mat4 ProjectionMatrix = getP();
        //Mat CameraPose = SLAM.TrackStereo(frame_left_rectified, frame_right_rectified, 1);
        //trackStereo(CameraPose);
        //glm::mat4 ViewMatrix = getViewMatrix();

        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs();
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        //glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = getModelMatrix();

        //glm::mat4 initModelMatrix = getInitModelMatrix();
        glm::mat4 initModelMatrix = orb_tracker.getInitModelMatrix();

        //glm::mat4 ModelMatrix, ScalingMatrix;
        //ModelMatrix = glm::mat4(1.0);
        //ModelMatrix = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3( -1, 0, 0));
        //ModelMatrix = glm::translate(ModelMatrix, glm::vec3(3.5,0,1.5));
        //ModelMatrix = glm::rotate(90, glm::vec3( 1, 0, 0));

        //ScalingMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(5.0f));




         MVP = ProjectionMatrix * ViewMatrix * initModelMatrix * ModelMatrix;
        //MVP = ProjectionMatrix * ViewMatrix * initModelMatrix;
        //MVP = ProjectionMatrix * initModelMatrix * ModelMatrix;

        glEnable(GL_DEPTH_TEST);

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);


        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() );



        /*width = 640*16/9-1;
        height = 480*16/9-1;
        cv::Mat pixels( height, width, CV_8UC3 );
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data );
        cv::Mat cv_pixels( height, width, CV_8UC3 );
        for( int y=0; y<height; y++ ) for( int x=0; x<width; x++ )
            {
                cv_pixels.at<cv::Vec3b>(y,x)[2] = pixels.at<cv::Vec3b>(height-y-1,x)[0];
                cv_pixels.at<cv::Vec3b>(y,x)[1] = pixels.at<cv::Vec3b>(height-y-1,x)[1];
                cv_pixels.at<cv::Vec3b>(y,x)[0] = pixels.at<cv::Vec3b>(height-y-1,x)[2];
            }
        out << cv_pixels;
         */

        // Swap buffers
        glfwSwapBuffers(window);

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_R ) == GLFW_PRESS){
            slamMode = false;
            SLAM.Reset();
        }

        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    //out.release();

    return 0;
}

