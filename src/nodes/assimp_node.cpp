/*#include "../assimp_loader/modelAssimp.h"
#include <GL/glut.h>

ModelAssimp *gAssimpObject;

void reshape(int width, int height)
{
    const double aspectRatio = (float) width / height, fieldOfView = 45.0;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfView, aspectRatio,
        1.0, 1000.0);
    gAssimpObject->SetViewport(500, 500);
}

void display(void)
{
    float tmp;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.f,1.f,1.f,1.f);
    gAssimpObject->Render();

}

int main(int argc, char **argv)
{

    gAssimpObject = new ModelAssimp();
    gAssimpObject->PerformGLInits();

    glutInitWindowSize(900,600);
    glutInitWindowPosition(100,100);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInit(&argc, argv);

    glutCreateWindow("Assimp - Very simple OpenGL sample");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);




    glutMainLoop();

    return 0;
}*/

//#include <glad/glad.h>
#include "../gui/imgui/imgui.h"
#include "../gui/imgui/gl3w.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../assimp_loader/shader.h"
#include "../assimp_loader/camera.h"
#include "../assimp_loader/model.h"

#include <iostream>

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings


// camera
Camera camera(glm::vec3(15.f, 25.f, -65.0f), glm::vec3(0.0f, 1.0f, 0.0f), 100.f);
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main()
{


    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    gl3wInit();
    ImGui_ImplGlfwGL3_Init(window, true);
    glfwHideWindow(window);
    //glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    //glfwSetCursorPosCallback(window, mouse_callback);
    //glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    /*if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }*/

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // build and compile shaders
    // -------------------------
    Shader ourShader("/home/ciro/uav_ws/src/ovs_controller/src/assimp_loader/assets/shaders/modelTextured.vs",
                     "/home/ciro/uav_ws/src/ovs_controller/src/assimp_loader/assets/shaders/modelTextured.fs");

    // load models
    // -----------
    Model ourModel("/home/ciro/uav_ws/src/ovs_controller/src/assimp_loader/assets/zeppelin/ZEPLIN_OBJ.obj");

    while (!glfwWindowShouldClose(window))
    {
           // per-frame time logic
           // --------------------
           float currentFrame = glfwGetTime();
           deltaTime = currentFrame - lastFrame;
           lastFrame = currentFrame;

           // input
           // -----
           processInput(window);

           // render
           // ------
           glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
           glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

           // don't forget to enable shader before setting uniforms
           ourShader.use();

           // view/projection transformations
           glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 250.0f);
           glm::mat4 view = camera.GetViewMatrix();
           ourShader.setMat4("projection", projection);
           ourShader.setMat4("view", view);

           // render the loaded model
           glm::mat4 model = glm::mat4(1.0f);
           model = glm::translate(model, glm::vec3(0.f, 0.f, 0.f)); // translate it down so it's at the center of the scene
           model = glm::scale(model, glm::vec3(0.1f, 0.1f, 0.1f));	// it's a bit too big for our scene, so scale it down
           ourShader.setMat4("model", model);
           ourModel.Draw(ourShader);

           // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
           // -------------------------------------------------------------------------------
           glfwSwapBuffers(window);


           cv::Mat prova = cv::Mat( cv::Size(SCR_WIDTH, SCR_HEIGHT), CV_8UC3 );


           glReadPixels ( 0, 0, SCR_WIDTH, SCR_HEIGHT, GL_BGR,
                          GL_UNSIGNED_BYTE, ( GLubyte * ) prova.data );

           cv::flip(prova, prova, 0);

           cv::imshow("ciao", prova);
           cv::waitKey(1);

           glfwPollEvents();
       }

       // glfw: terminate, clearing all previously allocated GLFW resources.
       // ------------------------------------------------------------------
       glfwTerminate();
       return 0;
   }

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime*10);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime*10);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime*10);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime*10);

    /*std::cout << camera.Position.x << " "
              << camera.Position.y << " "
              << camera.Position.z << "\n";

    std::cout << camera.Pitch << " " << camera.Yaw << "\n";*/
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);

}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
