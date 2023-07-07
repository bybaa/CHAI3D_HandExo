
// Created by YuanSu(963133358@qq.com) on 2023-June-21.
// You can get the position/orientation of your fingertip by using the HandExo device.
// Then check the data in this graphical interface.


//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
#include <GLFW/UDPSocket.h>
#include <GLFW/FingerKinematics.h>
#include <GLFW/DataPacketHandExo.h>
#include <GLFW/DeviceException.h>
#include <iostream>
#include "Eigen/Eigen"
#include <cmath>
//------------------------------------------------------------------------------
using namespace std;
using namespace chai3d;
//------------------------------------------------------------------------------
#define ADDRESS_STM32 "192.168.123.30"
#define PORT_STM32 8089
#define ADDRESS_PC "192.168.123.100"
#define PORT_PC 8089
//------------------------------------------------------------------------------
#define PI 3.1415926


//------------------------------------------------------------------------------
// For One Euro Filter
//------------------------------------------------------------------------------
#include<stdexcept>
#include<cmath>
#include<ctime>


//------------------------------------------------------------------------------
// GENERAL SETTINGS 
//------------------------------------------------------------------------------

/// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

/// fullscreen mode
bool fullscreen = false;

/// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES 
//------------------------------------------------------------------------------

/// a world that contains all objects of the virtual environment
cWorld* world;

/// a camera to render the world in the window display
cCamera* camera;

/// a light source to illuminate the objects in the world
//cSpotLight* light;
cDirectionalLight* light;

/// TODO: a small sphere (cursor) representing the haptic device
cShapeSphere* thumb_cursor;
cShapeSphere* index_cursor;
cShapeSphere* middle_cursor;
cShapeSphere* ring_cursor;
cShapeSphere* little_cursor;

///world frame
cShapeSphere* world_frame;

/// TODO: a line from world_frame to thumb_cursor
cShapeLine* lineWorldToF1;
cShapeLine* lineWorldToF2;
cShapeLine* lineWorldToF3;
cShapeLine* lineWorldToF4;
cShapeLine* lineWorldToF5;

cShapeSphere* test_cursor;

/// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

/// a haptic device handler
cHapticDeviceHandler* handler;
///a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;
/// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

/// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;
cLabel* labelHapticDeviceRotation;
/// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;
cMatrix3d hapticDeviceRotation;

/// a font for rendering text
cFontPtr font;

/// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;
cLabel* labelrotation;

/// a flag for using damping (ON/OFF)
bool useDamping = false;

/// a flag for using force field (ON/OFF)
bool useForceField = true;

/// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

/// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

/// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

/// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

/// haptic thread
cThread* hapticsThread;

/// a handle to window display context
GLFWwindow* window = NULL;

/// current width of window
int width = 0;

/// current height of window
int height = 0;

/// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

/// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

/// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

/// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

/// this function renders the scene
void updateGraphics(void);

/// this function contains the main haptics simulation loop
void updateHaptics(void);

/// this function closes the application
void close(void);

//------------------------------------------------------------------------------
// SOCKET
//------------------------------------------------------------------------------

/// sock to STM32
myDevice::ISocket<myDeviceUDP::datapacket::stm32_to_slave, myDeviceUDP::datapacket::stm32_to_slave>::ptr sock_to_stm32;
/// data packet
myDeviceUDP::datapacket::stm32_to_slave stm32_to_pc_data;
myDeviceUDP::datapacket::slave2ToM pose_and_angle;

///==============================================================================
///    DEMO:   01-HandExo_Cartesian.cpp
///==============================================================================

Eigen::Matrix4f finger1_pose;	//thumb
Eigen::Matrix4f finger2_pose;	//index
Eigen::Matrix4f finger3_pose;	//middle

Eigen::Vector4f finger1_joint;
Eigen::Vector4f finger2_joint;

///--------------------------------------------------------------------------
/// One Euro Filter
///--------------------------------------------------------------------------

typedef double TimeStamp; // in seconds

static const TimeStamp UndefinedTime = -1.0;

class LowPassFilter {

    double y, a, s;
    bool initialized;

    void setAlpha(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0)
            throw std::range_error("alpha should be in (0.0., 1.0]");
        a = alpha;
    }

public:

    LowPassFilter(double alpha, double initval = 0.0) {
        y = s = initval;
        setAlpha(alpha);
        initialized = false;
    }

    double filter(double value) {
        double result;
        if (initialized)
            result = a * value + (1.0 - a) * s;
        else {
            result = value;
            initialized = true;
        }
        y = value;
        s = result;
        return result;
    }

    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha);
        return filter(value);
    }

    bool hasLastRawValue(void) {
        return initialized;
    }

    double lastRawValue(void) {
        return y;
    }

};

// -----------------------------------------------------------------

class OneEuroFilter {

    double freq;
    double mincutoff;
    double beta_;
    double dcutoff;
    LowPassFilter* x;
    LowPassFilter* dx;
    TimeStamp lasttime;

    double alpha(double cutoff) {
        double te = 1.0 / freq;
        double tau = 1.0 / (2 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / te);
    }

    void setFrequency(double f) {
        if (f <= 0) throw std::range_error("freq should be >0");
        freq = f;
    }

    void setMinCutoff(double mc) {
        if (mc <= 0) throw std::range_error("mincutoff should be >0");
        mincutoff = mc;
    }

    void setBeta(double b) {
        beta_ = b;
    }

    void setDerivateCutoff(double dc) {
        if (dc <= 0) throw std::range_error("dcutoff should be >0");
        dcutoff = dc;
    }

public:

    OneEuroFilter(double freq,
        double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
        setFrequency(freq);
        setMinCutoff(mincutoff);
        setBeta(beta_);
        setDerivateCutoff(dcutoff);
        x = new LowPassFilter(alpha(mincutoff));
        dx = new LowPassFilter(alpha(dcutoff));
        lasttime = UndefinedTime;
    }

    double filter(double value, TimeStamp timestamp = UndefinedTime) {
        // update the sampling frequency based on timestamps
        if (lasttime != UndefinedTime && timestamp != UndefinedTime)
            freq = 1.0 / (timestamp - lasttime);
        lasttime = timestamp;
        // estimate the current variation per second
        double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
        double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
        // use it to update the cutoff frequency
        double cutoff = mincutoff + beta_ * fabs(edvalue);
        // filter the given value
        return x->filterWithAlpha(value, alpha(cutoff));
    }

    ~OneEuroFilter(void) {
        delete x;
        delete dx;
    }

};


int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    std::cout << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "CHAI3D" << std::endl;
    std::cout << "Demo: HandExo_Cartesian" << std::endl;
    std::cout << "Copyright 2003-2016" << std::endl;
    std::cout << "Created by YuanSu on 2023-June-21 !"<<std::endl;
    std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
    std::cout << "Keyboard Options:" << std::endl << std::endl;
    std::cout << "[1] - Enable/Disable potential field" << std::endl;
    std::cout << "[2] - Enable/Disable damping" << std::endl;
    std::cout << "[f] - Enable/Disable full screen mode" << std::endl;
    std::cout << "[m] - Enable/Disable vertical mirroring" << std::endl;
    std::cout << "[q] - Exit application" << std::endl;
    std::cout << std::endl << std::endl;

    ///
    sock_to_stm32 = std::make_shared<myDevice::UdpSocket<myDeviceUDP::datapacket::stm32_to_slave,myDeviceUDP::datapacket::stm32_to_slave>>();

    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        std::cout << "failed initialization" << std::endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 1.2 * mode->height;
    int h = 0.8 * mode->height;
    int x = 0.8 * (mode->width - w);
    int y = 0.8 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        std::cout << "failed to create window" << std::endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        std::cout << "failed to initialize GLEW library" << std::endl;
        glfwTerminate();
        return 1;
    }
#endif

    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    /// TODO:set background color
    world->m_backgroundColor.setGrayGainsboro();
    //world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    //   camera->set( cVector3d(-0.5, 0, 0),    // camera position (eye)
    //                cVector3d(0.5, 0, 0),     // look at position (target)
    //                cVector3d(0.0, 0.0, 1.0));    // direction of the (up) vector

    ///TODO:set camera position and orient !
    camera->set(cVector3d(-2.5, 0, 0),           // camera position (eye)
                cVector3d(0.5, 0, 0),            // look at position (target)
                cVector3d(0.0, 0.0, 1.0));       // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    //light = new cSpotLight(world);
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    //light->setLocalPos(0.0, 0.5, 0);
    //light->setLocalPos(-0.3, -0.3, 0.0);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // enable this light source to generate shadows
    //light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityHigh();

    // set shadow factor
    world->setShadowIntensity(0.3);
    // set light cone half angle
    
    //light->setCutOffAngleDeg(30);

    ///TODO: set the world,thumb,index,middle,ring,little fingertip sphere
    // create a sphere (cursor) to represent the haptic device
    thumb_cursor  = new cShapeSphere(0.02);
    index_cursor  = new cShapeSphere(0.02);
    middle_cursor = new cShapeSphere(0.02);
    ///Add the ring and little
    ring_cursor = new cShapeSphere(0.02);
    middle_cursor = new cShapeSphere(0.02);

    world_frame = new cShapeSphere(0.02);
    test_cursor = new cShapeSphere(0.02);
   
    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));

    /// TODO: a line connect world frame sphere to the other sphere
    lineWorldToF1 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF2 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF3 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF4 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF5 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));

    // insert cursor inside world
    world->addChild(thumb_cursor);
    world->addChild(index_cursor);
    world->addChild(middle_cursor);
    world->addChild(world_frame);
    world->addChild(test_cursor);
    ///TODO: if use the ring and little , insert it !
    //world->addChild(ring_cursor);
    //world->addChild(little_cursor);

    // insert line inside world
    world->addChild(velocity);
    world->addChild(lineWorldToF1);
    world->addChild(lineWorldToF2);
    world->addChild(lineWorldToF3);
    ///TODO: if use the ring and little , insert it !
    //world->addChild(lineWorldToF4);
    //world->addChild(lineWorldToF5);

    /// TODO: set color
    world_frame->m_material->setBlack();
    thumb_cursor->m_material->setBlack();
    index_cursor->m_material->setBlack();
    middle_cursor->m_material->setBlack();
    /// TODO: if use the ring and little , insert it !
    //ring_cursor->m_material->setBlack();
    //middle_cursor->m_material->setBlack();

    /// TODO: set line width
    lineWorldToF1->setLineWidth(4);
    lineWorldToF2->setLineWidth(4);
    lineWorldToF3->setLineWidth(4);
    /// TODO: if use the ring and little , insert it !
    //lineWorldToF4->setLineWidth(4);
    //lineWorldToF5->setLineWidth(4);

    /// TODO: set line color
    lineWorldToF1->m_colorPointA.setBlack();
    lineWorldToF1->m_colorPointB.setBlack();
    lineWorldToF2->m_colorPointA.setBlack();
    lineWorldToF2->m_colorPointB.setBlack();
    lineWorldToF3->m_colorPointA.setBlack();
    lineWorldToF3->m_colorPointB.setBlack();
    /// TODO: if use the ring and little , insert it !
    //lineWorldToF4->m_colorPointA.setBlack();
    //lineWorldToF4->m_colorPointB.setBlack();
    //lineWorldToF5->m_colorPointA.setBlack();
    //lineWorldToF5->m_colorPointB.setBlack();

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame

    }

    ///TODO: set a frame to the sphere
    thumb_cursor->setShowFrame(true);
    thumb_cursor->setFrameSize(0.1);

    index_cursor->setShowFrame(true);
    index_cursor->setFrameSize(0.1);

    middle_cursor->setShowFrame(true);
    middle_cursor->setFrameSize(0.1);

    world_frame->setShowFrame(true);
    world_frame->setFrameSize(0.1);

    test_cursor->setShowFrame(true);
    test_cursor->setFrameSize(0.1);

    /// TODO: if use the ring and little , insert it !
    //ring_cursor->setShowFrame(true);
    //ring_cursor->setFrameSize(0.1);
    //little_cursor->setShowFrame(true);
    //little_cursor->setFrameSize(0.1);

 
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    ///TODO: create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    labelHapticDeviceRotation = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceRotation);

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 60, 0);
}


void errorCallback(int a_error, const char* a_description)
{
    std::cout << "Error: " << a_description << std::endl;
}


void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField)
            std::cout << "> Enable force field     \r";
        else
            std::cout << "> Disable force field    \r";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            std::cout << "> Enable damping         \r";
        else
            std::cout << "> Disable damping        \r";
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------



void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------



void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    /// UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));
    labelHapticDeviceRotation->setText(hapticDeviceRotation.str(3));
    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
    //labelrotation->getRotation(rotation);

    /////////////////////////////////////////////////////////////////////
    /// RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) std::cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

bool sockInit(){

    bool ret_value = true;
    try {
        sock_to_stm32->sock_init();
        size_t udp_send_buffer_size = sizeof(myDeviceUDP::datapacket::stm32_to_slave) * 2;
        sock_to_stm32->set_recv_buf(udp_send_buffer_size);
        sock_to_stm32->sock_bind(ADDRESS_PC, PORT_PC);
        sock_to_stm32->sock_connect(ADDRESS_STM32, PORT_STM32);
        std::cout<<"sock to stm32"<<std::endl;
    }
    catch (const myDevice::DeviceException& exception)
    {
        std::cout<<"socket failed ! "<<std::endl;
        ret_value = false;
    }
    return ret_value;
}


void updateHaptics(void)
{
    /// TODO: sock to STM32
    sockInit();

    /// sensor data
    float angle[30] = {0};

    /// simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    /// TODO: first set sensor data to zero, 6 * 5 = 30

    while (simulationRunning)
    {

        cVector3d thumb_position;
        cVector3d index_position;
        cVector3d middle_position;

        cMatrix3d thumb_rotation;
        cMatrix3d index_rotation;
        cMatrix3d middle_rotation;

        double thumb_x;
        double thumb_y;
        double thumb_z;

        double index_x;
        double index_y;
        double index_z;

        double middle_x;
        double middle_y;
        double middle_z;

        double aux_thumb_rot[9];
        double aux_index_rot[9];
        double aux_middle_rot[9];

        ///sock receive
        if(sock_to_stm32->sock_receive(stm32_to_pc_data)>0){
            for(int i=0;i<30;i++){
                //std::cout<<stm32_to_pc_data.sensor_data[i]<<std::endl;
                if(stm32_to_pc_data.sensor_data[i] >= -180 && stm32_to_pc_data.sensor_data[i] <= 180){
                    angle[i] = stm32_to_pc_data.sensor_data[i];
                }
                std::cout<<angle[i]<<std::endl;
            }
        }

        //------------------------------------------------------------------------------
        // One Euro Filter
        //------------------------------------------------------------------------------

        double duration = 1000000000;     // seconds

        double frequency = 120;           // Hz
        double mincutoff = 0.05;          // FIXME
        double beta = 0.0001;             // FIXME
        double dcutoff = 1.0;             // this one should be ok

        OneEuroFilter angle0(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle1(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle2(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle3(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle4(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle5(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle6(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle7(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle8(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle9(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle10(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle11(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle12(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle13(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle14(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle15(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle16(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle17(frequency, mincutoff, beta, dcutoff);


        float filtered_angle0 = angle0.filter(angle[0], -1);
        float filtered_angle1 = angle1.filter(angle[1], -1);
        float filtered_angle2 = angle2.filter(angle[2], -1);
        float filtered_angle3 = angle3.filter(angle[3], -1);
        float filtered_angle4 = angle4.filter(angle[4], -1);
        float filtered_angle5 = angle5.filter(angle[5], -1);

        float filtered_angle6 = angle6.filter(angle[6], -1);
        float filtered_angle7 = angle7.filter(angle[7], -1);
        float filtered_angle8 = angle8.filter(angle[8], -1);
        float filtered_angle9 = angle9.filter(angle[9], -1);
        float filtered_angle10 = angle10.filter(angle[10], -1);
        float filtered_angle11 = angle11.filter(angle[11], -1);

        float filtered_angle12 = angle12.filter(angle[12], -1);
        float filtered_angle13 = angle13.filter(angle[13], -1);
        float filtered_angle14 = angle14.filter(angle[14], -1);
        float filtered_angle15 = angle15.filter(angle[15], -1);
        float filtered_angle16 = angle16.filter(angle[16], -1);
        float filtered_angle17 = angle17.filter(angle[17], -1);

        /// TODO: get fingertip pose
        finger1_pose = FingerMath::getF1TransMatrix(filtered_angle0, filtered_angle1, filtered_angle2, filtered_angle3, filtered_angle4,  filtered_angle5);
        finger2_pose = FingerMath::getF2TransMatrix(filtered_angle6, filtered_angle7, filtered_angle8, filtered_angle9, filtered_angle10, filtered_angle11);
        finger3_pose = FingerMath::getF3TransMatrix(filtered_angle12, filtered_angle13, filtered_angle14, filtered_angle15, filtered_angle16, filtered_angle17);



        /// TODO: set a scale to fingertip pose then set into CHAI3D
        /// Thumb
        thumb_x = finger1_pose(0, 3) * 0.01;
        thumb_y = finger1_pose(1, 3) * 0.01;
        thumb_z = finger1_pose(2, 3) * 0.01;
        aux_thumb_rot[0] = finger1_pose(0, 0);
        aux_thumb_rot[1] = finger1_pose(0, 1);
        aux_thumb_rot[2] = finger1_pose(0, 2);
        aux_thumb_rot[3] = finger1_pose(1, 0);
        aux_thumb_rot[4] = finger1_pose(1, 1);
        aux_thumb_rot[5] = finger1_pose(1, 2);
        aux_thumb_rot[6] = finger1_pose(2, 0);
        aux_thumb_rot[7] = finger1_pose(2, 1);
        aux_thumb_rot[8] = finger1_pose(2, 2);

        thumb_position.set(thumb_x, thumb_y, thumb_z);
        thumb_rotation.set(aux_thumb_rot[0], aux_thumb_rot[1], aux_thumb_rot[2],
                           aux_thumb_rot[3], aux_thumb_rot[4], aux_thumb_rot[5],
                           aux_thumb_rot[6], aux_thumb_rot[7], aux_thumb_rot[8]);

        ///Index
        index_x = finger2_pose(0, 3) * 0.01;
        index_y = finger2_pose(1, 3) * 0.01;
        index_z = finger2_pose(2, 3) * 0.01;
        aux_index_rot[0] = finger2_pose(0, 0);
        aux_index_rot[1] = finger2_pose(0, 1);
        aux_index_rot[2] = finger2_pose(0, 2);
        aux_index_rot[3] = finger2_pose(1, 0);
        aux_index_rot[4] = finger2_pose(1, 1);
        aux_index_rot[5] = finger2_pose(1, 2);
        aux_index_rot[6] = finger2_pose(2, 0);
        aux_index_rot[7] = finger2_pose(2, 1);
        aux_index_rot[8] = finger2_pose(2, 2);

        index_position.set(index_x, index_y, index_z);
        index_rotation.set(aux_index_rot[0], aux_index_rot[1], aux_index_rot[2], 
                           aux_index_rot[3], aux_index_rot[4], aux_index_rot[5], 
                           aux_index_rot[6], aux_index_rot[7], aux_index_rot[8]);

        ///Middle
        middle_x = finger3_pose(0, 3) * 0.01;
        middle_y = finger3_pose(1, 3) * 0.01;
        middle_z = finger3_pose(2, 3) * 0.01;

        aux_middle_rot[0] = finger3_pose(0, 0);
        aux_middle_rot[1] = finger3_pose(0, 1);
        aux_middle_rot[2] = finger3_pose(0, 2);
        aux_middle_rot[3] = finger3_pose(1, 0);
        aux_middle_rot[4] = finger3_pose(1, 1);
        aux_middle_rot[5] = finger3_pose(1, 2);
        aux_middle_rot[6] = finger3_pose(2, 0);
        aux_middle_rot[7] = finger3_pose(2, 1);
        aux_middle_rot[8] = finger3_pose(2, 2);

        middle_position.set(middle_x, middle_y, middle_z);
        middle_rotation.set(aux_middle_rot[0], aux_middle_rot[1], aux_middle_rot[2],
                            aux_middle_rot[3], aux_middle_rot[4], aux_middle_rot[5],
                            aux_middle_rot[6], aux_middle_rot[7], aux_middle_rot[8]);

    // read gripper position
    double gripperAngle;
    hapticDevice->getGripperAngleRad(gripperAngle);

    // read linear velocity
    cVector3d linearVelocity;
    hapticDevice->getLinearVelocity(linearVelocity);

    // read angular velocity
    cVector3d angularVelocity;
    hapticDevice->getAngularVelocity(angularVelocity);

    // read gripper angular velocity
    double gripperAngularVelocity;
    hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

    // read user-switch status (button 0)
    bool button0, button1, button2, button3;
    button0 = false;
    button1 = false;
    button2 = false;
    button3 = false;

    hapticDevice->getUserSwitch(0, button0);
    hapticDevice->getUserSwitch(1, button1);
    hapticDevice->getUserSwitch(2, button2);
    hapticDevice->getUserSwitch(3, button3);


    /////////////////////////////////////////////////////////////////////
    /// UPDATE 3D CURSOR MODEL
    /////////////////////////////////////////////////////////////////////

    // update arrow
    velocity->m_pointA = index_position;
    velocity->m_pointB = cAdd(index_position, linearVelocity);

    /// TODO: update position and orientation of cursor
    thumb_cursor->setLocalPos(thumb_position);
    thumb_cursor->setLocalRot(thumb_rotation);

    index_cursor->setLocalPos(index_position);
    index_cursor->setLocalRot(index_rotation);

    middle_cursor->setLocalPos(middle_position);
    middle_cursor->setLocalRot(middle_rotation);

    //
    lineWorldToF1->m_pointB = thumb_position;
    lineWorldToF2->m_pointB = index_position;
    lineWorldToF3->m_pointB = middle_position;

    hapticDevicePosition = index_position;
    hapticDeviceRotation = index_rotation;

    freqCounterHaptics.signal(1);
}

// exit haptics thread
simulationFinished = true;

}




