/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkPointCloud.h>
#include <RobotsViz/VtkiCubHand.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <vtkWindowToImageFilter.h>
#include <vtkNew.h>

#include <functional>
#include <cstring>
#include <thread>
#include <csignal>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>

using namespace RobotsIO::Camera;
using namespace RobotsViz;

std::function<void ()> customHandlerLambda;

void my_handler(int sig)
{
    static int ct = 0;

    if (sig == SIGABRT)
    {
        yInfo() << "Aborted.";
        if (ct > 3) //to avoid that std::abort is called again
        {
            return;
        }
    }

    ct++;
    if (ct > 3) {
        yInfo() <<  "Aborting (calling abort())...";
        std::abort();
    }
    yInfo() << "[try " << ct << " of 3] Trying to shut down.";

    customHandlerLambda();
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleSignals(std::function<void ()> customHandler)
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
    customHandlerLambda = customHandler;
}

void vtkImageDataToYarpimage(vtkImageData* imageData, yarp::sig::FlexImage& image)
{
    if (!imageData)
    {
        return;
    }

    /// \todo retrieve just the UpdateExtent
    int width = imageData->GetDimensions()[0];
    int height = imageData->GetDimensions()[1];

    bool useRGBA = false;
    if (imageData->GetNumberOfScalarComponents() == 3)
    {
        image.setPixelCode(VOCAB_PIXEL_RGB);
    }
    else if (imageData->GetNumberOfScalarComponents() > 3)
    {
        image.setPixelCode(VOCAB_PIXEL_RGBA);
        useRGBA = true;
    }
    else
    {
        return;
    }

    image.resize(width, height);

    unsigned char* colorsPtr =
            reinterpret_cast<unsigned char*>(imageData->GetScalarPointer());

    // Loop over the vtkImageData contents.
    for (int row = height - 1; row >= 0; row--)
    {
        for (int col = 0; col < width; col++)
        {
            if (useRGBA)
            {
                yarp::sig::PixelRgba& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgba*>(
                                                        image.getPixelAddress(col, row)));

                //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                pixelYarp.r = colorsPtr[0];
                pixelYarp.g = colorsPtr[1];
                pixelYarp.b = colorsPtr[2];
                pixelYarp.a = colorsPtr[3];
            }
            else
            {
                yarp::sig::PixelRgb& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgb*>(
                                                       image.getPixelAddress(col, row)));

                //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                pixelYarp.r = colorsPtr[0];
                pixelYarp.g = colorsPtr[1];
                pixelYarp.b = colorsPtr[2];
            }
            // Swap the vtkImageData RGB values with an equivalent QColor
            colorsPtr += imageData->GetNumberOfScalarComponents();
        }
    }
}


int main(int argc, char** argv)
{
    yarp::os::Network yarp; //to initialize the network

    bool closing = false;
    handleSignals([&](){closing = true;});

    if (argc < 7)
    {
        std::cout << "Synopsis: robmo-icub-viz <blocking> <robot_name> <hand_laterality> <use_fingers> <use_analogs> <hand_fk>" << std::endl;
        std::cout << "          Camera name <camera> is required only if <point_cloud> = true." << std::endl;
        return EXIT_FAILURE;
    }
    bool blocking = false;
    bool use_fingers = false;
    bool use_analogs = false;
    bool show_hand_fk = false;

    if (std::string(argv[1]) == "true")
        blocking = true;
    const std::string robot_name = std::string(argv[2]);
    const std::string hand_laterality = std::string(argv[3]);
    if (std::string(argv[4]) == "true")
        use_fingers = true;
    if (std::string(argv[5]) == "true")
        use_analogs = true;
    if (std::string(argv[6]) == "true")
        show_hand_fk = true;

    double fps = 30.0;
    VtkContainer container(1.0 / fps, 600, 600, blocking);

    /* Show hand according to forward kinematics. */
    if (show_hand_fk)
    {
        std::unique_ptr<VtkContent> hand = std::unique_ptr<VtkiCubHand>
        (
            new VtkiCubHand(robot_name, hand_laterality, "test-visualization/hand_fk", use_fingers, use_analogs, {100.0 / 255.0, 160 / 255.0, 255.0 / 255.0}, 1.0)
        );
        container.add_content("hand_fk", std::move(hand));
    }

    yarp::sig::FlexImage yarpImage;
    yarp::os::BufferedPort<yarp::sig::FlexImage> outputPort;
    if (!outputPort.open("/hand-visualizer/" + hand_laterality + "/image"))
    {
        return EXIT_FAILURE;
    }

    container.initialize();
    container.setOrientationWidgetEnabled(false);
    container.render_window()->SetAlphaBitPlanes(1);
    container.renderer()->SetBackground(0, 0, 0);
    while(!closing)
    {
        container.update();
        vtkNew<vtkWindowToImageFilter> w2if ;
        w2if->SetInputBufferTypeToRGBA();
        w2if->SetInput(container.render_window());
        w2if->Update();
        vtkImageDataToYarpimage(w2if->GetOutput(), yarpImage);

        yarp::sig::FlexImage& imageToBeSent = outputPort.prepare();
        imageToBeSent.setPixelCode(yarpImage.getPixelCode());
        imageToBeSent.setExternal(yarpImage.getRawImage(), yarpImage.width(), yarpImage.height()); //Avoid to copy
        outputPort.write();

    }

    return EXIT_SUCCESS;
}
