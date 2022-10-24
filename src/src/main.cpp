/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <functional>
#include <thread>
#include <csignal>
#include <atomic>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <HandsVisualizer.h>

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



int main(int argc, char** argv)
{
    //TODO
    //Add arms meshes
    //Test the analog readings
    //Allowing using only one hand and only one camera
    //Test on windows

    yarp::os::Network yarp; //to initialize the network

    std::atomic<bool> closing = false;
    handleSignals([&](){closing = true;});

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.configure(argc, argv);

    HandsVisualizer viz;
    viz.configure(rf);

    while(!closing)
    {
        if (!viz.update())
        {
            return EXIT_FAILURE;
        }
    }

    viz.close();

    return EXIT_SUCCESS;
}
