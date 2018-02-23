/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file Appsuperballmove.cpp
 * @brief move X (1-12) string of the super ball to test its locomotion
 * and log its COM to draw pictures.
 * @author hayden sun
 * $Id$
 */

// This application
#include "T6Model.h"
#include "T6SineWaves.h"
#include "tgSimpleLogger.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <fstream>
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppT6SinWave" << std::endl;

    // First create the ground and world
    
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    //const double pitch = M_PI/15.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    // const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.
    const tgWorld::Config config(981);

    tgWorld* world = new tgWorld(config, ground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    // to start trial multi times, we  use tgSimView instead of tgSimViewGraphics
    // tgSimViewGraphics* view = new tgSimViewGraphics(world, timestep_physics, timestep_graphics);
    tgSimView* view = new tgSimView(*world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation* simulation = new tgSimulation(*view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    T6Model* const myModel = new T6Model("Testdata");
    // T6Model* const myModel = new T6Model();
    // Fifth, select the controller to use, and attach it to the model.
    // Required for setting up serialization file input/output.
    const std::string suffix((argc > 1) ? argv[1] : "controlVars.json");
      
    T6SineWaves* const myControl = new T6SineWaves(suffix);
    myModel->attach(myControl);

    // Sixth, use data logger to get COM.
    tgSimpleLogger* const myLogger = new tgSimpleLogger("ballCOM.txt");    
    myModel->attach(myLogger);

    // Finally, add out model to the simulation
    simulation->addModel(myModel);
    
    // Run until the user stops
    // simulation->run();
    int nEpisodes = 5; // Number of episodes ("trial runs")
    int nSteps = 6000; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
    for (int i=0; i<nEpisodes; i++) 
    {
        std::cout << "trial time:" << (i+1) << std::endl;
        simulation->run(nSteps);
        simulation->reset();
    }

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
