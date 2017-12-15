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
 * @file SimpleModel.cpp
 * @brief Contains the definition of the members of the class SimpleModel.
 * $Id$
 */

// This module
#include "SimpleModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double pretension;
        double planar_length;
        double planar_width;
        double friction;
        double rollFriction;
        double restitution;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       0.0,     // pretension (mass * length / sec^2)
       40.0,     // planar_length (length)
       20.0,     // planar_width (length)
       0.99,      // friction (unitless)
       0.01,     // rollFriction (unitless)
       0.0,      // restitution (?)
       0,			// History logging (boolean)
       100000,   // maxTens
       10000,    // targetVelocity
  };
} // namespace

SimpleModel::SimpleModel() :
tgModel() 
{
}

SimpleModel::~SimpleModel()
{
}

void SimpleModel::addNodes(tgStructure& s,
                            double depth,
                            double width,
                            double height)
{
    // start from the origin point
    s.addNode(0, 0, 0); // 1
    // along x - axis
    s.addNode(width, 0, 0); // 2
    // along z - axis
    s.addNode(width, 0, depth); // 3
    // back to x - axis
    s.addNode(0, 0, depth); // 4
}

void SimpleModel::addRods(tgStructure& s)
{
    s.addPair( 0,  2, "rod");
    s.addPair( 1,  3, "rod");
}

void SimpleModel::addMuscles(tgStructure& s)
{
    // for planar tensegrity structure on xz surface
    s.addPair(0, 1,  "muscle");
    s.addPair(1, 2,  "muscle");
    s.addPair(2, 3,  "muscle");
    s.addPair(3, 0,  "muscle");
}

void SimpleModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    const tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.planar_width, c.planar_length, 0);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void SimpleModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void SimpleModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& SimpleModel::getAllActuators() const
{
    return allActuators;
}
    
void SimpleModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
