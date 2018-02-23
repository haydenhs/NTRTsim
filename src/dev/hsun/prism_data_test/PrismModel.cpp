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
 * @file PrismModel.cpp
 * @brief Contains the implementation of the class PrismModel.
 * $Id$
 */

// This module
#include "PrismModel.h"
// This library
#include "core/abstractMarker.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "sensors/tgDataObserver.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>

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
        double triangle_length;
        double triangle_height;
        double prism_height;  
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       500.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
  };
} // namespace

/*
 * helper arrays for node and rod numbering schema
 */

/*returns the number of the rod for a given node */
const int rodNumbersPerNode[6]={0,1,2,2,0,1};

PrismModel::PrismModel() :
tgModel() 
{
	m_pObserver = NULL;
}

PrismModel::PrismModel(const std::string& fileName) :
tgModel() 
{
	m_pObserver = new tgDataObserver(fileName);
}

PrismModel::~PrismModel()
{
}

void PrismModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
    // bottom right
    nodePositions.push_back(btVector3(-edge / 2.0, 0, 0)); //1
    // bottom left
    nodePositions.push_back(btVector3( edge / 2.0, 0, 0)); //2
    // bottom front
    nodePositions.push_back(btVector3(0, 0, width)); //3
    // top right
    nodePositions.push_back(btVector3(-edge / 2.0, height, 0)); //4
    // top left
    nodePositions.push_back(btVector3( edge / 2.0, height, 0)); //5
    // top front
    nodePositions.push_back(btVector3(0, height, width)); //6

    for(int i=0;i<6;i++)
    {
        s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}

void PrismModel::addRods(tgStructure& s)
{
    s.addPair( 0,  4, "r1 rod");
    s.addPair( 1,  5, "r2 rod");
    s.addPair( 2,  3, "r3 rod");
}

void PrismModel::addMuscles(tgStructure& s)
{
    // Bottom Triangle
    s.addPair(0, 1,  "muscle");
    s.addPair(1, 2,  "muscle");
    s.addPair(2, 0,  "muscle");
    
    // Top
    s.addPair(3, 4,  "muscle");
    s.addPair(4, 5,  "muscle");
    s.addPair(5, 3,  "muscle");

    //Edges
    s.addPair(0, 3,  "muscle");
    s.addPair(1, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
}

void PrismModel::addMarkers(tgStructure &s)
{
    std::vector<tgRod *> rods=find<tgRod>("rod");

    for(int i=0;i<6;i++)
    {
        const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
        btTransform inverseTransform = bt->getWorldTransform().inverse();
        btVector3 pos = inverseTransform * (nodePositions[i]);
        abstractMarker tmp=abstractMarker(bt,pos,btVector3(0.08*i,1.0 - 0.08*i,.0),i);
        this->addMarker(tmp);
    }
}

void PrismModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);

    
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
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);

    //map the rods and add the markers to them.
    addMarkers(s);

    // start an observer
    if (m_pObserver != NULL)
    {
        m_pObserver->onSetup(*this);
    }  

    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));    
}

void PrismModel::step(double dt)
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
        if (m_pObserver != NULL)
        {
            m_pObserver->onStep(*this, dt);
        }
    }
}

void PrismModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& PrismModel::getAllActuators() const
{
    return allActuators;
}
    
void PrismModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
    delete m_pObserver;
}
