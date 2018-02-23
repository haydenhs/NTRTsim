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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
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

/*namespace
{
    // Parameters obtained from form-finding algorithm in MATLAB
    // In MATLAB file, the rest length for muscle is 8.5 dm.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
        bool moveCPA;
        bool moveCPB;
    } c =
   {
     0.688,    // density (kg / length^3), adjust the mass
     0.3,      // radius (length) in dm
     352.94,   // stiffness (kg / sec^2) stiffness/10 to obtain tension in (kg * m / sec^2)
     400.0,    // damping (kg / sec) 
     15,       // rod_length (length) in dm
     6.545,    // rod_space (length) in dm
     0.0,      // friction (unitless), we do not use
     0.0,      // rollFriction (unitless), we do not use
     0.0,      // restitution (?), we do not use
     529.41,   // pretension -> set to 1.5 * 352.94, pretension = stiffness * (start_length-rest_length) 
     0,		   // History logging (boolean), we do not use
     100000,   // maxTens , motor parameter
     10000,    // targetVelocity, motor parameter
     false,    // moveCablePointAToEdge
     false,    // moveCablePointBToEdge
  };
} // namespace*/

// parameters for superBall
/*namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
        bool moveCPA;
        bool moveCPB;
    } c =
   {
     0.688,    // density (kg / length^3)
     0.31,     // radius (length)
     613.0,   // stiffness (kg / sec^2) was 1500
     200.0,    // damping (kg / sec)
     16.84,     // rod_length (length)
     7.5,      // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     2452.0,        // pretension -> set to 4 * 613, the previous value of the rest length controller
     0,         // History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity
     false,    // moveCablePointAToEdge
     false,    // moveCablePointBToEdge

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace
*/

// parameters for real world model in July, 2017
namespace
{
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
        bool moveCPA;
        bool moveCPB;
    } c =
   {
     2.855,    // density (kg / length^3) ok
     0.1,     // radius (length) ok
     206.19,   // stiffness (kg / sec^2) ok
     100.0,    // damping (kg / sec) ok
     6,     // rod_length (length) ok
     1.5,      // rod_space (length) ok
     0.99,      // friction (unitless) ok
     0.01,     // rollFriction (unitless) ok
     0.0,      // restitution (?) ok
     206.19,   // pretension -> set to 1 * stiffness ok
     0,         // History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity
     true,    // moveCablePointAToEdge
     true,    // moveCablePointBToEdge
  };
} // namespace



/*
 * helper arrays for node and rod numbering schema
 */
/*returns the number of the rod for a given node */
const int rodNumbersPerNode[12]={0,0,1,1,2,2,3,3,4,4,5,5};

T6Model::T6Model() : tgModel() 
{
    m_pDataObserver = NULL;
}

T6Model::T6Model(const std::string& fileName) :
tgModel() 
{
    m_pDataObserver = new tgDataObserver(fileName);
}

T6Model::~T6Model()
{
}

/* Front image of the T6 tensegrity model
 * -----1-----3-----
 * --------7--------
 * --8-----------9--
 * --------5--------
 * -----0-----2----- 
 * Back image of the T6 tensegrity model
 * -----3-----1-----
 * --------6--------
 * -11-----------10-
 * --------4--------
 * -----2-----0-----   
 */

void T6Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    nodePositions.push_back(btVector3(-c.rod_space,  -half_length, 0));            // 0
    nodePositions.push_back(btVector3(-c.rod_space,   half_length, 0));            // 1
    nodePositions.push_back(btVector3( c.rod_space,  -half_length, 0));            // 2
    nodePositions.push_back(btVector3( c.rod_space,   half_length, 0));            // 3
    nodePositions.push_back(btVector3(0,           -c.rod_space,   -half_length)); // 4
    nodePositions.push_back(btVector3(0,           -c.rod_space,    half_length)); // 5
    nodePositions.push_back(btVector3(0,            c.rod_space,   -half_length)); // 6
    nodePositions.push_back(btVector3(0,            c.rod_space,    half_length)); // 7
    nodePositions.push_back(btVector3(-half_length, 0,            c.rod_space));   // 8
    nodePositions.push_back(btVector3( half_length, 0,            c.rod_space));   // 9
    nodePositions.push_back(btVector3(-half_length, 0,           -c.rod_space));   // 10
    nodePositions.push_back(btVector3( half_length, 0,           -c.rod_space));   // 11

    for(int i=0;i<12;i++)
    {
        s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}

void T6Model::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "r1 rod");
    s.addPair( 2,  3, "r2 rod");
    s.addPair( 4,  5, "r3 rod");
    s.addPair( 6,  7, "r4 rod");
    s.addPair( 8,  9, "r5 rod");
    s.addPair(10, 11, "r6 rod");
}

void T6Model::addMuscles(tgStructure& s)
{

    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");

    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");

    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");

    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");
}

void T6Model::addMarkers(tgStructure &s)
{
    // the order of markers is the same as the order of nodes.
    std::vector<tgRod *> rods=find<tgRod>("rod");

    for(int i=0;i<12;i++)
    {
        const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
        btTransform inverseTransform = bt->getWorldTransform().inverse();
        btVector3 pos = inverseTransform * (nodePositions[i]);
        abstractMarker tmp=abstractMarker(bt,pos,btVector3(0.08*i,1.0 - 0.08*i,.0),i);
        this->addMarker(tmp);
    }
}

void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity, 0.1, 0.1, 0, c.moveCPA, c.moveCPB);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
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
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();
   
    // Actually setup the children
    tgModel::setup(world);

    //map the rods and add the markers to them.
    addMarkers(s); 

    btVector3 location(0,10.0,0);
    //btVector3 rotation(0.0,0.6,0.8);
    //btVector3 speed(0,20,100);
    btVector3 rotation(0.0,0.0,0.0);
    btVector3 speed(0,0,0);
    this->moveModel(location,rotation,speed);

    // start an observer    
    if (m_pDataObserver != NULL)
    {
        m_pDataObserver->onSetup(*this);
    } 
}

void T6Model::step(double dt)
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
        if (m_pDataObserver != NULL)
        {
            m_pDataObserver->onStep(*this, dt);
        }
    }
}

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T6Model::getAllActuators() const
{
    return allActuators;
}
    
void T6Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
    delete m_pDataObserver;
}

// Return the center of mass of this model
// Pre-condition: This model has 6 rods
std::vector<double> T6Model::getBallCOM() {   
    std::vector <tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());

    btVector3 ballCenterOfMass(0, 0, 0);
    double ballMass = 0.0; 
    for (std::size_t i = 0; i < rods.size(); i++) {   
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }

    assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = ballCenterOfMass[i]; }
    //std::cout<<"COM: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";

    return result;
}

void T6Model::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
{
    std::vector<tgRod *> rods=find<tgRod>("rod");

    btQuaternion initialRotationQuat;
    initialRotationQuat.setEuler(rotationVector[0],rotationVector[1],rotationVector[2]);
    btTransform initialTransform;
    initialTransform.setIdentity();
    initialTransform.setRotation(initialRotationQuat);
    initialTransform.setOrigin(positionVector);
    for(int i=0;i<rods.size();i++)
    {
            rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
            rods[i]->getPRigidBody()->setWorldTransform(initialTransform * rods[i]->getPRigidBody()->getWorldTransform());
    }
}
