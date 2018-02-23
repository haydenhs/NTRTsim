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
 * @file T6SineWaves.cpp
 * @brief Contains the implementation of class T6SineWaves
 * @author
 * $Id$
 */

// This module
#include "T6SineWaves.h"
#include "T6Model.h"

// NTRTSim
#include "core/tgBasicActuator.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <string>
#include <cmath>
// JSON Serialization
#include "helpers/FileHelpers.h"
#include <json/json.h>

T6SineWaves::Config::Config(std::string fileName)
{
    // begin JSON
    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString("controlVars.json"), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        return ;
    }

    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
    Json::Value sin_A = root.get("sin_amplitude", "UTF-8");
    Json::Value sin_F = root.get("sin_frequency", "UTF-8");
    Json::Value sin_PH = root.get("sin_phase_offset", "UTF-8");
    Json::Value sin_PO = root.get("sin_position_offset", "UTF-8");
    
    for(int i=0; i < sin_A.size(); i++)
    {
        sinAmplitude.push_back((sin_A[i]).asDouble());
        sinFrequency.push_back((sin_F[i]).asDouble());
        sinPhaseOffset.push_back((sin_PH[i]).asDouble());
        sinPosOffset.push_back((sin_PO[i]).asDouble());
    }

	updateFrequency = root.get("updateFrequency", "UTF-8").asDouble();

	//end JSON
}

T6SineWaves::Config::~Config()
{
    //classes in std are deleted automatically ?
}


T6SineWaves::T6SineWaves(std::string fileName) :
m_config(fileName),
simTime(0.0),
updateTime(0.0),
m_tensions(24,0)
{

}

T6SineWaves::~T6SineWaves()
{

}

void T6SineWaves::calculateTensions()
{
    for(std::size_t i = 0; i < m_controllers.size(); i++)
    {
		// This will reproduce the same value until simTime is updated.
        double cycle = sin(simTime * m_config.sinFrequency[i] + m_config.sinPhaseOffset[i]);
        double target = m_config.sinPosOffset[i] + cycle * m_config.sinAmplitude[i];
        m_tensions.push_back(target);
    }    
}

void T6SineWaves::onSetup(T6Model& subject)
{
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    //note: one problem is size_t without .size works only
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        tgBasicActuator * const pActuator = actuators[i];
        assert(pActuator != NULL);
        //calculateTensions();
        tgTensionController* m_tensController = new tgTensionController(pActuator, m_tensions[i]);
        m_controllers.push_back(m_tensController);
    }
}

void T6SineWaves::onStep(T6Model& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        updateTime += dt;
        if (updateTime >= 1.0/m_config.updateFrequency)
        {
            simTime += updateTime;
            updateTime = 0.0;
            calculateTensions();
            for(std::size_t i = 0; i < m_controllers.size(); i++)
            {
                m_controllers[i]->control(dt, m_tensions[i]);
            }
        }
    }	
}
    
void T6SineWaves::onTeardown(T6Model& subject) {
    /*std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    double distance = displacement(subject);
    double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    scores.push_back(distance);
    scores.push_back(energySpent);
*/
    std::cout << "Tearing down" << std::endl;
    for(std::size_t i = 0; i < m_controllers.size(); i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
    // If any of subject's dynamic objects need to be freed, this is the place to do so
}
