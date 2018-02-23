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

#ifndef T6MODEL_SINE_WAVES_H
#define T6MODEL_SINE_WAVES_H

/**
 * @file T6SineWaves.h
 * @brief Sine wave controllers for T6Model using JSON serialization
 * @author 
 * $Id$
 */

// NTRTSim
#include "core/tgObserver.h"
#include "controllers/tgTensionController.h"
// The C++ Standard Library
#include <vector>
#include <string>

// Forward Declarations
//class tgTensionController;
//class tgSpringCableActuator;
class T6Model;

/**
 * Control the T6Model with a series of sine wave controlllers
 */
class T6SineWaves : public tgObserver<T6Model>
{
public:

struct Config
{
	Config(std::string fileName);
	
	~Config();
		
    /**
     * Amplitudes for sine waves.
     */
    std::vector<double> sinAmplitude;

    /**
     * Angular frenquencies for sine waves.
     * Units are radians
     */
    std::vector<double> sinFrequency;

    /**
     * Phase offsets for sine waves.
     * Units are radians
     */
    std::vector<double> sinPhaseOffset;

    /**
     * Position offset for sine waves.
     */
    std::vector<double> sinPosOffset;

    /**
     *
     */
    double updateFrequency;

};
	
public:
	
	/**
	 * Construct the controller. Typically occurs in the main function.
	 * The controller will need to be attached to a subject (model)
	 * Parameters are currently set in the initalizer lists.
	 */
    T6SineWaves(std::string fileName);
    
    /**
     * Destructor. Frees the tgTensionController pointers
     */
    virtual ~T6SineWaves();

    virtual void onSetup(T6Model& subject);

    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the T6Model that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(T6Model& subject, double dt);

    virtual void onTeardown(T6Model& subject);    
    
private:
 
    /**
     * Calculate tensions. 
     * Called during this classes onStep function.
     */
    void calculateTensions();

    const Config m_config;
    
    /**
     * Parameters that determine when to apply controllers. 
     */
    double simTime;
    double updateTime;

    /**
     * value of tensions for controllers
     */
    std::vector<double> m_tensions;

    /**
     * Pointers to tension controllers 
     */    
    std::vector<tgTensionController*> m_controllers;
};

#endif // MY_MODEL_CONTROLLER_H
