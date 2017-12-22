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
 * @file tgSimpleLogger.cpp
 * @brief Contains the implementation of class tgSimpleLogger.
 * @author
 * $Id$
 */
//This module
#include "T6Model.h"
#include "tgSimpleLogger.h"
// The C++ standard library
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

/**
 * Construct our data logger
 */
tgSimpleLogger::tgSimpleLogger(std::string fileName) :
time (0.0),
m_fileName(fileName)
{
}

/** Virtual base classes must have a virtual destructor. */
tgSimpleLogger::~tgSimpleLogger()
{ }



void tgSimpleLogger::onStep(T6Model& subject, double dt)
{
	time += dt;
    // Then, open our output stream and render!
    std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);

    std::vector<double> result(subject.getBallCOM());
	
	//tgOutput << "Time:" << time << std::endl;
	//tgOutput << "COM: (" << result[0] << ", " << result[1] << ", " << result[2] << ")" << std::endl;
	tgOutput << result[0] << ", " << result[1] << ", " << result[2] << std::endl;

	tgOutput.close();
}
