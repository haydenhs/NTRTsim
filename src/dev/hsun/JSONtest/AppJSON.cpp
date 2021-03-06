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


// This application

// This library
#include "helpers/FileHelpers.h"
// JSON
#include <json/json.h>
#include <json/value.h>
// The C++ Standard Library
#include <iostream>
#include <exception>
#include <vector>
/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name; argv[1], if supplied, is the
 * suffix for the controller
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppJSONTests" << std::endl;
    

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    std::string fileStr = FileHelpers::getFileString("controlVars.json");

    bool parsingSuccessful = reader.parse( fileStr, root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        return 1;
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
    Json::Value sin_A = root.get("sin_amplitude", "UTF-8");
    
    std::cout << sin_A.size() << std::endl;
    
    //Json::Value::iterator sinIt = sin_A.begin();
    
    std::vector<double> sin_amplitude;
    
    /*for(sinIt = sin_A.begin(); sinIt != sin_A.end(); sinIt++)
    {
        std::cout << *sinIt << " ";
        sin_amplitude.push_back((*sinIt).asDouble());
    }*/

    for(int i=0;i<24;i++)
    {
        std::cout << sin_A[i] << " ";
        sin_amplitude.push_back((sin_A[i]).asDouble());
    }
    
    std::cout << std::endl;

    for(int i=0; i<sin_amplitude.size(); i++)
    {
        std::cout << sin_amplitude[i] << std::endl;
    }
    
//    string myName = root.get("name", "UTF-8").asString();
//    std::cout << myName << std::endl;
    
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
