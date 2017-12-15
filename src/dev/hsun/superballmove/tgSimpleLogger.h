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

// @todo:understand the meaning of TGDLR before H

//#ifndef TG_SIMPLE_LOGGER_TGDLR_H
//#define TG_SIMPLE_LOGGER_TGDLR_H

#ifndef TG_SIMPLE_LOGGER_H
#define TG_SIMPLE_LOGGER_H

/**
 * @file tgSimpleLogger.h
 * @brief Contains the definition of interface class tgSimpleLogger
 * @author 
 * $Id$
 */


// This library
#include "core/tgObserver.h"
#include <string>

// Forward declarations
class T6Model;

/**
 * Interface for an observer of the simple values
 */
class tgSimpleLogger : public tgObserver <T6Model>
{
    
public:

  /** Constructor
   * @param[in[ fileName, the filename where the data is logged
   */
  tgSimpleLogger (std::string fileName);

  /** Virtual base classes must have a virtual destructor. */
  virtual ~tgSimpleLogger ();


  virtual void onStep(T6Model& subject, double dt);
  
private:
	double time;
	std::string m_fileName;
};

#endif
