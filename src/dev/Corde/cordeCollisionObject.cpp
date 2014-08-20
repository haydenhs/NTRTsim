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
 * @file cordeCollisionObject.cpp
 * @brief Interface Between Corde Model and Bullet
 * @author Brian Mirletz
 * $Id$
 */

// This Module
#include "cordeCollisionObject.h"

cordeCollisionObject::cordeCollisionObject(std::vector<btVector3>& centerLine, CordeModel::Config& Config) :
CordeModel(centerLine, Config)
{
	// Enum from btCollisionObject
	m_internalType		=	CO_USER_TYPE;
///@todo examine how to reconfigure collision shape defaults (m_friction, etc)	
}
	
cordeCollisionObject::~cordeCollisionObject() {}