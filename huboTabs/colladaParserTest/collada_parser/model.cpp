/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <boost/algorithm/string.hpp>
//#include <ros/ros.h>
#include <vector>
#include "model.h"
#include <fstream>
#include <iostream>

namespace urdf{

bool IsColladaData(const std::string& data)
{
  return data.find("<COLLADA") != std::string::npos;
}


bool Model::initFile(const std::string& filename)
{

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return Model::initString(xml_string);
  }
  else
  {
    printf( "[ERROR] Could not open file [%s] for parsing.",filename.c_str() );
    return false;
  }

}


bool Model::initParam(const std::string& param)
{/*
  ros::NodeHandle nh;
  std::string xml_string;
  
  // gets the location of the robot description on the parameter server
  std::string full_param;
  if (!nh.searchParam(param, full_param)){
    printf("[ERROR] Could not find parameter %s on parameter server", param.c_str() );
    return false;
  }

  // read the robot description from the parameter server
  if (!nh.getParam(full_param, xml_string)){
    printf("[ERROR] Could not read parameter %s on parameter server", full_param.c_str() );
    return false;
  }
  return Model::initString(xml_string); */
  return Model::initString( param );
}

bool Model::initXml(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    printf("[ERROR] Could not parse the xml document");
    return false;
  }

  std::stringstream ss;
  ss << *xml_doc;

  return Model::initString(ss.str());
}

bool Model::initXml(TiXmlElement *robot_xml)
{
  if (!robot_xml)
  {
    printf("[ERROR] Could not parse the xml element");
    return false;
  }

  std::stringstream ss;
  ss << (*robot_xml);

  return Model::initString(ss.str());
}

bool Model::initString(const std::string& xml_string)
{
  boost::shared_ptr<ModelInterface> model;

  // necessary for COLLADA compatibility
  if( IsColladaData(xml_string) ) {
    //ROS_DEBUG("Parsing robot collada xml string");
printf("Parsing robot collada xml string \n");
    model = parseCollada(xml_string);
  }
  else {
    //ROS_DEBUG("Parsing robot urdf xml string");
printf("Parsing robot urdf xml string \n");
    model = parseURDF(xml_string);
  }

  // copy data from model into this object
  if (model){
    this->links_ = model->links_;
    this->joints_ = model->joints_;
    this->materials_ = model->materials_;
    this->name_ = model->name_;
    this->root_link_ = model->root_link_;
    return true;
  }
  else
    return false;
}



}// namespace
