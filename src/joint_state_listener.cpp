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

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl_parser/model_format.hpp"
#include "urdf/model.h"

#include "rclcpp/rclcpp.hpp"

#include "robot_state_publisher/joint_state_listener.h"
#include "robot_state_publisher/robot_state_publisher.h"

#include <sys/types.h>
#include <sys/stat.h>

struct stat info;

using namespace std::chrono_literals;

namespace robot_state_publisher
{

JointStateListener::JointStateListener(
  rclcpp::Node::SharedPtr node, const KDL::Tree & tree,
  const MimicMap & m, const std::string & urdf_xml, const urdf::Model & model)
: node_(node),
  state_publisher_(node, tree, model, urdf_xml),
  mimic_(m)
{
  /*
   * legacy code
  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  // set whether to use the /tf_static latched static transform broadcaster
  n_tilde.param("use_tf_static", use_tf_static_, true);
  // ignore_timestamp_ == true, joins_states messages are accepted, no matter their timestamp
  n_tilde.param("ignore_timestamp", ignore_timestamp_, false);
  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
  publish_interval_ = ros::Duration(1.0/max(publish_freq, 1.0));
  */

  node->declare_parameter("prefix", "");
  tf_prefix_ = node->get_parameter("prefix").as_string();

  use_tf_static_ = true;
  ignore_timestamp_ = false;
  // auto publish_freq = 50.0;
  // publish_interval_ = std::chrono::seconds(1.0/std::max(publish_freq, 1.0));
  publish_interval_ = 1s;

  // subscribe to joint state
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(
      &JointStateListener::callbackJointState, this,
      std::placeholders::_1));

  // trigger to publish fixed joints
  // if using static transform broadcaster, this will be a oneshot trigger and only run once
  timer_ = node_->create_wall_timer(1s, std::bind(&JointStateListener::callbackFixedJoint, this));
}

JointStateListener::~JointStateListener()
{}

void JointStateListener::callbackFixedJoint()
{
  state_publisher_.publishFixedTransforms(tf_prefix_, use_tf_static_);
}

void JointStateListener::callbackJointState(const sensor_msgs::msg::JointState::SharedPtr state)
{
  if (state->name.size() != state->position.size()) {
    if (state->position.empty()) {
      fprintf(stderr, "Robot state publisher ignored a JointState message about joint(s) \"%s\"\n",
        state->name[0].c_str());
    } else {
      fprintf(stderr, "Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  auto now = std::chrono::system_clock::now();
  if (last_callback_time_ > now) {
    // force re-publish of joint ransforms
    fprintf(stderr,
      "Moved backwards in time, re-publishing joint transforms!\n");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  auto last_published = now;
  for (unsigned int i = 0; i < state->name.size(); i++) {
    auto t = last_publish_time_[state->name[i]];
    last_published = (t < last_published) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  if (ignore_timestamp_ || true) {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (unsigned int i = 0; i < state->name.size(); i++) {
      joint_positions.insert(std::make_pair(state->name[i], state->position[i]));
    }

    for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++) {
      if (joint_positions.find(i->second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i->second->joint_name] * i->second->multiplier +
          i->second->offset;
        joint_positions.insert(std::make_pair(i->first, pos));
      }
    }

    state_publisher_.publishTransforms(
      joint_positions, state->header.stamp, tf_prefix_);

    // store publish time in joint map
    for (unsigned int i = 0; i < state->name.size(); i++) {
      // last_publish_time_[state->name[i]]
      // = std::chrono::time_point<std::chrono::system_clock>(msg_nanoseconds);
    }
  }
}
}  // namespace robot_state_publisher

bool read_xml(const std::string & filename, std::string & xml_string)
{
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open()) {
    while (xml_file.good()) {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return true;
  } else {
    fprintf(stderr, "Could not open file [%s] for parsing.\n", filename.c_str());
    return false;
  }
}

TiXmlNode* copy_model_into_sibling(TiXmlDocument & doc, TiXmlElement* name, TiXmlElement* pose,
  TiXmlElement* is_static)
{
  auto sdf_tag = doc.FirstChildElement("sdf");
  auto model_tag = sdf_tag->FirstChildElement("model");

  if (name != nullptr)
    model_tag->SetAttribute("name", name->GetText());
  if (pose != nullptr)
  {
    auto pose_tag = model_tag->FirstChildElement("pose");
    if (!pose_tag)
    {
	    pose_tag = new TiXmlElement("pose");
      model_tag->LinkEndChild(pose_tag);
    }
    pose_tag->LinkEndChild(new TiXmlText(pose->GetText()));
  }
  if (is_static != nullptr)
  {
    auto is_static_tag = model_tag->FirstChildElement("static");
    if (!is_static_tag)
    {
	    is_static_tag = new TiXmlElement("static");
      model_tag->LinkEndChild(is_static_tag);
    }
    is_static_tag->LinkEndChild(new TiXmlText(is_static->GetText()));
  }

  return sdf_tag->FirstChild("model")->Clone();
}

void recursive(TiXmlElement* ele)
{
  if (ele->ValueStr() == "include")
  {
    auto child_uri = ele->FirstChildElement("uri");
    if (child_uri != NULL)
    {
      std::string uri_txt = child_uri->GetText();
      std::string model_path = "";
      if (uri_txt.substr(0, 8) == "model://")
      {
        auto model = uri_txt.substr(8, uri_txt.size() - 8);
        std::string paths = std::getenv("GAZEBO_MODEL_PATH");
        if (paths == "")
          std::cout << "GAZEBO_MODEL_PATH not defined, using only ~/.gazebo/models" << std::endl;
        paths = "~/.gazebo/models:" + paths;

        std::stringstream ss(paths);
        std::vector<std::string> path_vec;
        while (ss.good())
        {
          std::string substr;
          std::getline(ss, substr, ':');
          if (substr.back() != '/')
            substr += "/" + model + "/model.sdf";
          path_vec.push_back(substr);
        }

        for (auto & path : path_vec)
        {
          if (stat(path.c_str(), &info) != 0)
            std::cout << "cannot access " << path << std::endl;
          else if (!(info.st_mode & S_IFDIR))
          {
            model_path = path;
            break;
          }
        }
      }
      else
        model_path = uri_txt;

      if (model_path != "")
      {
        auto child_name = ele->FirstChildElement("name");
        auto child_pose = ele->FirstChildElement("pose");
        auto child_static = ele->FirstChildElement("static");

        std::ifstream file_stream(model_path.c_str());
        std::string included_model_str((std::istreambuf_iterator<char>(file_stream)),
          std::istreambuf_iterator<char>());

        TiXmlDocument included_model;
        included_model.Parse(included_model_str.c_str());

        auto new_ele = copy_model_into_sibling(included_model, child_name, child_pose,
          child_static);
        ele->Parent()->InsertAfterChild(ele, *new_ele);
        ele->Parent()->RemoveChild(ele);
        ele = new_ele->ToElement();
      }
    }
  }

  if (ele == nullptr)
    return;
  for (TiXmlElement* e = ele->FirstChildElement(); e != NULL;)
  {
    // the current "e" could be removed if it's an include element, so we save the next element
    auto next = e->NextSiblingElement();
    recursive(e);
    e = next;
  }
}


// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char ** argv)
{
  // Initialize ros
  rclcpp::init(argc, argv);

  // KDL::Tree temp_tree;
  // kdl_parser::treeFromFile(argv[1], temp_tree);
  // std::cout << "tree done with " << temp_tree.getNrOfSegments() << " segments" << std::endl;

  // auto root = temp_tree.getRootSegment();
  // std::cout << "root segment is " << root->first << std::endl;
  // auto segments = temp_tree.getSegments();
  // for (auto & s : segments)
  // {
  //   std::cout << "segment named " << s.second.segment.getName() << std::endl;
  //   auto j = s.second.segment.getJoint();
  //   std::cout << "joint name " << j.getName() << " and type " << j.getType() << std::endl;
  //   auto axis = j.JointAxis();
  //   std::cout << "axis is " << axis.x() << " " << axis.y() << " " << axis.z() << std::endl;
  // }
  // KDL::Chain chain;
  // if (!temp_tree.getChain("gimbal::link_base", "gimbal::link_camera", chain))
  // {
  //   std::cout << "couldn't create chain" << std::endl;
  //   return -1;
  // }

  // std::cout << "number of joints is " << chain.getNrOfJoints() << std::endl;
  // for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
  // {
  //   // std::cout << "joint " << i << " as " << chain. << std::endl;
  // }

  // return 0;

  urdf::Model model;
  KDL::Tree tree;
  std::map<std::string, urdf::JointMimicSharedPtr> mimic;

  // gets the location of the robot description on the parameter server
  if (argc < 2) {
    fprintf(stderr, "Robot State Publisher 2 requires a file name\n");
    return -1;
  }

  // Read the URDF XML data
  std::string xml_str;
  if (!read_xml(argv[1], xml_str)) {
    fprintf(stderr, "failed to read xml '%s'\n", argv[1]);
    return -1;
  }

  auto format = kdl_parser::guessFormatFromFilename(argv[1]);
  if (format == kdl_parser::MODEL_UNKNOWN)
  {
    format = kdl_parser::guessFormatFromString(xml_str);
    if (format == kdl_parser::MODEL_UNKNOWN)
    {
      fprintf(stderr, "Unable to initialize guess file format from %s\n", argv[1]);
      return -1;
    }
  }

  if (format == kdl_parser::MODEL_URDF)
  {
    fprintf(stderr, "Initialize urdf model from file: %s\n", argv[1]);
    if (!model.initFile(argv[1])) {
      fprintf(stderr, "Unable to initialize urdf::model from %s\n", argv[1]);
      return -1;
    }

    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      fprintf(stderr, "Failed to extract kdl tree from urdf robot description\n");
      return -1;
    }

    for (auto i = model.joints_.begin(); i != model.joints_.end(); i++) {
      if (i->second->mimic) {
        mimic.insert(make_pair(i->first, i->second->mimic));
      }
    }
  }
  else
  {
    std::ifstream file_stream(argv[1]);
    std::string str((std::istreambuf_iterator<char>(file_stream)),
      std::istreambuf_iterator<char>());

    TiXmlDocument xml;
    xml.Parse(str.c_str());
    recursive(xml.FirstChildElement());

    TiXmlPrinter printer;
    printer.SetIndent( "    " );
    xml.Accept( &printer );
    std::string xml_str = printer.CStr();
    // std::cout << xml_str << std::endl;

    // if (!kdl_parser::treeFromFile(argv[1], tree)) {
    if (!kdl_parser::treeFromString(xml_str, tree)) {
      fprintf(stderr, "Failed to extract kdl tree from sdf robot description\n");
      return -1;
    }
  }

  auto segments_map = tree.getSegments();
  for (auto segment : segments_map) {
    fprintf(stderr, "got segment %s\n", segment.first.c_str());
  }

  auto node = std::make_shared<rclcpp::Node>("robot_state_publisher");
  robot_state_publisher::JointStateListener state_publisher(node, tree, mimic, xml_str, model);

  rclcpp::spin(node);
  return 0;
}
