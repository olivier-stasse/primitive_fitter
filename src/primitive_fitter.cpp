// Authors: André Brandenburger <brandenburger@dismail.de>
//          Diego Rodriguez <rodriguez@ais.uni-bonn.de>
//          Arindam Roychoud

/**
 * \file src/primitive_fitter.cpp
 *
 * \brief Automatic shape fitter for collision complexity reduction of URDF files.
 */

#include <nimbro_primitive_fitter/primitive_fitter.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <iterator>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <math.h>
#include <functional>
#include <regex>

#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/unordered_set.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pugixml.hpp>

#include "../contrib/approx_mvbb/include/ApproxMVBB/ComputeApproxMVBB.hpp"

#include "nimbro_primitive_fitter/inertia.h"
#include <roboptim/capsule/util.hh>

#define NODE_NAME "nimbro_primitive_fitter"

/*******************************************************************************************************************************************************/



// Shape fitter for boxes
BoxFit::BoxFit(std::vector<double> points, Eigen::Vector3d translation,
               Eigen::Vector3d rotation, Eigen::Vector3d scale,
               Eigen::Matrix4d localTransformation,
               std::shared_ptr<rclcpp::Node> nh
               ) : ShapeFit(nh)
{
  // Transform points to 3xLEN matrix
  checkPoints(points);
  readPoints(points);

  // Call MVBB routine
  RCLCPP_INFO(nh->get_logger(), "Computing MVBB... ");

  bool pointSamples_is_relative;
  double epsilon, pointSamples, gridSize, mvbbDiamOptLoops, mvbbGridSearchOptLoops;

  std::string nodeName = NODE_NAME;

  pointSamples = nh_->get_parameter("/" + nodeName +
                                    "/boxfit/pointSamples_is_relative").as_double();
  epsilon = nh_->get_parameter("/" + nodeName +
                               "/boxfit/epsilon").as_double();
  pointSamples = nh_->get_parameter("/" + nodeName +
                                    "/boxfit/pointSamples").as_double();
  gridSize = nh_->get_parameter("/" + nodeName +
                                "/boxfit/gridSize").as_double();
  mvbbDiamOptLoops = nh_->get_parameter("/" + nodeName +
                                        "/boxfit/mvbbDiamOptLoops").as_double();
  mvbbGridSearchOptLoops = nh_->get_parameter("/" + nodeName +
                                              "/boxfit/mvbbGridSearchOptLoops").as_double();

  int num_samples;

  if(pointSamples_is_relative)
  {
    num_samples = std::floor(pointSamples * m_points.cols());
  }
  else
  {
    num_samples = std::floor(pointSamples);
  }

  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "using " << num_samples
                     << " samples(out of "
                     << m_points.cols() << ")...");

  m_oobb = ApproxMVBB::approximateMVBB(m_points, epsilon, num_samples, gridSize,
                                       mvbbDiamOptLoops,
                                       mvbbGridSearchOptLoops);

  ApproxMVBB::Matrix33 A_KI = m_oobb.m_q_KI.matrix().transpose();
  auto size = m_points.cols();

  for( unsigned int i = 0;  i < size; ++i )
  {
    m_oobb.unite(A_KI * m_points.col(i));
  }

  // Extract results
  m_box_center = (m_oobb.m_q_KI * m_oobb.center()).eval();
  m_box_size = m_oobb.extent();

  // Check if an error occurred in the process
  if ( std::isinf(m_box_size[0]*m_box_size[1]*m_box_size[2]) )
  {
    RCLCPP_ERROR(nh_->get_logger(),"Faulty box-size vector! (Infinity detected)");
    RCLCPP_ERROR(nh_->get_logger(),
                 "Most likely this is caused by a bad mesh. Try reducing the detail of the mesh.");

    m_box_size[0] = 0;
    m_box_size[1] = 0;
    m_box_size[2] = 0;
  }

  // Calculate RPY from the rotation matrix
  Eigen::Vector3d rpy;
  geometry::matrix2rpy(m_oobb.m_q_KI.matrix(), rpy);
  m_roll = rpy[2];
  m_pitch = rpy[1];
  m_yaw = rpy[0];

  RCLCPP_INFO_STREAM(nh_->get_logger(),"Box size: " << m_box_size );
  RCLCPP_INFO_STREAM(nh_->get_logger(),"Box center: " << m_box_center );
  RCLCPP_INFO_STREAM(nh_->get_logger(), "r: p: y:" << m_roll << "   "
                     << m_pitch
                     << "   "  << m_yaw ) ;
}


// Check if the points are OK to be read in
bool BoxFit::checkPoints(const std::vector<double> points)
{
  // Test if number of coordinates is multiple of 3
  if (points.size() % 3 != 0)
  {
    throw std::invalid_argument(
        "Error: points should be an array of 3D points, e.g. x0 y0 z0 x1 y1 z1 etc.");
  }

  return true;
}


// Read the points from the flat vector
void BoxFit::readPoints(const std::vector<double> points)
{
  int num_coords = points.size();

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),"File contains: " << num_coords / 3 << " points");

  // Parse points
  boost::unordered_set<std::vector<float>> u_pointset;
  u_pointset.clear();

  for (int i = 0; i < num_coords; i += 3)
  {
    std::vector<float> f;
    f.push_back(points.at(i));
    f.push_back(points.at(i + 1));
    f.push_back(points.at(i + 2));
    u_pointset.insert(f);
  }

  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "(" << u_pointset.size() << " unique) ");

  boost::unordered_set<std::vector<float>>::iterator u_iter;

  for (u_iter = u_pointset.begin(); u_iter != u_pointset.end(); u_iter++)
  {
    m_points.conservativeResize(m_points.rows(), m_points.cols() + 1);
    Eigen::Vector3d vec((*u_iter)[0], (*u_iter)[1], (*u_iter)[2]);
    m_points.col(m_points.cols() - 1) = vec;
  }
}


// Writes the collision model to the URDF
void BoxFit::writeUrdf(std::shared_ptr<pugi::xml_node> lastLinkNode)
{
  // Box
  pugi::xml_node collision = lastLinkNode->append_child("collision");
  pugi::xml_node origin = collision.append_child("origin");

  // Actual collision model
  pugi::xml_node geom = collision.append_child("geometry");
  pugi::xml_node box = geom.append_child("box");
  pugi::xml_attribute dimensions = box.append_attribute("size");

  // Box dimension
  std::stringstream dimSS;
  dimSS << m_box_size[0] << " " << m_box_size[1] << " " << m_box_size[2];
  dimensions.set_value(dimSS.str().c_str());

  // Translation ...
  std::stringstream xyzSS;
  xyzSS << (m_box_center[0]) << " " << (m_box_center[1]) << " " << (m_box_center[2]);
  origin.append_attribute("xyz").set_value(xyzSS.str().c_str());

  // ... and rotation of box
  std::stringstream rpySS;
  rpySS << m_roll << " " << m_pitch << " " << m_yaw;
  origin.append_attribute("rpy").set_value(rpySS.str().c_str());
}


void BoxFit::writeUrdfXacro(std::shared_ptr<pugi::xml_node> lastLinkNode)
{
  // Same deal, since natively supported structure
  writeUrdf(lastLinkNode);
}


/*******************************************************************************************************************************************************/

double CapsuleFit::getLength()
{
  return m_length;
}


Eigen::Vector3d CapsuleFit::getMidpoint()
{
  return m_midpoint;
}


Eigen::Vector3d CapsuleFit::getRotation()
{
  return m_rot;
}

// Shape fitter for capsules
CapsuleFit::CapsuleFit(std::vector<double> points, Eigen::Vector3d translation,
                       Eigen::Vector3d rotation, Eigen::Vector3d scale,
                       Eigen::Matrix4d localTransformation,
                       std::shared_ptr<rclcpp::Node> nh) : ShapeFit(nh)
{
  m_points = points;

  /* Get the capsule endpoints and radius using roboptim capsule*/
  assert(m_points.size() > 0 && "Cannot compute capsule for point set.");

  std::string solver = "ipopt";

  if (m_points.size() % 3 != 0)
  {
    throw std::invalid_argument(
        "Error: points should be an array of 3D points, e.g. x0 y0 z0 x1 y1 z1 etc.");
  }

  // Load polyhedron
  polyhedron_t polyhedron;

  for (size_t i = 0; i < m_points.size(); i += 3)
  {
    point_t p(m_points[i], m_points[i + 1], m_points[i + 2]);
    polyhedron.push_back(p);
  }

  // Fitter expects a vector of polyhedrons
  polyhedrons_t polyhedrons;
  polyhedrons.push_back(polyhedron);

  // Create fitter
  Fitter fitter(polyhedrons, solver);

  fitter.logDirectory() = "log";

  // Compute initial guess
  point_t P0;
  point_t P1;
  value_type r = 0.;
  argument_t initParam(7);

  polyhedrons_t convexPolyhedrons;
  computeConvexPolyhedron(polyhedrons, convexPolyhedrons);
  computeBoundingCapsulePolyhedron(convexPolyhedrons, P0, P1, r);
  convertCapsuleToSolverParam(initParam, P0, P1, r);

  // Compute optimal capsule
  fitter.computeBestFitCapsule(initParam);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "best capsule computed" );

  // assign values to e1,e2 and radius
  m_endpoint1 = Eigen::Vector3d(fitter.solutionParam()(0, 0),
                                fitter.solutionParam()(1, 0), fitter.solutionParam()(2, 0));
  m_endpoint2 = Eigen::Vector3d(fitter.solutionParam()(3, 0),
                                fitter.solutionParam()(4, 0), fitter.solutionParam()(5, 0));
  m_radius = fitter.solutionParam()(6, 0);

  Eigen::Matrix4d transform = geometry::create_transformation_matrix(rotation, translation, scale);

  transformParameters(transform, localTransformation);
}


/* Transform capsule parameters based on urdf and collada files' original orientation for the mesh*/
void CapsuleFit::transformParameters(const Eigen::Matrix4d &transform, const Eigen::Matrix4d &localTransform)
{
  Eigen::Vector4d homoe1 = m_endpoint1.homogeneous();
  Eigen::Vector4d homoe2 = m_endpoint2.homogeneous();

  homoe1 = transform * localTransform * homoe1;
  homoe2 = transform * localTransform * homoe2;
  m_radius = m_radius * localTransform(0, 0);

  m_endpoint1 = homoe1.hnormalized();
  m_endpoint2 = homoe2.hnormalized();

  Eigen::Vector3d capAxis = m_endpoint2 - m_endpoint1;

  m_length = (m_endpoint1 - m_endpoint2).norm();
  m_midpoint = (m_endpoint1 + m_endpoint2) * 0.5;

  m_rot = geometry::rpyFromaxisAngles(capAxis);
}


void CapsuleFit::writeUrdf(std::shared_ptr<pugi::xml_node> lastLinkNode)
{
  // The first sphere
  pugi::xml_node s1Collision = lastLinkNode->append_child("collision");
  pugi::xml_node s1Origin = s1Collision.append_child("origin");

  std::stringstream s1rpySS;
  s1rpySS << m_rot(0) << " " << m_rot(1) << " " << m_rot(2);
  s1Origin.append_attribute("rpy").set_value(s1rpySS.str().c_str());

  std::stringstream s1xyzSS;
  s1xyzSS << (m_endpoint1(0)) << " " << (m_endpoint1(1)) << " " << (m_endpoint1(2));
  s1Origin.append_attribute("xyz").set_value(s1xyzSS.str().c_str());

  pugi::xml_node s1Geom = s1Collision.append_child("geometry");
  pugi::xml_node s1 = s1Geom.append_child("sphere");
  pugi::xml_attribute rs1 = s1.append_attribute("radius");
  rs1.set_value(m_radius);

  // The middle cylinder
  pugi::xml_node crCollision = lastLinkNode->append_child("collision");
  pugi::xml_node crOrigin = crCollision.append_child("origin");

  std::stringstream crrpySS;
  crrpySS << m_rot(0) << " " << m_rot(1) << " " << m_rot(2);
  crOrigin.append_attribute("rpy").set_value(crrpySS.str().c_str());

  std::stringstream crxyzSS;
  crxyzSS << (m_midpoint(0)) << " " << (m_midpoint(1)) << " " << (m_midpoint(2));
  crOrigin.append_attribute("xyz").set_value(crxyzSS.str().c_str());

  pugi::xml_node crGeom = crCollision.append_child("geometry");
  pugi::xml_node cyl = crGeom.append_child("cylinder");
  pugi::xml_attribute crl = cyl.append_attribute("length");
  crl.set_value(m_length);
  pugi::xml_attribute crr = cyl.append_attribute("radius");
  crr.set_value(m_radius);

  // The second sphere
  pugi::xml_node s2Collision = lastLinkNode->append_child("collision");
  pugi::xml_node s2Origin = s2Collision.append_child("origin");

  std::stringstream s2rpySS;
  s2rpySS << m_rot(0) << " " << m_rot(1) << " " << m_rot(2);
  s2Origin.append_attribute("rpy").set_value(s1rpySS.str().c_str());

  std::stringstream s2xyzSS;
  s2xyzSS << (m_endpoint2(0)) << " " << (m_endpoint2(1)) << " " << (m_endpoint2(2));
  s2Origin.append_attribute("xyz").set_value(s2xyzSS.str().c_str());

  pugi::xml_node s2Geom = s2Collision.append_child("geometry");
  pugi::xml_node s2 = s2Geom.append_child("sphere");
  pugi::xml_attribute s2r = s2.append_attribute("radius");
  s2r.set_value(m_radius);
}


void CapsuleFit::writeUrdfXacro(std::shared_ptr<pugi::xml_node> lastLinkNode)
{
  pugi::xml_node collision = lastLinkNode->append_child(
      "xacro:capsule-collision");

  std::stringstream e1SS;
  e1SS << (m_endpoint1(0)) << " " << (m_endpoint1(1)) << " " << (m_endpoint1(2));
  collision.append_attribute("e1").set_value(e1SS.str().c_str());

  std::stringstream e2SS;
  e2SS << (m_endpoint2(0)) << " " << (m_endpoint2(1)) << " " << (m_endpoint2(2));
  collision.append_attribute("e2").set_value(e2SS.str().c_str());

  collision.append_attribute("rad").set_value(m_radius);

  collision.append_attribute("len").set_value(m_length);

  std::stringstream rpySS;
  rpySS << (m_rot(0)) << " " << (m_rot(1)) << " " << (m_rot(2));
  collision.append_attribute("rpy").set_value(rpySS.str().c_str());

  std::stringstream midSS;
  midSS << (m_midpoint(0)) << " " << (m_midpoint(1)) << " " << (m_midpoint(2));
  collision.append_attribute("mid").set_value(midSS.str().c_str());
}


// static!
void CapsuleFit::writeXacroMacro(const pugi::xml_document* doc)
{
  pugi::xml_node robotNode = doc->child("robot");

  pugi::xml_node xacroNode = robotNode.prepend_child("xacro:macro");
  xacroNode.append_attribute("name").set_value("capsule-collision");
  xacroNode.append_attribute("params").set_value("e1 e2 rad len rpy mid");

  // The first sphere
  pugi::xml_node s1Collision = xacroNode.append_child("collision");
  pugi::xml_node s1Origin = s1Collision.append_child("origin");
  s1Origin.append_attribute("rpy").set_value("${rpy}");
  s1Origin.append_attribute("xyz").set_value("${e1}");

  pugi::xml_node s1Geom = s1Collision.append_child("geometry");
  pugi::xml_node s1 = s1Geom.append_child("sphere");
  pugi::xml_attribute rs1 = s1.append_attribute("radius");
  rs1.set_value("${rad}");

  // The middle cylinder
  pugi::xml_node crCollision = xacroNode.append_child("collision");
  pugi::xml_node crOrigin = crCollision.append_child("origin");
  crOrigin.append_attribute("rpy").set_value("${rpy}");
  crOrigin.append_attribute("xyz").set_value("${mid}");

  pugi::xml_node crGeom = crCollision.append_child("geometry");
  pugi::xml_node cyl = crGeom.append_child("cylinder");
  pugi::xml_attribute crl = cyl.append_attribute("length");
  crl.set_value("${len}");
  pugi::xml_attribute crr = cyl.append_attribute("radius");
  crr.set_value("${rad}");

  // The second sphere
  pugi::xml_node s2Collision = xacroNode.append_child("collision");
  pugi::xml_node s2Origin = s2Collision.append_child("origin");
  s2Origin.append_attribute("rpy").set_value("${rpy}");
  s2Origin.append_attribute("xyz").set_value("${e2}");

  pugi::xml_node s2Geom = s2Collision.append_child("geometry");
  pugi::xml_node s2 = s2Geom.append_child("sphere");
  pugi::xml_attribute s2r = s2.append_attribute("radius");
  s2r.set_value("${rad}");
}


//******************************************************************************************************************
// URDF class
Urdf::Urdf(const char *filepath, bool xacro_support,
           std::shared_ptr<PrimitiveFitterNode> nh) : nh_(nh)
{
  m_filepath = filepath;
  m_xacro = xacro_support; // boolean

  // Load URDF XML
  pugi::xml_parse_result result = m_doc.load_file(filepath);
  RCLCPP_DEBUG(nh_->get_logger(),
               "URDF File: %s", filepath);
  RCLCPP_DEBUG(nh_->get_logger(),
               "URDF File parse result: %s", result.description());

  // check whether XML could be read
  m_URDFLoaded = (result.status == pugi::status_ok);

  if (!m_URDFLoaded)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "The URDF could NOT be loaded");
    rclcpp::shutdown();
  }

  m_nodes = m_doc.select_nodes(m_collision_path.c_str());
  m_visual_nodes = m_doc.select_nodes(m_visual_path.c_str());

  m_it = m_nodes.begin();
  m_visual_it = m_visual_nodes.begin();

  // write the xacro macros for all shapes
  if (xacro_support)
  {
    addXacroDef();
  }
}


bool Urdf::save(const char* path)
{
  RCLCPP_INFO(nh_->get_logger(),
              "************************************************");
  RCLCPP_INFO(nh_->get_logger(),
              "Saved file with path: %s", path);
  RCLCPP_INFO(nh_->get_logger(),
              "************************************************");
  return m_doc.save_file(path);
}


void Urdf::addXacroDef()
{
  RCLCPP_DEBUG(nh_->get_logger(),
               "Adding xacro definitions...");

  // NOTE: write macros for each shape!
  CapsuleFit::writeXacroMacro(&m_doc);
  BoxFit::writeXacroMacro(&m_doc);
}


bool Urdf::isXacro()
{
  return m_xacro;
}


std::string Urdf::getNextMesh()
{
  m_lastMeshNode = m_it->node();
  m_lastCollisionNode = m_lastMeshNode.parent().parent();
  m_lastLinkNode = m_lastCollisionNode.parent();

  std::string filename( m_lastMeshNode.attribute("filename").value() );
  m_it++;

  return filename;
}

std::string Urdf::getNextVisualMesh()
{
  m_lastVisualMeshNode = m_visual_it->node();
  m_lastVisualNode = m_lastVisualMeshNode.parent().parent();
  m_lastVisualLinkNode = m_lastVisualNode.parent();

  std::string filename( m_lastVisualMeshNode.attribute("filename").value() );
  m_visual_it++;

  return filename;
}


bool Urdf::hasMoreMesh()
{
  return m_it != m_nodes.end();
}


bool Urdf::hasMoreVisualMesh()
{
  return m_visual_it != m_visual_nodes.end();
}


void Urdf::replaceMesh(std::shared_ptr<ShapeFit> shape)
{
  //remove old collision element
  if (!m_lastLinkNode.remove_child("collision"))
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Can't remove collision element. Skipping..." );
  }

  shape->writeUrdf(std::make_shared<pugi::xml_node>(m_lastLinkNode));
}


void Urdf::replaceMeshWithXacro(std::shared_ptr<ShapeFit> shape)
{
  //remove old collision element
  if (!m_lastLinkNode.remove_child("collision"))   // BUG Says skipping, but is not?
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),
                       "Can't remove collision element. Skipping..." );
  }

  shape->writeUrdfXacro(std::make_shared<pugi::xml_node>(m_lastLinkNode));
}


Eigen::Vector3d Urdf::getMeshRotation() const
{
  pugi::xml_node originNode = m_lastCollisionNode.child("origin");

  std::vector<double> rpy;

  if (!originNode.attribute("rpy"))
  {
    return Eigen::Vector3d(0, 0, 0);
  }

  const char* rpyStr = originNode.attribute("rpy").value();
  geometry::getAsVector(rpyStr, &rpy);

  return Eigen::Vector3d(rpy.data());
}


Eigen::Vector3d Urdf::getMeshTranslation() const
{
  pugi::xml_node originNode = m_lastCollisionNode.child("origin");

  std::vector<double> xyz;

  if (!originNode.attribute("xyz"))
  {
    return Eigen::Vector3d(0, 0, 0);
  }

  const char* xyzStr = originNode.attribute("xyz").value();
  geometry::getAsVector(xyzStr, &xyz);

  return Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
}


Eigen::Vector3d Urdf::getMeshScale() const
{
  std::vector<double> scaleVec;

  const char* scaleStr = "";

  if (!m_lastMeshNode.attribute("scale"))
  {
    return Eigen::Vector3d(1, 1, 1);
  }

  scaleStr = m_lastMeshNode.attribute("scale").value();
  RCLCPP_INFO_STREAM(nh_->get_logger(),"scaleStr: " << scaleStr );
  geometry::getAsVector(scaleStr, &scaleVec);

  return Eigen::Vector3d(scaleVec[0], scaleVec[1], scaleVec[2]);
}


float Urdf::getMass(bool visual)
{
  pugi::xml_node mass_node;

  if (visual)
  {
    mass_node = m_lastVisualLinkNode.child("inertial").child("mass");
  }
  else
  {
    mass_node = m_lastLinkNode.child("inertial").child("mass");
  }

  if (mass_node)
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Mass: " << mass_node.attribute("value").as_float() );
    return mass_node.attribute("value").as_float();
  }
  else
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Did not find node 'inertial/mass'. Skipping mass." );
    return 1.;
  }
}


void Urdf::setInertia(vcg::Matrix33f Inertia, bool visual)
{
  pugi::xml_node inertia_node;

  if (visual)
  {
    inertia_node = m_lastVisualLinkNode.child("inertial").child("inertia");
  }
  else
  {
    inertia_node = m_lastLinkNode.child("inertial").child("inertia");
  }

  if (inertia_node)
  {
    pugi::xml_attribute ixx = inertia_node.attribute("ixx");
    pugi::xml_attribute ixy = inertia_node.attribute("ixy");
    pugi::xml_attribute ixz = inertia_node.attribute("ixz");
    pugi::xml_attribute iyy = inertia_node.attribute("iyy");
    pugi::xml_attribute iyz = inertia_node.attribute("iyz");
    pugi::xml_attribute izz = inertia_node.attribute("izz");

    RCLCPP_INFO_STREAM(nh_->get_logger(),"Old Inertia:" );
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Ixx: " << ixx.as_float() << ", Ixy: " << ixy.as_float() << ", Ixz: " << ixz.as_float() << ", Iyy: "  << iyy.as_float() << ", Iyz: " << iyz.as_float() << ", Izz: " << izz.as_float() );

    ixx.set_value(Inertia[0][0]);
    ixy.set_value(Inertia[0][1]);
    ixz.set_value(Inertia[0][2]);
    iyy.set_value(Inertia[1][1]);
    iyz.set_value(Inertia[1][2]);
    izz.set_value(Inertia[2][2]);

    RCLCPP_INFO_STREAM(nh_->get_logger(),"New Inertia:" );
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Ixx: " << ixx.as_float() << ", Ixy: " << ixy.as_float() << ", Ixz: " << ixz.as_float() << ", Iyy: "  << iyy.as_float() << ", Iyz: " << iyz.as_float() << ", Izz: " << izz.as_float() );
  }
  else
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Did not find node 'inertial/inertia'. Skipping inertia." );
  }
}


void Urdf::setCenterOfMass(vcg::Point3f CenterOfMass, bool visual)
{
  pugi::xml_node com_node;

  if (visual)
  {
    com_node = m_lastVisualLinkNode.child("inertial").child("origin");
  }
  else
  {
    com_node = m_lastLinkNode.child("inertial").child("origin");
  }

  if (com_node)
  {
    pugi::xml_attribute xyz = com_node.attribute("xyz");

    RCLCPP_INFO_STREAM(nh_->get_logger(),"Old center of mass: " << xyz.value() );

    xyz.set_value((std::to_string(CenterOfMass[0]) + " " + std::to_string(CenterOfMass[1]) + " " + std::to_string(CenterOfMass[2])).c_str());

    RCLCPP_INFO_STREAM(nh_->get_logger(),"New center of mass: " << xyz.value() );
  }
  else
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Did not find node 'inertial/origin'. Skipping center of mass." );
  }
}


/*******************************************************************************************************************************************************/
Stl::Stl(std::string url,
         std::shared_ptr<rclcpp::Node> nh): MeshFile(nh)
{
  url::Url filepath(url);

  std::string pkgPath = ament_index_cpp::get_package_share_directory(filepath.getPackageName());
  std::string filename = pkgPath + filepath.getRelativePath();
  RCLCPP_INFO_STREAM(nh_->get_logger(),"STL file path: " << filename );

  m_filestream.open(filename.c_str(), std::ios::in | std::ios::binary);

  // Check if filestream is corrupted
  if (!m_filestream)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "ERROR OPENING STL FILESTREAM!");
    assert(false);
  }
  else
  {
    // Check if the filestream is OK
    if (m_filestream.fail())
    {
      RCLCPP_ERROR(nh_->get_logger(),
                   "! ! ! ERROR AFTER STL FILESTREAM OPENED ! ! !");

      if (m_filestream.eof())
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "Unexpectedly reached EOF!");
      }
    }
    else
    {
      RCLCPP_DEBUG(nh_->get_logger(),
                   "STL filestream successfully opened");
    }
  }

  // Read header
  char header_info[80] = "";
  m_filestream.read(header_info, 80);

  // Read number of faces/triangles
  char n_triangles[4];
  m_filestream.read(n_triangles, 4);
  unsigned int* r = (unsigned int*) n_triangles;
  m_num_faces = *r;
}


float Stl::parseFloat()
{
  char f_buf[sizeof(float)];
  m_filestream.read(f_buf, 4);
  float* fptr = (float*) f_buf;
  return *fptr;
}


void Stl::dumpPoint()
{
  char dummy[12];
  m_filestream.read(dummy, 12);
}


std::vector<double> Stl::getPoints()
{
  std::vector<double> points = {};

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                      "File contains " << m_num_faces << " faces");

  for (unsigned int i = 0; i < m_num_faces; i++)
  {
    dumpPoint(); // dump the normal vector

    // Read the actual points (3 points with 3 dimensions)
    for (uint j = 0; j < 3; j++)
    {
      for (uint k = 0; k < 3; k++)
      {
        points.push_back(parseFloat());
      }
    }

    // Check if the filestream is OK
    if (m_filestream.fail())
    {
      RCLCPP_ERROR(nh_->get_logger(),"! ! ! ERROR IN STL FILESTREAM ! ! !");

      if (m_filestream.eof())
      {
        RCLCPP_ERROR(nh_->get_logger(),"Unexpectedly reached EOF!");
      }

      RCLCPP_ERROR_STREAM(nh_->get_logger(),"Error No.:" << strerror(errno));
      break;
    }

    // another dummy
    char dummy[2];
    m_filestream.read(dummy, 2);
  }

  return points;
}


//******************************************************************************************************************
Dae::Dae(std::string filename,
         std::shared_ptr<rclcpp::Node> nh): MeshFile(nh)
{
  url::Url filepath(filename);
  std::string pkgName = filepath.getPackageName();
  RCLCPP_INFO_STREAM(nh_->get_logger(),"pkgName: " << pkgName );

  std::string pkgPath = ament_index_cpp::get_package_share_directory(pkgName);
  RCLCPP_INFO_STREAM(nh_->get_logger(),"pkgPath: " << pkgPath );

  std::string path = filepath.getRelativePath();
  RCLCPP_INFO_STREAM(nh_->get_logger(),"path: " << path );

  m_filename = (pkgPath + path).c_str();
  RCLCPP_INFO_STREAM(nh_->get_logger(),"DAE file path: " << m_filename );

  pugi::xml_parse_result result = m_doc.load_file(m_filename.c_str());
  RCLCPP_INFO_STREAM(nh_->get_logger(),"DAE file load result: " << result.description() );
}


/* Extract all polygon vertices from DAE file*/
std::vector<double> Dae::getPoints()
{
  std::vector<double> points;
  const std::string path =
      "/COLLADA/library_geometries/geometry/mesh/source[*]/float_array[contains(@id,'positions')]";

  const pugi::xpath_node_set nodes = m_doc.select_nodes(path.c_str());

  for (const pugi::xpath_node* it = nodes.begin(); it != nodes.end(); ++it)
  {
    const char* vertexString = it->node().text().get();
    geometry::getAsVector(vertexString, &points);
  }

  return points;
}


/* Get transformation matrix from DAE file*/
Eigen::Matrix4d Dae::getTransformationMatrix()
{
  Eigen::Matrix4d t;
  std::vector<double> els;

  const std::string path =
      "/COLLADA/library_visual_scenes/visual_scene[@id=\"Scene\"]/node/matrix[@sid=\"transform\"]";
  const pugi::xpath_node_set nodes = m_doc.select_nodes(path.c_str());
  RCLCPP_INFO_STREAM(nh_->get_logger(),"getting trans mat... " );

  for (const pugi::xpath_node* it = nodes.begin(); it != nodes.end(); ++it)
  {
    const char* elString = it->node().text().get();
    RCLCPP_INFO_STREAM(nh_->get_logger(),"elString: " << elString );
    geometry::getAsVector(elString, &els);
  }

  t = Eigen::Matrix4d(els.data());
  RCLCPP_INFO_STREAM(nh_->get_logger(),"trans mat: " << t );

  return t;
}


//*******************************************************************************************************************
VersatileFitter::VersatileFitter(std::shared_ptr<PrimitiveFitterNode> nh) : nh_(nh)
{
}

std::string VersatileFitter::getFileExtension(std::string meshfile)
{
  std::string file_extension("");

  // Get file extension
  std::regex file_regex("([^\\\\.]+)(\\.)([^\\\\.]+)");
  std::smatch r_match;

  if (std::regex_search(meshfile, r_match, file_regex))
  {
    file_extension = r_match[3];
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Detected file extension ." << r_match[3] );
  }
  else
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(),"NO FILE EXTENSION FOUND!" );
  }

  return file_extension;
}


void VersatileFitter::fit(std::string urdf_filename,
                          std::string &output_filename,
                          VersatileFitter::Shape shape, std::string shape_name)
{
  try
  {
    Urdf urdf(urdf_filename.c_str(), true,nh_);


    // Return if the urdf was not loaded
    if ( !urdf.loaded() )
    {
      RCLCPP_ERROR(nh_->get_logger(),"URDF could not be loaded." );
      return;
    }

    while (urdf.hasMoreMesh())
    {
      std::string meshfile = urdf.getNextMesh();

      std::string file_extension = getFileExtension(meshfile);

      // Abstract meshfile ptr
      std::shared_ptr<MeshFile> mf;

      // Initialize meshfile ptr to concrete filetype
      if (file_extension == "stl" || file_extension == "STL")
      {
        RCLCPP_DEBUG(nh_->get_logger(),"Handling STL file..." );
        mf = std::make_shared<Stl>(meshfile,nh_);
      }
      else if (file_extension == "dae" || file_extension == "DAE")
      {
        RCLCPP_DEBUG(nh_->get_logger(), "Handling DAE file..." );
        mf = std::make_shared<Dae>(meshfile,nh_);
      }
      else
      {
        RCLCPP_ERROR(nh_->get_logger(),"UNKNOWN FILE EXTENSION! (.stl or .dae supported)" );
        mf = nullptr;
      }

      std::vector<double> points = mf->getPoints();

      Eigen::Matrix4d M = mf->getTransformationMatrix();
      Eigen::Vector3d t = urdf.getMeshTranslation();
      Eigen::Vector3d r = urdf.getMeshRotation();
      Eigen::Vector3d s = urdf.getMeshScale();

      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Mesh translation: " << t );
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Mesh rotation: " << r );
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Mesh scale: " << s );

      std::shared_ptr<ShapeFit> shapefitter;

      if (shape == Shape::Box)
      {
        // Fit the box
        shapefitter = std::make_shared<BoxFit>(points, t, r, s, M, nh_);
      }
      else if (shape == Shape::Capsule)
        // Fit the capsule
      {
        shapefitter = std::make_shared<CapsuleFit>(points, t, r, s, M, nh_);
      }

      // Save the results
      if (urdf.isXacro())
      {
        urdf.replaceMeshWithXacro(shapefitter);
      }
      else
      {
        urdf.replaceMesh(shapefitter);
      }
    }

    if (urdf.save(output_filename.c_str()))
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(),"Primitives generated successfully!" );
      output_filename = output_filename;
    }
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),"Unhandled Exception: " << e.what() );
  }
}

PrimitiveFitterNode::PrimitiveFitterNode()
    : Node(NODE_NAME) {
  RCLCPP_INFO(this->get_logger(),
              " ********** NimbRo Primitive Fitter started ********** ");

  declare_parameter("urdf_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("output_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("fit_shape", rclcpp::PARAMETER_STRING);
  declare_parameter("use_fitter", rclcpp::PARAMETER_BOOL);
  declare_parameter("use_inertia", rclcpp::PARAMETER_BOOL);
  declare_parameter("mesh_type", rclcpp::PARAMETER_STRING);
};

PrimitiveFitterNode::~PrimitiveFitterNode()
{};

void PrimitiveFitterNode::setSharedPointer(std::shared_ptr<PrimitiveFitterNode> a_shr_ptr) {
  shr_ptr_to_this_ = a_shr_ptr;
}

std::string PrimitiveFitterNode::getFileExtension(std::string meshfile)
{
  std::string file_extension("");

  // Get file extension
  std::regex file_regex("([^\\\\.]+)(\\.)([^\\\\.]+)");
  std::smatch r_match;

  if (std::regex_search(meshfile, r_match, file_regex))
  {
    file_extension = r_match[3];
    RCLCPP_DEBUG_STREAM(get_logger(), "Detected file extension ." << r_match[3] );
  }
  else
  {
    RCLCPP_INFO(get_logger(), "NO FILE EXTENSION FOUND!" );
  }

  return file_extension;
}

void PrimitiveFitterNode::update_inertia(std::string in_filename,
                                         std::string out_filename,
                                         bool visual)
{
  Urdf urdf(in_filename.c_str(), false, shr_ptr_to_this_);

  // Return if the urdf was not loaded
  if ( !urdf.loaded() )
  {
    RCLCPP_ERROR(get_logger(), "URDF could not be loaded." );
    return;
  }

  bool hasMoreMesh;

  if (visual)
  {
    hasMoreMesh = urdf.hasMoreVisualMesh();
  }
  else
  {
    hasMoreMesh = urdf.hasMoreMesh();
  }

  while (hasMoreMesh)
  {
    std::string meshfile;

    if (visual)
    {
      meshfile = urdf.getNextVisualMesh();
      hasMoreMesh = urdf.hasMoreVisualMesh();
    }
    else
    {
      meshfile = urdf.getNextMesh();
      hasMoreMesh = urdf.hasMoreMesh();
    }

    std::string file_extension = getFileExtension(meshfile);

    if (file_extension == "stl" || file_extension == "STL")
    {
      RCLCPP_DEBUG(get_logger(), "Handling STL file ..." );
      RCLCPP_DEBUG_STREAM(get_logger(), "path: " << meshfile );

      // get global meshfile path
      url::Url filepath(meshfile);
      std::string pkgName(filepath.getPackageName());
      // may throw ament_index_cpp::PackageNotFoundError exception
      std::string pkgPath =
          ament_index_cpp::get_package_share_directory(pkgName);

      std::string path(filepath.getRelativePath());
      std::string global_meshfile((pkgPath + path).c_str());
      RCLCPP_INFO_STREAM(get_logger(), "global path: " << global_meshfile );

      // open STL file
      MyMesh mesh;
      int mask = 0;

      if(vcg::tri::io::ImporterSTL<MyMesh>::Open(mesh, global_meshfile.c_str(), mask))
      {
        RCLCPP_INFO_STREAM(get_logger(), "Error reading file " << global_meshfile );
        return;
      }

      // read mass from file
      float mass = urdf.getMass(visual);

      // compute inertia and center of mass
      vcg::Matrix33f Inertia;
      vcg::tri::Inertia<MyMesh> I(mesh);
      I.InertiaTensor(Inertia);
      vcg::Point3f CenterOfMass = I.CenterOfMass();

      // scale inertia according to mass
      float volume = I.Mass();
      Inertia /= volume;
      Inertia *= mass;

      // set new values
      urdf.setCenterOfMass(CenterOfMass, visual);
      urdf.setInertia(Inertia, visual);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "UNKNOWN FILE EXTENSION! (only .stl supported)" );
      return;
    }
  }

  if (urdf.save(out_filename.c_str()))
  {
    RCLCPP_INFO(get_logger(), "Inertia updated successfully!" );
  }

  return;
}

void PrimitiveFitterNode::main(void) {
  RCLCPP_INFO(this->get_logger(), "step 1 " );
  std::string in_filename;
  try {
    in_filename =
        this->get_parameter("urdf_filename")
        .as_string();
  } catch (...) {
    std::cerr << "Unable to find parameter urdf_filename" << std::endl;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "step 2 ");
  std::string out_filename("");
  out_filename = this->get_parameter("output_filename").as_string();

  RCLCPP_INFO(this->get_logger(), "step 3 " );
  if(out_filename.empty())
  {
    size_t lastindex = in_filename.find_last_of(".");
    out_filename = in_filename.substr(0, lastindex) + "_optimized.urdf.xacro";
  }

  RCLCPP_INFO(this->get_logger(), "step 4 " );
  bool fitter = true;
  fitter = this->get_parameter("use_fitter").as_bool();

  bool inertia = false;
  inertia = this->get_parameter("use_inertia").as_bool();

  RCLCPP_INFO(this->get_logger(), "step 5 " );
  std::string capsule_filename("");

  if (inertia)
  {
    RCLCPP_INFO(this->get_logger(),
                "Inertia calculation started.");

    std::string mesh_type = "visual";
    bool visual = true;
    mesh_type = this->get_parameter("mesh_type").as_string();

    if (mesh_type == "visual")
    {
      visual = true;
      RCLCPP_INFO(this->get_logger(),"Using visual meshes.");
    }
    else if (mesh_type == "collision")
    {
      visual = false;
      RCLCPP_INFO(this->get_logger(),
                  "Using collision meshes.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "PLEASE SET ROSPARAM /nimbro_primitive_fitter/mesh_type"
                  " TO EITHER visual OR collsion -> Using 'visual'.");
      visual = true;
    }

    update_inertia(in_filename, out_filename, visual);
    in_filename = out_filename;
  }

  if (fitter)
  {
    std::string shape = "box";
    shape = this->get_parameter("fit_shape").as_string();
    VersatileFitter::Shape vf_shape;

    if (shape == "box" || shape == "Box")
    {
      RCLCPP_INFO(this->get_logger(),
                  "Will try to fit URDF to Box.");
      vf_shape = VersatileFitter::Shape::Box;
    }
    else if (shape == "capsule" || shape == "Capsule")
    {
      RCLCPP_INFO(this->get_logger(),
                  "Will try to fit URDF to Capsule.");
      vf_shape = VersatileFitter::Shape::Capsule;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "PLEASE SET ROSPARAM /nimbro_primitive_fitter/fit_shape TO EITHER box OR capsule");
      RCLCPP_INFO(this->get_logger(),"Will try to fit URDF to Box.");
      vf_shape = VersatileFitter::Shape::Box;
    }

    VersatileFitter vf(shr_ptr_to_this_);
    vf.fit(in_filename, out_filename, vf_shape, shape);
  }
};
