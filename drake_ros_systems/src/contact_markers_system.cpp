// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_properties.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_set>
#include <utility>

#include <iostream>

#include "drake_ros_systems/contact_markers_system.hpp"
#include "drake_ros_systems/utilities/name_conventions.hpp"
#include "drake_ros_systems/utilities/type_conversion.hpp"


namespace drake_ros_systems
{

namespace
{

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  geom_point.z = point.z();
  return geom_point;
}

/*
geometry_msgs::msg::Vector3 ToScale(const Eigen::Vector3d& vector)
{
  geometry_msgs::msg::Vector3 geom_vector;
  geom_vector.x = vector.x();
  geom_vector.y = vector.y();
  geom_vector.z = vector.z();
  return geom_vector;
}
*/

class ContactGeometryToMarkers : public drake::geometry::ShapeReifier
{
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactGeometryToMarkers)

  explicit ContactGeometryToMarkers(const ContactMarkersParams & params)
  : params_(params)
  {
  }

  ~ContactGeometryToMarkers() override = default;

  void Populate(
    const std::vector<drake::geometry::ContactSurface<double>> & surfaces,
    const std::vector<drake::geometry::PenetrationAsPointPair<double>> & points,
    visualization_msgs::msg::MarkerArray * marker_array)
  {
    DRAKE_ASSERT(nullptr != marker_array);
    marker_array_ = marker_array;

    const int num_surfaces = static_cast<int>(surfaces.size());
    const int num_pairs = static_cast<int>(points.size());

    marker_array_->markers.reserve(num_surfaces + num_pairs);

    // Translate Drake Contact Surface into ROS List of Triangles.
    int name_index = 0;
    for (const drake::geometry::ContactSurface<double>& surface: surfaces) {
      visualization_msgs::msg::Marker face_msg;
      face_msg.header.frame_id = params_.origin_frame_name;
      face_msg.ns = std::to_string(name_index) + "_faces";
      face_msg.id = marker_array_->markers.size();
      face_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      face_msg.action = visualization_msgs::msg::Marker::ADD;

      face_msg.lifetime = rclcpp::Duration::from_nanoseconds(0);
      face_msg.frame_locked = true;

      drake::geometry::Rgba color = params_.default_color;
      face_msg.color.r = color.r();
      face_msg.color.g = color.g();
      face_msg.color.b = color.b();
      face_msg.color.a = color.a();

      face_msg.scale.x = 1.0;
      face_msg.scale.y = 1.0;
      face_msg.scale.z = 1.0;

      const drake::geometry::SurfaceMesh<double>& mesh_W = surface.mesh_W();
      face_msg.points.clear();
      face_msg.points.resize(mesh_W.num_faces() * 3);

      // Make lines for the edges
      visualization_msgs::msg::Marker edge_msg;
      edge_msg.header.frame_id = params_.origin_frame_name;
      edge_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
      edge_msg.action = visualization_msgs::msg::Marker::ADD;
      edge_msg.lifetime = rclcpp::Duration::from_nanoseconds(0);
      edge_msg.frame_locked = true;
      edge_msg.ns = std::to_string(name_index++) + "_edges";
      edge_msg.id = 1;
      // Set the size of the individual markers (depends on scale)
      //edge_msg.scale = ToScale(edge_scale * scale);
      edge_msg.scale.x = 0.02;
      edge_msg.scale.y = 0.02;
      edge_msg.scale.z = 1.0;

      // Set the overall color
      edge_msg.color.r = 0.0;
      edge_msg.color.g = 0.0;
      edge_msg.color.b = 0.0;
      edge_msg.color.a = 1.0;

      // Generate the surface markers for each mesh.
      size_t index = 0;
      for (drake::geometry::SurfaceFaceIndex j(0); j < mesh_W.num_faces(); ++j) {
        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const drake::geometry::SurfaceVertex<double>& vA = mesh_W.vertex(face.vertex(0));
        const drake::geometry::SurfaceVertex<double>& vB = mesh_W.vertex(face.vertex(1));
        const drake::geometry::SurfaceVertex<double>& vC = mesh_W.vertex(face.vertex(2));

        face_msg.points.at(index + 0) = ToPoint(vA.r_MV());
        face_msg.points.at(index + 1) = ToPoint(vB.r_MV());
        face_msg.points.at(index + 2) = ToPoint(vC.r_MV());
        index += 3;

        /*
        write_double3(vA.r_MV(), tri_msg.p_WA);
        write_double3(vB.r_MV(), tri_msg.p_WB);
        write_double3(vC.r_MV(), tri_msg.p_WC);
        write_double3((vA.r_MV() + vB.r_MV() + vC.r_MV()) / 3.0, quad_msg.p_WQ);

        tri_msg.pressure_A = field.EvaluateAtVertex(face.vertex(0));
        tri_msg.pressure_B = field.EvaluateAtVertex(face.vertex(1));
        tri_msg.pressure_C = field.EvaluateAtVertex(face.vertex(2));

        // Face contact *traction* and *slip velocity* data.
        write_double3(Vector3<double>(0, 0.2, 0), quad_msg.vt_BqAq_W);
        write_double3(Vector3<double>(0, -0.2, 0), quad_msg.traction_Aq_W);
        */

        // 0->1
        edge_msg.points.push_back(ToPoint(vA.r_MV()));
        edge_msg.points.push_back(ToPoint(vB.r_MV()));
        // 1->2
        edge_msg.points.push_back(ToPoint(vB.r_MV()));
        edge_msg.points.push_back(ToPoint(vC.r_MV()));
        // 2->0
        edge_msg.points.push_back(ToPoint(vC.r_MV()));
        edge_msg.points.push_back(ToPoint(vA.r_MV()));
      }

      marker_array_->markers.push_back(face_msg);

      marker_array_->markers.push_back(edge_msg);
    }
  }

private:

  const ContactMarkersParams & params_;
  visualization_msgs::msg::MarkerArray * marker_array_{nullptr};
  drake::math::RigidTransform<double> X_FG_{};
};

}  // namespace

class ContactMarkersSystem::ContactMarkersSystemPrivate
{
public:
  explicit ContactMarkersSystemPrivate(ContactMarkersParams _params)
  : params(std::move(_params))
  {
  }

  const ContactMarkersParams params;
  drake::systems::InputPortIndex graph_query_port_index;
  drake::systems::OutputPortIndex contact_markers_port_index;
  std::unordered_set<const drake::multibody::MultibodyPlant<double> *> plants;
  mutable drake::geometry::GeometryVersion version;
};

ContactMarkersSystem::ContactMarkersSystem(ContactMarkersParams params)
: impl_(new ContactMarkersSystemPrivate(std::move(params)))
{
  impl_->graph_query_port_index =
    this->DeclareAbstractInputPort(
    "graph_query", drake::Value<drake::geometry::QueryObject<double>>{}).get_index();

  impl_->contact_markers_port_index = this->DeclareAbstractOutputPort(
    "contact_markers", &ContactMarkersSystem::CalcContactMarkers).get_index();
}

ContactMarkersSystem::~ContactMarkersSystem()
{
}

void
ContactMarkersSystem::RegisterMultibodyPlant(
  const drake::multibody::MultibodyPlant<double> * plant)
{
  DRAKE_THROW_UNLESS(plant != nullptr);
  impl_->plants.insert(plant);
}

namespace
{

visualization_msgs::msg::Marker MakeDeleteAllMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

void
ContactMarkersSystem::CalcContactMarkers(
  const drake::systems::Context<double> & context,
  visualization_msgs::msg::MarkerArray * output_value) const
{
  output_value->markers.clear();
  output_value->markers.insert(
    output_value->markers.begin(),
    MakeDeleteAllMarker());

  const auto& query_object =
    get_input_port(impl_->graph_query_port_index)
    .Eval<drake::geometry::QueryObject<double>>(context);

  std::vector<drake::geometry::ContactSurface<double>> surfaces;
  std::vector<drake::geometry::PenetrationAsPointPair<double>> points;

  if (impl_->params.use_strict_hydro_) {
    surfaces = query_object.ComputeContactSurfaces();
  } else {
    query_object.ComputeContactSurfacesWithFallback(&surfaces, &points);
  }

  ContactGeometryToMarkers(impl_->params).Populate(
    surfaces, points, output_value);
}

const ContactMarkersParams &
ContactMarkersSystem::params() const
{
  return impl_->params;
}

const drake::systems::InputPort<double> &
ContactMarkersSystem::get_graph_query_port() const
{
  return get_input_port(impl_->graph_query_port_index);
}

const drake::systems::OutputPort<double> &
ContactMarkersSystem::get_markers_output_port() const
{
  return get_output_port(impl_->contact_markers_port_index);
}

}  // namespace drake_ros_systems
