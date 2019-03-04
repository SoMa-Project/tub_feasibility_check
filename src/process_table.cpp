#include <Eigen/Dense>
#include <list>
#include <eigen_conversions/eigen_msg.h>
#include "process_table.h"

typedef Eigen::Matrix<double, 3, 2> IntersectionMatrix;

LineIntersection findLineIntersection(const Line& line1, const Line& line2)
{
  IntersectionMatrix A;
  for (unsigned i = 0; i < 3; i++)
  {
    A(i, 0) = line1.direction(i);
    A(i, 1) = -line2.direction(i);
  }

  Eigen::Vector3d b = line2.base - line1.base;
  Eigen::Vector2d ts = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  LineIntersection intersection;
  intersection.distances_along_lines[0] = ts(0);
  intersection.distances_along_lines[1] = ts(1);
  intersection.point = (line1.base + ts(0) * line1.direction + line2.base + ts(1) * line2.direction) / 2;
  intersection.residual = (A * ts - b).norm() / b.norm();

  return intersection;
}

std::vector<Edge> createEdgesFromIntersections(const std::vector<std::vector<LineIntersection>>& line_intersections,
                                               const std::vector<Line>& lines, double intersection_threshold)
{
  assert(line_intersections.size() == lines.size());

  std::vector<Edge> edges;

  for (unsigned i = 0; i < line_intersections.size(); ++i)
  {
    double max_negative = -std::numeric_limits<double>::infinity();
    double min_positive = std::numeric_limits<double>::infinity();

    boost::optional<Eigen::Vector3d> negative_intersection;
    boost::optional<Eigen::Vector3d> positive_intersection;

    for (auto& intersection : line_intersections[i])
    {
      if (std::abs(intersection.distances_along_lines[0]) > intersection_threshold)
        continue;

      if (intersection.distances_along_lines[0] < 0 &&
          (!negative_intersection.is_initialized() || intersection.distances_along_lines[0] > max_negative))
      {
        max_negative = intersection.distances_along_lines[0];
        negative_intersection = intersection.point;
      }

      if (intersection.distances_along_lines[0] > 0 &&
          (!positive_intersection.is_initialized() || intersection.distances_along_lines[0] < min_positive))
      {
        min_positive = intersection.distances_along_lines[0];
        positive_intersection = intersection.point;
      }
    }

    Edge e;
    e.start = negative_intersection.is_initialized() ? *negative_intersection :
                                                       lines[i].base - min_positive * lines[i].direction;
    e.end = positive_intersection.is_initialized() ? *positive_intersection :
                                                     lines[i].base - max_negative * lines[i].direction;

    edges.push_back(std::move(e));
  }

  return edges;
}

boost::optional<LoosePoints> findLoosePoints(const std::vector<Edge>& edges)
{
  std::list<Eigen::Vector3d> points;

  for (auto& edge : edges)
    for (auto point : { edge.start, edge.end })
    {
      bool duplicate_removed = false;

      for (auto it = points.begin(); it != points.end(); ++it)
      {
        if (it->isApprox(point))
        {
          it = points.erase(it);
          duplicate_removed = true;
          break;
        }
      }

      if (!duplicate_removed)
        points.push_back(point);
    }

  if (points.empty())
    return boost::none;

  assert(points.size() == 2);

  auto pair = std::make_pair(points.front(), points.back());
  return pair;
}

Eigen::Vector3d projectIntoLinesPlane(const Eigen::Vector3d& point, const Line& line1, const Line& line2)
{
  IntersectionMatrix A;
  for (unsigned i = 0; i < 3; ++i)
  {
    A(i, 0) = line1.direction(i);
    A(i, 1) = line2.direction(i);
  }

  Eigen::Vector3d b = point - line1.base;
  Eigen::Vector2d ts = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  return line1.base + ts(0) * line1.direction + ts(1) * line2.direction;
}

bool isCenterInside(const Eigen::Vector3d& center, const std::vector<Edge>& edges, const LoosePoints& loose_points)
{
  Line loose_points_connection;
  loose_points_connection.base = loose_points.first;
  loose_points_connection.direction = (loose_points.second - loose_points.first).normalized();

  Line forward;

  for (auto& edge : edges)
  {
    if (edge.start.isApprox(loose_points.first))
    {
      forward.base = edge.end;
      forward.direction = (center - forward.base).normalized();
      break;
    }

    if (edge.end.isApprox(loose_points.second))
    {
      forward.base = edge.start;
      forward.direction = (center - forward.base).normalized();
      break;
    }
  }

  auto intersection = findLineIntersection(forward, loose_points_connection);
  return intersection.distances_along_lines[0] > (center - forward.base).norm();
}

std::vector<Line> convertEdgeFramesToLines(const std::vector<Eigen::Affine3d>& edge_frames)
{
  std::vector<Line> lines(edge_frames.size());

  for (std::size_t i = 0; i < edge_frames.size(); ++i)
  {
    lines[i].base = edge_frames[i] * Eigen::Vector3d::Zero();
    lines[i].direction =
        (edge_frames[i] * Eigen::Vector3d::UnitX() - edge_frames[i] * Eigen::Vector3d::Zero()).normalized();
  }

  return lines;
}

Eigen::Vector3d correctNormal(const Eigen::Vector3d& almost_normal, const Line& line1, const Line& line2)
{
  Eigen::Matrix<double, 3, 3> vectors_matrix;
  for (std::size_t i = 0; i < 3; ++i)
  {
    vectors_matrix(0, i) = line1.direction(i);
    vectors_matrix(1, i) = line2.direction(i);
    vectors_matrix(2, i) = 1;
  }

  auto b = Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d common_normal =
      vectors_matrix.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b).normalized();

  // pick the direction so it is the same with almost_normal
  return common_normal.dot(almost_normal) > 0 ? common_normal : Eigen::Vector3d(-common_normal);
}

boost::optional<TableDescription>
createTableFromFrames(const tub_feasibility_check::CheckKinematicsTabletopRequest& request)
{
  std::vector<Eigen::Affine3d> edge_frames(request.edge_frames.size());
  for (std::size_t i = 0; i < request.edge_frames.size(); ++i)
    tf::poseMsgToEigen(request.edge_frames[i], edge_frames[i]);

  auto lines = convertEdgeFramesToLines(edge_frames);
  std::vector<std::vector<LineIntersection>> line_intersections(lines.size());
  for (std::size_t i = 0; i < lines.size(); ++i)
    for (std::size_t j = 0; j < lines.size(); ++j)
      if (i != j)
        line_intersections[i].push_back(findLineIntersection(lines[i], lines[j]));

  auto edges = createEdgesFromIntersections(line_intersections, lines);
  auto loose_points = findLoosePoints(edges);

  if (edges.size() < 2)
    return boost::none;

  auto convertEdgesToTableDescription = [&edge_frames, &edges, &lines]() {
    TableDescription table_description;
    Eigen::Vector3d current = edges.front().start;
    std::list<Edge> edge_list(edges.begin(), edges.end());

    while(!edge_list.empty())
    {
      table_description.points.push_back(current);
      for (auto it = edge_list.begin(); it != edge_list.end(); ++it)
      {
        auto edge = *it;

        if (edge.start.isApprox(current))
        {
          current = edge.end;
          edge_list.erase(it);
          break;
        }
        else if (edge.end.isApprox(current))
        {
          current = edge.start;
          edge_list.erase(it);
          break;
        }
      }
    }

    table_description.normal = correctNormal(edge_frames.front() * Eigen::Vector3d::UnitZ(), lines[0], lines[1]);
    return table_description;
  };

  if (!loose_points)
    return convertEdgesToTableDescription();

  Eigen::Vector3d table_surface_pose_position;
  tf::pointMsgToEigen(request.table_surface_pose.position, table_surface_pose_position);
  Eigen::Vector3d table_center_in_edges_plane = projectIntoLinesPlane(table_surface_pose_position, lines[0], lines[1]);
  bool center_inside = isCenterInside(table_center_in_edges_plane, edges, *loose_points);

  if (center_inside)
    edges.emplace_back(Edge{ loose_points->first, loose_points->second });
  else
  {
    edges.emplace_back(Edge{ loose_points->first, table_center_in_edges_plane });
    edges.emplace_back(Edge{ table_center_in_edges_plane, loose_points->second });
  }

  return convertEdgesToTableDescription();
}
