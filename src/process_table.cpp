#include <Eigen/Dense>
#include <list>
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

Eigen::Vector3d projectIntoEdgesPlane(const Eigen::Vector3d& point, const std::vector<Edge>& edges,
                                      const std::vector<Line>& lines)
{
  assert(edges.size() > 1);

  IntersectionMatrix A;
  for (unsigned i = 0; i < 3; ++i)
  {
    A(i, 0) = lines[0].direction(i);
    A(i, 1) = lines[1].direction(i);
  }

  Eigen::Vector3d b = point - lines[0].base;
  Eigen::Vector2d ts = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  return lines[0].base + ts(0) * lines[0].direction + ts(1) * lines[1].direction;
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
