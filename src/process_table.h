#ifndef PROCESS_TABLE_H
#define PROCESS_TABLE_H

#include <Eigen/Core>
#include <boost/optional.hpp>

struct Line
{
  Eigen::Vector3d base;
  Eigen::Vector3d direction;
};

struct Edge
{
  Eigen::Vector3d start;
  Eigen::Vector3d end;
};

struct LineIntersection
{
  Eigen::Vector3d point;
  double residual;
  std::array<double, 2> distances_along_lines;
};

struct TableDescription
{
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d normal;
};

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> LoosePoints;

std::vector<Line> convertEdgeFramesToLines(const std::vector<Eigen::Affine3d>& edge_frames);
LineIntersection findLineIntersection(const Line& line1, const Line& line2);
std::vector<Edge> createEdgesFromIntersections(const std::vector<std::vector<LineIntersection> > &line_intersections,
                                               const std::vector<Line>& lines,
                                               double intersection_threshold = 1.);
boost::optional<LoosePoints> findLoosePoints(const std::vector<Edge>& edges);
Eigen::Vector3d projectIntoEdgesPlane(const Eigen::Vector3d& point, const std::vector<Edge>& edges,
                                      const std::vector<Line>& lines);
bool isCenterInside(const Eigen::Vector3d& center, const std::vector<Edge>& edges, const LoosePoints& loose_points);

#endif  // PROCESS_TABLE_H
