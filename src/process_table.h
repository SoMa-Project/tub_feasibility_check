#ifndef PROCESS_TABLE_H
#define PROCESS_TABLE_H

#include <Eigen/Core>
#include <boost/optional.hpp>
#include "tub_feasibility_check/CheckKinematicsTabletopRequest.h"

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
std::vector<Edge> createEdgesFromIntersections(const std::vector<std::vector<LineIntersection> >& line_intersections,
                                               const std::vector<Line>& lines, double intersection_threshold = 1.);
boost::optional<LoosePoints> findLoosePoints(const std::vector<Edge>& edges);
Eigen::Vector3d projectIntoEdgesPlane(const Eigen::Vector3d& point, const std::vector<Edge>& edges,
                                      const std::vector<Line>& lines);
bool isCenterInside(const Eigen::Vector3d& center, const std::vector<Edge>& edges, const LoosePoints& loose_points);
Eigen::Vector3d correctNormal(const Eigen::Vector3d& almost_normal, const Line& line1, const Line& line2);

boost::optional<TableDescription>
createTableFromFrames(const tub_feasibility_check::CheckKinematicsTabletopRequest& request);

#endif  // PROCESS_TABLE_H
