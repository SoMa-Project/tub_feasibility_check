#ifndef PROCESS_TABLE_H
#define PROCESS_TABLE_H

#include <Eigen/Core>
#include <boost/optional.hpp>
#include "tub_feasibility_check/CheckKinematicsTabletopRequest.h"

/* The vector representation of the line: p(t) = base + t * direction. */
struct Line
{
  /* A point on the line. */
  Eigen::Vector3d base;

  /* A vector parallel to the direction of the line. */
  Eigen::Vector3d direction;
};

/* The description of a table edge with start and end points. */
struct Edge
{
  Eigen::Vector3d start;
  Eigen::Vector3d end;
};

/* The solution of the closest points on two lines equation. */
struct LineIntersection
{
  /* (p1 + p2) /2, where p1 and p2 are the closest points between line1 and line2. */
  Eigen::Vector3d point;

  /* The residual, showing how far the lines are from perfectly intersecting. */
  double residual;

  /* t1 and t2, such that base1 + t1 * direction1 and base2 + t2 * direction2 are the closest points
   * between two lines on line1 and line2 respectively.
   */
  std::array<double, 2> distances_along_lines;
};

/* The decription of the table through a vector of consecutive points and a normal. */
struct TableDescription
{
  /* Consecutive points of the table surface polygon: every two successive points form an edge, and additionally
   * the last point forms and edge with the first one.
   */
  std::vector<Eigen::Vector3d> points;

  /* The normal of the table surface plane. This can be calculated from points, but is provided for convenience. */
  Eigen::Vector3d normal;
};

struct LoosePoint
{
  Eigen::Vector3d point;
  Eigen::Vector3d outward_direction;
};

/* A pair of loose points - the points on the table polygon that are part of only one edge.
 * Such points exists when not all edge grasp frames were visible in the scene.
 */
typedef std::pair<LoosePoint, LoosePoint> LoosePoints;

/* Create a parameteric description of the line for every edge frame X axis. */
std::vector<Line> convertEdgeFramesToLines(const std::vector<Eigen::Affine3d>& edge_frames);

/* Find the two closest points on the lines and compute a point that lies in the middle between them. */
LineIntersection findLineIntersection(const Line& line1, const Line& line2);

/* Filter out intersections that are not real table corners. This assumes that the table is convex.
 *
 * @param line_intersections For every line there is a vector that contains all intersections of line with other lines.
 * line_intersections elements should be in the same order as lines in the lines argument.
 * @param lines Lines, in the same order as corresponding line_intersections.
 * @param intersection_threshold If the intersection is further that that from the base point, filter it out.
 * It is needed to filter out intersection of almost parallel unclosed edges that intersect in the far distance.
 */
std::vector<Edge> createEdgesFromIntersections(const std::vector<std::vector<LineIntersection> >& line_intersections,
                                               const std::vector<Line>& lines, double intersection_threshold = 1.);

/* Find loose points based on the edges, that is points that are only part of one edge.
 *
 * @return A pair of points that each belong to only one edge, if they exist, and boost::none otherwise.
 */
boost::optional<LoosePoints> findLoosePoints(const std::vector<Edge>& edges);

/* Project the point into the plane defined by two lines.
 *
 * @param point The point to be projected.
 * @return The projection of point into the plane defined by line1 and line2.
 */
Eigen::Vector3d projectIntoLinesPlane(const Eigen::Vector3d& point, const Line& line1, const Line& line2);

/* Find the common normal between line1 and line2 that goes in the same direction as almost_normal. */
Eigen::Vector3d correctNormal(const Eigen::Vector3d& almost_normal, const Line& line1, const Line& line2);

/* Create a table description using edge frames and table surface position from the request.
 *
 * @return A table description, if it is possible to build a table from the edge frames supplied by request, and
 * boost::none otherwise. It is not possible to build a table when there are less than two edges. If there
 * are loose ends in the supplied edges, they will be connected with an extension.
 *
 * @param extend_loose_ends_distance Extend loose ends so far in the outwards direction.
 */
boost::optional<TableDescription> createTableFromFrames(
    const tub_feasibility_check::CheckKinematicsTabletopRequest& request, double extend_loose_ends_distance = 0.5);

#endif  // PROCESS_TABLE_H
