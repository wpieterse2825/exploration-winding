#include "exploration/winding/library_winding.hpp"

idWinding2D::idWinding2D(void) {
  numPoints = 0;
}

idWinding2D& idWinding2D::operator=(idWinding2D const& winding) {
  int i;

  for(i = 0; i < winding.numPoints; i++) {
    p[i] = winding.p[i];
  }
  numPoints = winding.numPoints;
  return *this;
}

idVec2 const& idWinding2D::operator[](int const index) const {
  return p[index];
}

idVec2& idWinding2D::operator[](int const index) {
  return p[index];
}

void idWinding2D::Clear(void) {
  numPoints = 0;
}

void idWinding2D::AddPoint(idVec2 const& point) {
  p[numPoints++] = point;
}

int idWinding2D::GetNumPoints(void) const {
  return numPoints;
}

idVec3 idWinding2D::Plane2DFromPoints(
  idVec2 const& start, idVec2 const& end, bool const normalize
) {
  idVec3 plane;
  plane.x = start.y - end.y;
  plane.y = end.x - start.x;
  if(normalize) {
    plane.ToVec2().Normalize();
  }
  plane.z = -(start.x * plane.x + start.y * plane.y);
  return plane;
}

idVec3 idWinding2D::Plane2DFromVecs(
  idVec2 const& start, idVec2 const& dir, bool const normalize
) {
  idVec3 plane;
  plane.x = -dir.y;
  plane.y = dir.x;
  if(normalize) {
    plane.ToVec2().Normalize();
  }
  plane.z = -(start.x * plane.x + start.y * plane.y);
  return plane;
}

bool idWinding2D::Plane2DIntersection(
  idVec3 const& plane1, idVec3 const& plane2, idVec2& point
) {
  float n00, n01, n11, det, invDet, f0, f1;

  n00 = plane1.x * plane1.x + plane1.y * plane1.y;
  n01 = plane1.x * plane2.x + plane1.y * plane2.y;
  n11 = plane2.x * plane2.x + plane2.y * plane2.y;
  det = n00 * n11 - n01 * n01;

  if(fabs(det) < 1e-6f) {
    return false;
  }

  invDet  = 1.0f / det;
  f0      = (n01 * plane2.z - n11 * plane1.z) * invDet;
  f1      = (n01 * plane1.z - n00 * plane2.z) * invDet;
  point.x = f0 * plane1.x + f1 * plane2.x;
  point.y = f0 * plane1.y + f1 * plane2.y;
  return true;
}

/*
============
GetAxialBevel
============
*/
bool GetAxialBevel(
  idVec3 const& plane1, idVec3 const& plane2, idVec2 const& point, idVec3& bevel
) {
  if(FLOATSIGNBITSET(plane1.x) ^ FLOATSIGNBITSET(plane2.x)) {
    if(fabs(plane1.x) > 0.1f && fabs(plane2.x) > 0.1f) {
      bevel.x = 0.0f;
      if(FLOATSIGNBITSET(plane1.y)) {
        bevel.y = -1.0f;
      } else {
        bevel.y = 1.0f;
      }
      bevel.z = -(point.x * bevel.x + point.y * bevel.y);
      return true;
    }
  }
  if(FLOATSIGNBITSET(plane1.y) ^ FLOATSIGNBITSET(plane2.y)) {
    if(fabs(plane1.y) > 0.1f && fabs(plane2.y) > 0.1f) {
      bevel.y = 0.0f;
      if(FLOATSIGNBITSET(plane1.x)) {
        bevel.x = -1.0f;
      } else {
        bevel.x = 1.0f;
      }
      bevel.z = -(point.x * bevel.x + point.y * bevel.y);
      return true;
    }
  }
  return false;
}

/*
============
idWinding2D::ExpandForAxialBox
============
*/
void idWinding2D::ExpandForAxialBox(idVec2 const bounds[2]) {
  int    i, j, numPlanes;
  idVec2 v;
  idVec3 planes[MAX_POINTS_ON_WINDING_2D], plane, bevel;

  // get planes for the edges and add bevels
  for(numPlanes = i = 0; i < numPoints; i++) {
    j = (i + 1) % numPoints;
    if((p[j] - p[i]).LengthSqr() < 0.01f) {
      continue;
    }
    plane = Plane2DFromPoints(p[i], p[j], true);
    if(i) {
      if(GetAxialBevel(planes[numPlanes - 1], plane, p[i], bevel)) {
        planes[numPlanes++] = bevel;
      }
    }
    assert(numPlanes < MAX_POINTS_ON_WINDING_2D);
    planes[numPlanes++] = plane;
  }
  if(GetAxialBevel(planes[numPlanes - 1], planes[0], p[0], bevel)) {
    planes[numPlanes++] = bevel;
  }

  // expand the planes
  for(i = 0; i < numPlanes; i++) {
    v.x          = bounds[FLOATSIGNBITSET(planes[i].x)].x;
    v.y          = bounds[FLOATSIGNBITSET(planes[i].y)].y;
    planes[i].z += v.x * planes[i].x + v.y * planes[i].y;
  }

  // get intersection points of the planes
  for(numPoints = i = 0; i < numPlanes; i++) {
    if(Plane2DIntersection(
         planes[(i + numPlanes - 1) % numPlanes], planes[i], p[numPoints]
       )) {
      numPoints++;
    }
  }
}

/*
============
idWinding2D::Expand
============
*/
void idWinding2D::Expand(float const d) {
  int    i;
  idVec2 edgeNormals[MAX_POINTS_ON_WINDING_2D];

  for(i = 0; i < numPoints; i++) {
    idVec2& start    = p[i];
    idVec2& end      = p[(i + 1) % numPoints];
    edgeNormals[i].x = start.y - end.y;
    edgeNormals[i].y = end.x - start.x;
    edgeNormals[i].Normalize();
    edgeNormals[i] *= d;
  }

  for(i = 0; i < numPoints; i++) {
    p[i] += edgeNormals[i] + edgeNormals[(i + numPoints - 1) % numPoints];
  }
}

/*
=============
idWinding2D::Split
=============
*/
int idWinding2D::Split(
  idVec3 const& plane,
  float const   epsilon,
  idWinding2D** front,
  idWinding2D** back
) const {
  float         dists[MAX_POINTS_ON_WINDING_2D];
  int8_t        sides[MAX_POINTS_ON_WINDING_2D];
  int           counts[3];
  float         dot;
  int           i, j;
  idVec2 const *p1, *p2;
  idVec2        mid;
  idWinding2D*  f;
  idWinding2D*  b;
  // int           maxpts;

  counts[0] = counts[1] = counts[2] = 0;

  // determine sides for each point
  for(i = 0; i < numPoints; i++) {
    dists[i] = dot = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(dot > epsilon) {
      sides[i] = SIDE_FRONT;
    } else if(dot < -epsilon) {
      sides[i] = SIDE_BACK;
    } else {
      sides[i] = SIDE_ON;
    }
    counts[sides[i]]++;
  }
  sides[i] = sides[0];
  dists[i] = dists[0];

  *front = *back = NULL;

  // if nothing at the front of the clipping plane
  if(!counts[SIDE_FRONT]) {
    *back = Copy();
    return SIDE_BACK;
  }
  // if nothing at the back of the clipping plane
  if(!counts[SIDE_BACK]) {
    *front = Copy();
    return SIDE_FRONT;
  }

  // maxpts = numPoints + 4; // cant use counts[0]+2 because of fp grouping
  // errors

  *front = f = new idWinding2D;
  *back = b = new idWinding2D;

  for(i = 0; i < numPoints; i++) {
    p1 = &p[i];

    if(sides[i] == SIDE_ON) {
      f->p[f->numPoints] = *p1;
      f->numPoints++;
      b->p[b->numPoints] = *p1;
      b->numPoints++;
      continue;
    }

    if(sides[i] == SIDE_FRONT) {
      f->p[f->numPoints] = *p1;
      f->numPoints++;
    }

    if(sides[i] == SIDE_BACK) {
      b->p[b->numPoints] = *p1;
      b->numPoints++;
    }

    if(sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i]) {
      continue;
    }

    // generate a split point
    p2 = &p[(i + 1) % numPoints];

    // always calculate the split going from the same side
    // or minor epsilon issues can happen
    if(sides[i] == SIDE_FRONT) {
      dot = dists[i] / (dists[i] - dists[i + 1]);
      for(j = 0; j < 2; j++) {
        // avoid round off error when possible
        if(plane[j] == 1.0f) {
          mid[j] = plane.z;
        } else if(plane[j] == -1.0f) {
          mid[j] = -plane.z;
        } else {
          mid[j] = (*p1)[j] + dot * ((*p2)[j] - (*p1)[j]);
        }
      }
    } else {
      dot = dists[i + 1] / (dists[i + 1] - dists[i]);
      for(j = 0; j < 2; j++) {
        // avoid round off error when possible
        if(plane[j] == 1.0f) {
          mid[j] = plane.z;
        } else if(plane[j] == -1.0f) {
          mid[j] = -plane.z;
        } else {
          mid[j] = (*p2)[j] + dot * ((*p1)[j] - (*p2)[j]);
        }
      }
    }

    f->p[f->numPoints] = mid;
    f->numPoints++;
    b->p[b->numPoints] = mid;
    b->numPoints++;
  }

  return SIDE_CROSS;
}

/*
============
idWinding2D::ClipInPlace
============
*/
bool idWinding2D::ClipInPlace(
  idVec3 const& plane, float const epsilon, bool const keepOn
) {
  int     i, j, maxpts, newNumPoints;
  int     sides[MAX_POINTS_ON_WINDING_2D + 1], counts[3];
  float   dot, dists[MAX_POINTS_ON_WINDING_2D + 1];
  idVec2 *p1, *p2, mid, newPoints[MAX_POINTS_ON_WINDING_2D + 4];

  counts[SIDE_FRONT] = counts[SIDE_BACK] = counts[SIDE_ON] = 0;

  for(i = 0; i < numPoints; i++) {
    dists[i] = dot = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(dot > epsilon) {
      sides[i] = SIDE_FRONT;
    } else if(dot < -epsilon) {
      sides[i] = SIDE_BACK;
    } else {
      sides[i] = SIDE_ON;
    }
    counts[sides[i]]++;
  }
  sides[i] = sides[0];
  dists[i] = dists[0];

  // if the winding is on the plane and we should keep it
  if(keepOn && !counts[SIDE_FRONT] && !counts[SIDE_BACK]) {
    return true;
  }
  if(!counts[SIDE_FRONT]) {
    numPoints = 0;
    return false;
  }
  if(!counts[SIDE_BACK]) {
    return true;
  }

  maxpts = numPoints + 4; // cant use counts[0]+2 because of fp grouping errors
  newNumPoints = 0;

  for(i = 0; i < numPoints; i++) {
    p1 = &p[i];

    if(newNumPoints + 1 > maxpts) {
      return true; // can't split -- fall back to original
    }

    if(sides[i] == SIDE_ON) {
      newPoints[newNumPoints] = *p1;
      newNumPoints++;
      continue;
    }

    if(sides[i] == SIDE_FRONT) {
      newPoints[newNumPoints] = *p1;
      newNumPoints++;
    }

    if(sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i]) {
      continue;
    }

    if(newNumPoints + 1 > maxpts) {
      return true; // can't split -- fall back to original
    }

    // generate a split point
    p2 = &p[(i + 1) % numPoints];

    dot = dists[i] / (dists[i] - dists[i + 1]);
    for(j = 0; j < 2; j++) {
      // avoid round off error when possible
      if(plane[j] == 1.0f) {
        mid[j] = plane.z;
      } else if(plane[j] == -1.0f) {
        mid[j] = -plane.z;
      } else {
        mid[j] = (*p1)[j] + dot * ((*p2)[j] - (*p1)[j]);
      }
    }

    newPoints[newNumPoints] = mid;
    newNumPoints++;
  }

  if(newNumPoints >= MAX_POINTS_ON_WINDING_2D) {
    return true;
  }

  numPoints = newNumPoints;
  memcpy(p, newPoints, newNumPoints * sizeof(idVec2));

  return true;
}

/*
=============
idWinding2D::Copy
=============
*/
idWinding2D* idWinding2D::Copy(void) const {
  idWinding2D* w;

  w            = new idWinding2D;
  w->numPoints = numPoints;
  memcpy(w->p, p, numPoints * sizeof(p[0]));
  return w;
}

/*
=============
idWinding2D::Reverse
=============
*/
idWinding2D* idWinding2D::Reverse(void) const {
  idWinding2D* w;
  int          i;

  w            = new idWinding2D;
  w->numPoints = numPoints;
  for(i = 0; i < numPoints; i++) {
    w->p[numPoints - i - 1] = p[i];
  }
  return w;
}

/*
============
idWinding2D::GetArea
============
*/
float idWinding2D::GetArea(void) const {
  int    i;
  idVec2 d1, d2;
  float  total;

  total = 0.0f;
  for(i = 2; i < numPoints; i++) {
    d1     = p[i - 1] - p[0];
    d2     = p[i] - p[0];
    total += d1.x * d2.y - d1.y * d2.x;
  }
  return total * 0.5f;
}

/*
============
idWinding2D::GetCenter
============
*/
idVec2 idWinding2D::GetCenter(void) const {
  int    i;
  idVec2 center;

  center.Zero();
  for(i = 0; i < numPoints; i++) {
    center += p[i];
  }
  center *= (1.0f / numPoints);
  return center;
}

/*
============
idWinding2D::GetRadius
============
*/
float idWinding2D::GetRadius(idVec2 const& center) const {
  int    i;
  float  radius, r;
  idVec2 dir;

  radius = 0.0f;
  for(i = 0; i < numPoints; i++) {
    dir = p[i] - center;
    r   = dir * dir;
    if(r > radius) {
      radius = r;
    }
  }
  return sqrtf(radius);
}

/*
============
idWinding2D::GetBounds
============
*/
void idWinding2D::GetBounds(idVec2 bounds[2]) const {
  int i;

  if(!numPoints) {
    bounds[0].x = bounds[0].y = INFINITY;
    bounds[1].x = bounds[1].y = -INFINITY;
    return;
  }
  bounds[0] = bounds[1] = p[0];
  for(i = 1; i < numPoints; i++) {
    if(p[i].x < bounds[0].x) {
      bounds[0].x = p[i].x;
    } else if(p[i].x > bounds[1].x) {
      bounds[1].x = p[i].x;
    }
    if(p[i].y < bounds[0].y) {
      bounds[0].y = p[i].y;
    } else if(p[i].y > bounds[1].y) {
      bounds[1].y = p[i].y;
    }
  }
}

/*
=============
idWinding2D::IsTiny
=============
*/
#define EDGE_LENGTH 0.2f

bool idWinding2D::IsTiny(void) const {
  int    i;
  float  len;
  idVec2 delta;
  int    edges;

  edges = 0;
  for(i = 0; i < numPoints; i++) {
    delta = p[(i + 1) % numPoints] - p[i];
    len   = delta.Length();
    if(len > EDGE_LENGTH) {
      if(++edges == 3) {
        return false;
      }
    }
  }
  return true;
}

/*
=============
idWinding2D::IsHuge
=============
*/
bool idWinding2D::IsHuge(void) const {
  int i, j;

  for(i = 0; i < numPoints; i++) {
    for(j = 0; j < 2; j++) {
      if(p[i][j] <= MIN_WORLD_COORD || p[i][j] >= MAX_WORLD_COORD) {
        return true;
      }
    }
  }
  return false;
}

/*
=============
idWinding2D::PlaneDistance
=============
*/
float idWinding2D::PlaneDistance(idVec3 const& plane) const {
  int   i;
  float d, min, max;

  min = INFINITY;
  max = -min;
  for(i = 0; i < numPoints; i++) {
    d = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(d < min) {
      min = d;
      if(FLOATSIGNBITSET(min) & FLOATSIGNBITNOTSET(max)) {
        return 0.0f;
      }
    }
    if(d > max) {
      max = d;
      if(FLOATSIGNBITSET(min) & FLOATSIGNBITNOTSET(max)) {
        return 0.0f;
      }
    }
  }
  if(FLOATSIGNBITNOTSET(min)) {
    return min;
  }
  if(FLOATSIGNBITSET(max)) {
    return max;
  }
  return 0.0f;
}

/*
=============
idWinding2D::PlaneSide
=============
*/
int idWinding2D::PlaneSide(idVec3 const& plane, float const epsilon) const {
  bool  front, back;
  int   i;
  float d;

  front = false;
  back  = false;
  for(i = 0; i < numPoints; i++) {
    d = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(d < -epsilon) {
      if(front) {
        return SIDE_CROSS;
      }
      back = true;
      continue;
    } else if(d > epsilon) {
      if(back) {
        return SIDE_CROSS;
      }
      front = true;
      continue;
    }
  }

  if(back) {
    return SIDE_BACK;
  }
  if(front) {
    return SIDE_FRONT;
  }
  return SIDE_ON;
}

/*
============
idWinding2D::PointInside
============
*/
bool idWinding2D::PointInside(idVec2 const& point, float const epsilon) const {
  int    i;
  float  d;
  idVec3 plane;

  for(i = 0; i < numPoints; i++) {
    plane = Plane2DFromPoints(p[i], p[(i + 1) % numPoints]);
    d     = plane.x * point.x + plane.y * point.y + plane.z;
    if(d > epsilon) {
      return false;
    }
  }
  return true;
}

/*
============
idWinding2D::LineIntersection
============
*/
bool idWinding2D::LineIntersection(idVec2 const& start, idVec2 const& end)
  const {
  int    i, numEdges;
  int    sides[MAX_POINTS_ON_WINDING_2D + 1], counts[3];
  float  d1, d2, epsilon = 0.1f;
  idVec3 plane, edges[2];

  counts[SIDE_FRONT] = counts[SIDE_BACK] = counts[SIDE_ON] = 0;

  plane = Plane2DFromPoints(start, end);
  for(i = 0; i < numPoints; i++) {
    d1 = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(d1 > epsilon) {
      sides[i] = SIDE_FRONT;
    } else if(d1 < -epsilon) {
      sides[i] = SIDE_BACK;
    } else {
      sides[i] = SIDE_ON;
    }
    counts[sides[i]]++;
  }
  sides[i] = sides[0];

  if(!counts[SIDE_FRONT]) {
    return false;
  }
  if(!counts[SIDE_BACK]) {
    return false;
  }

  numEdges = 0;
  for(i = 0; i < numPoints; i++) {
    if(sides[i] != sides[i + 1] && sides[i + 1] != SIDE_ON) {
      edges[numEdges++] = Plane2DFromPoints(p[i], p[(i + 1) % numPoints]);
      if(numEdges >= 2) {
        break;
      }
    }
  }
  if(numEdges < 2) {
    return false;
  }

  d1 = edges[0].x * start.x + edges[0].y * start.y + edges[0].z;
  d2 = edges[0].x * end.x + edges[0].y * end.y + edges[0].z;
  if(FLOATSIGNBITNOTSET(d1) & FLOATSIGNBITNOTSET(d2)) {
    return false;
  }
  d1 = edges[1].x * start.x + edges[1].y * start.y + edges[1].z;
  d2 = edges[1].x * end.x + edges[1].y * end.y + edges[1].z;
  if(FLOATSIGNBITNOTSET(d1) & FLOATSIGNBITNOTSET(d2)) {
    return false;
  }
  return true;
}

/*
============
idWinding2D::RayIntersection
============
*/
bool idWinding2D::RayIntersection(
  idVec2 const& start,
  idVec2 const& dir,
  float&        scale1,
  float&        scale2,
  int*          edgeNums
) const {
  int    i, numEdges, localEdgeNums[2];
  int    sides[MAX_POINTS_ON_WINDING_2D + 1], counts[3];
  float  d1, d2, epsilon = 0.1f;
  idVec3 plane, edges[2];

  scale1 = scale2    = 0.0f;
  counts[SIDE_FRONT] = counts[SIDE_BACK] = counts[SIDE_ON] = 0;

  plane = Plane2DFromVecs(start, dir);
  for(i = 0; i < numPoints; i++) {
    d1 = plane.x * p[i].x + plane.y * p[i].y + plane.z;
    if(d1 > epsilon) {
      sides[i] = SIDE_FRONT;
    } else if(d1 < -epsilon) {
      sides[i] = SIDE_BACK;
    } else {
      sides[i] = SIDE_ON;
    }
    counts[sides[i]]++;
  }
  sides[i] = sides[0];

  if(!counts[SIDE_FRONT]) {
    return false;
  }
  if(!counts[SIDE_BACK]) {
    return false;
  }

  numEdges = 0;
  for(i = 0; i < numPoints; i++) {
    if(sides[i] != sides[i + 1] && sides[i + 1] != SIDE_ON) {
      localEdgeNums[numEdges] = i;
      edges[numEdges++]       = Plane2DFromPoints(p[i], p[(i + 1) % numPoints]);
      if(numEdges >= 2) {
        break;
      }
    }
  }
  if(numEdges < 2) {
    return false;
  }

  d1 = edges[0].x * start.x + edges[0].y * start.y + edges[0].z;
  d2 = -(edges[0].x * dir.x + edges[0].y * dir.y);
  if(d2 == 0.0f) {
    return false;
  }
  scale1 = d1 / d2;
  d1     = edges[1].x * start.x + edges[1].y * start.y + edges[1].z;
  d2     = -(edges[1].x * dir.x + edges[1].y * dir.y);
  if(d2 == 0.0f) {
    return false;
  }
  scale2 = d1 / d2;

  if(fabs(scale1) > fabs(scale2)) {
    std::swap(scale1, scale2);
    std::swap(localEdgeNums[0], localEdgeNums[1]);
  }

  if(edgeNums) {
    edgeNums[0] = localEdgeNums[0];
    edgeNums[1] = localEdgeNums[1];
  }
  return true;
}

void Plane2DFromPoints_Generic(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize
) {
  for(int32_t Index = 0; Index < Count; Index++) {
    auto StartPoint = idVec2 { StartX[Index], StartY[Index] };
    auto EndPoint   = idVec2 { EndX[Index], EndY[Index] };

    auto ComputedPlane =
      idWinding2D::Plane2DFromPoints(StartPoint, EndPoint, Normalize);

    OutputX[Index] = ComputedPlane.x;
    OutputY[Index] = ComputedPlane.y;
    OutputZ[Index] = ComputedPlane.z;
  }
}

void Plane2DFromPoints_Optimized(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize
) {
  if(Normalize) {
    for(int32_t Index = 0; Index < Count; Index++) {
      auto StartPointX = StartX[Index];
      auto StartPointY = StartX[Index];
      auto EndPointX   = EndX[Index];
      auto EndPointY   = EndX[Index];

      idVec3 ComputedPlane;

      ComputedPlane.x = StartPointY - EndPointY;
      ComputedPlane.y = EndPointX - StartPointX;

      float sqrLength =
        ComputedPlane.x * ComputedPlane.x + ComputedPlane.y * ComputedPlane.y;

      float invLength = sqrtf(1.0f / sqrLength);

      ComputedPlane.x *= invLength;
      ComputedPlane.y *= invLength;
      ComputedPlane.z =
        -(StartPointX * ComputedPlane.x + StartPointY * ComputedPlane.y);

      OutputX[Index] = ComputedPlane.x;
      OutputY[Index] = ComputedPlane.y;
      OutputZ[Index] = ComputedPlane.z;
    }
  } else {
    for(int32_t Index = 0; Index < Count; Index++) {
      auto StartPointX = StartX[Index];
      auto StartPointY = StartX[Index];
      auto EndPointX   = EndX[Index];
      auto EndPointY   = EndX[Index];

      idVec3 ComputedPlane;

      ComputedPlane.x = StartPointY - EndPointY;
      ComputedPlane.y = EndPointX - StartPointX;
      ComputedPlane.z =
        -(StartPointX * ComputedPlane.x + StartPointY * ComputedPlane.y);

      OutputX[Index] = ComputedPlane.x;
      OutputY[Index] = ComputedPlane.y;
      OutputZ[Index] = ComputedPlane.z;
    }
  }
}

void Plane2DIntersection_Generic(
  int8_t*      DidIntersect,
  float*       IntersectionX,
  float*       IntersectionY,
  int32_t      Count,
  float const* Plane1X,
  float const* Plane1Y,
  float const* Plane1Z,
  float const* Plane2X,
  float const* Plane2Y,
  float const* Plane2Z
) {
  for(int32_t Index = 0; Index < Count; Index++) {
    idVec3 Plane1 { Plane1X[Index], Plane1Y[Index], Plane1Z[Index] };
    idVec3 Plane2 { Plane2X[Index], Plane2Y[Index], Plane2Z[Index] };

    idVec2 IntersectionPoint;

    DidIntersect[Index] =
      idWinding2D::Plane2DIntersection(Plane1, Plane2, IntersectionPoint);

    IntersectionX[Index] = IntersectionPoint.x;
    IntersectionY[Index] = IntersectionPoint.y;
  }
}

void Plane2DIntersection_Optimized(
  int8_t*      DidIntersect,
  float*       IntersectionX,
  float*       IntersectionY,
  int32_t      Count,
  float const* Plane1X,
  float const* Plane1Y,
  float const* Plane1Z,
  float const* Plane2X,
  float const* Plane2Y,
  float const* Plane2Z
) {
  for(int32_t Index = 0; Index < Count; Index++) {
    float n00 =
      Plane1X[Index] * Plane1X[Index] + Plane1Y[Index] * Plane1Y[Index];

    float n01 =
      Plane1X[Index] * Plane2X[Index] + Plane1Y[Index] * Plane2Y[Index];

    float n11 =
      Plane2X[Index] * Plane2X[Index] + Plane2Y[Index] * Plane2Y[Index];

    float det = n00 * n11 - n01 * n01;

    if(fabs(det) < 1e-6f) {
      DidIntersect[Index] = 0;
      continue;
    }

    float invDet = 1.0f / det;
    float f0     = (n01 * Plane2Z[Index] - n11 * Plane1Z[Index]) * invDet;
    float f1     = (n01 * Plane1Z[Index] - n00 * Plane2Z[Index]) * invDet;

    IntersectionX[Index] = f0 * Plane1X[Index] + f1 * Plane2X[Index];
    IntersectionY[Index] = f0 * Plane1Y[Index] + f1 * Plane2Y[Index];

    DidIntersect[Index] = 1;
  }
}

#if defined(__SSE__) || defined(__AVX__)

#  include <immintrin.h>

#endif

#if defined(__SSE__)

void Plane2DFromPoints_SSE(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize
) {
  if(Normalize) {
    for(int32_t Index = 0; Index < Count; Index += 4) {
      auto StartPointX = _mm_loadu_ps(StartX + Index);
      auto StartPointY = _mm_loadu_ps(StartY + Index);
      auto EndPointX   = _mm_loadu_ps(EndX + Index);
      auto EndPointY   = _mm_loadu_ps(EndY + Index);

      auto ComputedPlaneX = _mm_sub_ps(StartPointY, EndPointY);
      auto ComputedPlaneY = _mm_sub_ps(EndPointX, StartPointX);

      auto SquareLength = _mm_add_ps(
        _mm_mul_ps(ComputedPlaneX, ComputedPlaneX),
        _mm_mul_ps(ComputedPlaneY, ComputedPlaneY)
      );

      auto InverseLength = _mm_rsqrt_ps(SquareLength);

      ComputedPlaneX = _mm_mul_ps(ComputedPlaneX, InverseLength);
      ComputedPlaneY = _mm_mul_ps(ComputedPlaneY, InverseLength);

      auto ComputedPlaneZ = _mm_sub_ps(
        _mm_set1_ps(0.0f),
        _mm_add_ps(
          _mm_mul_ps(StartPointX, ComputedPlaneX),
          _mm_mul_ps(StartPointY, ComputedPlaneY)
        )
      );

      _mm_storeu_ps(OutputX + Index, ComputedPlaneX);
      _mm_storeu_ps(OutputY + Index, ComputedPlaneY);
      _mm_storeu_ps(OutputZ + Index, ComputedPlaneZ);
    }
  } else {
    for(int32_t Index = 0; Index < Count; Index += 4) {
      auto StartPointX = _mm_loadu_ps(StartX + Index);
      auto StartPointY = _mm_loadu_ps(StartY + Index);
      auto EndPointX   = _mm_loadu_ps(EndX + Index);
      auto EndPointY   = _mm_loadu_ps(EndY + Index);

      auto ComputedPlaneX = _mm_sub_ps(StartPointY, EndPointY);
      auto ComputedPlaneY = _mm_sub_ps(EndPointX, StartPointX);
      auto ComputedPlaneZ = _mm_sub_ps(
        _mm_set1_ps(0.0f),
        _mm_add_ps(
          _mm_mul_ps(StartPointX, ComputedPlaneX),
          _mm_mul_ps(StartPointY, ComputedPlaneY)
        )
      );

      _mm_storeu_ps(OutputX + Index, ComputedPlaneX);
      _mm_storeu_ps(OutputY + Index, ComputedPlaneY);
      _mm_storeu_ps(OutputZ + Index, ComputedPlaneZ);
    }
  }
}

void Plane2DIntersection_SSE(
  int8_t*      DidIntersect,
  float*       IntersectionX,
  float*       IntersectionY,
  int32_t      Count,
  float const* Plane1X,
  float const* Plane1Y,
  float const* Plane1Z,
  float const* Plane2X,
  float const* Plane2Y,
  float const* Plane2Z
) {
  for(int32_t Index = 0; Index < Count; Index += 4) {
    auto Plane1XLoad = _mm_loadu_ps(Plane1X + Index);
    auto Plane1YLoad = _mm_loadu_ps(Plane1Y + Index);
    auto Plane1ZLoad = _mm_loadu_ps(Plane1Z + Index);

    auto Plane2XLoad = _mm_loadu_ps(Plane2X + Index);
    auto Plane2YLoad = _mm_loadu_ps(Plane2Y + Index);
    auto Plane2ZLoad = _mm_loadu_ps(Plane2Z + Index);

    auto n00 = _mm_add_ps(
      _mm_mul_ps(Plane1XLoad, Plane1XLoad), _mm_mul_ps(Plane1YLoad, Plane1YLoad)
    );

    auto n01 = _mm_add_ps(
      _mm_mul_ps(Plane1XLoad, Plane2XLoad), _mm_mul_ps(Plane1YLoad, Plane2YLoad)
    );

    auto n11 = _mm_add_ps(
      _mm_mul_ps(Plane2XLoad, Plane2XLoad), _mm_mul_ps(Plane2YLoad, Plane2YLoad)
    );

    auto det = _mm_sub_ps(_mm_mul_ps(n00, n11), _mm_mul_ps(n01, n01));

    auto const sign_mask     = _mm_set1_ps(-0.0f);
    auto const minimum_value = _mm_set1_ps(1e-6f);
    auto const one           = _mm_set1_ps(1.0f);

    auto det_abs      = _mm_andnot_ps(sign_mask, det);
    auto compare_mask = _mm_cmplt_ps(det_abs, minimum_value);

    // TODO(wpieterse): Populate intersection mask

    auto invDet = _mm_div_ps(one, det);

    auto f0 = _mm_mul_ps(
      _mm_sub_ps(_mm_mul_ps(n01, Plane2ZLoad), _mm_mul_ps(n11, Plane1ZLoad)),
      invDet
    );

    auto f1 = _mm_mul_ps(
      _mm_sub_ps(_mm_mul_ps(n01, Plane1ZLoad), _mm_mul_ps(n00, Plane2ZLoad)),
      invDet
    );

    _mm_storeu_ps(
      IntersectionX + Index,
      _mm_add_ps(_mm_mul_ps(f0, Plane1XLoad), _mm_mul_ps(f1, Plane2XLoad))
    );

    _mm_storeu_ps(
      IntersectionY + Index,
      _mm_add_ps(_mm_mul_ps(f0, Plane1YLoad), _mm_mul_ps(f1, Plane2YLoad))
    );

    DidIntersect[Index] = 1;
  }
}

#endif

#if defined(__AVX__)

void Plane2DFromPoints_AVX(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize
) {
  if(Normalize) {
    for(int32_t Index = 0; Index < Count; Index += 8) {
      auto StartPointX = _mm256_loadu_ps(StartX + Index);
      auto StartPointY = _mm256_loadu_ps(StartY + Index);
      auto EndPointX   = _mm256_loadu_ps(EndX + Index);
      auto EndPointY   = _mm256_loadu_ps(EndY + Index);

      auto ComputedPlaneX = _mm256_sub_ps(StartPointY, EndPointY);
      auto ComputedPlaneY = _mm256_sub_ps(EndPointX, StartPointX);

      auto SquareLength = _mm256_add_ps(
        _mm256_mul_ps(ComputedPlaneX, ComputedPlaneX),
        _mm256_mul_ps(ComputedPlaneY, ComputedPlaneY)
      );

      auto InverseLength = _mm256_rsqrt_ps(SquareLength);

      ComputedPlaneX = _mm256_mul_ps(ComputedPlaneX, InverseLength);
      ComputedPlaneY = _mm256_mul_ps(ComputedPlaneY, InverseLength);

      auto ComputedPlaneZ = _mm256_sub_ps(
        _mm256_set1_ps(0.0f),
        _mm256_add_ps(
          _mm256_mul_ps(StartPointX, ComputedPlaneX),
          _mm256_mul_ps(StartPointY, ComputedPlaneY)
        )
      );

      _mm256_storeu_ps(OutputX + Index, ComputedPlaneX);
      _mm256_storeu_ps(OutputY + Index, ComputedPlaneY);
      _mm256_storeu_ps(OutputZ + Index, ComputedPlaneZ);
    }
  } else {
    for(int32_t Index = 0; Index < Count; Index += 8) {
      auto StartPointX = _mm256_loadu_ps(StartX + Index);
      auto StartPointY = _mm256_loadu_ps(StartY + Index);
      auto EndPointX   = _mm256_loadu_ps(EndX + Index);
      auto EndPointY   = _mm256_loadu_ps(EndY + Index);

      auto ComputedPlaneX = _mm256_sub_ps(StartPointY, EndPointY);
      auto ComputedPlaneY = _mm256_sub_ps(EndPointX, StartPointX);
      auto ComputedPlaneZ = _mm256_sub_ps(
        _mm256_set1_ps(0.0f),
        _mm256_add_ps(
          _mm256_mul_ps(StartPointX, ComputedPlaneX),
          _mm256_mul_ps(StartPointY, ComputedPlaneY)
        )
      );

      _mm256_storeu_ps(OutputX + Index, ComputedPlaneX);
      _mm256_storeu_ps(OutputY + Index, ComputedPlaneY);
      _mm256_storeu_ps(OutputZ + Index, ComputedPlaneZ);
    }
  }
}

void Plane2DIntersection_AVX(
  int8_t*      DidIntersect,
  float*       IntersectionX,
  float*       IntersectionY,
  int32_t      Count,
  float const* Plane1X,
  float const* Plane1Y,
  float const* Plane1Z,
  float const* Plane2X,
  float const* Plane2Y,
  float const* Plane2Z
) {
  for(int32_t Index = 0; Index < Count; Index += 8) {
    auto Plane1XLoad = _mm256_loadu_ps(Plane1X + Index);
    auto Plane1YLoad = _mm256_loadu_ps(Plane1Y + Index);
    auto Plane1ZLoad = _mm256_loadu_ps(Plane1Z + Index);

    auto Plane2XLoad = _mm256_loadu_ps(Plane2X + Index);
    auto Plane2YLoad = _mm256_loadu_ps(Plane2Y + Index);
    auto Plane2ZLoad = _mm256_loadu_ps(Plane2Z + Index);

    auto n00 = _mm256_add_ps(
      _mm256_mul_ps(Plane1XLoad, Plane1XLoad),
      _mm256_mul_ps(Plane1YLoad, Plane1YLoad)
    );

    auto n01 = _mm256_add_ps(
      _mm256_mul_ps(Plane1XLoad, Plane2XLoad),
      _mm256_mul_ps(Plane1YLoad, Plane2YLoad)
    );

    auto n11 = _mm256_add_ps(
      _mm256_mul_ps(Plane2XLoad, Plane2XLoad),
      _mm256_mul_ps(Plane2YLoad, Plane2YLoad)
    );

    auto det = _mm256_sub_ps(_mm256_mul_ps(n00, n11), _mm256_mul_ps(n01, n01));

    auto const sign_mask     = _mm256_set1_ps(-0.0f);
    auto const minimum_value = _mm256_set1_ps(1e-6f);
    auto const one           = _mm256_set1_ps(1.0f);

    auto det_abs = _mm256_andnot_ps(sign_mask, det);
    // auto compare_mask = _mm256_cmplt_ps(det_abs, minimum_value);

    // TODO(wpieterse): Populate intersection mask

    auto invDet = _mm256_div_ps(one, det);

    auto f0 = _mm256_mul_ps(
      _mm256_sub_ps(
        _mm256_mul_ps(n01, Plane2ZLoad), _mm256_mul_ps(n11, Plane1ZLoad)
      ),
      invDet
    );

    auto f1 = _mm256_mul_ps(
      _mm256_sub_ps(
        _mm256_mul_ps(n01, Plane1ZLoad), _mm256_mul_ps(n00, Plane2ZLoad)
      ),
      invDet
    );

    _mm256_storeu_ps(
      IntersectionX + Index,
      _mm256_add_ps(
        _mm256_mul_ps(f0, Plane1XLoad), _mm256_mul_ps(f1, Plane2XLoad)
      )
    );

    _mm256_storeu_ps(
      IntersectionY + Index,
      _mm256_add_ps(
        _mm256_mul_ps(f0, Plane1YLoad), _mm256_mul_ps(f1, Plane2YLoad)
      )
    );

    DidIntersect[Index] = 1;
  }
}

#endif

#if defined(__ARM_FEATURE_SVE)

#  include <arm_sve.h>

void Plane2DIntersection_SVE(
  int8_t*      DidIntersect,
  float*       IntersectionX,
  float*       IntersectionY,
  int32_t      Count,
  float const* Plane1X,
  float const* Plane1Y,
  float const* Plane1Z,
  float const* Plane2X,
  float const* Plane2Y,
  float const* Plane2Z
) {
  svbool_t    predicate;
  svfloat32_t n00;
  svfloat32_t n01;
  svfloat32_t n11;

  uint64_t numVals = svlen_f32(n00);

  auto c_min = svdup_n_f32(1e-6f);
  auto c_one = svdup_n_f32(1.0f);

  for(int32_t Index = 0; Index < Count; Index += numVals) {
    predicate = svwhilelt_b32_s32(Index, Count);

    auto p1x_load = svld1_f32(predicate, Plane1X + Index);
    auto p1y_load = svld1_f32(predicate, Plane1Y + Index);
    auto p1z_load = svld1_f32(predicate, Plane1Z + Index);

    auto p2x_load = svld1_f32(predicate, Plane2X + Index);
    auto p2y_load = svld1_f32(predicate, Plane2Y + Index);
    auto p2z_load = svld1_f32(predicate, Plane2Z + Index);

    n00 = svadd_f32_z(
      predicate,
      svmul_f32_z(predicate, p1x_load, p1x_load),
      svmul_f32_z(predicate, p1y_load, p1y_load)
    );

    n01 = svadd_f32_z(
      predicate,
      svmul_f32_z(predicate, p1x_load, p2x_load),
      svmul_f32_z(predicate, p1y_load, p2y_load)
    );

    n11 = svadd_f32_z(
      predicate,
      svmul_f32_z(predicate, p2x_load, p2x_load),
      svmul_f32_z(predicate, p2y_load, p2y_load)
    );

    auto det = svsub_f32_z(
      predicate,
      svmul_f32_z(predicate, n00, n11),
      svmul_f32_z(predicate, n01, n01)
    );

    auto abs_v = svabs_f32_z(predicate, det);

    auto compare_p = svcmplt_f32(predicate, abs_v, c_min);

    auto invDet = svdiv_f32_z(compare_p, c_one, det);

    auto f0 = svmul_f32_z(
      compare_p,
      svsub_f32_z(
        predicate,
        svmul_f32_z(compare_p, n01, p2z_load),
        svmul_f32_z(compare_p, n11, p1z_load)
      ),
      invDet
    );

    auto f1 = svmul_f32_z(
      compare_p,
      svsub_f32_z(
        predicate,
        svmul_f32_z(compare_p, n01, p1z_load),
        svmul_f32_z(compare_p, n00, p2z_load)
      ),
      invDet
    );

    auto intersection_x = svadd_f32_z(
      compare_p,
      svmul_f32_z(compare_p, f0, p1x_load),
      svmul_f32_z(compare_p, f1, p2x_load)
    );

    auto intersection_y = svadd_f32_z(
      compare_p,
      svmul_f32_z(compare_p, f0, p1y_load),
      svmul_f32_z(compare_p, f1, p2y_load)
    );

    svst1_f32(predicate, IntersectionX + Index, intersection_x);
    svst1_f32(predicate, IntersectionY + Index, intersection_y);

    // DidIntersect[Index] = 1;
  }
}

#endif
