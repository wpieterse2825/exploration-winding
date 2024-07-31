#pragma once

#include "exploration/winding/library_vector.hpp"

#define MAX_POINTS_ON_WINDING_2D 16
#define ON_EPSILON               0.1f

class idWinding2D {
  public:
    idWinding2D(void);

    idWinding2D&  operator=(idWinding2D const& winding);
    idVec2 const& operator[](int const index) const;
    idVec2&       operator[](int const index);

    void Clear(void);
    void AddPoint(idVec2 const& point);
    int  GetNumPoints(void) const;

    void Expand(float const d);
    void ExpandForAxialBox(idVec2 const bounds[2]);

    // splits the winding into a front and back winding, the winding itself
    // stays unchanged returns a SIDE_?
    int Split(
      idVec3 const& plane,
      float const   epsilon,
      idWinding2D** front,
      idWinding2D** back
    ) const;
    // cuts off the part at the back side of the plane, returns true if some
    // part was at the front if there is nothing at the front the number of
    // points is set to zero
    bool ClipInPlace(
      idVec3 const& plane,
      float const   epsilon = ON_EPSILON,
      bool const    keepOn  = false
    );

    idWinding2D* Copy(void) const;
    idWinding2D* Reverse(void) const;

    float  GetArea(void) const;
    idVec2 GetCenter(void) const;
    float  GetRadius(idVec2 const& center) const;
    void   GetBounds(idVec2 bounds[2]) const;

    bool IsTiny(void) const;
    bool IsHuge(void) const; // base winding for a plane is typically huge
    void Print(void) const;

    float PlaneDistance(idVec3 const& plane) const;
    int PlaneSide(idVec3 const& plane, float const epsilon = ON_EPSILON) const;

    bool PointInside(idVec2 const& point, float const epsilon) const;
    bool LineIntersection(idVec2 const& start, idVec2 const& end) const;
    bool RayIntersection(
      idVec2 const& start,
      idVec2 const& dir,
      float&        scale1,
      float&        scale2,
      int*          edgeNums = nullptr
    ) const;

    static idVec3 Plane2DFromPoints(
      idVec2 const& start, idVec2 const& end, bool const normalize = false
    );
    static idVec3 Plane2DFromVecs(
      idVec2 const& start, idVec2 const& dir, bool const normalize = false
    );
    static bool Plane2DIntersection(
      idVec3 const& plane1, idVec3 const& plane2, idVec2& point
    );

  private:
    int    numPoints;
    idVec2 p[MAX_POINTS_ON_WINDING_2D];
};

void Plane2DFromPoints_Generic(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize = false
);

void Plane2DFromPoints_Optimized(
  float*       OutputX,
  float*       OutputY,
  float*       OutputZ,
  int32_t      Count,
  float const* StartX,
  float const* StartY,
  float const* EndX,
  float const* EndY,
  bool         Normalize = false
);

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
);

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
);

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
  bool         Normalize = false
);

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
);

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
  bool         Normalize = false
);

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
);

#endif

#if defined(__ARM_FEATURE_SVE)

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
);

#endif
