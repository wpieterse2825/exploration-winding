#pragma once

#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include <algorithm>

#define FLOATSIGNBITSET(f)    ((*(const unsigned long*) &(f)) >> 31)
#define FLOATSIGNBITNOTSET(f) ((~(*(const unsigned long*) &(f))) >> 31)
#define FLOATNOTZERO(f)       ((*(const unsigned long*) &(f)) & ~(1 << 31))
#define INTSIGNBITSET(i)      (((const unsigned long) (i)) >> 31)
#define INTSIGNBITNOTSET(i)   ((~((const unsigned long) (i))) >> 31)

// #define INFINITY 1e30f

#define PI        3.14159265358979323846f
#define M_DEG2RAD PI / 180.0f
#define M_RAD2DEG 180.0f / PI

#define DEG2RAD(a) ((a) * M_DEG2RAD)
#define RAD2DEG(a) ((a) * M_RAD2DEG)

#define MAX_WORLD_COORD (128 * 1024)
#define MIN_WORLD_COORD (-128 * 1024)
#define MAX_WORLD_SIZE  (MAX_WORLD_COORD - MIN_WORLD_COORD)

#define DEGENERATE_DIST_EPSILON 1e-4f

#define SIDE_FRONT 0
#define SIDE_BACK  1
#define SIDE_ON    2
#define SIDE_CROSS 3

// plane sides
#define PLANESIDE_FRONT 0
#define PLANESIDE_BACK  1
#define PLANESIDE_ON    2
#define PLANESIDE_CROSS 3

// plane types
#define PLANETYPE_X         0
#define PLANETYPE_Y         1
#define PLANETYPE_Z         2
#define PLANETYPE_NEGX      3
#define PLANETYPE_NEGY      4
#define PLANETYPE_NEGZ      5
#define PLANETYPE_TRUEAXIAL 6 // all types < 6 are true axial planes
#define PLANETYPE_ZEROX     6
#define PLANETYPE_ZEROY     7
#define PLANETYPE_ZEROZ     8
#define PLANETYPE_NONAXIAL  9

float InvSqrt(float x);

class idVec2 {
  public:
    float x;
    float y;

    idVec2(void);
    explicit idVec2(float const x, float const y);

    void Set(float const x, float const y);
    void Zero(void);

    float   operator[](int index) const;
    float&  operator[](int index);
    idVec2  operator-() const;
    float   operator*(idVec2 const& a) const;
    idVec2  operator*(float const a) const;
    idVec2  operator/(float const a) const;
    idVec2  operator+(idVec2 const& a) const;
    idVec2  operator-(idVec2 const& a) const;
    idVec2& operator+=(idVec2 const& a);
    idVec2& operator-=(idVec2 const& a);
    idVec2& operator/=(idVec2 const& a);
    idVec2& operator/=(float const a);
    idVec2& operator*=(float const a);

    friend idVec2 operator*(float const a, idVec2 const b);

    bool Compare(idVec2 const& a) const; // exact compare, no epsilon
    bool
    Compare(idVec2 const& a, float const epsilon) const; // compare with epsilon
    bool operator==(idVec2 const& a) const; // exact compare, no epsilon
    bool operator!=(idVec2 const& a) const; // exact compare, no epsilon

    float   Length(void) const;
    float   LengthFast(void) const;
    float   LengthSqr(void) const;
    float   Normalize(void);        // returns length
    float   NormalizeFast(void);    // returns length
    idVec2& Truncate(float length); // cap length
    void    Clamp(idVec2 const& min, idVec2 const& max);
    void    Snap(void);    // snap to closest integer value
    void    SnapInt(void); // snap towards integer (floor)

    int GetDimension(void) const;

    float const* ToFloatPtr(void) const;
    float*       ToFloatPtr(void);
    char const*  ToString(int precision = 2) const;

    void Lerp(idVec2 const& v1, idVec2 const& v2, float const l);
};

class idVec3 {
  public:
    float x;
    float y;
    float z;

    idVec3(void);
    explicit idVec3(float const x, float const y, float const z);

    void Set(float const x, float const y, float const z);
    void Zero(void);

    float   operator[](int const index) const;
    float&  operator[](int const index);
    idVec3  operator-() const;
    idVec3& operator=(idVec3 const& a); // required because of a msvc 6 & 7 bug
    float   operator*(idVec3 const& a) const;
    idVec3  operator*(float const a) const;
    idVec3  operator/(float const a) const;
    idVec3  operator+(idVec3 const& a) const;
    idVec3  operator-(idVec3 const& a) const;
    idVec3& operator+=(idVec3 const& a);
    idVec3& operator-=(idVec3 const& a);
    idVec3& operator/=(idVec3 const& a);
    idVec3& operator/=(float const a);
    idVec3& operator*=(float const a);

    friend idVec3 operator*(float const a, idVec3 const b);

    bool Compare(idVec3 const& a) const; // exact compare, no epsilon
    bool
    Compare(idVec3 const& a, float const epsilon) const; // compare with epsilon
    bool operator==(idVec3 const& a) const; // exact compare, no epsilon
    bool operator!=(idVec3 const& a) const; // exact compare, no epsilon

    bool FixDegenerateNormal(void); // fix degenerate axial cases
    bool FixDenormals(void);        // change tiny numbers to zero

    idVec3  Cross(idVec3 const& a) const;
    idVec3& Cross(idVec3 const& a, idVec3 const& b);
    float   Length(void) const;
    float   LengthSqr(void) const;
    float   LengthFast(void) const;
    float   Normalize(void);        // returns length
    float   NormalizeFast(void);    // returns length
    idVec3& Truncate(float length); // cap length
    void    Clamp(idVec3 const& min, idVec3 const& max);
    void    Snap(void);    // snap to closest integer value
    void    SnapInt(void); // snap towards integer (floor)

    int GetDimension(void) const;

    float         ToYaw(void) const;
    float         ToPitch(void) const;
    idVec2 const& ToVec2(void) const;
    idVec2&       ToVec2(void);
    float const*  ToFloatPtr(void) const;
    float*        ToFloatPtr(void);
    char const*   ToString(int precision = 2) const;

    void NormalVectors(idVec3& left, idVec3& down)
      const; // vector should be normalized
    void OrthogonalBasis(idVec3& left, idVec3& up) const;

    void ProjectOntoPlane(idVec3 const& normal, float const overBounce = 1.0f);
    bool ProjectAlongPlane(
      idVec3 const& normal, float const epsilon, float const overBounce = 1.0f
    );
    void ProjectSelfOntoSphere(float const radius);

    void Lerp(idVec3 const& v1, idVec3 const& v2, float const l);
    void SLerp(idVec3 const& v1, idVec3 const& v2, float const l);
};

extern idVec2 vec2_origin;
#define vec2_zero vec2_origin

extern idVec3 vec3_origin;
#define vec3_zero vec3_origin
