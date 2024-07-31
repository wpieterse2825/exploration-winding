#include "exploration/winding/library_vector.hpp"

idVec2 vec2_origin(0.0f, 0.0f);
idVec3 vec3_origin(0.0f, 0.0f, 0.0f);

idVec2::idVec2(void) {
}

idVec2::idVec2(float const x, float const y) {
  this->x = x;
  this->y = y;
}

void idVec2::Set(float const x, float const y) {
  this->x = x;
  this->y = y;
}

void idVec2::Zero(void) {
  x = y = 0.0f;
}

bool idVec2::Compare(idVec2 const& a) const {
  return ((x == a.x) && (y == a.y));
}

bool idVec2::Compare(idVec2 const& a, float const epsilon) const {
  if(fabs(x - a.x) > epsilon) {
    return false;
  }

  if(fabs(y - a.y) > epsilon) {
    return false;
  }

  return true;
}

bool idVec2::operator==(idVec2 const& a) const {
  return Compare(a);
}

bool idVec2::operator!=(idVec2 const& a) const {
  return !Compare(a);
}

float idVec2::operator[](int index) const {
  return (&x)[index];
}

float& idVec2::operator[](int index) {
  return (&x)[index];
}

float idVec2::Length(void) const {
  return (float) sqrtf(x * x + y * y);
}

float InvSqrt(float x) {
  return sqrtf(1.0f / x);
}

float RSqrt(float x) {

  long  i;
  float y, r;

  y = x * 0.5f;
  i = *reinterpret_cast<long*>(&x);
  i = 0x5f3759df - (i >> 1);
  r = *reinterpret_cast<float*>(&i);
  r = r * (1.5f - r * r * y);
  return r;
}

float idVec2::LengthFast(void) const {
  float sqrLength;

  sqrLength = x * x + y * y;
  return sqrLength * RSqrt(sqrLength);
}

float idVec2::LengthSqr(void) const {
  return (x * x + y * y);
}

float idVec2::Normalize(void) {
  float sqrLength, invLength;

  sqrLength  = x * x + y * y;
  invLength  = InvSqrt(sqrLength);
  x         *= invLength;
  y         *= invLength;
  return invLength * sqrLength;
}

float idVec2::NormalizeFast(void) {
  float lengthSqr, invLength;

  lengthSqr  = x * x + y * y;
  invLength  = RSqrt(lengthSqr);
  x         *= invLength;
  y         *= invLength;
  return invLength * lengthSqr;
}

idVec2& idVec2::Truncate(float length) {
  float length2;
  float ilength;

  if(!length) {
    Zero();
  } else {
    length2 = LengthSqr();
    if(length2 > length * length) {
      ilength  = length * InvSqrt(length2);
      x       *= ilength;
      y       *= ilength;
    }
  }

  return *this;
}

void idVec2::Clamp(idVec2 const& min, idVec2 const& max) {
  if(x < min.x) {
    x = min.x;
  } else if(x > max.x) {
    x = max.x;
  }
  if(y < min.y) {
    y = min.y;
  } else if(y > max.y) {
    y = max.y;
  }
}

void idVec2::Snap(void) {
  x = floor(x + 0.5f);
  y = floor(y + 0.5f);
}

void idVec2::SnapInt(void) {
  x = float(int(x));
  y = float(int(y));
}

idVec2 idVec2::operator-() const {
  return idVec2(-x, -y);
}

idVec2 idVec2::operator-(idVec2 const& a) const {
  return idVec2(x - a.x, y - a.y);
}

float idVec2::operator*(idVec2 const& a) const {
  return x * a.x + y * a.y;
}

idVec2 idVec2::operator*(float const a) const {
  return idVec2(x * a, y * a);
}

idVec2 idVec2::operator/(float const a) const {
  float inva = 1.0f / a;
  return idVec2(x * inva, y * inva);
}

idVec2 operator*(float const a, idVec2 const b) {
  return idVec2(b.x * a, b.y * a);
}

idVec2 idVec2::operator+(idVec2 const& a) const {
  return idVec2(x + a.x, y + a.y);
}

idVec2& idVec2::operator+=(idVec2 const& a) {
  x += a.x;
  y += a.y;

  return *this;
}

idVec2& idVec2::operator/=(idVec2 const& a) {
  x /= a.x;
  y /= a.y;

  return *this;
}

idVec2& idVec2::operator/=(float const a) {
  float inva  = 1.0f / a;
  x          *= inva;
  y          *= inva;

  return *this;
}

idVec2& idVec2::operator-=(idVec2 const& a) {
  x -= a.x;
  y -= a.y;

  return *this;
}

idVec2& idVec2::operator*=(float const a) {
  x *= a;
  y *= a;

  return *this;
}

int idVec2::GetDimension(void) const {
  return 2;
}

float const* idVec2::ToFloatPtr(void) const {
  return &x;
}

float* idVec2::ToFloatPtr(void) {
  return &x;
}

void idVec2::Lerp(idVec2 const& v1, idVec2 const& v2, float const l) {
  if(l <= 0.0f) {
    (*this) = v1;
  } else if(l >= 1.0f) {
    (*this) = v2;
  } else {
    (*this) = v1 + l * (v2 - v1);
  }
}

idVec3::idVec3(void) {
}

idVec3::idVec3(float const x, float const y, float const z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

float idVec3::operator[](int const index) const {
  return (&x)[index];
}

float& idVec3::operator[](int const index) {
  return (&x)[index];
}

void idVec3::Set(float const x, float const y, float const z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

void idVec3::Zero(void) {
  x = y = z = 0.0f;
}

idVec3 idVec3::operator-() const {
  return idVec3(-x, -y, -z);
}

idVec3& idVec3::operator=(idVec3 const& a) {
  x = a.x;
  y = a.y;
  z = a.z;
  return *this;
}

idVec3 idVec3::operator-(idVec3 const& a) const {
  return idVec3(x - a.x, y - a.y, z - a.z);
}

float idVec3::operator*(idVec3 const& a) const {
  return x * a.x + y * a.y + z * a.z;
}

idVec3 idVec3::operator*(float const a) const {
  return idVec3(x * a, y * a, z * a);
}

idVec3 idVec3::operator/(float const a) const {
  float inva = 1.0f / a;
  return idVec3(x * inva, y * inva, z * inva);
}

idVec3 operator*(float const a, idVec3 const b) {
  return idVec3(b.x * a, b.y * a, b.z * a);
}

idVec3 idVec3::operator+(idVec3 const& a) const {
  return idVec3(x + a.x, y + a.y, z + a.z);
}

idVec3& idVec3::operator+=(idVec3 const& a) {
  x += a.x;
  y += a.y;
  z += a.z;

  return *this;
}

idVec3& idVec3::operator/=(idVec3 const& a) {
  x /= a.x;
  y /= a.y;
  z /= a.z;

  return *this;
}

idVec3& idVec3::operator/=(float const a) {
  float inva  = 1.0f / a;
  x          *= inva;
  y          *= inva;
  z          *= inva;

  return *this;
}

idVec3& idVec3::operator-=(idVec3 const& a) {
  x -= a.x;
  y -= a.y;
  z -= a.z;

  return *this;
}

idVec3& idVec3::operator*=(float const a) {
  x *= a;
  y *= a;
  z *= a;

  return *this;
}

bool idVec3::Compare(idVec3 const& a) const {
  return ((x == a.x) && (y == a.y) && (z == a.z));
}

bool idVec3::Compare(idVec3 const& a, float const epsilon) const {
  if(fabs(x - a.x) > epsilon) {
    return false;
  }

  if(fabs(y - a.y) > epsilon) {
    return false;
  }

  if(fabs(z - a.z) > epsilon) {
    return false;
  }

  return true;
}

bool idVec3::operator==(idVec3 const& a) const {
  return Compare(a);
}

bool idVec3::operator!=(idVec3 const& a) const {
  return !Compare(a);
}

float idVec3::NormalizeFast(void) {
  float sqrLength, invLength;

  sqrLength  = x * x + y * y + z * z;
  invLength  = RSqrt(sqrLength);
  x         *= invLength;
  y         *= invLength;
  z         *= invLength;
  return invLength * sqrLength;
}

bool idVec3::FixDegenerateNormal(void) {
  if(x == 0.0f) {
    if(y == 0.0f) {
      if(z > 0.0f) {
        if(z != 1.0f) {
          z = 1.0f;
          return true;
        }
      } else {
        if(z != -1.0f) {
          z = -1.0f;
          return true;
        }
      }
      return false;
    } else if(z == 0.0f) {
      if(y > 0.0f) {
        if(y != 1.0f) {
          y = 1.0f;
          return true;
        }
      } else {
        if(y != -1.0f) {
          y = -1.0f;
          return true;
        }
      }
      return false;
    }
  } else if(y == 0.0f) {
    if(z == 0.0f) {
      if(x > 0.0f) {
        if(x != 1.0f) {
          x = 1.0f;
          return true;
        }
      } else {
        if(x != -1.0f) {
          x = -1.0f;
          return true;
        }
      }
      return false;
    }
  }
  if(fabs(x) == 1.0f) {
    if(y != 0.0f || z != 0.0f) {
      y = z = 0.0f;
      return true;
    }
    return false;
  } else if(fabs(y) == 1.0f) {
    if(x != 0.0f || z != 0.0f) {
      x = z = 0.0f;
      return true;
    }
    return false;
  } else if(fabs(z) == 1.0f) {
    if(x != 0.0f || y != 0.0f) {
      x = y = 0.0f;
      return true;
    }
    return false;
  }
  return false;
}

bool idVec3::FixDenormals(void) {
  bool denormal = false;
  if(fabs(x) < 1e-30f) {
    x        = 0.0f;
    denormal = true;
  }
  if(fabs(y) < 1e-30f) {
    y        = 0.0f;
    denormal = true;
  }
  if(fabs(z) < 1e-30f) {
    z        = 0.0f;
    denormal = true;
  }
  return denormal;
}

idVec3 idVec3::Cross(idVec3 const& a) const {
  return idVec3(y * a.z - z * a.y, z * a.x - x * a.z, x * a.y - y * a.x);
}

idVec3& idVec3::Cross(idVec3 const& a, idVec3 const& b) {
  x = a.y * b.z - a.z * b.y;
  y = a.z * b.x - a.x * b.z;
  z = a.x * b.y - a.y * b.x;

  return *this;
}

float idVec3::Length(void) const {
  return (float) sqrtf(x * x + y * y + z * z);
}

float idVec3::LengthSqr(void) const {
  return (x * x + y * y + z * z);
}

float idVec3::LengthFast(void) const {
  float sqrLength;

  sqrLength = x * x + y * y + z * z;
  return sqrLength * RSqrt(sqrLength);
}

float idVec3::Normalize(void) {
  float sqrLength, invLength;

  sqrLength  = x * x + y * y + z * z;
  invLength  = InvSqrt(sqrLength);
  x         *= invLength;
  y         *= invLength;
  z         *= invLength;
  return invLength * sqrLength;
}

idVec3& idVec3::Truncate(float length) {
  float length2;
  float ilength;

  if(!length) {
    Zero();
  } else {
    length2 = LengthSqr();
    if(length2 > length * length) {
      ilength  = length * InvSqrt(length2);
      x       *= ilength;
      y       *= ilength;
      z       *= ilength;
    }
  }

  return *this;
}

void idVec3::Clamp(idVec3 const& min, idVec3 const& max) {
  if(x < min.x) {
    x = min.x;
  } else if(x > max.x) {
    x = max.x;
  }
  if(y < min.y) {
    y = min.y;
  } else if(y > max.y) {
    y = max.y;
  }
  if(z < min.z) {
    z = min.z;
  } else if(z > max.z) {
    z = max.z;
  }
}

void idVec3::Snap(void) {
  x = floor(x + 0.5f);
  y = floor(y + 0.5f);
  z = floor(z + 0.5f);
}

void idVec3::SnapInt(void) {
  x = float(int(x));
  y = float(int(y));
  z = float(int(z));
}

int idVec3::GetDimension(void) const {
  return 3;
}

idVec2 const& idVec3::ToVec2(void) const {
  return *reinterpret_cast<idVec2 const*>(this);
}

idVec2& idVec3::ToVec2(void) {
  return *reinterpret_cast<idVec2*>(this);
}

float const* idVec3::ToFloatPtr(void) const {
  return &x;
}

float* idVec3::ToFloatPtr(void) {
  return &x;
}

void idVec3::NormalVectors(idVec3& left, idVec3& down) const {
  float d;

  d = x * x + y * y;
  if(!d) {
    left[0] = 1;
    left[1] = 0;
    left[2] = 0;
  } else {
    d       = InvSqrt(d);
    left[0] = -y * d;
    left[1] = x * d;
    left[2] = 0;
  }
  down = left.Cross(*this);
}

void idVec3::OrthogonalBasis(idVec3& left, idVec3& up) const {
  float l, s;

  if(fabs(z) > 0.7f) {
    l       = y * y + z * z;
    s       = InvSqrt(l);
    up[0]   = 0;
    up[1]   = z * s;
    up[2]   = -y * s;
    left[0] = l * s;
    left[1] = -x * up[2];
    left[2] = x * up[1];
  } else {
    l       = x * x + y * y;
    s       = InvSqrt(l);
    left[0] = -y * s;
    left[1] = x * s;
    left[2] = 0;
    up[0]   = -z * left[1];
    up[1]   = z * left[0];
    up[2]   = l * s;
  }
}

void idVec3::ProjectOntoPlane(idVec3 const& normal, float const overBounce) {
  float backoff;

  backoff = *this * normal;

  if(overBounce != 1.0) {
    if(backoff < 0) {
      backoff *= overBounce;
    } else {
      backoff /= overBounce;
    }
  }

  *this -= backoff * normal;
}

bool idVec3::ProjectAlongPlane(
  idVec3 const& normal, float const epsilon, float const overBounce
) {
  idVec3 cross;
  float  len;

  cross = this->Cross(normal).Cross((*this));
  // normalize so a fixed epsilon can be used
  cross.Normalize();
  len = normal * cross;
  if(fabs(len) < epsilon) {
    return false;
  }
  cross   *= overBounce * (normal * (*this)) / len;
  (*this) -= cross;
  return true;
}

float idVec3::ToYaw(void) const {
  float yaw;

  if((y == 0.0f) && (x == 0.0f)) {
    yaw = 0.0f;
  } else {
    yaw = RAD2DEG(atan2(y, x));
    if(yaw < 0.0f) {
      yaw += 360.0f;
    }
  }

  return yaw;
}

/*
=============
idVec3::ToPitch
=============
*/
float idVec3::ToPitch(void) const {
  float forward;
  float pitch;

  if((x == 0.0f) && (y == 0.0f)) {
    if(z > 0.0f) {
      pitch = 90.0f;
    } else {
      pitch = 270.0f;
    }
  } else {
    forward = (float) sqrtf(x * x + y * y);
    pitch   = RAD2DEG(atan2(z, forward));
    if(pitch < 0.0f) {
      pitch += 360.0f;
    }
  }

  return pitch;
}

/*
=============
Lerp

Linearly inperpolates one vector to
 *
 *
 * another.
=============
*/
void idVec3::Lerp(idVec3 const& v1, idVec3 const& v2, float const l) {
  if(l <= 0.0f) {
    (*this) = v1;
  } else if(l >= 1.0f) {
    (*this) = v2;
  } else {
    (*this) = v1 + l * (v2 - v1);
  }
}

/*
=============
SLerp

Spherical linear interpolation from v1 to v2.
Vectors

 * *
 * are expected to be normalized.
=============
*/
#define LERP_DELTA 1e-6

void idVec3::SLerp(idVec3 const& v1, idVec3 const& v2, float const t) {
  float omega, cosom, sinom, scale0, scale1;

  if(t <= 0.0f) {
    (*this) = v1;
    return;
  } else if(t >= 1.0f) {
    (*this) = v2;
    return;
  }

  cosom = v1 * v2;
  if((1.0f - cosom) > LERP_DELTA) {
    omega  = acos(cosom);
    sinom  = sin(omega);
    scale0 = sin((1.0f - t) * omega) / sinom;
    scale1 = sin(t * omega) / sinom;
  } else {
    scale0 = 1.0f - t;
    scale1 = t;
  }

  (*this) = (v1 * scale0 + v2 * scale1);
}

/*
=============
ProjectSelfOntoSphere

Projects the z component onto a
 *
 *
 * sphere.
=============
*/
void idVec3::ProjectSelfOntoSphere(float const radius) {
  float rsqr = radius * radius;
  float len  = Length();
  if(len < rsqr * 0.5f) {
    z = sqrt(rsqr - len);
  } else {
    z = rsqr / (2.0f * sqrt(len));
  }
}
