#pragma once

#include <gtest/gtest.h>

#include "exploration/winding/library_winding.hpp"

static constexpr size_t EVEN_COUNT = 128;
static constexpr size_t ODD_COUNT  = (EVEN_COUNT * 2) - 1;

static float const MAXIMUM_ERROR = 0.01f;

template <class T>
class Plane2DFromPoints_Implementation : public testing::Test { };

TYPED_TEST_SUITE_P(Plane2DFromPoints_Implementation);

template <size_t ElementCount, typename TypeParam, bool Normalized>
void IsUndefinedPointsCorrect() {
  static constexpr float EXPECTED_X = 0.0f;
  static constexpr float EXPECTED_Y = 0.0f;
  static constexpr float EXPECTED_Z = 0.0f;

  float LHS_X[ElementCount] = { 0.0f };
  float LHS_Y[ElementCount] = { 0.0f };

  float RHS_X[ElementCount] = { 0.0f };
  float RHS_Y[ElementCount] = { 0.0f };

  float PlaneX[ElementCount] = { 0.0f };
  float PlaneY[ElementCount] = { 0.0f };
  float PlaneZ[ElementCount] = { 0.0f };

  TypeParam::Compute(
    PlaneX, PlaneY, PlaneZ, ElementCount, LHS_X, LHS_Y, RHS_X, RHS_Y, Normalized
  );

  for(int32_t Index = 0; Index < ElementCount; Index++) {
    EXPECT_NEAR(EXPECTED_X, PlaneX[Index], MAXIMUM_ERROR);
    EXPECT_NEAR(EXPECTED_Y, PlaneY[Index], MAXIMUM_ERROR);
    EXPECT_NEAR(EXPECTED_Z, PlaneZ[Index], MAXIMUM_ERROR);
  }
}

TYPED_TEST_P(
  Plane2DFromPoints_Implementation,
  IsUndefinedPointsCorrectWhenUnnormalized_Even
) {
  IsUndefinedPointsCorrect<EVEN_COUNT, TypeParam, false>();
}

TYPED_TEST_P(
  Plane2DFromPoints_Implementation, IsUndefinedPointsCorrectWhenUnnormalized_Odd
) {
  IsUndefinedPointsCorrect<ODD_COUNT, TypeParam, false>();
}

TYPED_TEST_P(
  Plane2DFromPoints_Implementation, IsUndefinedPointsCorrectWhenNormalized_Even
) {
  IsUndefinedPointsCorrect<EVEN_COUNT, TypeParam, true>();
}

TYPED_TEST_P(
  Plane2DFromPoints_Implementation, IsUndefinedPointsCorrectWhenNormalized_Odd
) {
  IsUndefinedPointsCorrect<ODD_COUNT, TypeParam, true>();
}

REGISTER_TYPED_TEST_SUITE_P(
  Plane2DFromPoints_Implementation,
  IsUndefinedPointsCorrectWhenUnnormalized_Even,
  IsUndefinedPointsCorrectWhenUnnormalized_Odd,
  IsUndefinedPointsCorrectWhenNormalized_Even,
  IsUndefinedPointsCorrectWhenNormalized_Odd
);

template <class T>
class Plane2DFromPoints_WithinError : public testing::Test { };

TYPED_TEST_SUITE_P(Plane2DFromPoints_WithinError);

inline float ToRadians(float Degrees) {
  return ((Degrees) *M_PI / 180.0);
}

template <typename TypeParam, bool Normalized>
void HasCorrectErrorOnCirlce() {
  static constexpr size_t ELEMENT_COUNT   = 360;
  static constexpr float  CONSTANT_OFFSET = 100.0f;

  float LHS_X[ELEMENT_COUNT] = { 0.0f };
  float LHS_Y[ELEMENT_COUNT] = { 0.0f };

  float RHS_X[ELEMENT_COUNT] = { 0.0f };
  float RHS_Y[ELEMENT_COUNT] = { 0.0f };

  for(int32_t Index = 0; Index < ELEMENT_COUNT; Index++) {
    float IndexLHS = ToRadians(static_cast<float>(Index) + 0.0f);

    LHS_X[Index] = sinf(IndexLHS) + CONSTANT_OFFSET;
    LHS_Y[Index] = cosf(IndexLHS) + CONSTANT_OFFSET;

    float IndexRHS = ToRadians(static_cast<float>(Index) + 180.0f);

    RHS_X[Index] = sinf(IndexRHS) + CONSTANT_OFFSET;
    RHS_Y[Index] = cosf(IndexRHS) + CONSTANT_OFFSET;
  }

  float PlaneX_Generic[ELEMENT_COUNT] = { 0.0f };
  float PlaneY_Generic[ELEMENT_COUNT] = { 0.0f };
  float PlaneZ_Generic[ELEMENT_COUNT] = { 0.0f };

  float PlaneX_Type[ELEMENT_COUNT] = { 0.0f };
  float PlaneY_Type[ELEMENT_COUNT] = { 0.0f };
  float PlaneZ_Type[ELEMENT_COUNT] = { 0.0f };

  Plane2DFromPoints_Generic(
    PlaneX_Generic,
    PlaneY_Generic,
    PlaneZ_Generic,
    ELEMENT_COUNT,
    LHS_X,
    LHS_Y,
    RHS_X,
    RHS_Y,
    Normalized
  );

  TypeParam::Compute(
    PlaneX_Type,
    PlaneY_Type,
    PlaneZ_Type,
    ELEMENT_COUNT,
    LHS_X,
    LHS_Y,
    RHS_X,
    RHS_Y,
    Normalized
  );

  for(int32_t Index = 0; Index < ELEMENT_COUNT; Index++) {
    EXPECT_NEAR(PlaneX_Type[Index], PlaneX_Generic[Index], MAXIMUM_ERROR);
    EXPECT_NEAR(PlaneY_Type[Index], PlaneY_Generic[Index], MAXIMUM_ERROR);
    EXPECT_NEAR(PlaneZ_Type[Index], PlaneZ_Generic[Index], MAXIMUM_ERROR);
  }
}

TYPED_TEST_P(
  Plane2DFromPoints_WithinError, HasCorrectErrorOnCircle_Unnormalized
) {
  HasCorrectErrorOnCirlce<TypeParam, false>();
}

TYPED_TEST_P(
  Plane2DFromPoints_WithinError, HasCorrectErrorOnCircle_Normalized
) {
  HasCorrectErrorOnCirlce<TypeParam, true>();
}

REGISTER_TYPED_TEST_SUITE_P(
  Plane2DFromPoints_WithinError,
  HasCorrectErrorOnCircle_Unnormalized,
  HasCorrectErrorOnCircle_Normalized
);
