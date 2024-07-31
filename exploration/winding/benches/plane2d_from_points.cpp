#include <vector>

#include <celero/Celero.h>

#include "exploration/winding/library_winding.hpp"

CELERO_MAIN

class Plane2DFromPointsFixture : public celero::TestFixture {
  public:
    Plane2DFromPointsFixture()
      : PlaneCount { 0 }
      , StartPointX {}
      , StartPointY {}
      , EndPointX {}
      , EndPointY {}
      , PlaneX {}
      , PlaneY {}
      , PlaneZ {} {
    }

    virtual std::vector<std::shared_ptr<celero::TestFixture::ExperimentValue>>
    getExperimentValues() const override {
      std::vector<std::shared_ptr<celero::TestFixture::ExperimentValue>>
        ExperimentValues {
          std::make_shared<celero::TestFixture::ExperimentValue>(128),
          std::make_shared<celero::TestFixture::ExperimentValue>(256),
          std::make_shared<celero::TestFixture::ExperimentValue>(512),
          std::make_shared<celero::TestFixture::ExperimentValue>(1024)
        };

      return ExperimentValues;
    }

    virtual void
    setUp(celero::TestFixture::ExperimentValue const* ExperimentValue
    ) override {
      PlaneCount = ExperimentValue->Value;

      StartPointX.resize(PlaneCount);
      StartPointY.resize(PlaneCount);
      EndPointX.resize(PlaneCount);
      EndPointY.resize(PlaneCount);
      PlaneX.resize(PlaneCount);
      PlaneY.resize(PlaneCount);
      PlaneZ.resize(PlaneCount);
    }

    virtual void tearDown() override {
      PlaneZ.clear();
      PlaneY.clear();
      PlaneX.clear();
      EndPointY.clear();
      EndPointX.clear();
      StartPointY.clear();
      StartPointX.clear();

      PlaneCount = 0;
    }

  protected:
    int32_t PlaneCount;

    std::vector<float> StartPointX;
    std::vector<float> StartPointY;
    std::vector<float> EndPointX;
    std::vector<float> EndPointY;
    std::vector<float> PlaneX;
    std::vector<float> PlaneY;
    std::vector<float> PlaneZ;
};

static constexpr auto SAMPLE_COUNT    = 100;
static constexpr auto ITERATION_COUNT = 1000;

BASELINE_F(
  Plane2DFromPoints,
  Generic,
  Plane2DFromPointsFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DFromPoints_Generic(
    PlaneX.data(),
    PlaneY.data(),
    PlaneZ.data(),
    PlaneCount,
    StartPointX.data(),
    StartPointY.data(),
    EndPointX.data(),
    EndPointY.data(),
    true
  );

  celero::DoNotOptimizeAway(PlaneX[0] == 0.0f);
}

BENCHMARK_F(
  Plane2DFromPoints,
  Optimized,
  Plane2DFromPointsFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DFromPoints_Optimized(
    PlaneX.data(),
    PlaneY.data(),
    PlaneZ.data(),
    PlaneCount,
    StartPointX.data(),
    StartPointY.data(),
    EndPointX.data(),
    EndPointY.data(),
    true
  );

  celero::DoNotOptimizeAway(PlaneX[0] == 0.0f);
}

#if defined(__SSE__)

BENCHMARK_F(
  Plane2DFromPoints,
  SSE,
  Plane2DFromPointsFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DFromPoints_SSE(
    PlaneX.data(),
    PlaneY.data(),
    PlaneZ.data(),
    PlaneCount,
    StartPointX.data(),
    StartPointY.data(),
    EndPointX.data(),
    EndPointY.data(),
    true
  );

  celero::DoNotOptimizeAway(PlaneX[0] == 0.0f);
}

#endif

#if defined(__AVX__)

BENCHMARK_F(
  Plane2DFromPoints,
  AVX,
  Plane2DFromPointsFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DFromPoints_AVX(
    PlaneX.data(),
    PlaneY.data(),
    PlaneZ.data(),
    PlaneCount,
    StartPointX.data(),
    StartPointY.data(),
    EndPointX.data(),
    EndPointY.data(),
    true
  );

  celero::DoNotOptimizeAway(PlaneX[0] == 0.0f);
}

#endif
