#ifndef RAISIM_TERRAIN_GENERATOR_HPP
#define RAISIM_TERRAIN_GENERATOR_HPP

#include <vector>
# include <cstdint>
# include <numeric>
# include <algorithm>
# include <random>

namespace raisim {

class noiseUtils {

 public:
  inline static size_t fastFloor(double f) noexcept { return (f >= 0 ? (int) f : (int) f - 1); }

  inline static size_t fastRound(double f) {
    return (f >= 0) ? (int) (f + double(0.5)) : (int) (f - double(0.5));
  }

  inline static double map(double value, double lower0, double upper0, double lower1, double upper1) {
    return ((value - lower0) / (upper0 - lower0)) * (upper1 - lower1) + lower1;
  }

};

class PerlinNoise {

 private:

  size_t p[512];

  static double Fade(double t) noexcept {
    return t * t * t * (t * (t * 6 - 15) + 10);
  }

  static double Lerp(double t, double a, double b) noexcept {
    return a + t * (b - a);
  }

  static double Grad(size_t hash, double x, double y, double z) noexcept {
    const size_t h = hash & 15;
    const double u = h < 8 ? x : y;
    const double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
  }
 public:
  explicit PerlinNoise(std::uint32_t seed = std::default_random_engine::default_seed) {
    reseed(seed);
  }

  void reseed(std::uint32_t seed) {
    for (size_t i = 0; i < 256; ++i) {
      p[i] = i;
    }

    std::shuffle(std::begin(p), std::begin(p) + 256, std::default_random_engine(seed));

    for (size_t i = 0; i < 256; ++i) {
      p[256 + i] = p[i];
    }
  }

  double sample(double x, double y, double z) const {
    const size_t X = static_cast<size_t>(noiseUtils::fastFloor(x)) & 255;
    const size_t Y = static_cast<size_t>(noiseUtils::fastFloor(y)) & 255;
    const size_t Z = static_cast<size_t>(noiseUtils::fastFloor(z)) & 255;

    x -= noiseUtils::fastFloor(x);
    y -= noiseUtils::fastFloor(y);
    z -= noiseUtils::fastFloor(z);

    const double u = Fade(x);
    const double v = Fade(y);
    const double w = Fade(z);

    const size_t A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
    const size_t B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;

    return Lerp(w, Lerp(v, Lerp(u, Grad(p[AA], x, y, z),
                                Grad(p[BA], x - 1, y, z)),
                        Lerp(u, Grad(p[AB], x, y - 1, z),
                             Grad(p[BB], x - 1, y - 1, z))),
                Lerp(v, Lerp(u, Grad(p[AA + 1], x, y, z - 1),
                             Grad(p[BA + 1], x - 1, y, z - 1)),
                     Lerp(u, Grad(p[AB + 1], x, y - 1, z - 1),
                          Grad(p[BB + 1], x - 1, y - 1, z - 1))));
  }

};

struct TerrainProperties {
  TerrainProperties() = default;
  TerrainProperties(double frequency,
                    double xSize,
                    double ySize,
                    double zScale,
                    size_t xSamples,
                    size_t ySamples,
                    size_t fractalOctaves,
                    double fractalLacunarity,
                    double fractalGain,
                    double stepSize,
                    std::uint32_t seed) :
      frequency(frequency),
      xSize(xSize),
      ySize(ySize),
      zScale(zScale),
      xSamples(xSamples),
      ySamples(ySamples),
      fractalOctaves(fractalOctaves),
      fractalLacunarity(fractalLacunarity),
      fractalGain(fractalGain),
      stepSize(stepSize),
      seed(seed) {
  };

  double frequency = 0.1;
  double xSize = 10.0;
  double ySize = 10.0;
  double zScale = 2.0;
  double fractalLacunarity = 2.0;
  double fractalGain = 0.5;
  double stepSize = 0;

  size_t fractalOctaves = 5;
  size_t xSamples = 100;
  size_t ySamples = 100;
  std::uint32_t seed = std::default_random_engine::default_seed;
};

//https://github.com/Auburns/FastNoise/blob/master/FastNoise.cpp
class TerrainGenerator {
 public:

  static std::vector<double> generatePerlinFractalTerrain(const TerrainProperties& prop) {
    std::vector<double> height_;
    height_.resize(prop.xSamples * prop.ySamples);
    auto bound = calculateFractalBounding(prop);

    double xScale =
        prop.frequency * prop.xSize / prop.xSamples; // freq * stepSize
    double yScale = prop.frequency * prop.ySize / prop.ySamples;

    // sample noise
    for (int y = 0; y < prop.ySamples; y++)
      for (int x = 0; x < prop.xSamples; x++) {
        double x_temp = xScale * x;
        double y_temp = yScale * y;
        double e = singlePerlinFractalNoise_01(prop, bound, x_temp, y_temp, 0.0)* prop.zScale;

        if(prop.stepSize > 0){
          e = (double) noiseUtils::fastFloor(e / prop.stepSize) * prop.stepSize;
        }
        height_[y * prop.xSamples + x] = e;
      }

    return height_;
  }

 private:
  //within [-1.0, 1.0]
  static double singlePerlinFractalNoise(const TerrainProperties& prop, double bound, double x, double y, double z) {
    PerlinNoise perlinNoise;
    double sum = perlinNoise.sample(x, y, z);
    double amp = 1;
    int i = 0;

    while (++i < prop.fractalOctaves) {
      x *= prop.fractalLacunarity;
      y *= prop.fractalLacunarity;
      z *= prop.fractalLacunarity;

      amp *= prop.fractalGain;
      sum += perlinNoise.sample(x, y, z) * amp;
    }
    return sum * bound;
  }

  //within [0.0, 1.0]
  static double singlePerlinFractalNoise_01(const TerrainProperties& prop, double bound, double x, double y, double z) {
    PerlinNoise perlinNoise;
    double sum = perlinNoise.sample(x, y, z);
    double amp = 1;
    int i = 0;

    while (++i < prop.fractalOctaves) {
      x *= prop.fractalLacunarity;
      y *= prop.fractalLacunarity;
      z *= prop.fractalLacunarity;

      amp *= prop.fractalGain;
      sum += perlinNoise.sample(x, y, z) * amp;
    }
    return sum * bound * 0.5 + 0.5;
  }

  static double calculateFractalBounding(const TerrainProperties& prop) {
    double amp = prop.fractalGain;
    double ampFractal = 1.0f;
    for (int i = 1; i < prop.fractalOctaves; i++) {
      ampFractal += amp;
      amp *= prop.fractalGain;
    }
    return 1.0f / ampFractal;
  }

};

} // namespace raisim
#endif //RAISIM_TERRAIN_GENERATOR_HPP