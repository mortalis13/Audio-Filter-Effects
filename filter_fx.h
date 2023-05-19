#ifndef FILTER_FX_H
#define FILTER_FX_H

#include <string.h>
#include <stdint.h>
#include <math.h>


const double kSmallestPositiveFloatValue = 1.175494351e-38;         /* min positive value */
const double kSmallestNegativeFloatValue = -1.175494351e-38;        /* min negative value */
const double kPi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899;


enum class FilterAlgorithm {
  kLPF1, kHPF1,
  kLPF2, kHPF2,
  kBPF2, kBSF2,
  kLowShelf, kHiShelf,
  kNCQParaEQ, kCQParaEQ
};


inline bool checkFloatUnderflow(double& value) {
  bool retValue = false;
  if (value > 0.0 && value < kSmallestPositiveFloatValue) {
    value = 0;
    retValue = true;
  }
  else if (value < 0.0 && value > kSmallestNegativeFloatValue) {
    value = 0;
    retValue = true;
  }
  return retValue;
}


class AudioFilter {
public:
  AudioFilter() {}
  ~AudioFilter() {}

  void reset() {
    resetStates();
  }

  void setSampleRate(double sampleRate) {
    this->sampleRate = sampleRate;
    calculateFilterCoeffs();
  }
  
  void setFilterType(FilterAlgorithm type) {
    this->algorithm = type;
    calculateFilterCoeffs();
  }
  
  void setCornerFrequency(double frequency) {
    this->fc = frequency;
    calculateFilterCoeffs();
  }
  
  void setQualityFactor(double qFactor) {
    switch (algorithm) {
      case FilterAlgorithm::kLPF2:
      case FilterAlgorithm::kHPF2:
      case FilterAlgorithm::kBPF2:
      case FilterAlgorithm::kBSF2:
      case FilterAlgorithm::kNCQParaEQ:
      case FilterAlgorithm::kCQParaEQ:
        this->Q = qFactor;
        calculateFilterCoeffs();
    }
  }
  
  void setDb(double db) {
    switch (algorithm) {
      case FilterAlgorithm::kLowShelf:
      case FilterAlgorithm::kHiShelf:
      case FilterAlgorithm::kNCQParaEQ:
      case FilterAlgorithm::kCQParaEQ:
        this->db = db;
        calculateFilterCoeffs();
    }
  }
  
  double processAudioSample(double xn);

protected:
  bool calculateFilterCoeffs();
  
  void resetCoeffs() {
    cf_a0 = cf_a1 = cf_a2 = cf_b1 = cf_b2 = cf_c0 = cf_d0 = 0.0;
  }
  
  void resetStates() {
    x_z1 = x_z2 = y_z1 = y_z2 = 0.0;
  }

private:
  double cf_a0 = 0.0;
  double cf_a1 = 0.0;
  double cf_a2 = 0.0;
  double cf_b1 = 0.0;
  double cf_b2 = 0.0;

  double cf_c0 = 0.0;
  double cf_d0 = 0.0;
  
  double x_z1 = 0.0;
  double x_z2 = 0.0;
  double y_z1 = 0.0;
  double y_z2 = 0.0;
  
  double sampleRate = 44100.0;
  
  FilterAlgorithm algorithm;

  double fc = 100.0;
  double Q = 0.707;
  double db = 0.0;
};

#endif //FILTER_FX_H
