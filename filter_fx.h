#ifndef FILTER_FX_H
#define FILTER_FX_H

#include <stdint.h>
#include <math.h>


const double kSmallestPositiveFloatValue = 1.175494351e-38;         /* min positive value */
const double kSmallestNegativeFloatValue = -1.175494351e-38;        /* min negative value */
const double kPi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899;


enum class filterAlgorithm { kLPF1, kHPF1, kLPF2, kHPF2, kBPF2, kBSF2, kLowShelf, kHiShelf, kNCQParaEQ, kCQParaEQ };


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

  bool reset(double _sampleRate) {
    sampleRate = _sampleRate;
    resetStates();
    return true;
  }

  void setSampleRate(double _sampleRate) {
    sampleRate = _sampleRate;
    calculateFilterCoeffs();
  }
  
  double processAudioSample(double xn);

protected:
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
  
  filterAlgorithm algorithm = filterAlgorithm::kLPF1;
  double fc = 100.0;
  double Q = 0.707;
  double boostCut_dB = 0.0;

  bool calculateFilterCoeffs();
  
  void resetCoeffs() {
    cf_a0 = cf_a1 = cf_a2 = cf_b1 = cf_b2 = cf_c0 = cf_d0 = 0.0;
  }
  
  void resetStates() {
    x_z1 = x_z2 = y_z1 = y_z2 = 0.0;
  }
};

#endif //FILTER_FX_H
