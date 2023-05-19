#ifndef FILTER_FX_H
#define FILTER_FX_H

#include <stdint.h>
#include <math.h>


const double kSmallestPositiveFloatValue = 1.175494351e-38;         /* min positive value */
const double kSmallestNegativeFloatValue = -1.175494351e-38;        /* min negative value */
const double kPi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899;


enum filterCoeff { a0, a1, a2, b1, b2, numCoeffs };
enum stateReg { x_z1, x_z2, y_z1, y_z2, numStates };
enum class filterAlgorithm { kLPF1, kHPF1, kLPF2, kHPF2, kBPF2, kBSF2, kLowShelf, kHiShelf, kCQParaEQ };


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


class IAudioSignalProcessor {
public:
  virtual bool reset(double _sampleRate) = 0;
  virtual void setSampleRate(double _sampleRate) {}
  virtual double processAudioSample(double xn) = 0;
};


class Biquad : public IAudioSignalProcessor {
public:
  Biquad() {}
  ~Biquad() {}

  virtual bool reset(double _sampleRate) {
    memset(&stateArray[0], 0, sizeof(double) * numStates);
    return true;
  }
  
  virtual double processAudioSample(double xn);

  void setCoefficients(double* coeffs) {
    memcpy(&coeffArray[0], &coeffs[0], sizeof(double) * numCoeffs);
  }

protected:
  double coeffArray[numCoeffs] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  double stateArray[numStates] = { 0.0, 0.0, 0.0, 0.0 };
};


class AudioFilter : public IAudioSignalProcessor {
public:
  AudioFilter() {}
  ~AudioFilter() {}

  virtual bool reset(double _sampleRate) {
    sampleRate = _sampleRate;
    return biquad.reset(_sampleRate);
  }

  virtual void setSampleRate(double _sampleRate) {
    sampleRate = _sampleRate;
    calculateFilterCoeffs();
  }
  
  virtual double processAudioSample(double xn);

protected:
  Biquad biquad;
  double coeffArray[numCoeffs] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  
  double sampleRate = 44100.0;
  
  filterAlgorithm algorithm = filterAlgorithm::kLPF1;
  double fc = 100.0;
  double Q = 0.707;
  double boostCut_dB = 0.0;

  bool calculateFilterCoeffs();
};

#endif //FILTER_FX_H
