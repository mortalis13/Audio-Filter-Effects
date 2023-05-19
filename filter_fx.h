#ifndef FILTER_FX_H
#define FILTER_FX_H

#include <string.h>
#include <stdint.h>
#include <math.h>


const double kSmallestPositiveFloatValue = 1.175494351e-38;         /* min positive value */
const double kSmallestNegativeFloatValue = -1.175494351e-38;        /* min negative value */
const double kPi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899;


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
  
  void setFrequency(double frequency) {
    this->fc = frequency;
    calculateFilterCoeffs();
  }
  
  double processAudioSample(double xn);

protected:
  virtual void calculateFilterCoeffs() = 0;
  
  void resetCoeffs() {
    cf_a0 = cf_a1 = cf_a2 = cf_b1 = cf_b2 = cf_c0 = cf_d0 = 0.0;
  }
  
  void resetStates() {
    x_z1 = x_z2 = y_z1 = y_z2 = 0.0;
  }

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
  
  double fc = 100.0;
};


class LowPassFilter : public AudioFilter {
public:
  void setOrder(int order) {
    if (order != 1 && order != 2) order = 1;
    this->order = order;
    calculateFilterCoeffs();
  }
  void setQFactor(double qFactor) {
    this->Q = qFactor;
    calculateFilterCoeffs();
  }
protected:
  virtual void calculateFilterCoeffs();
private:
  int order = 1;
  double Q = 0.707;
};


class HighPassFilter : public AudioFilter {
public:
  void setOrder(int order) {
    if (order != 1 && order != 2) order = 1;
    this->order = order;
    calculateFilterCoeffs();
  }
  void setQFactor(double qFactor) {
    this->Q = qFactor;
    calculateFilterCoeffs();
  }
protected:
  virtual void calculateFilterCoeffs();
private:
  int order = 1;
  double Q = 0.707;
};


class BandFilter : public AudioFilter {
public:
  enum FilterType { BandPass, BandStop };
  void setType(FilterType type) {
    this->type = type;
    calculateFilterCoeffs();
  }
  void setQFactor(double qFactor) {
    this->Q = qFactor;
    calculateFilterCoeffs();
  }
protected:
  virtual void calculateFilterCoeffs();
private:
  FilterType type = BandPass;
  double Q = 0.707;
};


class ShelfFilter : public AudioFilter {
public:
  enum FilterType { LowShelf, HighShelf };
  void setType(FilterType type) {
    this->type = type;
    calculateFilterCoeffs();
  }
  void setDB(double db) {
    this->db = db;
    calculateFilterCoeffs();
  }
protected:
  virtual void calculateFilterCoeffs();
private:
  FilterType type = LowShelf;
  double db = 0.0;
};


class PeakingFilter : public AudioFilter {
public:
  void setQFactor(double qFactor) {
    this->Q = qFactor;
    calculateFilterCoeffs();
  }
  void setDB(double db) {
    this->db = db;
    calculateFilterCoeffs();
  }
protected:
  virtual void calculateFilterCoeffs();
private:
  double Q = 0.707;
  double db = 0.0;
};

#endif //FILTER_FX_H
