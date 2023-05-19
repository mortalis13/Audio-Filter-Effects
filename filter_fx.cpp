#include <string.h>

#include "filter_fx.h"


double Biquad::processAudioSample(double xn) {
  double yn = coeffArray[a0] * xn +
        coeffArray[a1] * stateArray[x_z1] + coeffArray[a2] * stateArray[x_z2] -
        coeffArray[b1] * stateArray[y_z1] - coeffArray[b2] * stateArray[y_z2];

  checkFloatUnderflow(yn);

  stateArray[x_z2] = stateArray[x_z1];
  stateArray[x_z1] = xn;

  stateArray[y_z2] = stateArray[y_z1];
  stateArray[y_z1] = yn;

  return yn;
}


double AudioFilter::processAudioSample(double xn) {
  return coeffArray[d0] * xn + coeffArray[c0] * biquad.processAudioSample(xn);
}


bool AudioFilter::calculateFilterCoeffs() {
  memset(&coeffArray[0], 0, sizeof(double) * numCoeffs);

  coeffArray[a0] = 1.0;

  if (algorithm == filterAlgorithm::kLPF1) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double gamma = cos(theta_c) / (1.0 + sin(theta_c));

    coeffArray[a0] = (1.0 - gamma) / 2.0;
    coeffArray[a1] = (1.0 - gamma) / 2.0;
    coeffArray[a2] = 0.0;
    coeffArray[b1] = -gamma;
    coeffArray[b2] = 0.0;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kHPF1) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double gamma = cos(theta_c) / (1.0 + sin(theta_c));

    coeffArray[a0] = (1.0 + gamma) / 2.0;
    coeffArray[a1] = -(1.0 + gamma) / 2.0;
    coeffArray[a2] = 0.0;
    coeffArray[b1] = -gamma;
    coeffArray[b2] = 0.0;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kLPF2) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double d = 1.0 / Q;
    double betaNumerator = 1.0 - ((d / 2.0) * (sin(theta_c)));
    double betaDenominator = 1.0 + ((d / 2.0) * (sin(theta_c)));

    double beta = 0.5 * (betaNumerator / betaDenominator);
    double gamma = (0.5 + beta) * (cos(theta_c));
    double alpha = (0.5 + beta - gamma) / 2.0;

    coeffArray[a0] = alpha;
    coeffArray[a1] = 2.0 * alpha;
    coeffArray[a2] = alpha;
    coeffArray[b1] = -2.0 * gamma;
    coeffArray[b2] = 2.0 * beta;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kHPF2) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double d = 1.0 / Q;

    double betaNumerator = 1.0 - ((d / 2.0) * (sin(theta_c)));
    double betaDenominator = 1.0 + ((d / 2.0) * (sin(theta_c)));

    double beta = 0.5 * (betaNumerator / betaDenominator);
    double gamma = (0.5 + beta) * (cos(theta_c));
    double alpha = (0.5 + beta + gamma) / 2.0;

    coeffArray[a0] = alpha;
    coeffArray[a1] = -2.0 * alpha;
    coeffArray[a2] = alpha;
    coeffArray[b1] = -2.0 * gamma;
    coeffArray[b2] = 2.0 * beta;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kBPF2) {
    double K = tan(kPi * fc / sampleRate);
    double delta = K * K * Q + K + Q;

    coeffArray[a0] = K / delta;;
    coeffArray[a1] = 0.0;
    coeffArray[a2] = -K / delta;
    coeffArray[b1] = 2.0 * Q * (K * K - 1) / delta;
    coeffArray[b2] = (K * K * Q - K + Q) / delta;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kBSF2) {
    double K = tan(kPi * fc / sampleRate);
    double delta = K * K * Q + K + Q;

    coeffArray[a0] = Q * (1 + K * K) / delta;
    coeffArray[a1] = 2.0 * Q * (K * K - 1) / delta;
    coeffArray[a2] = Q * (1 + K * K) / delta;
    coeffArray[b1] = 2.0 * Q * (K * K - 1) / delta;
    coeffArray[b2] = (K * K * Q - K + Q) / delta;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kLowShelf) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double mu = pow(10.0, boostCut_dB / 20.0);

    double beta = 4.0 / (1.0 + mu);
    double delta = beta * tan(theta_c / 2.0);
    double gamma = (1.0 - delta) / (1.0 + delta);

    coeffArray[a0] = (1.0 - gamma) / 2.0;
    coeffArray[a1] = (1.0 - gamma) / 2.0;
    coeffArray[a2] = 0.0;
    coeffArray[b1] = -gamma;
    coeffArray[b2] = 0.0;
    
    coeffArray[c0] = mu - 1.0;
    coeffArray[d0] = 1.0;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kHiShelf) {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double mu = pow(10.0, boostCut_dB / 20.0);

    double beta = (1.0 + mu) / 4.0;
    double delta = beta * tan(theta_c / 2.0);
    double gamma = (1.0 - delta) / (1.0 + delta);

    coeffArray[a0] = (1.0 + gamma) / 2.0;
    coeffArray[a1] = -coeffArray[a0];
    coeffArray[a2] = 0.0;
    coeffArray[b1] = -gamma;
    coeffArray[b2] = 0.0;
    
    coeffArray[c0] = mu - 1.0;
    coeffArray[d0] = 1.0;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kNCQParaEQ)
  {
    double theta_c = 2.0 * kPi * fc / sampleRate;
    double mu = pow(10.0, boostCut_dB / 20.0);

    double tanArg = theta_c / (2.0 * Q);
    if (tanArg >= 0.95 * kPi / 2.0) tanArg = 0.95 * kPi / 2.0;

    double zeta = 4.0 / (1.0 + mu);
    double betaNumerator = 1.0 - zeta * tan(tanArg);
    double betaDenominator = 1.0 + zeta * tan(tanArg);

    double beta = 0.5 * (betaNumerator / betaDenominator);
    double gamma = (0.5 + beta) * (cos(theta_c));
    double alpha = (0.5 - beta);

    coeffArray[a0] = alpha;
    coeffArray[a1] = 0.0;
    coeffArray[a2] = -alpha;
    coeffArray[b1] = -2.0 * gamma;
    coeffArray[b2] = 2.0 * beta;

    coeffArray[c0] = mu - 1.0;
    coeffArray[d0] = 1.0;

    biquad.setCoefficients(coeffArray);
    return true;
  }
  
  else if (algorithm == filterAlgorithm::kCQParaEQ) {
    double K = tan(kPi * fc / sampleRate);
    double Vo = pow(10.0, boostCut_dB / 20.0);

    double d0 = 1.0 + (1.0 / Q) * K + K * K;
    double e0 = 1.0 + (1.0 / (Vo * Q)) * K + K * K;
    double alpha = 1.0 + (Vo / Q) * K + K * K;
    double beta = 2.0 * (K * K - 1.0);
    double gamma = 1.0 - (Vo / Q) * K + K * K;
    double delta = 1.0 - (1.0 / Q) * K + K * K;
    double eta = 1.0 - (1.0 / (Vo * Q)) * K + K * K;
    
    double val_a0, val_a1, val_a2, val_b1, val_b2;
    
    if (boostCut_dB >= 0) {
      val_a0 = alpha / d0;
      val_a1 = beta  / d0;
      val_a2 = gamma / d0;
      val_b1 = beta  / d0;
      val_b2 = delta / d0;
    }
    else {
      val_a0 = d0    / e0;
      val_a1 = beta  / e0;
      val_a2 = delta / e0;
      val_b1 = beta  / e0;
      val_b2 = eta   / e0;
    }

    coeffArray[a0] = val_a0;
    coeffArray[a1] = val_a1;
    coeffArray[a2] = val_a2;
    coeffArray[b1] = val_b1;
    coeffArray[b2] = val_b2;

    biquad.setCoefficients(coeffArray);
    return true;
  }

  return false;
}
