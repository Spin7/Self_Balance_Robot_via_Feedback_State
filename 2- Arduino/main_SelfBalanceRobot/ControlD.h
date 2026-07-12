#ifndef CONTROLD_H
#define CONTROLD_H

class ControlD {
  public:
    static const int n = 4; // Number of states

    // Constructor
    ControlD();

    // Control method
    float update(float y_medido, float ref);

    // System matrices
    float G[n][n];
    float H[n];
    float K[n];
    float Kep[n];
    float C[n];
    float N;

  private:
    float x_hat[n]; // Estimated state

    float dot(const float* a, const float* b, int len);
};

ControlD::ControlD() {
  // Initialize matrices with MATLAB values
float G_init[n][n] = {
  {1.0000,    0.0100,    0.0001,    0.0000},
  { 0,    0.9972,    0.0170,    0.0001},
  {0,    0.0003,    1.0114,    0.0100},
  {0,    0.0564,    2.2811,    1.0114}
};

  float H_init[n] = {  0.0001,0.0284,-0.0028,-0.5640};
  float K_init[n] = {-3.3441,   -2.7845,  -16.0467,   -1.7897};
  float Kep_init[n] = {    0.4032,6.2152,54.8919,850.9455};
  float C_init[n] = {1.0, 0.0, 0.0, 0.0};

  N = 500; // Adjust if necessary

//float G_init[n][n] = {
  //{1.0000, 0.1000, 0.0102, 0.0003},
  //{0.0003, 1.0000, 0.2413, 0.0102},
  //{0.0060, 0.0002, 2.3680, 0.1424},
  //{0.1424, 0.0060, 32.3549, 2.3680}
//};

//float H_init[n] = {0.0147, 0.2959, 0.0666, -4.1181};
//float K_init[n] = {-0.2622, -0.1958, -22.7089, -1.5069};
//float Kep_init[n] = {0.2*0.9674, 0.2*0.4892, 0.2*4.4952, 0.2*67.8646};
//float C_init[n] = {0.0, 0.0, 1.0, 0.0};

//N = 1000.0;

  // Copy to class attributes
  for (int i = 0; i < n; i++) {
    H[i] = H_init[i];
    K[i] = K_init[i];
    Kep[i] = Kep_init[i];
    C[i] = C_init[i];
    for (int j = 0; j < n; j++) {
      G[i][j] = G_init[i][j];
    }
    x_hat[i] = 0.0; // Initialization
  }
}

float ControlD::dot(const float* a, const float* b, int len) {
  float sum = 0.0;
  for (int i = 0; i < len; i++) sum += a[i] * b[i];
  return sum;
}

float ControlD::update(float y_medido, float ref) {
  // y_hat = C * x_hat
  float y_hat = dot(C, x_hat, n);

  // u = -K * x_hat + N * ref
  float u = -dot(K, x_hat, n) + N * ref;
  u = constrain(u, -200, -200); 

  // x_hat = G * x_hat + H * u + Kep * (y - y_hat)
  float x_hat_next[n] = {0.0};
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      x_hat_next[i] += G[i][j] * x_hat[j];
    }
    x_hat_next[i] += H[i] * u;
    x_hat_next[i] += Kep[i] * (y_medido - y_hat);
  }

  for (int i = 0; i < n; i++) {
    x_hat[i] = x_hat_next[i];
    x_hat[i] = constrain(x_hat[i], -1000, 1000);
  }

  return u;
}

#endif

