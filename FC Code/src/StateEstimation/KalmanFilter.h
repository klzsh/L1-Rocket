#include "datatypes.h"
#include "config.h"
#include "LinearKalmanFilter.h"
#include "Matrix.h"
#include "Mahony.h"


class SixStateLKF: public LinearKalmanFilter{
  public:
    SixStateLKF();
    ~SixStateLKF() = default;

    void initialize() override {};
    Matrix getF(double dt) override;
    Matrix getG(double dt) override;
    Matrix getH() override;
    Matrix getR() override;
    Matrix getQ(double dt) override;
};

