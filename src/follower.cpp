#include <iostream>
#include <C:\Users\aswer\Documents\Lab\path\Eigen\Eigen\Dense>
using namespace std;
using namespace Eigen;

struct state
{
    double LateralError;
    double LateralErrorDot;
    double thetaError;
    double thetaErrorDot;
};

class DynamicModel
{
  private:
    const double m = 4.76;
    const double lf = 0.35;
    const double lr = 0.35;
    const double cf = 150;
    const double cr = 150;
    const double Iz = 0.0687;
    const double Vx = 5;
    const double a = -(cf * +cr) / (m * Vx);
    const double b = (cf + cr) / (m);
    const double c = ((lr * cr) - (lf * cf)) / (m * Vx);
    const double d = ((lr * cr) - (lf * cf)) / (Iz * Vx);
    const double e = ((lf * cf) - (lr * cr)) / (Iz);
    const double f = -(((lf * lf) * cf) + ((lr * lr) * cr)) / (Iz * Vx);
    const double g = (cf / m);
    const double h = (lf * cf) / Iz;
    const double i = (((lr * cr) - (lf * cf)) / (m * Vx)) - (Vx);
    const double j = -(((lf * lf) * cf) + ((lr * lr) * cr)) / (Iz * Vx);
    double Ks = 0;
    double rDes = Vx * Ks;
    double U = 0;
    MatrixXd A = MatrixXd::Zero(4, 4);
    MatrixXd B = MatrixXd::Zero(4, 1);
    MatrixXd C = MatrixXd::Zero(4, 1);
    MatrixXd Xdot = MatrixXd::Zero(4, 1);
    MatrixXd Xprev = MatrixXd::Zero(4, 1);
    MatrixXd K = MatrixXd::Zero(4, 1);
    MatrixXd X = MatrixXd::Zero(4, 1);
    state currentState = {0, 0, 0, 0};

  public:
    DynamicModel();
    void updateState(double);
    state getState();
};
DynamicModel::DynamicModel()
{
    A << 0.0, 1.0, 0.0, 0.0,
        0.0, a, b, c,
        0.0, 0.0, 0.0, 1.0,
        0.0, d, e, f;

    B << 0,
        g,
        0,
        h;

    C << 0,
        i,
        0,
        j;
    Xprev << 0,
        0,
        0,
        0;

    X << 0,
        0,
        0,
        0;

    Xdot << 0,
        0,
        0,
        0;

    K << 1.000000000000002,
        0.070921085243448,
        0.895791151436776,
        0.007988442727476;
}
void DynamicModel::updateState(double)
{
    Eigen::MatrixXd steering(4, 1);
    steering = K.cwiseProduct(X);
    U = steering(0, 0) + steering(1, 0) + steering(2, 0) + steering(3, 0);
    Xdot = A * X + B * U + C * rDes;
    //Fix
    currentState = {Xdot(0, 0), Xdot(1, 0), Xdot(2, 0), Xdot(3, 0)};
}

state DynamicModel::getState()
{
    return currentState;
}