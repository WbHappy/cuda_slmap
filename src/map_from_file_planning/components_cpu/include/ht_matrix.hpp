#ifndef HOMOGENEOUS_TRASFORMATION_MATRIX_HPP_
#define HOMOGENEOUS_TRASFORMATION_MATRIX_HPP_

#include <stdio.h>
#include <cmath>

#define PI 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899

struct Quaternion{
    double x,y,z,w;
};

class HTMatrix{
public:
    double m[4][4];
    struct Quaternion q;

public:
    HTMatrix();

    void print();

    void set(double e00, double e01, double e02, double e03,
             double e10, double e11, double e12, double e13,
             double e20, double e21, double e22, double e23,
             double e30, double e31, double e32, double e33);

    void calcQuat();

};

HTMatrix operator*(const HTMatrix &m1, const HTMatrix &m2);

HTMatrix rotZ(double theta);
HTMatrix transZ(double z);
HTMatrix transX(double x);
HTMatrix rotX(double alfa);

double sqr(double x);

#endif
