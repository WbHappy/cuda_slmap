#include "../include/ht_matrix.hpp"

HTMatrix::HTMatrix(){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            this->m[i][j] = 0;
        }
    }
}

void HTMatrix::print(){
    printf("\n");
    for(int i = 0; i < 4; i++){
        printf("[ %f\t%f\t%f\t%f ]\n", m[i][0], m[i][1], m[i][2], m[i][3]);
    }
    printf("\n");
}


void HTMatrix::set(double e00, double e01, double e02, double e03,
                   double e10, double e11, double e12, double e13,
                   double e20, double e21, double e22, double e23,
                   double e30, double e31, double e32, double e33){

    m[0][0] = e00;
    m[0][1] = e01;
    m[0][2] = e02;
    m[0][3] = e03;

    m[1][0] = e10;
    m[1][1] = e11;
    m[1][2] = e12;
    m[1][3] = e13;

    m[2][0] = e20;
    m[2][1] = e21;
    m[2][2] = e22;
    m[2][3] = e23;

    m[3][0] = e30;
    m[3][1] = e31;
    m[3][2] = e32;
    m[3][3] = e33;
}

void HTMatrix::calcQuat(){
     float trace = m[0][0] + m[1][1] + m[2][2];
     if( trace > 0 ) {
           float s = 0.5f / sqrtf(trace+ 1.0f);
           q.w = 0.25f / s;
           q.x = ( m[2][1] - m[1][2] ) * s;
           q.y = ( m[0][2] - m[2][0] ) * s;
           q.z = ( m[1][0] - m[0][1] ) * s;
     } else {
           if ( m[0][0] > m[1][1] && m[0][0] > m[2][2] ) {
                 float s = 2.0f * sqrtf( 1.0f + m[0][0] - m[1][1] - m[2][2]);
                 q.w = (m[2][1] - m[1][2] ) / s;
                 q.x = 0.25f * s;
                 q.y = (m[0][1] + m[1][0] ) / s;
                 q.z = (m[0][2] + m[2][0] ) / s;
           } else if (m[1][1] > m[2][2]) {
                 float s = 2.0f * sqrtf( 1.0f + m[1][1] - m[0][0] - m[2][2]);
                 q.w = (m[0][2] - m[2][0] ) / s;
                 q.x = (m[0][1] + m[1][0] ) / s;
                 q.y = 0.25f * s;
                 q.z = (m[1][2] + m[2][1] ) / s;
           } else {
                 float s = 2.0f * sqrtf( 1.0f + m[2][2] - m[0][0] - m[1][1] );
                 q.w = (m[1][0] - m[0][1] ) / s;
                 q.x = (m[0][2] + m[2][0] ) / s;
                 q.y = (m[1][2] + m[2][1] ) / s;
                 q.z = 0.25f * s;
           }
     }
}

HTMatrix operator*(const HTMatrix &m1, const HTMatrix &m2){
    HTMatrix result;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                result.m[i][j] += m1.m[i][k] * m2.m[k][j];
            }
        }
    }

    return result;
}

HTMatrix rotZ(double theta){
    HTMatrix m;
    m.set( cos(theta), -sin(theta), 0, 0,
           sin(theta),  cos(theta), 0, 0,
               0,           0,      1, 0,
               0,           0,      0, 1);
    return m;
}

HTMatrix transZ(double z){
    HTMatrix m;
    m.set(1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, z,
          0, 0, 0, 1);
    return m;
}


HTMatrix transX(double x){
    HTMatrix m;
    m.set(1, 0, 0, x,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1);
    return m;
}

HTMatrix rotX(double alfa){
    HTMatrix m;
    m.set(1,     0,         0,      0,
          0, cos(alfa), -sin(alfa), 0,
          0, sin(alfa),  cos(alfa), 0,
          0,     0,         0,      1);
    return m;
}

double sqr(double x){
    return x*x;
}
