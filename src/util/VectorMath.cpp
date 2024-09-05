#include "util/VectorMath.hpp"

cartesian_vector copy(cartesian_vector other) {
    return {other.x, other.y, other.z};
}

double magnitude(cartesian_vector v) {
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}


cartesian_vector add_vectors(cartesian_vector v1, cartesian_vector v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}


cartesian_vector diff_vectors(cartesian_vector v1, cartesian_vector v2) {
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}


cartesian_vector cross_product(cartesian_vector v1, cartesian_vector v2) {
    double x = (v1.y * v2.z) - (v1.z * v2.y);
    double y = -((v1.x * v2.z) - (v1.z * v2.x));
    double z = (v1.x * v2.y) - (v1.y * v2.x);
    return {x, y, z};
}


double dot_product(cartesian_vector v1, cartesian_vector v2) {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}


double angleDiff_rad(double a1, double a2) {
    double diff = a2 - a1;
    if(diff > M_PI) diff -= 2 * M_PI;
    else if(diff < -M_PI) diff += 2 * M_PI;
    
    return fabs(diff);
}


// Rotates a 2D vector by theta radians. Converts to polar
// adds the angle and then converts back
cartesian_vector rotateVector(cartesian_vector v, double dtheta_rad) {
    double r = sqrt(pow(v.x, 2) + pow(v.y, 2));
    double theta_rad = atan2(v.y, v.x) + dtheta_rad;

    cartesian_vector v2 = {r * cos(theta_rad), r * sin(theta_rad), 0};
    return v2;
}

void scale_vector(cartesian_vector target, cartesian_vector max){
    double M = magnitude(max);
    target.x = target.x/M;
    target.y = target.y/M;
}

bool greaterThan(cartesian_vector v1, cartesian_vector v2){
    return magnitude(v1) > magnitude(v2);
}