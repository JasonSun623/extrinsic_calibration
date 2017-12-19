/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <cstdint>

namespace extrinsic_calibration
{

#define PRINT(a) std::cout << #a << ": " << a << "\n";

#ifndef LOGF
#define LOGF(a,b) std::cout << "[" <<  #a << "] " << b << "\n";
#endif 

#ifndef PI
#define PI 3.14159265
#endif


typedef unsigned char BYTE;

struct Point3f
{
    Point3f()
    {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
    Point3f(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    float x;
    float y;
    float z;
};

struct Point2f
{
    Point2f()
    {
        this->x = 0;
        this->y = 0;
    }
    Point2f(float x, float y)
    {
        this->x = x;
        this->y = y;
    }
    float x;
    float y;
};

struct RGB
{
    BYTE    rgbBlue;
    BYTE    rgbGreen;
    BYTE    rgbRed;
    BYTE    rgbReserved;

    RGB(){}
    RGB(BYTE r, BYTE g, BYTE b): rgbBlue(b), rgbGreen(g), rgbRed(r)
    {

    }
};

typedef struct PointXYZ {
    float x;
    float y;
    float z;
} PointXYZ;

struct PointRGB {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    PointRGB(){};
    PointRGB(uint8_t r,uint8_t g,uint8_t b): r(r), g(g), b(b)
    {
    }
};

typedef struct PointXYZRGB {
    float x;
    float y;
    float z;

    uint8_t r;
    uint8_t g;
    uint8_t b;
} PointXYZRGB;

}

#endif
