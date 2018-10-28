#ifndef CAMERA_H
#define CAMERA_H
#include <QtMath>
#include <QMatrix>
#include <QMatrix4x4>
#include <QVector>
#include <QPoint>
class Camera
{
public:
    Camera();
    void resetCam();
    void setCamFront(QVector3D& newFront);
    void rotateCam(float pitch, float yaw);
    QVector3D getDirToPoint(QVector3D& origin);
    void moveLeft();
    void moveRight();
    void moveForward();
    void moveBackward();
    QMatrix4x4 genViewMatrix();
private:
    QVector3D frontVec;
    QVector3D camPos;
    QVector3D upVector;
    float radius = 10.0;
    float camSpeed = 0.2;
};

#endif // CAMERA_H
