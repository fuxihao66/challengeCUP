#include "camera.h"

Camera::Camera()
{
    camPos = QVector3D(0.0,0.0,radius);
    frontVec = QVector3D(0.0,0.0,-1.0);
    upVector = QVector3D(0.0,1.0,0.0);
}

void Camera::setCamFront(QVector3D& newFront){
    frontVec = newFront;
}

QVector3D Camera::getDirToPoint(QVector3D& origin){
    return origin - camPos;
}

void Camera::rotateCam(float pitch, float yaw){ // theta上下  phi左右
    frontVec.setX(cos(qDegreesToRadians(pitch)) * cos( qDegreesToRadians(yaw)));
    frontVec.setY(sin(qDegreesToRadians(pitch)));
    frontVec.setZ(cos(qDegreesToRadians(pitch)) * sin(qDegreesToRadians(yaw)));
    frontVec.normalize();
}
void Camera::moveLeft(){
    camPos -= QVector3D::crossProduct(frontVec, upVector).normalized()*camSpeed;
}
void Camera::moveRight(){
    camPos += QVector3D::crossProduct(frontVec, upVector).normalized()*camSpeed;
}
void Camera::moveForward(){
    camPos += frontVec.normalized()*camSpeed;
}
void Camera::moveBackward(){
    camPos -= frontVec.normalized()*camSpeed;
}
void Camera::resetCam(){
    camPos = QVector3D(0.0,0.0,3.0);
    frontVec = QVector3D(0.0,0.0,-1.0);
    upVector = QVector3D(0.0,1.0,0.0);
}

QMatrix4x4 Camera::genViewMatrix(){
    QMatrix4x4 view;
    view.setToIdentity();
    view.lookAt(camPos, camPos+frontVec, upVector);
    return view;
}
