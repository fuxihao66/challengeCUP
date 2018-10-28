#ifndef OPGLWINDOW_H
#define OPGLWINDOW_H
#include <qopenglfunctions_4_5_compatibility.h>
#include <QOpenGLWidget>
#include <QObject>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <vector>
#include <QDir>
#include <QMouseEvent>
#include <QPainter>
#include <QtMath>
#include <QTime>
#include <QBasicTimer>
#include "camera.h"
#include "model.h"

class opglWindow : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    opglWindow(QWidget * parent = 0);
    ~opglWindow();
    void setMode(GLuint mode);
    void addModelFromPath(QString& path);
    void clearModel();

public slots:
    void changeBGColor(QVector4D& color);
    void changePointColor(QVector4D& color);
    void changeDrawMode(int mode);
protected:
    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* e) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;
    void checkKey();

    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;

    void initShader();
private:
    Camera* mainCam;
    QTime time;
    QPoint lastCursorPos;
    qreal yaw, pitch;
    GLfloat zNear, zFar, fov;
    bool keys[1024];

    QVector<QOpenGLShaderProgram*> shaderArray;
    QMatrix4x4 viewMatrix;
    QMatrix4x4 projMatrix;
    QMatrix4x4 modelMatrix;

    GLuint drawMode;
    QVector4D bgColor;
    QVector4D vertexColor;

    Model * model;
};

#endif // OPGLWINDOW_H
