#ifndef MODEL_H
#define MODEL_H
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#include <iterator>
#include "utility.h"
#include "stdlib.h"
#include "stdio.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
class Model : protected QOpenGLFunctions
{
public:
    Model(QString& path);
    ~Model();
    void Draw(QOpenGLShaderProgram* currentProgram, GLuint mode);
private:
    void loadModel(QString& path);
    void setup();
    QVector<Vertex> modelVertices;              // 顶点
    QVector<GLuint> modelIndices;
    QOpenGLBuffer indexBuf;
    QOpenGLBuffer arrayBuf;
};

#endif // MODEL_H
