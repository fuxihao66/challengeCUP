#ifndef UTILITY_H
#define UTILITY_H
#include <QtMath>
#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <string>
#include <QMatrix>
#include <QMatrix4x4>

#include <QOpenGLTexture>
#include <algorithm>
#define INFIDESITIMAL (0.0001)
using std::min;
using std::max;
using std::string;

inline string QS2S(QString& qs){
    return qs.toLocal8Bit().constData();
}
inline QString S2QS(string& s){
    return QString::fromStdString(s);
}

struct Vertex{
    QVector3D Position;
    QVector3D Normal;
};

#endif // UTILITY_H
