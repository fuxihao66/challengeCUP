#include "opglwindow.h"

opglWindow::opglWindow(QWidget* parent):
    QOpenGLWidget(parent), model(nullptr), drawMode(0),
    bgColor(0.1,0.6,0.4, 1.0), vertexColor(0.8, 0.1,0.1, 1.0)
{
    resize(1600, 1080);
    setFocusPolicy(Qt::ClickFocus);
    for (int i = 0; i < 1024; i++){
        keys[i] = false;
    }
    zNear =0.1f;
    zFar = 100.0;
    fov = 45.0;
    yaw = -90;
    pitch = 0;
//    colors = QVector<QVector4D>{QVector4D(1.0,0.0,0.0,1.0),QVector4D(0.0,0.0,0.0,1.0),
//            QVector4D(0.0,0.0,1.0,1.0), QVector4D(0.0,1.0,0.0,1.0), QVector4D(1.0,1.0,0.0,1.0)};
}
opglWindow::~opglWindow(){

}

void opglWindow::initShader(){
    QString path = "rec/fshader/";
    QDir directory(path);

    QStringList f_shaders = directory.entryList(QStringList() << "*.glsl" ,QDir::Files);
	qDebug() << f_shaders.size();
    for (auto it = f_shaders.begin(); it != f_shaders.end(); it++){
        qDebug() << "f_shader name is " << (*it);
        QOpenGLShaderProgram* program = new QOpenGLShaderProgram();
        if (!program->addShaderFromSourceFile(QOpenGLShader::Vertex, "rec/vshader/vshader.glsl")){
            qDebug() << __FILE__<<__FUNCTION__<< " add vertex shader file failed.";
            close();
        }
        if (!program->addShaderFromSourceFile(QOpenGLShader::Fragment, path+(*it))){
            qDebug() << __FILE__<<__FUNCTION__<< " add fragment shader file failed.";
            close();
        }
        if (!program->link()){
            qDebug() << __FILE__<<__LINE__<< "program link failed";
            close();
        }
        if (!program->bind()){
            qDebug() << __FILE__<<__LINE__<< "program bind failed";
            close();
        }
        shaderArray.append(program);
    }
    qDebug() << "Shader Added Successful. Total shader amount is " << shaderArray.size();
}
void opglWindow::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0,0,0,1);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    mainCam = new Camera();
    initShader();

    startTimer(1);
    time.start();

}

void opglWindow::paintGL() {
    glClearColor(bgColor.x(), bgColor.y(), bgColor.z(), bgColor.w());
//    glClearColor(0.1f, 0.1f, 0.6f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	checkKey();

    viewMatrix = mainCam->genViewMatrix();

    if (shaderArray.empty()){
        //qDebug() << "No shader loaded";
        return;
    }

    if (model != nullptr){
        QOpenGLShaderProgram* currentProgram = shaderArray[0];
        currentProgram->bind();
        currentProgram->setUniformValue("mvp_matrix", projMatrix*viewMatrix);         // mvp矩阵
        currentProgram->setUniformValue("ver_color", vertexColor);
        model->Draw(currentProgram, drawMode);
        currentProgram->release();
    }

}
void opglWindow::resizeGL(int w, int h) {
    qreal aspect = qreal(w) / qreal(h ? h : 1);
    projMatrix.setToIdentity();
    projMatrix.perspective(fov, aspect, zNear, zFar);
}


void opglWindow::mousePressEvent(QMouseEvent *event)  //
{
    lastCursorPos = event->pos();

    update();
}


void opglWindow::mouseMoveEvent(QMouseEvent *event) // 一旦front快和upvec重合就停住，不能转了
{
    QPointF diff = QPointF(0, 0);

    diff = event->pos() - lastCursorPos;
//    qWarning() << diff;
    lastCursorPos = event->pos();

    qreal sensitivity = 0.05;
    qreal xoffset =  diff.x() * sensitivity;
    qreal yoffset = -diff.y() * sensitivity;
    yaw   += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f) {
        pitch = 89.0f;
    } else  if (pitch < -89.0f) {
        pitch = -89.0f;
    }
    mainCam->rotateCam(pitch, yaw);

    event->accept();
    update();
}
void opglWindow::checkKey()
{

    if (keys[Qt::Key_A]) {
        mainCam->moveLeft();
        qDebug() << "key a, left";
        update();
    }
    if (keys[Qt::Key_D])  {
        mainCam->moveRight();
        qDebug() << "key d, right";
        update();
    }
    if(keys[Qt::Key_W]) {
        mainCam->moveForward();
        qDebug() << "key w, front";
        update();
    }
    if (keys[Qt::Key_S])  {
        mainCam->moveBackward();
        qDebug() << "key s, back";
        update();
    }

}
void opglWindow::keyPressEvent(QKeyEvent *event)
{
    if (0 <= event->key() && event->key() < (int)(sizeof(keys)/ sizeof(keys[0]))) {
        keys[event->key()] = true;
    }

    event->accept();
}
void opglWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (0 <= event->key() && event->key() < (int)(sizeof(keys)/ sizeof(keys[0]))) {
        keys[event->key()] = false;
    }
    event->accept();
}
void opglWindow::timerEvent(QTimerEvent *)
{
    update();
}

void opglWindow::setMode(GLuint mode){
    drawMode = mode;
}

void opglWindow::addModelFromPath(QString &path){
    model = new Model(path);
}

void opglWindow::clearModel(){
    model = nullptr;
}


void opglWindow::changeBGColor(QVector4D& color){
    bgColor = color;
}
void opglWindow::changePointColor(QVector4D& color){
    vertexColor = color;
}
void opglWindow::changeDrawMode(int mode){
    setMode(mode);
}
