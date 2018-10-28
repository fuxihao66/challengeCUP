#include "mainwindow.h"
#include "opglwindow.h"
#include "inspector.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QSplitter>
#include <QListView>
#include <QTextEdit>
#include <QFrame>
#include <QGridLayout>
#include <QDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    resize(1920,1080);
    QSplitter *splitter = new QSplitter(this);
    splitter->setStyleSheet("QSplitter:handle{background-color:grey}");
    opglWindow * ol = new opglWindow(this);
    Inspector *inspector = new Inspector(this);

    splitter->addWidget(ol);
    splitter->addWidget(inspector);


    /*QObject::connect(inspector, &Inspector::modeChanged, ol, &opglWindow::changeDrawMode);
    QObject::connect(inspector, &Inspector::bgColorChanged, ol, &opglWindow::changeBGColor);
    QObject::connect(inspector, &Inspector::pointColorChanged, ol, &opglWindow::changePointColor);*/

	QObject::connect(inspector, SIGNAL(modeChanged(int)), ol, SLOT(changeDrawMode(int)));
	QObject::connect(inspector, SIGNAL(bgColorChanged(QVector4D&)), ol, SLOT(changeBGColor(QVector4D&)));
	QObject::connect(inspector, SIGNAL(pointColorChanged(QVector4D&)), ol, SLOT(changePointColor(QVector4D&)));

    setCentralWidget(splitter);


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionOpen_File_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "",  tr("PLY File(*.ply);;Allfile(*.*)"));
    opglWindow* current = findChild<opglWindow*>();
    if (current == nullptr){
        qDebug() << "get og window fail";
    }
    else{
        current->addModelFromPath(fileName);
//        current->addModelFromPath(QS2S(fileName));
    }
}

void MainWindow::on_actionClear_triggered()
{
    opglWindow* current = findChild<opglWindow*>();
    if (current == nullptr){
        qDebug() << "get og window fail";
    }
    else{
        current->clearModel();
    }
}
