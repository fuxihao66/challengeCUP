#include "inspector.h"

Inspector::Inspector(QWidget* parent):QWidget(parent)
{
    colors = QVector<QVector4D>{QVector4D(0.6, 0.1,0.1, 1.0),QVector4D(0.0,0.0,0.0,1.0),
             QVector4D(0.1,0.3,0.5,1.0), QVector4D(0.1,0.6,0.4, 1.0), QVector4D(0.8,0.8,0.09,1.0)};
    colorList = QStringList(QList<QString>{"red", "black", "blue", "green", "yellow"});

    QGroupBox * modeArea = new QGroupBox(tr("模式选择"));
    QGroupBox * colorArea = new QGroupBox(tr("背景颜色"));
    QGroupBox * modelArea = new QGroupBox(tr("模型颜色"));


    modeLabel = new QLabel("显示模式");
    bgLabel = new QLabel("背景颜色");
    modelLabel = new QLabel("模型颜色");
    mode = new QComboBox();
    bgColor = new QComboBox();
    modelColor = new QComboBox();


	mode->addItem("模型");
    mode->addItem("点云");
    
    bgColor->addItems(colorList);
    modelColor->addItems(colorList);
    bgColor->setCurrentIndex(3);
    modelColor->setCurrentIndex(0);

    QHBoxLayout * modeLayout = new QHBoxLayout;
    modeLayout->addWidget(modeLabel);
    modeLayout->addWidget(mode);
    modeArea->setLayout(modeLayout);

    QHBoxLayout * bgLayout = new QHBoxLayout;
    bgLayout->addWidget(bgLabel);
    bgLayout->addWidget(bgColor);
    colorArea->setLayout(bgLayout);

    QHBoxLayout * modelLayout = new QHBoxLayout;
    modelLayout->addWidget(modelLabel);
    modelLayout->addWidget(modelColor);
    modelArea->setLayout(modelLayout);

    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addWidget(modeArea);
    mainLayout->addWidget(colorArea);
    mainLayout->addWidget(modelArea);

    QObject::connect(mode, SIGNAL(currentIndexChanged(int)), this, SLOT(indexToModeIndex(int)));
	QObject::connect(bgColor, SIGNAL(currentIndexChanged(int)), this, SLOT(indexToBgColor(int)));
	QObject::connect(modelColor, SIGNAL(currentIndexChanged(int)), this, SLOT(indexToPointColor(int)));
    setLayout(mainLayout);
}


Inspector::~Inspector(){

}
void Inspector::indexToModeIndex(int modeIndex){
    modeChanged(modeIndex);

}
void Inspector::indexToBgColor(int bgIndex){
    bgColorChanged(colors[bgIndex]);

}
void Inspector::indexToPointColor(int pIndex){
    pointColorChanged(colors[pIndex]);
}
