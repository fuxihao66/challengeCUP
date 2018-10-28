#ifndef INSPECTOR_H
#define INSPECTOR_H
#include <QWidget>
#include <QObject>
#include <QPushButton>
#include <QLabel>
#include <QSplitter>
#include <QFrame>
#include <QTextEdit>
#include <QLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QComboBox>
#include <QStringList>
#include <QVector4D>
class Inspector : public QWidget{
    Q_OBJECT
public:
    Inspector(QWidget * parent = 0);
    ~Inspector();
signals:
    void modeChanged(int mode);
    void bgColorChanged(QVector4D& color);
    void pointColorChanged(QVector4D& color);
public slots:
    void indexToModeIndex(int modeIndex);
    void indexToBgColor(int bgIndex);
    void indexToPointColor(int pIndex);
private:
    QLabel * modeLabel;
    QLabel * bgLabel;
    QLabel * modelLabel;
    QComboBox * mode;
    QComboBox * bgColor;
    QComboBox * modelColor;
    QStringList colorList;

    QVector<QVector4D> colors;
};

#endif // INSPECTOR_H
