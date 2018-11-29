#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QPushButton>
#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <QLayout>
#include <QTextCodec>
#include <QLabel>
#include <pcl/io/pcd_io.h>
#include <memory>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/surface/gp3.h>
#include<pcl/io/ply_io.h>
#include "Client.h"

#include <winsock2.h> 
#pragma comment(lib,"ws2_32.lib")  

typedef pcl::PointXYZRGBA PointType;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
	void startVisualizer();
	void startSaving();
	void reconstruct();
	void connectToHost();
private:
    Ui::MainWindow *ui;
	

	QPushButton * visBtn;
	QPushButton * saveBtn;
	QPushButton * reconsBtn;
	QPushButton * connectBtn;
	QLabel * label;
	Client * client;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<PointType>::Ptr cloud;
	pcl::PointCloud<PointType>::ConstPtr cloudRev;

	int numOfKinect = 3;

	boost::mutex mutex;
	boost::signals2::signal<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveSignal;
};

#endif // MAINWINDOW_H
