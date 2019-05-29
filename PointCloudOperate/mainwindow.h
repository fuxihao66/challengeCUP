#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QPushButton>
#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <QLayout>
#include <QTextCodec>
#include <vtkRenderWindow.h>
#include <QLabel>
#include <QProgressBar>
#include <QFileDialog>
#include <pcl/io/pcd_io.h>
#include <memory>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/surface/gp3.h>

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
	void reconstruct();
	void VoxelFilter();
	void StatisticalRemoval();
	void Resampling();
	void getPointCloud();
	void saveCloud();
	void savePly();
	void scanPointOnce();
private:
    Ui::MainWindow *ui;
	
	QPushButton * scanOnce;
	QPushButton * viewerBtn;
	QPushButton * reconsBtn;
	QPushButton * getCloudBtn;
	QPushButton * statisBtn;
	QPushButton * filterBtn;
	QPushButton * resampBtn;
	QPushButton * saveCloudBtn;
	QPushButton * savePlyBtn;
	QProgressBar * progressBar;


	// Parameters read from config
	int numClouds = 4;
	float minX = -0.2;
	float maxX = 0.2;
	float minY = -0.6;
	float maxY = 0.6;
	float minZ = -2.0;
	float maxZ = 2.0;
	float statisticalRemovalStd = 0.1;
	float voxelFilterLeafSize = 0.005;
	float resampleRadius = 0.05;


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr representCloud;
	pcl::PolygonMesh triangles;

	int numOfKinect = 3;

	boost::mutex mutex;
	boost::signals2::signal<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveSignal;
};

#endif // MAINWINDOW_H
