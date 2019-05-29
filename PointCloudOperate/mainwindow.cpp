#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ctime>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include "kinect2_grabber.h"
#include "utility.h"
#include "readConfig.h"

/*
TODO: 
1. 点云是否确实合并了
2. statistical removal没效果？
*/

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
	resize(1920,1080);

	//readConfig("config.xml", numClouds, minX, maxX, minY, maxY, minZ, maxZ, statisticalRemovalStd, voxelFilterLeafSize, resampleRadius);
	

	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveFunction =
		[this](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		this->mutex.unlock();
		boost::mutex::scoped_lock lock(this->mutex);
		pcl::io::savePCDFile("resultFile.pcd", *representCloud);
	};
	representCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	saveSignal.connect(saveFunction);

	QTextCodec *codec = QTextCodec::codecForName("UTF-8");//情况2
	QTextCodec::setCodecForLocale(codec);
	
	scanOnce = new QPushButton(tr("Scan Once"));
	viewerBtn = new QPushButton(tr("Visualize"));
	getCloudBtn = new QPushButton(tr("Get Point Cloud"));
	statisBtn = new QPushButton(tr("Statistical Removal"));
	filterBtn = new QPushButton("Voxel Filter");
	resampBtn = new QPushButton("Resampling");
	reconsBtn = new QPushButton(tr("Reconstruction"));
	saveCloudBtn = new QPushButton(tr("Save Cloud"));
	savePlyBtn = new QPushButton(tr("Save PLY"));
	
	scanOnce->setFont(QFont("Microsoft YaHei", 20, 1));
	viewerBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	getCloudBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	statisBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	filterBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	resampBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	reconsBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	saveCloudBtn->setFont(QFont("Microsoft YaHei", 20, 1));
	savePlyBtn->setFont(QFont("Microsoft YaHei", 20, 1));


	progressBar = new QProgressBar();
	QVBoxLayout * mainLayout = new QVBoxLayout;

	mainLayout->addWidget(viewerBtn);
	mainLayout->addWidget(scanOnce);
	mainLayout->addWidget(getCloudBtn);
	mainLayout->addWidget(statisBtn);
	mainLayout->addWidget(filterBtn);
	mainLayout->addWidget(resampBtn);
	mainLayout->addWidget(reconsBtn);
	mainLayout->addWidget(saveCloudBtn);
	mainLayout->addWidget(savePlyBtn);
	mainLayout->addWidget(progressBar);

	connect(scanOnce, SIGNAL(clicked()), this, SLOT(scanPointOnce()));
	connect(viewerBtn, SIGNAL(clicked()), this, SLOT(startVisualizer()));
	connect(getCloudBtn, SIGNAL(clicked()), this, SLOT(getPointCloud()));
	connect(statisBtn, SIGNAL(clicked()), this, SLOT(StatisticalRemoval()));
	connect(filterBtn, SIGNAL(clicked()), this, SLOT(VoxelFilter()));
	connect(resampBtn, SIGNAL(clicked()), this, SLOT(Resampling()));
	connect(reconsBtn, SIGNAL(clicked()), this, SLOT(reconstruct()));
	connect(saveCloudBtn, SIGNAL(clicked()), this, SLOT(saveCloud()));
	connect(savePlyBtn, SIGNAL(clicked()), this, SLOT(savePly()));

	QWidget *widget = new QWidget();
	widget->setLayout(mainLayout);
	setCentralWidget(widget);
}

void MainWindow::scanPointOnce() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	pcl::PointCloud<PointType>::ConstPtr cloud;

	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);
		cloud = ptr->makeShared();
	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();
	std::cout << "has been started" << std::endl;
	while (cloud == nullptr) {
		std::cout << "cloud has not been initialized yet" << endl;
	}
	std::cout << "Kinect初始化成功" << std::endl;

	pcl::copyPointCloud(*cloud, *representCloud);

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}



	progressBar->setValue(1);

	viewer->addPointCloud(representCloud);

}

void MainWindow::saveCloud() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	QString fileName;
	fileName = QFileDialog::getSaveFileName(this,
		tr("Save File"), "cloud.pcd", tr("Point Cloud File (*.pcd)"));

	if (!fileName.isNull())
	{
		pcl::io::savePCDFile(fileName.toStdString(), *representCloud);
	}
	else {

	}
	
	progressBar->setValue(1);
}
void MainWindow::savePly() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	QString fileName;
	fileName = QFileDialog::getSaveFileName(this,
		tr("Save File"), "mesh.ply", tr("Polygon Mesh File (*.ply)"));

	if (!fileName.isNull())
	{
		pcl::io::savePLYFile(fileName.toStdString(), triangles);
	}
	else {

	}
	
	progressBar->setValue(1);
}

void MainWindow::startVisualizer() {
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	
}
/*
void MainWindow::getPointCloud() {
	progressBar->setRange(0, numClouds*2);
	progressBar->setValue(0);
	pcl::PointCloud<PointType>::ConstPtr cloud;

	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;

	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);
		cloud = ptr->makeShared();
	};

	boost::function<void()> addCloudToArrayFunc =
		[&cloud, &mutex, &cloudQueues, this]() {
		boost::mutex::scoped_lock lock(mutex);
		pcl::PointCloud<PointType>::Ptr ptr;
		ptr = cloud->makeShared();
		cloudQueues.push_back(ptr);

	};



	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);


	// Start Grabber
	grabber->start();
	std::cout << "has been started" << std::endl;
	while (cloud == nullptr) {
		std::cout << "cloud has not been initialized yet" << endl;
	}
	std::cout << "Kinect初始化成功" << std::endl;

	while (true)
	{
		Sleep(2974);
		std::cout << "添加到队列中" << std::endl;
		addCloudToArrayFunc();
		std::stringstream ss;
		ss << cloudQueues.size();
		
		//viewer->addPointCloud(cloudQueues[cloudQueues.size() - 1], ss.str());
		progressBar->setValue(cloudQueues.size());
		if (cloudQueues.size() == numClouds)
			break;
	}

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	// 预处理  去除框外的
	preCutOffClouds(cloudQueues, minX, maxX, minY, maxY, minZ, maxZ);

	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::copyPointCloud(*cloudQueues[0], *testCloud);
	getParameter(testCloud, parameter);

	std::cout << "参数生成成功" << std::endl;

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
		filterWithPlane(cloudQueues[i], cloudFiltered, parameter);
		cloudQueues[i] = cloudFiltered;
		std::cout << cloudFiltered->points.size() << std::endl;
		progressBar->setValue(cloudQueues.size() + i);
	}
	operateClouds(cloudQueues);

	pcl::PointCloud<PointType>::Ptr combined = combineClouds(cloudQueues);
	pcl::copyPointCloud(*combined, *representCloud);

	progressBar->setValue(cloudQueues.size() * 2);

	viewer->addPointCloud(representCloud);
	
	std::cout << representCloud->size() << std::endl;

}

*/

void MainWindow::getPointCloud() {
	progressBar->setRange(0, 33);
	progressBar->setValue(0);

	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;
	for (size_t i = 1; i <= 32; i++) {
		std::stringstream ss;
		ss << i;
		pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile("cloud_data/"+ss.str()+".pcd", *tempCloud);
		cloudQueues.push_back(tempCloud);
		progressBar->setValue(i);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::copyPointCloud(*cloudQueues[0], *testCloud);
	getParameter(testCloud, parameter);

	std::cout << "参数生成成功" << std::endl;

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
		filterWithPlane(cloudQueues[i], cloudFiltered, parameter);
		cloudQueues[i] = cloudFiltered;
		std::cout << cloudFiltered->points.size() << std::endl;
		progressBar->setValue(cloudQueues.size() + i);
	}

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		std::stringstream ss;
		ss << i;
		pcl::io::savePCDFile(ss.str()+"_.pcd", *cloudQueues[i]);
	}

	operateClouds(cloudQueues);

	pcl::PointCloud<PointType>::Ptr combined = combineClouds(cloudQueues);
	pcl::io::savePCDFile("c.pcd", *combined);
	pcl::copyPointCloud(*combined, *representCloud);

	progressBar->setValue(33);

	viewer->addPointCloud(representCloud);
}

void MainWindow::VoxelFilter() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	clock_t start, finish;
	start = clock();//计时开始；

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


	// 创建滤波器对象
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(representCloud);
	sor.setLeafSize(voxelFilterLeafSize, voxelFilterLeafSize, voxelFilterLeafSize);
	sor.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
	//pcl::io::savePCDFile("final_voxel.pcd", *cloud_filtered);

	finish = clock();//计时结束；
	double time;
	time = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("The running time is %f seconds\n", time);
	
	representCloud = cloud_filtered;
	progressBar->setValue(1);
	viewer->updatePointCloud(representCloud);
}
void MainWindow::StatisticalRemoval() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	clock_t start, finish;
	start = clock();//计时开始；

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(representCloud);
	sor.setMeanK(30);
	sor.setStddevMulThresh(statisticalRemovalStd);
	sor.filter(*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	finish = clock();//计时结束；
	double time;
	time = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("The running time is %f seconds\n", time);

	std::cout << representCloud->size() << std::endl;
	std::cout << cloud_filtered->size() << std::endl;
	representCloud =  cloud_filtered;
	progressBar->setValue(1);
	viewer->updatePointCloud(representCloud);

	std::cout << "统计去除结束" << std::endl;
}
void MainWindow::Resampling() {
	progressBar->setRange(0, 1);
	progressBar->setValue(0);
	clock_t start, finish;
	start = clock();//计时开始；


	// 创建一个KD树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//设置参数
	mls.setInputCloud(representCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(resampleRadius);
	// 曲面重建
	mls.process(mls_points);

	//pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal>::PointCloudOut mls_points;
	//pcl::io::loadPCDFile ("D:\\pcl_study\\urban_reconstrution\\b4_mls.pcd", mls_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(mls_points, *copied_cloud);

	finish = clock();//计时结束；
	double time;
	time = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("The running time is %f seconds\n", time);
	representCloud = copied_cloud;
	progressBar->setValue(1);
	viewer->updatePointCloud(representCloud);
}

void MainWindow::reconstruct() {
	progressBar->setRange(0, 10);
	progressBar->setValue(0);
	std::cout << "Begin to reconstruct" << std::endl;

	if (representCloud->points.size() == 0) {

	}

	/*std::string fileName = "Data/resultFile.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReconstruct(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(fileName, *cloudReconstruct) == 1) return;

	std::cout << "file loaded success" << std::endl;*/

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(representCloud);
	n.setInputCloud(representCloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	std::cout << "before compute" << std::endl;
	n.compute(*normals);
	std::cout << "normal computation success" << std::endl;

	progressBar->setValue(2);

	std::cout << "before concate" << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	concatenateFields(*representCloud, *normals, *cloud_with_normals);
	std::cout << "normals have been processed" << std::endl;

	progressBar->setValue(6);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	

	gp3.setSearchRadius(1.5f);
	gp3.setMu(2.5f);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	cout << "begin to reconstruct" << endl;
	gp3.reconstruct(triangles);

	viewer->addPolygonMesh(triangles);
	
	progressBar->setValue(10);
}


MainWindow::~MainWindow()
{
    delete ui;
}
