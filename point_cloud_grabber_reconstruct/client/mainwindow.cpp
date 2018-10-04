#include "mainwindow.h"
#include "build\ui_mainwindow.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "kinect2_grabber.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
	resize(400,200);

	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveFunction =
		[this](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		this->mutex.unlock();
		boost::mutex::scoped_lock lock(this->mutex);
		pcl::io::savePCDFile("Data/resultFile.pcd", *cloud);
		label->setText("Save successfully");
	};
	
	boost::function<void()> cloudSync =
		[this]() {
		boost::mutex::scoped_lock lock(this->mutex);

		std::cout << "sync start" << std::endl;

		const char * sendData = "来自客户端的数据包.\n";
		/*  第一次请求  获取总数据包长度  */
		sendto(client->sclient, sendData, strlen(sendData), 0, (sockaddr *)&(client->sin), client->len);

		char bufferSizeRaw[255];
		unsigned int bufferSize;
		int ret = recvfrom(client->sclient, bufferSizeRaw, 255, 0, (sockaddr *)&(client->sin), &client->len);
		if (ret > 0) {
			bufferSize = bytesToInt(bufferSizeRaw);
			std::cout << "total size is " << bufferSize << std::endl;
		}

		pcl::PointCloud<PointType>::Ptr cloudTemp(new pcl::PointCloud<PointType>);
		cloudTemp->width = bufferSize / (12);  // 6*sizeof(short)
		cloudTemp->height = 1;
		cloudTemp->points.resize(cloudTemp->width*cloudTemp->height);

		/* 第二次请求  获取所有数据包 */
		sendto(client->sclient, sendData, strlen(sendData), 0, (sockaddr *)&(client->sin), client->len);

		char recvRaw[60010];
		short recvData[32020];
		//short * recvData = new short[bufferSize/2];
		unsigned int leftBytes = bufferSize;
		
		unsigned int headPointer = 0;
		unsigned int offset = 0;


		/* 增加接收缓冲区的大小 防止丢包*/
		/*  TODO: 需要增加timeout*/
		int recvBufSize = 1300000;
		int iRet = setsockopt(client->sclient,
			SOL_SOCKET,
			SO_RCVBUF,
			(const char *)&recvBufSize,
			sizeof(recvBufSize));
		while (leftBytes > 0) {
			int ret = recvfrom(client->sclient, recvRaw, 60000, 0, (sockaddr *)&(client->sin), &client->len);
			
			memcpy(recvData, recvRaw, ret);
			leftBytes -= ret;
			offset += (ret / 2);

			int numPoints = ret / (12);
			std::cout << "ret size is " << ret << std::endl;

			for (size_t i = 0; i < numPoints; i++) {
				cloudTemp->points[headPointer].x = recvData[i * 6 + 0] / 500.0;
				cloudTemp->points[headPointer].y = recvData[i * 6 + 1] / 500.0;
				cloudTemp->points[headPointer].z = recvData[i * 6 + 2] / 500.0;
				cloudTemp->points[headPointer].r = recvData[i * 6 + 3];
				cloudTemp->points[headPointer].g = recvData[i * 6 + 4];
				cloudTemp->points[headPointer].b = recvData[i * 6 + 5];
				headPointer++;
			}

			
			//std::cout << "leftBytes is " << leftBytes << std::endl;
		}


		std::cout << "one phase ended " << std::endl;
		cloudRev = cloudTemp->makeShared();
		std::cout << cloudRev->width;
	};

	saveSignal.connect(saveFunction);

	QTextCodec *codec = QTextCodec::codecForName("UTF-8");//情况2
	QTextCodec::setCodecForLocale(codec);
	

	visBtn = new QPushButton(tr("Visualization"));
	saveBtn = new QPushButton(tr("Saving"));
	reconsBtn = new QPushButton(tr("Reconstruction"));
	connectBtn = new QPushButton("connect to host");
	QVBoxLayout * mainLayout = new QVBoxLayout;
	label = new QLabel("Idle");
	label->setMargin(0);
	label->setAlignment(Qt::AlignCenter);
	mainLayout->addWidget(visBtn);
	mainLayout->addWidget(saveBtn);
	mainLayout->addWidget(reconsBtn);
	mainLayout->addWidget(connectBtn);
	mainLayout->addWidget(label);

	client = new Client();
	client->registerCallback(cloudSync);


	connect(visBtn, SIGNAL(clicked()), this, SLOT(startVisualizer()));
	connect(saveBtn, SIGNAL(clicked()), this, SLOT(startSaving()));
	connect(connectBtn, SIGNAL(clicked()), this, SLOT(connectToHost()));
	connect(reconsBtn, SIGNAL(clicked()), this, SLOT(reconstruct()));

	QWidget *widget = new QWidget();
	widget->setLayout(mainLayout);
	setCentralWidget(widget);
}

void MainWindow::startSaving() {
	if (cloud != nullptr) {
		mutex.unlock();
		label->setText("Begin to save");
		saveSignal(cloud);
		viewer->close();
	}
}
void MainWindow::startVisualizer() {
	
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[this](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(this->mutex);

		this->cloud = ptr->makeShared();
		if (cloudRev)
			cloud->operator+=(*cloudRev);
	};
	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();

	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		if (lock.owns_lock() && cloud) {
			// Update Point Cloud

			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");
				std::cout << "has been called" << std::endl;
			}
		}
	}

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}
	
}

void MainWindow::reconstruct() {

	std::cout << "Begin to reconstruct" << std::endl;
	std::string fileName = "Data/resultFile.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReconstruct(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(fileName, *cloudReconstruct) == 1) return;

	std::cout << "file loaded success" << std::endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudReconstruct);
	n.setInputCloud(cloudReconstruct);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	std::cout << "before compute" << std::endl;
	n.compute(*normals);
	std::cout << "normal computation success" << std::endl;

	std::cout << "before concate" << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	concatenateFields(*cloudReconstruct, *normals, *cloud_with_normals);
	std::cout << "normals have been processed" << std::endl;


	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

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

	pcl::io::savePLYFile("Data/resultMesh.ply", triangles);

	cout << "has been successfully saved";
	
	label->setText("Reconstruction succeeds");
}

void MainWindow::connectToHost() {
	client->start();
}
MainWindow::~MainWindow()
{
    delete ui;
}
