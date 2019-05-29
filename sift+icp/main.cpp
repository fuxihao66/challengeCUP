#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <time.h>
#include <cmath>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/eigen.h>
#include <pcl/correspondence.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/geometry.h>
#include "kinect2_grabber.h"


typedef pcl::PointXYZRGBA PointType;
typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;
typedef boost::shared_ptr<const Correspondences > CorrespondencesConstPtr;
//using namespace cv;
using namespace cv::xfeatures2d;

/*
TODO:
1. SIFT坐标是否相反；img2Mat是否正确
2. SIFT参数
3. 变换后cloud1空？？  
*/
template <typename PointSource, typename PointTarget, typename Scalar = float>
class WeightedTransformationEstimationSVD : public pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar> {
public:
	typedef typename pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;
	WeightedTransformationEstimationSVD() : pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>() {

	}
	void estimateRigidTransformation(
		const pcl::PointCloud<PointSource> &cloud_src,
		const pcl::PointCloud<PointTarget> &cloud_tgt,
		const pcl::Correspondences &correspondences,
		Matrix4 &transformation_matrix,
		const pcl::PointCloud<PointSource> &cloud_pattern_source,
		const pcl::PointCloud<PointTarget> &cloud_pattern_target) const
	{
		pcl::ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
		pcl::ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
		estimateRigidTransformation(source_it, target_it, transformation_matrix, cloud_pattern_source, cloud_pattern_target);
	}

	void estimateRigidTransformation(
		pcl::ConstCloudIterator<PointSource>& source_it,
		pcl::ConstCloudIterator<PointTarget>& target_it,
		Matrix4 &transformation_matrix,
		const pcl::PointCloud<PointSource> &cloud_pattern_source,
		const pcl::PointCloud<PointTarget> &cloud_pattern_target) const
	{
		// Convert to Eigen format
		const int npts = static_cast <int> (source_it.size());

		if (use_umeyama_)
		{
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts + cloud_pattern_source.size());
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts + cloud_pattern_source.size());


			// calculate weights
			for (int i = 0; i < npts; ++i)
			{
				cloud_src(0, i) = source_it->x;
				cloud_src(1, i) = source_it->y;
				cloud_src(2, i) = source_it->z;
				++source_it;

				cloud_tgt(0, i) = target_it->x;
				cloud_tgt(1, i) = target_it->y;
				cloud_tgt(2, i) = target_it->z;
				++target_it;
			}

			for (int i = npts; i < npts + cloud_pattern_source.size(); i++) {
				cloud_src(0, i) = 10 * cloud_pattern_source.points[i - npts].x;
				cloud_src(0, i) = 10 * cloud_pattern_source.points[i - npts].y;
				cloud_src(0, i) = 10 * cloud_pattern_source.points[i - npts].z;

				cloud_tgt(0, i) = 10 * cloud_pattern_target.points[i - npts].x;
				cloud_tgt(0, i) = 10 * cloud_pattern_target.points[i - npts].y;
				cloud_tgt(0, i) = 10 * cloud_pattern_target.points[i - npts].z;
			}
			// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
			transformation_matrix = pcl::umeyama(cloud_src, cloud_tgt, false);
		}
		else
		{
			source_it.reset(); target_it.reset();
			// <cloud_src,cloud_src> is the source dataset
			transformation_matrix.setIdentity();

			Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
			// Estimate the centroids of source, target
			compute3DCentroid(source_it, centroid_src);
			compute3DCentroid(target_it, centroid_tgt);
			source_it.reset(); target_it.reset();

			// Subtract the centroids from source, target
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
			demeanPointCloud(source_it, centroid_src, cloud_src_demean);
			demeanPointCloud(target_it, centroid_tgt, cloud_tgt_demean);

			getTransformationFromCorrelation(cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
		}
	}
};

template <typename PointSource, typename PointTarget, typename Scalar = float>
class ICPWithPenalty : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
private:
	boost::shared_ptr<WeightedTransformationEstimationSVD<PointSource, PointTarget, Scalar> > weighted_transformation_estimation_;
	boost::shared_ptr<pcl::PointCloud<PointSource>> pcloud_pattern_source;
	boost::shared_ptr<pcl::PointCloud<PointTarget>> pcloud_pattern_target;
public:
	ICPWithPenalty() : IterativeClosestPoint<PointSource, PointTarget, Scalar>() {

		weighted_transformation_estimation_.reset(new WeightedTransformationEstimationSVD<PointSource, PointTarget, Scalar>());
	}

	void setPattern(boost::shared_ptr<pcl::PointCloud<PointSource>> pattern_source,
					boost::shared_ptr<pcl::PointCloud<PointTarget>> pattern_target) {
		pcloud_pattern_source = pattern_source;
		pcloud_pattern_target = pattern_target;
	}


	virtual void computeTransformation(
		PointCloudSource &output, const Matrix4 &guess)
	{
		// Point cloud containing the correspondences of each point in <input, indices>
		PointCloudSourcePtr input_transformed(new PointCloudSource);

		nr_iterations_ = 0;
		converged_ = false;

		// Initialise final transformation to the guessed one
		final_transformation_ = guess;

		// If the guessed transformation is non identity
		if (guess != Matrix4::Identity())
		{
			input_transformed->resize(input_->size());
			// Apply guessed transformation prior to search for neighbours
			transformCloud(*input_, *input_transformed, guess);
		}
		else
			*input_transformed = *input_;

		transformation_ = Matrix4::Identity();

		// Make blobs if necessary
		determineRequiredBlobData();
		pcl::PCLPointCloud2::Ptr target_blob(new pcl::PCLPointCloud2);
		if (need_target_blob_)
			pcl::toPCLPointCloud2(*target_, *target_blob);

		// Pass in the default target for the Correspondence Estimation/Rejection code
		correspondence_estimation_->setInputTarget(target_);
		if (correspondence_estimation_->requiresTargetNormals())
			correspondence_estimation_->setTargetNormals(target_blob);
		// Correspondence Rejectors need a binary blob
		for (size_t i = 0; i < correspondence_rejectors_.size(); ++i)
		{
			pcl::registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
			if (rej->requiresTargetPoints())
				rej->setTargetPoints(target_blob);
			if (rej->requiresTargetNormals() && target_has_normals_)
				rej->setTargetNormals(target_blob);
		}

		convergence_criteria_->setMaximumIterations(max_iterations_);
		convergence_criteria_->setRelativeMSE(euclidean_fitness_epsilon_);
		convergence_criteria_->setTranslationThreshold(transformation_epsilon_);
		convergence_criteria_->setRotationThreshold(1.0 - transformation_epsilon_);

		// Repeat until convergence
		do
		{
			// Get blob data if needed
			pcl::PCLPointCloud2::Ptr input_transformed_blob;
			if (need_source_blob_)
			{
				input_transformed_blob.reset(new pcl::PCLPointCloud2);
				pcl::toPCLPointCloud2(*input_transformed, *input_transformed_blob);
			}
			// Save the previously estimated transformation
			previous_transformation_ = transformation_;

			// Set the source each iteration, to ensure the dirty flag is updated
			correspondence_estimation_->setInputSource(input_transformed);
			if (correspondence_estimation_->requiresSourceNormals())
				correspondence_estimation_->setSourceNormals(input_transformed_blob);
			// Estimate correspondences
			if (use_reciprocal_correspondence_)
				correspondence_estimation_->determineReciprocalCorrespondences(*correspondences_, corr_dist_threshold_);
			else
				correspondence_estimation_->determineCorrespondences(*correspondences_, corr_dist_threshold_);

			//if (correspondence_rejectors_.empty ())
			CorrespondencesPtr temp_correspondences(new Correspondences(*correspondences_));
			for (size_t i = 0; i < correspondence_rejectors_.size(); ++i)
			{
				pcl::registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
				PCL_DEBUG("Applying a correspondence rejector method: %s.\n", rej->getClassName().c_str());
				if (rej->requiresSourcePoints())
					rej->setSourcePoints(input_transformed_blob);
				if (rej->requiresSourceNormals() && source_has_normals_)
					rej->setSourceNormals(input_transformed_blob);
				rej->setInputCorrespondences(temp_correspondences);
				rej->getCorrespondences(*correspondences_);
				// Modify input for the next iteration
				if (i < correspondence_rejectors_.size() - 1)
					*temp_correspondences = *correspondences_;
			}

			size_t cnt = correspondences_->size();
			// Check whether we have enough correspondences
			if (static_cast<int> (cnt) < min_number_correspondences_)
			{
				PCL_ERROR("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName().c_str());
				convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
				converged_ = false;
				break;
			}

			// Estimate the transform
			//transformation_estimation_->estimateRigidTransformation(*input_transformed, *target_, *correspondences_, transformation_);
			weighted_transformation_estimation_->estimateRigidTransformation(*input_transformed, *target_, *correspondences_, transformation_, *pcloud_pattern_source, *pcloud_pattern_target);


			// Tranform the data
			transformCloud(*input_transformed, *input_transformed, transformation_);
			

			// 也要把匹配点进行变换
			pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformCloud(*pcloud_pattern_source, *pcloud_pattern_source, transformation_);



			// Obtain the final transformation    
			final_transformation_ = transformation_ * final_transformation_;

			++nr_iterations_;

			// Update the vizualization of icp convergence
			//if (update_visualizer_ != 0)
			//  update_visualizer_(output, source_indices_good, *target_, target_indices_good );

			converged_ = static_cast<bool> ((*convergence_criteria_));
		} while (!converged_);

		// Transform the input cloud using the final transformation
		PCL_DEBUG("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
			final_transformation_(0, 0), final_transformation_(0, 1), final_transformation_(0, 2), final_transformation_(0, 3),
			final_transformation_(1, 0), final_transformation_(1, 1), final_transformation_(1, 2), final_transformation_(1, 3),
			final_transformation_(2, 0), final_transformation_(2, 1), final_transformation_(2, 2), final_transformation_(2, 3),
			final_transformation_(3, 0), final_transformation_(3, 1), final_transformation_(3, 2), final_transformation_(3, 3));

		// Copy all the values
		output = *input_;
		// Transform the XYZ + normals
		transformCloud(*input_, output, final_transformation_);
	}
};

void myImage2Mat(const myMath::myImage& img, cv::Mat& m) {
	double radius = 0.22*img.height;
	cv::Point2f p(int(img.height/2), int(img.width/2));
	m.create(img.height, img.width, CV_8UC3);
	for (size_t i = 0; i < img.height; i++) {
		for (size_t j = 0; j < img.width; j++) {
			int index = i * img.width + j;
			if ( ((i-p.x)*(i-p.x)+(j-p.y)*(j-p.y)) > radius*radius) {
				m.at<cv::Vec3b>(i, j)[0] = 0;
				m.at<cv::Vec3b>(i, j)[1] = 0;
				m.at<cv::Vec3b>(i, j)[2] = 0;
			}
			else {
				m.at<cv::Vec3b>(i, j)[0] = img.points[index].b;
				m.at<cv::Vec3b>(i, j)[1] = img.points[index].g;
				m.at<cv::Vec3b>(i, j)[2] = img.points[index].r;
			}
			
		}
	}
}

void getCorner(const myMath::myImage& img1, const myMath::myImage& img2, pcl::PointCloud<pcl::PointXYZRGBA>& cloud_pattern1, pcl::PointCloud<pcl::PointXYZRGBA>& cloud_pattern2) {
	cv::Mat img_1;
	cv::Mat img_2;

	myImage2Mat(img1, img_1);
	myImage2Mat(img2, img_2);
	cv::Mat image_gray_1;
	cv::Mat image_gray_2;
	cv::cvtColor(img_1, image_gray_1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(img_2, image_gray_2, cv::COLOR_BGR2GRAY);
	std::vector<cv::Point2f> corners1;
	std::vector<cv::Point2f> corners2;

	bool ret1 = cv::findChessboardCorners(image_gray_1,
		cv::Size(6, 9),
		corners1,
		cv::CALIB_CB_ADAPTIVE_THRESH |
		cv::CALIB_CB_NORMALIZE_IMAGE);

	/*cv::TermCriteria criteria = cv::TermCriteria(
		cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
		40,
		0.1);*/
	//亚像素检测
	//cv::cornerSubPix(image_gray_1, corners1, cv::Size(5, 5), cv::Size(-1, -1), criteria);
	std::cout << ret1;
	cv::drawChessboardCorners(img_1, cv::Size(9, 6), corners1, ret1);


	bool ret2 = cv::findChessboardCorners(image_gray_2,
		cv::Size(9, 6),
		corners2,
		cv::CALIB_CB_ADAPTIVE_THRESH |
		cv::CALIB_CB_NORMALIZE_IMAGE);


	//cv::cornerSubPix(image_gray_2, corners2, cv::Size(5, 5), cv::Size(-1, -1), criteria);
	//角点绘制
	
	//cv::drawChessboardCorners(img_2, cv::Size(9, 6), corners2, ret2);

	
}

void mySIFT(const myMath::myImage& img1, const myMath::myImage& img2, pcl::PointCloud<pcl::PointXYZRGBA>& cloud_pattern1, pcl::PointCloud<pcl::PointXYZRGBA>& cloud_pattern2)
{
	cv::Mat img_1;
	cv::Mat img_2;


	myImage2Mat(img1, img_1);
	myImage2Mat(img2, img_2);


	//Create SIFT class pointer
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(2000);


	double t0 = cv::getTickCount();//当前滴答数
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	f2d->detect(img_1, keypoints_1);
	f2d->detect(img_2, keypoints_2);
	cout << "The keypoints number of img1 is:" << keypoints_1.size() << endl;
	cout << "The keypoints number of img2 is:" << keypoints_2.size() << endl;
	//Calculate descriptors (feature vectors)
	cv::Mat descriptors_1, descriptors_2;
	f2d->compute(img_1, keypoints_1, descriptors_1);
	f2d->compute(img_2, keypoints_2, descriptors_2);
	double freq = cv::getTickFrequency();
	double tt = ((double)cv::getTickCount() - t0) / freq;
	cout << "Extract SIFT Time:" << tt << "ms" << endl;
	//画关键点
	cv::Mat img_keypoints_1, img_keypoints_2;
	drawKeypoints(img_1, keypoints_1, img_keypoints_1, cv::Scalar::all(-1), 0);
	drawKeypoints(img_2, keypoints_2, img_keypoints_2, cv::Scalar::all(-1), 0);


	//Matching descriptor vector using BFMatcher
	cv::BFMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	cout << "The number of match:" << matches.size() << endl;
	//绘制匹配出的关键点
	cv::Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
	//imshow("Match image",img_matches);
	//计算匹配结果中距离最大和距离最小值
	double min_dist = matches[0].distance, max_dist = matches[0].distance;
	for (int m = 0; m < matches.size(); m++)
	{
		if (matches[m].distance < min_dist)
		{
			min_dist = matches[m].distance;
		}
		if (matches[m].distance > max_dist)
		{
			max_dist = matches[m].distance;
		}
	}
	cout << "min dist=" << min_dist << endl;
	cout << "max dist=" << max_dist << endl;
	//筛选出较好的匹配点
	std::vector<cv::DMatch> goodMatches;
	for (int m = 0; m < matches.size(); m++)												//        这一步可以调
	{
		if (matches[m].distance < 0.6*max_dist)
		{
			goodMatches.push_back(matches[m]);
		}
	}
	cout << "The number of good matches:" << goodMatches.size() << endl;
	//画出匹配结果
	cv::Mat img_out;
	//红色连接的是匹配的特征点数，绿色连接的是未匹配的特征点数
	//matchColor – Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
	//singlePointColor – Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
	//CV_RGB(0, 255, 0)存储顺序为R-G-B,表示绿色
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, goodMatches, img_out, cv::Scalar::all(-1), CV_RGB(0, 0, 255), cv::Mat(), 2);
	/*imshow("Before RANSAC", img_out);
	cv::imwrite("match_beforeRANCAC.png", img_out);*/


	//RANSAC匹配过程
	std::vector<cv::DMatch> m_Matches;
	m_Matches = goodMatches;
	int ptCount = goodMatches.size();
	/*if (ptCount < 100)
	{
		cout << "Don't find enough match points" << endl;
		return 0;
	}*/

	//坐标转换为float类型
	std::vector <cv::KeyPoint> RAN_KP1, RAN_KP2;
	//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		RAN_KP1.push_back(keypoints_1[goodMatches[i].queryIdx]);
		RAN_KP2.push_back(keypoints_2[goodMatches[i].trainIdx]);
		//RAN_KP1是要存储img01中能与img02匹配的点
		//goodMatches存储了这些匹配点对的img01和img02的索引值
	}
	//坐标变换
	std::vector <cv::Point2f> p01, p02;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		p01.push_back(RAN_KP1[i].pt);
		p02.push_back(RAN_KP2[i].pt);
	}
	/*vector <Point2f> img1_corners(4);
	img1_corners[0] = Point(0,0);
	img1_corners[1] = Point(img_1.cols,0);
	img1_corners[2] = Point(img_1.cols, img_1.rows);
	img1_corners[3] = Point(0, img_1.rows);
	vector <Point2f> img2_corners(4);*/
	////求转换矩阵
	//Mat m_homography;
	//vector<uchar> m;
	//m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
	//求基础矩阵 Fundamental,3*3的基础矩阵
	std::vector<uchar> RansacStatus;
	cv::Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, cv::FM_RANSAC);
	//重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	std::vector <cv::KeyPoint> RR_KP1, RR_KP2;
	std::vector <cv::DMatch> RR_matches;
	int index = 0;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_KP1.push_back(RAN_KP1[i]);
			RR_KP2.push_back(RAN_KP2[i]);
			m_Matches[i].queryIdx = index;
			m_Matches[i].trainIdx = index;
			RR_matches.push_back(m_Matches[i]);
			index++;
		}
	}
	std::cout << "RANSAC后匹配点数" << RR_matches.size() << std::endl;
	cv::Mat img_RR_matches;

	/*std::cout << RR_KP1[0].pt.x << ' ' << RR_KP1[0].pt.y << std::endl;
	std::cout << RR_KP2[0].pt.x << ' ' << RR_KP2[0].pt.y << std::endl;*/

	std::cout << RR_KP1.size() << std::endl;
	for (size_t i = 0; i < RR_KP1.size(); i++) {
		const myMath::myPoint& mp1 = img1.points[(int)RR_KP1[i].pt.y * img1.width + (int)RR_KP1[i].pt.x];
		const myMath::myPoint& mp2 = img2.points[(int)RR_KP2[i].pt.y * img1.width + (int)RR_KP2[i].pt.x];
		
		if ( (mp1.x == 0.0 && mp1.y == 0.0 && mp1.z == 0.0) || (mp2.x == 0.0 && mp2.y == 0.0 && mp2.z == 0.0) ) {
			continue;
		}


		if ( abs(RR_KP1[i].pt.y - RR_KP2[i].pt.y) <= 1 && abs(RR_KP1[i].pt.x - RR_KP2[i].pt.x) <= 1) {
			continue;
		}

		if (abs(RR_KP1[i].pt.y - RR_KP2[i].pt.y) > 100 && abs(RR_KP1[i].pt.x - RR_KP2[i].pt.x) > 100) {
			continue;
		}
		
		cloud_pattern1.push_back(*img1.points[(int)RR_KP1[i].pt.y * img1.width + (int)RR_KP1[i].pt.x].toPointXYZRGBA());
		cloud_pattern2.push_back(*img2.points[(int)RR_KP2[i].pt.y * img1.width + (int)RR_KP2[i].pt.x].toPointXYZRGBA());
	}

	std::cout << cloud_pattern1.size() << std::endl;

	drawMatches(img_1, RR_KP1, img_2, RR_KP2, RR_matches, img_RR_matches);
	/*imshow("After RANSAC", img_RR_matches);
	
	cv::waitKey(0);*/
}


// tr -> in 的坐标系
int ICPWithNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tr, Eigen::Matrix4d& matrix) {

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> myIcp;
	myIcp.setMaximumIterations(60);

	myIcp.setInputSource(cloud_icp);
	myIcp.setInputTarget(cloud_in);
	myIcp.align(*cloud_icp);
	if (myIcp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << myIcp.getFitnessScore() << std::endl;
		matrix = myIcp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}



	return 0;
}

// tr -> in 的坐标系  
// 使用特征点作为惩罚项（参数尚未调好）
int ICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix, pcl::PointCloud<PointType>::Ptr cloud_p1, pcl::PointCloud<PointType>::Ptr cloud_p2) {

	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	ICPWithPenalty<PointType, PointType> myIcp;
	myIcp.setMaximumIterations(60);
	
	myIcp.setInputSource(cloud_icp);
	myIcp.setInputTarget(cloud_in);
	myIcp.setPattern(cloud_p1, cloud_p2);
	myIcp.align(*cloud_icp);
	if (myIcp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << myIcp.getFitnessScore() << std::endl;
		matrix = myIcp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	

	return 0;
}

// tr -> in 的坐标系
int tempICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix) {

	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	// The Iterative Closest Point algorithm  
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaximumIterations(100);
	
	icp.setTransformationEpsilon(1e-10);
	std::cout << "icp" << std::endl;
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		matrix = icp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	return 0;
}

double getAvgDistance(pcl::PointCloud<PointType>::Ptr cloud1, pcl::PointCloud<PointType>::Ptr cloud2) {
	double avgDis = 0.0;

	for (size_t i = 0; i < cloud1->size(); i++) {
		avgDis += pcl::geometry::distance(cloud1->points[i], cloud2->points[i]);
	}
	avgDis /= cloud1->size();

	return avgDis;
}
/*自适应阈值*/
pcl::PointCloud<PointType>::Ptr statisRemoval(pcl::PointCloud<PointType>::Ptr cloud) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
	sor.setInputCloud(cloud);
	sor.setMeanK(int(0.9*cloud->size()));
	sor.setStddevMulThresh(0.2);// 0.1 和 0.05没区别

	sor.filter(*cloud_filtered);

	return cloud_filtered;
}
int patternICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix, double& score) {
	cloud_in = statisRemoval(cloud_in);
	cloud_tr = statisRemoval(cloud_tr);

	std::cout << "统计去除成功" << std::endl;

	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	// The Iterative Closest Point algorithm  
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaximumIterations(100);
	//icp.setMaxCorrespondenceDistance(0.5*getAvgDistance(cloud_in, cloud_tr));
	icp.setTransformationEpsilon(1e-10);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		score = icp.getFitnessScore();
		matrix = icp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	return 0;
}

void operateClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL - 1);


	for (size_t i = 1; i < numOfPCL; i++) {

		std::cout << "开始计算变换矩阵" << std::endl;

		int hr = tempICP(cloudQueues[i-1], cloudQueues[i], transformVecs[i - 1]);


		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}


	Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
	for (size_t i = 1; i < numOfPCL; i++) {
		currentMatrix = currentMatrix * transformVecs[i - 1];    // ?? 左乘 or 右乘 
		// 变换点云
		pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
		cloudQueues[i] = pPointCloudOut;
	}

}

void operateCloudsWithCorner(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues, std::vector<boost::shared_ptr<myMath::myImage>>& imgQueue) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL - 1);


	for (size_t i = 1; i < numOfPCL; i++) {
		// 开始SIFT求出匹配点
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern1(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		getCorner(*imgQueue[i - 1], *imgQueue[i], *pcloud_pattern1, *pcloud_pattern2);

		pcl::io::savePCDFile("patt1.pcd", *pcloud_pattern1);
		pcl::io::savePCDFile("patt2.pcd", *pcloud_pattern2);

		std::cout << "SIFT完成" << std::endl;

		std::cout << "开始计算变换矩阵" << std::endl;

		int hr = tempICP(pcloud_pattern1, pcloud_pattern2, transformVecs[i - 1]);

		pcl::transformPointCloud(*pcloud_pattern2, *pcloud_pattern2, transformVecs[i - 1]);
		pcl::io::savePCDFile("patt2_m.pcd", *pcloud_pattern2);

		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}


	Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
	for (size_t i = 1; i < numOfPCL; i++) {
		currentMatrix = currentMatrix * transformVecs[i - 1];    // ?? 左乘 or 右乘 
		// 变换点云
		pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
		cloudQueues[i] = pPointCloudOut;
	}

}
void operateCloudsWithGap(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues, std::vector<boost::shared_ptr<myMath::myImage>>& imgQueue) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL - 1);
	std::vector<double> scores(numOfPCL - 1);
	std::vector<double> fromWhich(numOfPCL - 1);  // 表示当前点云是从哪个变来的
	double score;
	for (size_t i = 1; i < numOfPCL; i++) {
		// 开始SIFT求出匹配点
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern1(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		mySIFT(*imgQueue[i - 1], *imgQueue[i], *pcloud_pattern1, *pcloud_pattern2);

		std::cout << "SIFT完成" << std::endl;

		std::cout << "开始计算变换矩阵" << std::endl;

		int hr = patternICP(pcloud_pattern1, pcloud_pattern2, transformVecs[i - 1], score);

		fromWhich[i - 1] = i - 1;
		scores[i - 1] = score;

		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}

	if (numOfPCL < 2) {
		Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
		for (size_t i = 1; i < numOfPCL; i++) {
			currentMatrix = currentMatrix * transformVecs[i - 1];    // ?? 左乘 or 右乘 
			// 变换点云
			pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
			pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
			cloudQueues[i] = pPointCloudOut;
		}
	}
	else {
		std::vector<Eigen::Matrix4d> transformVecsSecond(numOfPCL - 1);
		for (size_t i = 2; i < numOfPCL; i++) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern1(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern2(new pcl::PointCloud<pcl::PointXYZRGBA>);
			mySIFT(*imgQueue[i - 2], *imgQueue[i], *pcloud_pattern1, *pcloud_pattern2);

			int hr = patternICP(pcloud_pattern1, pcloud_pattern2, transformVecsSecond[i - 1], score);

			
			std::cout << "矩阵计算成功" << std::endl;
			if (hr != 0)
				std::cout << "错误，ICP不收敛" << std::endl;


			if (score < scores[i - 1]) {   // 如果新的更好
				fromWhich[i - 1] = i - 2;
			}

		}

		std::vector<Eigen::Matrix4d> matrixToThisCloud(numOfPCL);
		matrixToThisCloud[0] = Eigen::Matrix4d::Identity();
		for (size_t i = 1; i < numOfPCL; i++) {
			Eigen::Matrix4d currentMatrix;
			if (fromWhich[i - 1] == i - 1) {
				currentMatrix = matrixToThisCloud[fromWhich[i - 1]] * transformVecs[i - 1];
			}
			else {
				currentMatrix = matrixToThisCloud[fromWhich[i - 1]] * transformVecsSecond[i - 1];
			}
			matrixToThisCloud[i] = currentMatrix;

			pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
			pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
			cloudQueues[i] = pPointCloudOut;
		}
	}


}
void operateClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues, std::vector<boost::shared_ptr<myMath::myImage>>& imgQueue) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL - 1);
	std::vector<double> scores(numOfPCL - 1);
	double score;
	for (size_t i = 1; i < numOfPCL; i++) {
		// 开始SIFT求出匹配点
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern1(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_pattern2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		mySIFT(*imgQueue[i - 1], *imgQueue[i], *pcloud_pattern1, *pcloud_pattern2);

		std::cout << "SIFT完成" << std::endl;

		std::cout << "开始计算变换矩阵" << std::endl;

		//int hr = tempICP(pcloud_pattern1, pcloud_pattern2, transformVecs[i - 1]);
		int hr = patternICP(pcloud_pattern1, pcloud_pattern2, transformVecs[i - 1], score);

		scores[i - 1] = score;

		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}


	Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
	for (size_t i = 1; i < numOfPCL; i++) {
		currentMatrix = currentMatrix * transformVecs[i - 1];    // ?? 左乘 or 右乘 
		// 变换点云
		pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
		cloudQueues[i] = pPointCloudOut;
	}

}

void preCutOffClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues,
	float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
	int numOfPCL = cloudQueues.size();
	for (size_t i = 0; i < numOfPCL; i++) {
		pcl::PointCloud<PointType>::Ptr Filtered(new pcl::PointCloud<PointType>);
		pcl::CropBox<PointType> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
		boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
		boxFilter.setInputCloud(cloudQueues[i]);
		boxFilter.filter(*Filtered);

		pcl::PointCloud<PointType>::Ptr ZeroFiltered(new pcl::PointCloud<PointType>);
		for (size_t j = 0; j < Filtered->points.size(); j++) {
			
			if (Filtered->points[j].x != 0.0 || Filtered->points[j].y != 0.0 || Filtered->points[j].z != 0.0) {
				
				ZeroFiltered->push_back(Filtered->points[j]);
			}
			
		}

		cloudQueues[i] = ZeroFiltered;
	}
}



typedef typename pcl::registration::TransformationEstimationSVD<PointType, PointType, double>::Matrix4 Matrix4;
int charICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix) {

	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;    // source

	Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_src(3, cloud_icp->size());
	Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_tgt(3, cloud_icp->size());

	Matrix4 transformation_ = Matrix4::Identity();

	for (size_t j = 0; j < 1000; j++) {
		for (int i = 0; i < cloud_icp->size(); ++i)
		{
			cloud_src(0, i) = cloud_icp->points[i].x;
			cloud_src(1, i) = cloud_icp->points[i].y;
			cloud_src(2, i) = cloud_icp->points[i].z;

			cloud_tgt(0, i) = cloud_in->points[i].x;
			cloud_tgt(1, i) = cloud_in->points[i].y;
			cloud_tgt(2, i) = cloud_in->points[i].z;
		}

		transformation_ = pcl::umeyama(cloud_src, cloud_tgt, false);

		std::cout << transformation_ << std::endl;

		pcl::transformPointCloud(*cloud_icp, *cloud_icp, transformation_);

		// Obtain the final transformation    
		matrix = transformation_ * matrix;
	}

	std::cout << cloud_icp->size() << std::endl;
	//pcl::io::savePCDFile("patt1_m.pcd", *cloud_in);
	pcl::io::savePCDFile("patt2_m.pcd", *cloud_icp);


	return 0;
}


//void getCurvature(pcl::PointCloud<PointType>& cloud) {
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(inCloud);
//	ne.setInputCloud(inCloud);
//	ne.setSearchMethod(tree);
//	ne.setKSearch(50);
//	//ne->setRadiusSearch (0.03); 
//	ne.compute(*pcNormal);
//
//	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
//	pcl::concatenateFields(*inCloud, *pcNormal, *cloud_with_normals);
//	
//}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getNormalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud) {

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(inCloud);
	ne.setInputCloud(inCloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(50);
	//ne->setRadiusSearch (0.03); 
	ne.compute(*pcNormal);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*inCloud, *pcNormal, *cloud_with_normals);

	return cloud_with_normals;
}

int ICPWithCurvature(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tr, Eigen::Matrix4d& matrix, double thres) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);  // ICP output point cloud  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t i = 0; i < cloud_in->size(); i++) {
		if (cloud_in->points[i].curvature > thres)
			cloud_tgt->push_back(pcl::PointXYZ(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z));
	}
	for (size_t i = 0; i < cloud_tr->size(); i++) {
		if (cloud_tr->points[i].curvature > thres)
			cloud_icp->push_back(pcl::PointXYZ(cloud_tr->points[i].x, cloud_tr->points[i].y, cloud_tr->points[i].z));
	}

	std::cout << cloud_tgt->size() << std::endl;
	std::cout << cloud_icp->size() << std::endl;

	// The Iterative Closest Point algorithm  
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(100);

	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_tgt);
	icp.align(*cloud_icp);

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		matrix = icp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	return 0;
}








int main_icpnormal() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile("比较好的一次\\0.pcd", *cloud1);
	pcl::io::loadPCDFile("比较好的一次\\1.pcd", *cloud2);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_1 = getNormalCloud(cloud1);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_2 = getNormalCloud(cloud2);

	Eigen::Matrix4d matrix;

	//ICPWithCurvature(cloud_with_normals_1, cloud_with_normals_2, matrix, 0.2);
	clock_t startTime, endTime;
	startTime = clock();
	ICPWithNormal(cloud_with_normals_1, cloud_with_normals_2, matrix);
	endTime = clock();
	std::cout << "Totle Time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	pcl::transformPointCloud(*cloud2, *cloud2, matrix);

	//pcl::io::savePCDFile("比较好的一次\\1_withCurvature.pcd", *cloud2);
	return 0;
}
//int main_pattern() {
//	pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
//	pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);
//	pcl::PointCloud<PointType>::Ptr cloudT(new pcl::PointCloud<PointType>);
//	pcl::io::loadPCDFile("比较好的一次\\patt1.pcd", *cloud1);
//	pcl::io::loadPCDFile("比较好的一次\\patt2.pcd", *cloud2);
//	pcl::io::loadPCDFile("比较好的一次\\1.pcd", *cloudT);
//	/*cloud1 = statisRemoval(cloud1);
//	cloud2 = statisRemoval(cloud2);*/
//
//	/*pcl::io::savePCDFile("比较好的一次\\1_rem.pcd", *cloud1);
//	pcl::io::savePCDFile("比较好的一次\\2_rem.pcd", *cloud2);*/
//	Eigen::Matrix4d matrix;
//	patternICP(cloud1, cloud2, matrix);
//
//	pcl::transformPointCloud(*cloudT, *cloudT, matrix);
//
//	pcl::io::savePCDFile("比较好的一次\\1_setCorrDis.pcd", *cloudT);
//
//	return 0;
//}
int main_normal() {

	//pcl::PointXYZRGBNormal
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud2_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPCDFile("比较好的一次\\patt1.pcd", *cloud1);
	pcl::io::loadPCDFile("比较好的一次\\patt2.pcd", *cloud2);
	Eigen::Matrix4d matrix;


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud1_normals);

	ne.setInputCloud(cloud2);
	ne.compute(*cloud2_normals);

	pcl::concatenateFields(*cloud1, *cloud1_normals, *cloud1_with_normals);
	pcl::concatenateFields(*cloud2, *cloud2_normals, *cloud2_with_normals);

	ICPWithNormal(cloud1_with_normals, cloud2_with_normals, matrix);

	pcl::transformPointCloud(*cloud2, *cloud2, matrix);

	pcl::io::savePCDFile("比较好的一次\\patt2_normal.pcd", *cloud2);

	return 0;
}

int main1() {
	pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile("比较好的一次\\beforeTuning_0.pcd", *cloud1);
	pcl::io::loadPCDFile("比较好的一次\\beforeTuning_1.pcd", *cloud2);

	std::cout << cloud1->size() << std::endl; 
	int count = 0;
	for (size_t i = 0; i < cloud2->size(); i++) {
		if (cloud2->points[i].x == 0.0 && cloud2->points[i].y == 0.0 && cloud2->points[i].z == 0.0)
			count++;
	}
	std::cout << count << std::endl;

	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;
	cloudQueues.push_back(cloud1);
	cloudQueues.push_back(cloud2);

	Eigen::Matrix4d matrix;

	tempICP(cloud1, cloud2, matrix);

	pcl::transformPointCloud(*cloud2, *cloud2, matrix);
	pcl::io::savePCDFile("比较好的一次\\afterTuning_1.pcd", *cloud2);


	/*operateClouds(cloudQueues);

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
	}*/

	return 0;
}

int getParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float parameter[4]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 1cm

	//ne.setRadiusSearch(0.02);
	ne.setKSearch(15);


	std::cout << "开始计算法线" << std::endl;
	ne.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	std::cout << "法线计算成功" << std::endl;

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);

	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Create the segmentation object for the planar model and set all the parameters

	seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化

	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型

	seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数

	seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法

	seg.setMaxIterations(500); //设置迭代的最大次数

	seg.setDistanceThreshold(0.05); //设置内点到模型的距离允许最大值

	seg.setInputCloud(cloud);

	seg.setInputNormals(normals);

	// 开始segment
	seg.segment(*inliers_plane, *coefficients_plane);

	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud

	for (size_t i = 0; i < 4; i++) {
		parameter[i] = coefficients_plane->values[i];
	}
	return 0;
}

int filterWithPlane(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, float parameter[4]) {
	float a = parameter[0];
	float b = parameter[1];
	float c = parameter[2];
	float d = parameter[3];
	for (size_t i = 0; i < cloudIn->points.size(); i++) {
		PointType& p = cloudIn->points[i];
		float residual = a * p.x + b * p.y + c * p.z + d;
		if (abs(residual) > 0.04 && p.x != 0.0 && p.y != 0.0 && p.z != 0.0)
			cloudOut->push_back(p);
	}

	return 0;
}

void filterPlaneForQueue(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::copyPointCloud(*cloudQueues[0], *testCloud);
	getParameter(testCloud, parameter);

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
		filterWithPlane(cloudQueues[i], cloudFiltered, parameter);
		cloudQueues[i] = cloudFiltered;
	}
}

int main_() {
	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;

	for (size_t i = 0; i < 10; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::loadPCDFile("一圈\\volleyball\\sift\\" + fileName, *cloud);
		cloudQueues.push_back(cloud);
	}

	filterPlaneForQueue(cloudQueues);
	clock_t startTime, endTime;
	startTime = clock();
	operateClouds(cloudQueues);
	endTime = clock();
	std::cout << "Totle Time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	


	for (size_t i = 0; i < 10; i++) {
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile("一圈\\volleyball\\icp_no_floor\\icp_" + fileName, *cloudQueues[i]);
	}

	return 0;
}

int main() {

	pcl::PointCloud<PointType>::ConstPtr cloud;
	boost::shared_ptr<myMath::myImage> img;


	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;
	std::vector<boost::shared_ptr<myMath::myImage>> imgQueue;

	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&, const boost::shared_ptr<myMath::myImage>&)> function =
		[&cloud, &img, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr, const boost::shared_ptr<myMath::myImage>& imgPtr) {
		boost::mutex::scoped_lock lock(mutex);
		cloud = ptr->makeShared();
		img = imgPtr->makeShared();
	};

	boost::function<void()> addCloudToArrayFunc =
		[&cloud, &img, &mutex, &cloudQueues, &imgQueue]() {
		boost::mutex::scoped_lock lock(mutex);
		pcl::PointCloud<PointType>::Ptr ptr;
		ptr = cloud->makeShared();
		cloudQueues.push_back(ptr);

		boost::shared_ptr<myMath::myImage> imgPtr;
		imgPtr = img->makeShared();
		imgQueue.push_back(imgPtr);
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


	std::cout << "<--------------- kinect初始化成功" << std::endl;

	
	while (true)
	{
		Sleep(800);
		std::cout << "添加到队列中" << std::endl;
		addCloudToArrayFunc();
		if (cloudQueues.size() == 2)
			break;
	}
	

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	//// 预处理  去除框外的
	preCutOffClouds(cloudQueues, -0.25, 0.25, -0.6, 0.2, -2.0, 2.0);
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//float parameter[4];
	//pcl::copyPointCloud(*cloudQueues[0], *testCloud);
	//getParameter(testCloud, parameter);

	//std::cout << "参数生成成功" << std::endl;

	//for (size_t i = 0; i < cloudQueues.size(); i++) {
	//	pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
	//	filterWithPlane(cloudQueues[i], cloudFiltered, parameter);
	//	cloudQueues[i] = cloudFiltered;
	//	std::cout << cloudFiltered->points.size() << std::endl;
	//}
	for (size_t i = 0; i < cloudQueues.size(); i++) {
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
	}
	
	clock_t startTime, endTime;
	startTime = clock();
	//operateClouds(cloudQueues, imgQueue);
	operateCloudsWithGap(cloudQueues, imgQueue);
	endTime = clock();
	std::cout << "Totle Time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	
	for (size_t i = 0; i < cloudQueues.size(); i++) {
		std::stringstream ss;
		ss << i;
		std::string fileName = "beforeTuning_" + ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
	}
	/*operateClouds(cloudQueues);
	for (size_t i = 0; i < cloudQueues.size(); i++) {
		std::stringstream ss;
		ss << i;
		std::string fileName = "afterTuning_" + ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
	}*/

	

	return 0;
}