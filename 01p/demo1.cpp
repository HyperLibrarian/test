#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <sstream>
#include <iomanip> 

#include <boost/shared_ptr.hpp> 

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/search/kdtree.h>                      // KDtree搜索
#include <pcl/features/normal_3d.h>                 // 计算法向量
#include <pcl/ModelCoefficients.h>                  // 模型系数
#include <pcl/sample_consensus/ransac.h>            // RANSAC
#include <pcl/sample_consensus/method_types.h>      // 随机参数估计方法
#include <pcl/sample_consensus/model_types.h>       // 模型定义
#include <pcl/segmentation/sac_segmentation.h>      // RANSAC分割
#include <pcl/sample_consensus/sac_model_cylinder.h>// 圆柱
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/convolution_3d.h>             // 高斯滤波


using namespace std;
using namespace cv;

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

string pngnum;

//奥比中光TOF相机内参 fx:490.084 fy : 490.084 cx : 316.404 cy : 241.755 width : 640 height : 480
const double camera_factor = 1000; 
//默认
//const double camera_cx = 316.404; const double camera_cy = 241.755; const double camera_fx = 490.084; const double camera_fy = 490.084;
//const double color_cx = 316.404; const double color_cy = 241.755; const double color_fx = 490.084; const double color_fy = 490.084;
//自测标定结果
//const double camera_cx = 319.853; const double camera_cy = 244.390; const double camera_fx = 496.491; const double camera_fy = 496.879;
//const double color_cx = 328.146; const double color_cy = 230.062; const double color_fx = 518.169; const double color_fy = 518.627;
//软件读取结果
const double camera_cx = 316.393; const double camera_cy = 241.764; const double camera_fx = 490.121; const double camera_fy = 490.121;
const double color_cx = 301.393; const double color_cy = 241.764; const double color_fx = 515.432; const double color_fy = 515.432;
//const double color_cx = 333.739; const double color_cy = 236.986; const double color_fx = 515.432; const double color_fy = 515.432;


extern "C"
{

bool depth2cloud(Mat& depth, PointCloud::Ptr& cloud)
{
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值或超出所需范围，若如此，跳过此点
			if (d <= 0 || d > 3000)
				continue;
			// d 存在符合要求的值，则向点云增加一个点
			PointT p;

			// 计算这个点的空间坐标,注意矩阵的行列（m,n)对应的是坐标（y,x）。
			p.z = double(d) / camera_factor / 9.4;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("./Result/cloud/" + pngnum + ".pcd", *cloud);
	cout << "Point cloud is saved." << endl;
	return true;
}

int GaussFilters(PointCloud::Ptr& cloud, PointCloud::Ptr& gassFilter)
{
	// -----------------------------基于高斯核函数的卷积滤波实现---------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);//设置距离阈值，若点间距离大于阈值则不予考虑
	cout << "Kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	cout << "KdTree made" << endl;

	// -------------------------------设置Convolution 相关参数-----------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	cout << "Convolution Start" << endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr gassFilter(new pcl::PointCloud<pcl::PointXYZ>);
	convolution.convolve(*gassFilter);
	pcl::io::savePCDFileASCII("./Result/cloud/" + pngnum + "_GS.pcd", *gassFilter);
	cout << "Convolution End" << endl;
	return 1;
}

int CylinderSegment(PointCloud::Ptr& cloud, vector<vector<Point3d>>& axis, int dist0, int minPN ,int rMin, int rMax)
{
	
	//第一步：定义输入的原始数据以及分割获得的点、平面系数coefficients、存储内点的索引集合对象inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>);//创建分割对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//模型系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
	pcl::SACSegmentation<pcl::PointXYZ> seg;//分割对象

	pcl::ExtractIndices<pcl::PointXYZ> extract0;//提取器

	int n_piece = 1;//需要探测的面的个数

	//第二步：使用RANSAC获取点数最多的面
	for (int i = 0; i < n_piece; i++)
	{
		seg.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
		seg.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
		seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
		seg.setDistanceThreshold(0.003);          //设定距离阈值，决定点被认为是局内点时必须满足的条件
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() < (0.7 * cloud->points.size()))
		{
			extract0.setInputCloud(cloud);
			extract0.setIndices(inliers);
			extract0.setNegative(false);
			//提取探测出来的平面
			extract0.filter(*planar_segment);
			//planar_segment为该次探测出来的面片，可以单独进行保存，此处省略
			//剔除探测出的平面，在剩余点中继续探测平面
			extract0.setNegative(true);
			extract0.filter(*cloud);

			// ----------------------------------------分割结果分类保存------------------------------------------
			pcl::io::savePCDFileBinary("./Result/cloud/" + pngnum + "_0.pcd", *cloud);
			cout << "第[" << i + 1 << "]块点云保存完毕！" << endl;
			cout << "平面方程为：\n"
				<< coefficients->values[0] << "x + "
				<< coefficients->values[1] << "y + "
				<< coefficients->values[2] << "z + "
				<< coefficients->values[3] << " = 0"
				<< endl;
		}
	}
	
	// -----------------------------计算法线------------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(50);		// K近邻搜索个数
	n.compute(*normals);
	// ----------------------------定义所需容器--------------------------------
	std::vector<int> totalInners;
	std::vector<int> indices(cloud->points.size());
	std::iota(std::begin(indices), std::end(indices), (int)0);

	float dist = (float)dist0 * 0.0001;         // 与模型距离阈值，小于该阈值的视为内点
	float minPointNum = (float)minPN; // 最小点数
	float radiusMin = (float)rMin * 0.0001;    // 圆柱半径的最小值
	float radiusMax = (float)rMax * 0.0001;      // 圆柱半径的最大值
	int iters = 0;
	cout << "圆柱参数(轴线一点坐标a、轴线方向向量b、半径r）为：\n" << endl;
	do
	{
		std::vector<int> inners;   // 存储内点索引的容器

		// -----------------------创建圆柱模型---------------------------------
		pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >::Ptr
			model(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >(cloud, indices));
		model->setInputNormals(normals);
		//model->setNormalDistanceWeight(0.001);		 // 设置法向距离的权重
		//model->setAxis(Eigen::Vector3f(0, 0, 1));  // 设置圆柱的轴向
		model->setRadiusLimits(radiusMin, radiusMax);// 设置圆柱半径的范围
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
		ransac.setDistanceThreshold(dist);
		ransac.setMaxIterations(300);
		ransac.computeModel();
		ransac.getInliers(inners); // 获取拟合圆柱的内点
		totalInners.insert(totalInners.end(), inners.begin(), inners.end());
		Point3d a, b;
		// 如果内点个数大于最小点数，则保存提取结果，否则结束提取。
		if (inners.size() > minPointNum)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_segment(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloud, inners, *cylinder_segment);
			pcl::io::savePCDFileBinary("./Result/cloud/" + pngnum + "_cylinder_" + std::to_string(iters) + ".pcd", *cylinder_segment);
			Eigen::VectorXf coef;
			ransac.getModelCoefficients(coef);
			cout << "a[" << iters << "].x = " << coef[0] << "; a[" << iters << "].y = " << coef[1] << "; a[" << iters << "].z = " << coef[2] << "; b[" << iters << "].x = " << coef[3] << "; b[" << iters << "].y = " << coef[4] << "; b[" << iters << "].z = " << coef[5] << "; \n  r:" << coef[6] << "\n" << endl;
			
			a.x = coef[0]; a.y = coef[1]; a.z = coef[2];b.x = coef[3]; b.y = coef[4]; b.z = coef[5];
			axis[0].push_back(a); axis[1].push_back(b);
			iters++;

		}
		else
		{
			break;
		}
		// --------------------提取剩余的点云数据-------------------------------
		std::vector<int> newIndices;
		pcl::ExtractIndices<pcl::PointXYZ> extract(true);
		extract.setInputCloud(cloud);
		extract.setNegative(true); // false代表提取指定的索引点，true则代表提取未指定的索引点
		extract.setIndices(boost::make_shared <std::vector<int>>(totalInners));	// 指定点的索引
		extract.filter(newIndices);
		indices = newIndices;

		// 如果剩余点个数不满足分割要求的最小点数，则结束分割，退出循环。
	} while (indices.size() > minPointNum);

	return iters;
}

double getVecAngle(Vec3d v1, Vec3d v2)
{

	Vec3d VecAcross = v1.cross(v2);
	double vecsin = norm(VecAcross) / norm(v1) / norm(v2);
	return asin(vecsin) * 180 / 3.141592654;

}

double getXaxisAngle(Vec3d v1) {

    double cosa = v1[0] / norm(v1);
    double anglea = acos(cosa) * 180 / 3.141592654;
    if (anglea < 90)
        return anglea;
    else
        return 180 - anglea;

}

double getYaxisAngle(Vec3d v1) {

    double cosa = v1[1] / norm(v1);
    double anglea = acos(cosa) * 180 / 3.141592654;
    if (anglea < 90)
        return anglea;
    else
        return 180 - anglea;

}

double getZaxisAngle(Vec3d v1) {

    double cosa = v1[2] / norm(v1);
    double anglea = acos(cosa) * 180 / 3.141592654;
    if (anglea < 90)
        return anglea;
    else
        return 180 - anglea;

}

bool drawlines(Point3d a, Point3d b, Mat& src, int colornum) 
{
    cv::Point startPoint, endPoint;
    Point3d c;

    int klong = 1;

    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;

    int x1 = (int)(a.x * color_fx / a.z + color_cx);
    int y1 = (int)(a.y * color_fy / a.z + color_cy);
    int x2 = (int)(c.x * color_fx / c.z + color_cx);
    int y2 = (int)(c.y * color_fy / c.z + color_cy);

    startPoint = Point(x1, y1);
    endPoint = Point(x2, y2);

    double slope = static_cast<double>(endPoint.y - startPoint.y) / (endPoint.x - startPoint.x);

    // 计算延长线的端点
    cv::Point extendedStartPoint(0, startPoint.y - slope * startPoint.x);
    cv::Point extendedEndPoint(640 - 1, slope * (640 - 1) + extendedStartPoint.y);

    // 绘制延长线
    switch (colornum) {
    case 1:
        cv::line(src, extendedStartPoint, extendedEndPoint, cv::Scalar(225, 0, 0), 5);
        break;
    case 2:
        cv::line(src, extendedStartPoint, extendedEndPoint, cv::Scalar(0, 225, 0), 5);
        break;
    case 3:
        cv::line(src, extendedStartPoint, extendedEndPoint, cv::Scalar(0, 0, 255), 5);
        break;
    }

    return 1;
}

double CalDistance(Point3d a1, Point3d b1, Point3d a2, Point3d b2)
{

	// 计算方向向量
	// 第一条直线、第二条直线的方向向量分别为
	Vec3d VecFirstLine = (b1), VecSecondLine = (b2);
	// 直线上一点分别为
	Point3d PointFirst = a1, PointSecond = a2;

	// 依据异面直线公式
	double angle = getVecAngle(VecFirstLine, VecSecondLine);
	Vec3d VecCross = VecFirstLine.cross(VecSecondLine);
	VecCross = normalize(VecCross);
	Vec3d VecPoint(PointSecond.x - PointFirst.x, PointSecond.y - PointFirst.y, PointSecond.z - PointFirst.z);
	double Distance = abs((VecCross.dot(VecPoint))) * 1000;
	cout << getZaxisAngle(b1) << "°" << "\t" << getZaxisAngle(b2) << "°" << endl;
	cout << "Distance:\t" << Distance << "cm" << endl << "Angle:\t" << angle << "°" << endl;
	
	return Distance;
}


int ResultViewer(int cnt, int timestamp, vector<vector<Point3d>> axis) 
{

	// 记录每根杆子的x、y、z偏向
	int fx[20];

	string imgsrc = "./Img/Color_" + std::to_string(timestamp)+ ".png";
	Mat src = imread(imgsrc);

	int k = 0;
	double xangle, yangle, zangle;
	double xlong = 999;
	double ylong = 999;
	double zlong = 999;
	double anglelimit = 35;
	while (k < cnt)
	{
		xangle = getXaxisAngle(axis[1][k]);
		yangle = getYaxisAngle(axis[1][k]);
		zangle = getZaxisAngle(axis[1][k]);
		cout << xangle << "°    " << yangle << "°    " << zangle << "°    " << endl;
		if (xangle < anglelimit && xangle < yangle && xangle < zangle) {
			drawlines(axis[0][k], axis[1][k], src, 1);
			fx[k] = 1;

		}
		else {
			if (yangle < anglelimit && yangle < xangle && yangle < zangle) {
				drawlines(axis[0][k], axis[1][k], src, 2);
				fx[k] = 2;
			}
			else {
				if (zangle < anglelimit && zangle < xangle && zangle < yangle) {
					drawlines(axis[0][k], axis[1][k], src, 3);
					fx[k] = 3;
				}
			}
		}
		k++;
	}
	cout << "循环1" << endl;
	k = 0;
	double distance0;
	double zmax = 0;
	int zmaxk = 0;
	while (k < (cnt-1))
	{
		cout << "循环2" << endl;
		int cntk = k + 1;
		while (cntk < cnt)
		{
			if (fx[k] == fx[cntk] && abs(axis[0][k].z - axis[0][cntk].z) < 0.1) {

				distance0 = CalDistance(axis[0][k], axis[1][k], axis[0][cntk], axis[1][cntk]);
				if (fx[k] == 1 && xlong > distance0) {
					xlong = distance0;
				}
				else {
					if (fx[k] == 2 && ylong > distance0) {
						ylong = distance0;
					}
				}

			}
			cntk++;

		}
		if (axis[0][k].z > zmax) {
			zmax = axis[0][k].z;
			zmaxk = k;
		}
		k++;
	}
	if (axis[0][k].z > zmax) {
			zmax = axis[0][k].z;
			zmaxk = k;
	}

	cout << zmaxk << endl;

	k = 0;
	while (k < cnt) {

		cout << "循环3" << endl;

		if (fx[k] == fx[zmaxk] && abs(axis[0][k].z - axis[0][zmaxk].z) >= 0.1) {
			//cout << "zzz" << endl;
			distance0 = CalDistance(axis[0][k], axis[1][k], axis[0][zmaxk], axis[1][zmaxk]);
			if (zlong > distance0) {
				zlong = distance0;
			}
		}
		k++;
	}

	std::string text = "x=" + std::to_string(xlong) + "cm";
	int font_face = cv::FONT_HERSHEY_COMPLEX;
	double font_scale = 1;
	int thickness = 1;
	int baseline;
	//获取文本框的长宽
	cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

	//将文本框居中绘制
	cv::Point origin;
	origin.x = src.cols - text_size.width;
	origin.y = src.rows - text_size.height * 2;
	cv::putText(src, text, origin, font_face, font_scale, cv::Scalar(255, 0, 0), thickness, 8, 0);
	text = "y=" + std::to_string(ylong) + "cm";
	origin.y = src.rows - text_size.height * 1;
	cv::putText(src, text, origin, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
	text = "z=" + std::to_string(zlong) + "cm";
	origin.y = src.rows - text_size.height * 0;
	cv::putText(src, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
	string seg_path = "./Result/Seg_" + std::to_string(timestamp)+ ".png"; 
	cv::imwrite(seg_path, src);
	

	return 1;
}


// 主函数
int CalCylinder(int timestamp, int dist, int minPN ,int rMin, int rMax)
{
	pngnum = std::to_string(timestamp);

	//读取深度图并生成点云
	std::string image_path = "./Img/Depth_" + pngnum + ".png";
	cv::Mat depth;
	depth = cv::imread(image_path, CV_16UC1);
	PointCloud::Ptr cloud(new PointCloud);
	depth2cloud(depth, cloud);

	//点云预处理
	PointCloud::Ptr cloud0(new PointCloud);
	GaussFilters(cloud, cloud0);

	//圆柱分割
	int cnt;
	vector<vector<Point3d>> axis(2);
	cnt = CylinderSegment(cloud0, axis, dist, minPN, rMin, rMax);

	//结果可视化
	ResultViewer(cnt, timestamp, axis);

	return 1;
}

}

