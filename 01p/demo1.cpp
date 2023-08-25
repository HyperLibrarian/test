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

#include <pcl/search/kdtree.h>                     // KDtree搜索
#include <pcl/features/normal_3d.h>                // 计算法向量
#include <pcl/ModelCoefficients.h>                 // 模型系数
#include <pcl/sample_consensus/ransac.h>           // RANSAC
#include <pcl/sample_consensus/method_types.h>     // 随机参数估计方法
#include <pcl/sample_consensus/model_types.h>      // 模型定义
#include <pcl/segmentation/sac_segmentation.h>     // RANSAC分割
#include <pcl/sample_consensus/sac_model_cylinder.h>// 圆柱
#include <pcl/filters/extract_indices.h>


using namespace std;
using namespace cv;

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

string pngnum;

//奥比中光TOF相机内参 fx:490.084 fy : 490.084 cx : 316.404 cy : 241.755 width : 640 height : 480
const double camera_factor = 1000; const double camera_cx = 316.404; const double camera_cy = 241.755; const double camera_fx = 490.084; const double camera_fy = 490.084;

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

int CylinderSegment(PointCloud::Ptr& cloud, vector<vector<Point3d>>& axis)
{
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

	float dist = 0.001;         // 与模型距离阈值，小于该阈值的视为内点
	float minPointNum = 2400; // 最小点数
	float radiusMin = 0.0015;    // 圆柱半径的最小值
	float radiusMax = 0.0040;      // 圆柱半径的最大值
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
		ransac.setMaxIterations(200);
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

	return iters-1;
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

    int x1 = (int)(a.x * camera_fx / a.z + camera_cx);
    int y1 = (int)(a.y * camera_fy / a.z + camera_cy);
    int x2 = (int)(c.x * camera_fx / c.z + camera_cx);
    int y2 = (int)(c.y * camera_fy / c.z + camera_cy);

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
	while (k < cnt)
	{
		xangle = getXaxisAngle(axis[1][k]);
		yangle = getYaxisAngle(axis[1][k]);
		zangle = getZaxisAngle(axis[1][k]);
		k++;
		cout << xangle << "°    " << yangle << "°    " << zangle << "°    " << endl;
		if (xangle < 20 && xangle < yangle && xangle < zangle) {
			drawlines(axis[0][k], axis[1][k], src, 1);
			fx[k] = 1;

		}
		else {
			if (yangle < 20 && yangle < xangle && yangle < zangle) {
				drawlines(axis[0][k], axis[1][k], src, 2);
				fx[k] = 2;
			}
			else {
				if (zangle < 20 && zangle < xangle && zangle < yangle) {
					drawlines(axis[0][k], axis[1][k], src, 3);
					fx[k] = 3;
				}
			}
		}
	}
	cout << "循环1" << endl;
	k = 0;
	double distance0;
	double zmax = 0;
	int zmaxk = 0;
	while (k < cnt)
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
int CalCylinder(int timestamp)
{
	pngnum = std::to_string(timestamp);
	std::string image_path = "./Img/Depth_" + pngnum + ".png";
	cv::Mat depth;
	depth = cv::imread(image_path, CV_16UC1);
	PointCloud::Ptr cloud(new PointCloud);
	depth2cloud(depth, cloud);

	int cnt;
	vector<vector<Point3d>> axis(2);
	cnt = CylinderSegment(cloud, axis);

	ResultViewer(cnt, timestamp, axis);

	return 1;
}

}

