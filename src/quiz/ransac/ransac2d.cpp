/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int cloudSize = cloud->size();

	while (maxIterations--)
	{
		std::unordered_set<int> currInlier;
		pcl::PointXYZ point;
		int a, b, c;
		int x1, x2, x3;
		int y1, y2, y3;
		float dist;
		while (currInlier.size() < 2)
			currInlier.insert(rand() % cloudSize);
		auto itr = currInlier.begin();
		point = cloud->points[*itr];
		x1 = point.x;
		y1 = point.y;
		itr++;
		point = cloud->points[*itr];
		x2 = point.x;
		y2 = point.y;

		a = y1 - y2;
		b = x2 - x1;
		c = (x1 * y2) - (x2 * y1);
		for (int i = 0; i < cloudSize; i++)
		{
			if (currInlier.count(i) > 0)
				continue;
			point = cloud->points[*itr];
			x3 = point.x;
			y3 = point.y;
			dist = abs((a * x3) + (b * y3) + c) / sqrt((a * a) + (b * b));

			if (dist <= distanceTol)
				currInlier.insert(i);
		}
		if (currInlier.size() > inliersResult.size())
		{
			inliersResult = currInlier;
		}
	}

	return inliersResult;
}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int cloudSize = cloud->size();

	while (maxIterations--)
	{
		std::unordered_set<int> currInlier;
		int a, b, c, d;
		int x, x1, x2, x3;
		int y, y1, y2, y3;
		int z, z1, z2, z3;
		float dist;
		pcl::PointXYZ point;
		while (currInlier.size() < 3)
			currInlier.insert(rand() % cloudSize);

		auto itr = currInlier.begin();
		point = cloud->points[*itr];
		x1 = point.x;
		y1 = point.y;
		z1 = point.z;

		itr++;
		point = cloud->points[*itr];
		x2 = point.x;
		y2 = point.y;
		z2 = point.z;
		itr++;
		point = cloud->points[*itr];
		x3 = point.x;
		y3 = point.y;
		z3 = point.z;

		a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		d = -1 * ((a * x1) + (b * y1) + (c * z1));
		for (int i = 0; i < cloudSize; i++)
		{
			if (currInlier.count(i) > 0)
				continue;
			point = cloud->points[*itr];
			x = point.x;
			y = point.y;
			z = point.z;
			dist = abs((a * x) + (b * y) + (c * z) + d) / sqrt((a * a) + (b * b) + (c * c));
			if (dist <= distanceTol)
				currInlier.insert(i);
		}
		if (currInlier.size() > inliersResult.size())
		{
			inliersResult = currInlier;
		}
	}
	return inliersResult;
}
int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, .2);
	// std::unordered_set<int> inliers = Ransac3d(cloud, 100, .2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
