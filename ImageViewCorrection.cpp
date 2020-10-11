// ImageViewCorrection.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#include "view_change.h"
#include "SimpleMatrix.cpp"

//D:\zxbData\0824NineImgs\higher\youyisquare\images\DJI_0683.JPG
//D:\zxbData\0824NineImgs\higher\youyisquare\images\DJI_0678.JPG
int main()
{
	string src_name, match_name ,dst_name;
	
	cin >> src_name;// 基准片
	cin >> match_name;// 匹配片

	dst_name = "correction.bmp";
	Mat src_img = imread(src_name, IMREAD_GRAYSCALE);
	Mat match_img = imread(match_name, IMREAD_GRAYSCALE);
	namedWindow("test_one", WindowFlags::WINDOW_KEEPRATIO);
	imshow("test_one", src_img);
	namedWindow("test_two", WindowFlags::WINDOW_KEEPRATIO);
	imshow("test_two", match_img);
	
	int col_b = src_img.cols;
	int row_b = src_img.rows;
	int col_r=col_b, row_r=row_b;
	col_r = row_r = 11;
	pose base_pose; base_pose.phi = -0.772915; base_pose.omega = 0.169054; base_pose.kappa = 1.399974;
	//pose base_pose; base_pose.phi = 0; base_pose.omega = 0; base_pose.kappa = 1.4134881456;
	pose std_pose;//0,0,0
	pose match_pose; match_pose.phi = 0.76936; match_pose.omega = -0.165549; match_pose.kappa = -1.739349;

	BYTE* data=0;// = new BYTE[100000 * 100000];
	// compute the rotation matrix
	double R_b[9], R_bT[9], R_m[9], R_std[9];
	GetRotationMat(R_b, base_pose.phi, base_pose.omega, base_pose.kappa);
	GetRotationMat(R_m, match_pose.phi, match_pose.omega, match_pose.kappa);
	Transpose(R_b, R_bT, 3, 3);
	// transform matrix
	double R_transT[9];
	MatrixMulti(R_bT, R_m, R_transT, 3, 3, 3);
	// compute matrix over
	// use R_transT to perform affine transformation
	// window_affine(src_img.data, col_b, row_b, data, col_r, row_r, col_r/2, row_r/2, R_transT, 0);// To match_pose
	// window_affine2(src_img.data, col_b, row_b, data, col_r, row_r, 2749, 913, R_bT, R_m, 0);// Via std_pose

		//ChangeView(src_img.data, col_b, row_b, data, col_r, row_r, base_pose, match_pose, 0);
		//ChangeViewAffine2(src_img.data, col_b, row_b, data, col_r, row_r, bpose, mpose);
	window_semi_homography(src_img.data, col_b, row_b, data, col_r, row_r, 1998, 1887, R_bT, R_m, 0);
	Mat res_img(Size(col_r, row_r), CV_8UC1);
	memcpy(res_img.data, data, sizeof(BYTE)*col_r*row_r);
	delete[] data;
	data = 0;
	namedWindow("test_three", WindowFlags::WINDOW_AUTOSIZE);
	imshow("test_three", res_img);
	waitKey();
	return 0;
}

// Note:
// Direction Distortion is severe when the directions are perpendicular.
// TODO: improve the reprojection