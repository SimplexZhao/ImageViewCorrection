#pragma once

typedef unsigned char BYTE;

// 姿态参数按φ-ω-κ转角系统
typedef struct tagpose
{
	double phi;
	double omega;
	double kappa;
	tagpose()
	{
		phi = omega = kappa = 0.0;
	}
}pose;

// 将基准影像纠正到匹配影像的视角，采用影像中心为主点
//
// @param base 基准影像
// @param col_b,row_b 基准影像的列数行数
// @param result 结果影像
// @param col_res,row_res 结果影像尺寸
// @param base_pose 基准影像姿态角
// @param match_pose 匹配影像姿态角
// @param f focal length of image
bool ChangeView(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res, 
	pose base_pose, pose match_pose, int rgb_flag, double f = 3648.5);

// Deprecated Affine transform function
//
// use ASIFT-like method to compute transformation parameters (using phi, theta, var_phi):
// Try to solve the angel values from roatation matrix computed with base_pose and match_pose, and then compute the affine matrix 'A' with the angels.
// Unsuccessful & Deprecated in 2020_09_10
bool ChangeViewAffine(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res,
	pose base_pose, pose match_pose, double lambda = 1.0);


// Affine transformation on whole image
//
// use rotation matrix computed with base_pose and match_pose to convert the coordinates
// in **base image space system** to those in
// **match-posed result image space system**
// then use parallel projection to project the pixel along match image's z-axis on base image to get RGB value 
bool ChangeViewAffine2(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res,
	pose base_pose, pose match_pose, double lambda = 1.0);

// Affine transformation for a window. 
//
// Generate a small window(like 11*11) at match_pose viewing base image.
// @param window pointee of small window
// @param win_h,win_w height and width of the small window
// @param feature_x,feature_y coordinates of the feature point in image coordinate system
// @param R rotation matrix giving description of relative pose between base and match image( Corresponding to R_transT in above functions)
// @param lambda scale paramter of the small window, unused until now.
// @return true if excuted successfully
bool window_affine(BYTE* base, int col_b, int row_b, BYTE*& window, int win_w, int win_h,
	int feature_x, int feature_y, double R[9], int rgb_flag, double lambda=1.0);

// Affine transformation for a window. 
//
// Generate a small window(like 11*11) via `Projecting base image on std_pose plane, and viewing std_pose plane at match_pose`.
// And it seems only useful when object photographed is almost flat like Ground, e.g. Youyi Square
// @param window pointee of small window
// @param win_h,win_w height and width of the small window
// @param feature_x,feature_y coordinates of the feature point in image coordinate system
// @param R_1 rotation matrix giving description of relative pose from base to std image
// @param R_2 rotation matrix giving description of relative pose between std and match image
// @param lambda scale paramter of the small window, unused until now.
// @return true if excuted successfully
bool window_affine2(BYTE* base, int col_b, int row_b, BYTE*& window, int win_w, int win_h,
	int feature_x, int feature_y, double R_1[9], double R_2[9], int rgb_flag, double lambda = 1.0);


bool window_semi_affine(BYTE* base, int col_b, int row_b, BYTE*& window, int win_w, int win_h,
	int feature_x, int feature_y, double R_1[9], double R_2[9], int rgb_flag, int f=3600, double lambda = 1.0);