#include "view_change.h"
#include <iostream>
#include "SimpleMatrix.cpp"

// @param R 3*3 rotation matrix
// @param input x,y,-f
// @param outpu x',y'
template<typename T>
void computePt(T* R, T* input, T* output)
{
	T tmp = R[6] * input[0] + R[7] * input[1] + R[8] * input[2];
	tmp /= input[2];
	output[0] = R[0] * input[0] + R[1] * input[1] + R[2] * input[2];
	output[0] /= tmp;// x'
	output[1] = R[3] * input[0] + R[4] * input[1] + R[5] * input[2];
	output[1] /= tmp;// y'

}

bool ChangeView(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res, 
	pose base_pose, pose match_pose, int rgb_flag, double f)
{
	// compute the rotation matrix
	double R_b[9], R_m[9], R_mT[9];
	GetRotationMat(R_b, base_pose.phi, base_pose.omega, base_pose.kappa);
	GetRotationMat(R_m, match_pose.phi, match_pose.omega, match_pose.kappa);
	Transpose(R_m, R_mT, 3, 3);
	// transform matrix
	double R_trans[9];
	double R_transT[9];
	MatrixMulti(R_mT, R_b, R_trans, 3, 3, 3);
	Transpose(R_trans, R_transT, 3, 3);
	// compute four corner points' coordinates in match_pose image.
	double x[4], y[4];
	x[0] = -double(col_b) / 2;	y[0] = double(row_b) / 2;
	x[1] = -x[0];	y[1] = y[0]; // 0	1
	x[2] = x[0];	y[2] = -y[0];// 2	3
	x[3] = x[1];	y[3] = y[2];
	double u[4], v[4];
	int mMx[2], mMy[2];
	for (int i = 0; i < 4; ++i)
	{
		double tmp_i[3], tmp_o[2];
		tmp_i[0] = x[i]; tmp_i[1] = y[i]; tmp_i[2] = -f;
		computePt(R_trans, tmp_i, tmp_o);
		u[i] = tmp_o[0];
		v[i] = tmp_o[1];
		std::cout << u[i] << "," << v[i] << std::endl;
	}
	// compute over.

	mMx[0] = 100000;	mMx[1] = -100000;
	mMy[0] = 100000;	mMy[1] = -100000;

	for (int i = 0; i < 4; ++i)
	{
		if (u[i] < mMx[0])mMx[0] = (int)u[i];
		if (u[i] > mMx[1])mMx[1] = (int)u[i];
		if (v[i] < mMy[0])mMy[0] = (int)v[i];
		if (v[i] > mMy[1])mMy[1] = (int)v[i];
	}

	col_res = mMx[1] - mMx[0];
	row_res = mMy[1] - mMy[0];
	//col_res = col_b;
	//row_res = row_b;
	if (rgb_flag == 1)
		result = new BYTE[3 * col_res*row_res];
	else
		result = new BYTE[col_res*row_res];
	//generate a new image in match_pose at the same exposure station.
	// comment: bad.‪ See C:\Users\hp\Desktop\test_three.bmp
	double res_x[4], res_y[4];
	res_x[0] = -double(col_res) / 2;	res_y[0] = double(row_res) / 2;
	res_x[1] = -res_x[0];	res_y[1] = res_y[0]; // 0	1
	res_x[2] = res_x[0];	res_y[2] = -res_y[0];// 2	3
	res_x[3] = res_x[1];	res_y[3] = res_y[2];
	for (int i = mMx[0]; i < mMx[1]; ++i)
	{
		int tmp_ii = i - mMx[0];
		for (int j = mMy[1]; j > mMy[0]; --j)
		{
			int tmp_jj = mMy[1] - j;
			double tmp_i[3], tmp_o[2];
			tmp_i[0] = i; tmp_i[1] = j; tmp_i[2] = -f;
			computePt(R_transT, tmp_i, tmp_o);
			int tmp_x = tmp_o[0] - x[0];
			int tmp_y = y[0] - tmp_o[1];
			if (tmp_x<0||tmp_x>col_b-1||tmp_y<0||tmp_y>row_b-1)continue;
			else
			{
				if (rgb_flag == 1)
				{
					int pos_res = 3 * int((tmp_ii + tmp_jj *col_res));
					int pos_base = 3 * int(tmp_x + tmp_y * col_b);
					result[pos_res] = base[pos_base];
					result[pos_res + 1] = base[pos_base + 1];
					result[pos_res + 2] = base[pos_base + 2];
				}
				else if (rgb_flag == 0)
				{
					//int fail = 0;
					//Bilinear_Interp(tmp_x, tmp_y, base, row_b, col_b, window + pos_res, fail);
					int pos_res = int((tmp_ii + tmp_jj * col_res));
					int pos_base = int(tmp_x + tmp_y * col_b);
					result[pos_res] = base[pos_base];
					//pos_res++;// intensity
				}
			}
		}
	}

	// compute over

	return true;
}

//TODO: in-place affine transformation.
bool ChangeViewAffine(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res,
	pose base_pose, pose match_pose, double lambda)
{
	// compute the rotation matrix
	double R_b[9], R_m[9], R_mT[9];
	GetRotationMat(R_b, base_pose.phi, base_pose.omega, base_pose.kappa);
	GetRotationMat(R_m, match_pose.phi, match_pose.omega, match_pose.kappa);
	Transpose(R_m, R_mT, 3, 3);
	// transform matrix
	double R_trans[9];
	double R_transT[9];
	MatrixMulti(R_mT, R_b, R_trans, 3, 3, 3);
	Transpose(R_trans, R_transT, 3, 3);
	// compute matrix over

	// solve φ_1, θ, φ_2 from matrix R_transT
	double phi1, theta, phi2;// 
	double vecz[3];
	vecz[0] = R_transT[2];
	vecz[1] = R_transT[5];
	vecz[2] = R_transT[8];
	phi1 = atan2(vecz[1], vecz[0]);//
	phi2 = match_pose.kappa;

	theta = acos(vecz[2]/* /sqrt(vecz[0]*vecz[0]+vecz[1]*vecz[1]+vecz[2]*vecz[2])*/);
	// solve over.

	// compute some values to avoid repeated steps
	double t;
	// 不求θ,直接记录1/cosθ,节省一步
	// t = sqrt(vecz[0] * vecz[0] + vecz[1] * vecz[1] + vecz[2] * vecz[2]) / vecz[2]; //double t = 1.0 / cos(theta);
	double sin_phi1 = sin(phi1);
	double cos_phi1 = cos(phi1);
	double cos_theta = cos(theta);
	double sin_theta = sin(theta);
	double R_tmp[9];
	R_tmp[0] = cos_phi1 * cos_theta; R_tmp[3] = -sin_phi1; R_tmp[6] = -sin_phi1 * cos_theta;
	R_tmp[1] = sin_phi1 * cos_theta; R_tmp[4] = cos_phi1; R_tmp[7] = -sin_phi1 * sin_theta;
	R_tmp[2] = sin_theta; R_tmp[5] = 0; R_tmp[8] = cos_theta;
	//R_tmp[0] = cos_phi1 * cos_theta; R_tmp[1] = -sin_phi1; R_tmp[2] = -sin_phi1 * cos_theta;
	//R_tmp[3] = sin_phi1 * cos_theta; R_tmp[4] = cos_phi1; R_tmp[5] = cos_phi1 * cos_theta;
	//R_tmp[6] = sin_theta; R_tmp[7] = 0; R_tmp[8] = cos_theta;
	double dPhi2[9];
	MatrixMulti(R_tmp, R_transT, dPhi2, 3, 3, 3);
	double sin_phi2;// = sin(phi2);
	double cos_phi2;// = cos(phi2);
	// try compute φ2 from R_trans and φ1, θ
	// TODO: modify here 2020.9.8
	sin_phi2 = dPhi2[3];
	cos_phi2 = dPhi2[0];
	// TODO: over. 2020.9.9
	double sin1_sin2 = sin_phi1 * sin_phi2;
	double sin1_cos2 = sin_phi1 * cos_phi2;
	double cos1_cos2 = cos_phi1 * cos_phi2;
	double cos1_sin2 = cos_phi1 * sin_phi2;
	t = 1.0 / cos_theta;
	// compute over.


	// get matrix A=$[a&b\\c&d]$, assuming $\lambda=1.0$
	double a, b, c, d;
	a = t * cos1_cos2 - sin1_sin2; b = -t * cos1_sin2 - sin1_cos2;
	c = t * sin1_cos2 + cos1_sin2; d = -t * sin1_sin2 + cos1_cos2;
	// got A.


	// compute view-changed image
	col_res = col_b;
	row_res = row_b;
	result = new BYTE[3 * col_res*row_res];

	int tag_x = -col_res / 2;
	for (int i = 0; i < col_res; ++i, ++tag_x)
	{
		int tag_y = row_res / 2;
		for (int j = 0; j < row_res; ++j, --tag_y)
		{
			int tmp_x = 0;
			int tmp_y = 0;
			tmp_x = a * tag_x + b * tag_y + (col_b >> 1);
			tmp_y = (row_b >> 1) - c * tag_x + d * tag_y;
			if (tmp_x<0 || tmp_x>col_b - 1 || tmp_y<0 || tmp_y>row_b - 1)continue;
			else
			{
				int pos_res = 3 * (i + j * col_res);
				int pos_base = 3 * int(tmp_x + tmp_y * col_b);
				result[pos_res] = base[pos_base];
				result[pos_res + 1] = base[pos_base + 1];
				result[pos_res + 2] = base[pos_base + 2];
			}
		}
	}
	//over

	return true;
}

bool ChangeViewAffine2(BYTE* base, int col_b, int row_b, BYTE*& result, int& col_res, int& row_res,
	pose base_pose, pose match_pose, double lambda)
{
	// compute the rotation matrix
	double R_b[9], R_m[9], R_mT[9];
	GetRotationMat(R_b, base_pose.phi, base_pose.omega, base_pose.kappa);
	GetRotationMat(R_m, match_pose.phi, match_pose.omega, match_pose.kappa);
	Transpose(R_m, R_mT, 3, 3);
	// transform matrix
	double R_trans[9];
	double R_transT[9];
	MatrixMulti(R_mT, R_b, R_trans, 3, 3, 3);
	Transpose(R_trans, R_transT, 3, 3);
	// compute matrix over

	// use R_transT to perform affine transformation
	// assuming scale parameter $\lambda=1.0$

	// unit vector of z_{match} axis in base coordinate 
	double vecz[3];
	vecz[0] = R_transT[2];
	vecz[1] = R_transT[5];
	vecz[2] = R_transT[8];
	//

	// compute view-changed image
	col_res = col_b;
	row_res = row_b;
	result = new BYTE[3 * col_res*row_res];

	int half_x = col_res / 2;
	int half_y = row_res / 2;
	int tag_x = -half_x;
	for (int i = 0; i < col_res; ++i, ++tag_x)
	{
		int tag_y = half_y;
		for (int j = 0; j < row_res; ++j, --tag_y)
		{
			int tmp_x = 0;
			int tmp_y = 0;
			double tmp_z;
			double tmp_pt[3] = { tag_x, tag_y, 0.0 };
			double tmp_out[3];
			MatrixMulti(R_transT, tmp_pt, tmp_out, 3, 1, 3);
			tmp_x = tmp_out[0]; tmp_y = tmp_out[1]; tmp_z = tmp_out[2];

			double ratio_z = tmp_z / vecz[2];
			tmp_x -= vecz[0] * ratio_z;
			tmp_y -= vecz[1] * ratio_z;
			tmp_x += half_x;
			tmp_y = half_y - tmp_y;
			if (tmp_x<0 || tmp_x>col_b - 1 || tmp_y<0 || tmp_y>row_b - 1)continue;
			else
			{
				int pos_res = 3 * (i + j * col_res);
				int pos_base = 3 * int(tmp_x + tmp_y * col_b);
				result[pos_res] = base[pos_base];
				result[pos_res + 1] = base[pos_base + 1];
				result[pos_res + 2] = base[pos_base + 2];
			}
		}
	}
	//over

	return true;
}

// Bilinear Interpolation at single point (x,y) in image
//
// @param x,y float number, coordinates in image
// @param image uchar type image data pointee
// @param window uchar type pointee to a pixel in small window
// @param fail flag of running state
void Bilinear_Interp(double x, double y, unsigned char *image, int row, int col, unsigned char *window, int& fail)
{
	int x0 = int(x);	int y0 = int(y); // 取整
	if (x0 < 0 || x0 >= col - 1 || y0 < 0 || y0 >= row - 1) // 像点接近"影像边缘"
	{
		fail = 1;	return;
	}
	unsigned char *image0 = image + y0 * col + x0; // 求得大影像上待内插点左上像素地址
	unsigned char g = 0; // interpolation value
	double p = x - x0;	double q = y - y0;	// 小数部分
	double pq = p * q;
	// 双线性重采样
	g += unsigned char(*image0*(1.0 - p - q + pq));
	g += unsigned char(*(image0 + 1)*(p - pq));
	g += unsigned char(*(image0 + col)*(q - pq));
	g += unsigned char(*(image0 + col + 1)*pq);
	*window = g;
}


bool window_affine(BYTE* base, int col_b, int row_b, BYTE*& window, int win_w, int win_h,
	int feature_x, int feature_y, double R[9], int rgb_flag, double lambda)
{
	// assuming scale parameter $\lambda=1.0$
	// unit vector of z_{match} axis in base coordinate 
	double vecz[3];
	vecz[0] = R[2];
	vecz[1] = R[5];
	vecz[2] = R[8];
	double invz = 1.0/vecz[2];
	//

	// compute view-changed window
	if (rgb_flag == 1)
		window = new BYTE[3 * win_w*win_h];// mode:RGB
	else if (rgb_flag == 0)
		window = new BYTE[win_w*win_h];// mode:Intensity
	int half_win_w = win_w / 2;
	int half_win_h = win_h / 2;
	int tag_y = half_win_h;
	int pos_res=0;
	for (int j = 0; j < win_h; ++j, --tag_y)
	{
		int tag_x = -half_win_w;
		for (int i = 0; i < win_w; ++i, ++tag_x)
		{
			double tmp_x = 0;
			double tmp_y = 0;
			double tmp_z;
			double tmp_pt[3] = { tag_x, tag_y, 0.0 };
			double tmp_out[3];
			MatrixMulti(R, tmp_pt, tmp_out, 3, 1, 3);
			tmp_x = tmp_out[0]; tmp_y = tmp_out[1]; tmp_z = tmp_out[2];

			// Parallel Projection
			double ratio_z = tmp_z * invz;
			tmp_x -= vecz[0] * ratio_z;
			tmp_y -= vecz[1] * ratio_z;
			// Parallel Projection
			
			tmp_x += feature_x;
			tmp_y = feature_y - tmp_y;
			if (rgb_flag == 1)
			{
				int pos_base = 3 * (tmp_x + tmp_y * col_b);
				window[pos_res] = base[pos_base]; pos_res++;// R
				window[pos_res] = base[pos_base + 1]; pos_res++;// G
				window[pos_res] = base[pos_base + 2]; pos_res++;// B
			}
			else if (rgb_flag == 0)
			{
				int fail = 0;
				Bilinear_Interp(tmp_x, tmp_y, base, row_b, col_b, window+pos_res, fail);
				//int pos_base = tmp_x + tmp_y * col_b;
				//window[pos_res] = base[pos_base]; 
				pos_res++;// intensity
			}
		}
	}
	//over

	return true;
}

bool window_affine2(BYTE * base, int col_b, int row_b, BYTE *& window, int win_w, int win_h, int feature_x, int feature_y, double R_1[9], double R_2[9], int rgb_flag, double lambda)
{
	// assuming scale parameter $\lambda=1.0$
// unit vector of z_{match} axis in base coordinate 
	double vecz[3];
	vecz[0] = R_2[2];
	vecz[1] = R_2[5];
	vecz[2] = R_2[8];
	double invz = 1.0 / vecz[2];
	//

	// compute view-changed window
	if (rgb_flag == 1)
		window = new BYTE[3 * win_w*win_h];// mode:RGB
	else if (rgb_flag == 0)
		window = new BYTE[win_w*win_h];// mode:Intensity
	int half_win_w = win_w / 2;
	int half_win_h = win_h / 2;
	int tag_y = half_win_h;
	int pos_res = 0;
	for (int j = 0; j < win_h; ++j, --tag_y)
	{
		int tag_x = -half_win_w;
		for (int i = 0; i < win_w; ++i, ++tag_x)
		{
			double tmp_x = 0;
			double tmp_y = 0;
			double tmp_z;
			double tmp_pt[3] = { tag_x, tag_y, 0.0 };
			double tmp_out[3];
			MatrixMulti(R_2, tmp_pt, tmp_out, 3, 1, 3);
			tmp_x = tmp_out[0]; tmp_y = tmp_out[1]; tmp_z = tmp_out[2];

			// Parallel Projection to std_pose plane
			double ratio_z = tmp_z * invz;
			tmp_x -= vecz[0] * ratio_z;
			tmp_y -= vecz[1] * ratio_z;
			// Parallel Projection to std_pose plane
			// Projection to base_pose plane
			tmp_pt[0] = tmp_x;	tmp_pt[1] = tmp_y; tmp_pt[2]= 0;
			MatrixMulti(R_1, tmp_pt, tmp_out, 3, 1, 3);
			// Projection to base_pose plane
			tmp_x = feature_x + tmp_out[0];
			tmp_y = feature_y - tmp_out[1];
			if (rgb_flag == 1)
			{
				int pos_base = 3 * (tmp_x + tmp_y * col_b);
				window[pos_res] = base[pos_base]; pos_res++;// R
				window[pos_res] = base[pos_base + 1]; pos_res++;// G
				window[pos_res] = base[pos_base + 2]; pos_res++;// B
			}
			else if (rgb_flag == 0)
			{
				int fail = 0;
				Bilinear_Interp(tmp_x, tmp_y, base, row_b, col_b, window + pos_res, fail);
				//int pos_base = tmp_x + tmp_y * col_b;
				//window[pos_res] = base[pos_base]; 
				pos_res++;// intensity
			}
		}
	}
	//over
	return true;
}

// Project a point from a plane to target plane via central projection
//
// only used in window_semi_affine to simplify the code
inline void Proj2Plane(double vecz[3], double x_src, double y_src, double z_src, double& x, double& y)
{
	double ratio_z = z_src / (z_src-vecz[2]);
	x = x_src - (x_src-vecz[0])*ratio_z;
	y = y_src - (y_src-vecz[1])*ratio_z;
	return;
}
// Project a point from a plane to target plane via central projection
//
// only used in window_semi_affine to simplify the code
// @param vecz[3] a vector of positive z_src axis direction in target space coordinate, length==focal_length
// @param src_pt 3D point in target space coordinate
// @param x,y result coordinates
// @return none
inline void Proj2Plane(double vecz[3], double src_pt[3], double& x, double& y)
{
	Proj2Plane(vecz, src_pt[0], src_pt[1], src_pt[2], x, y);
	return;
}

// Get point location in target pose using R_1,R_2,vecz1,vecz2
//
// @param f_x,f_y Point coordinates in 'image center ORIGIN' coordinate, NOT 'left-top ORIGIN' TYPE
// @param R_1,R_2
// @param vecz1,vecz2 Length is required to be f, so f is needed before calling this function **to compute vecz** 
// @param x,y result point location **still in 'image center ORIGIN' coordinate**
// @return void
void GetFeaturePos(double f_x, double f_y, double R_1[9], double R_2[9], double vecz2[3], double vecz1[3], double& x, double &y)
{
	double tmp_x = 0;
	double tmp_y = 0;
	double tmp_z;
	double tmp_pt[3] = { f_x, f_y, 0.0 };
	double tmp_out[3];
	MatrixMulti(R_2, tmp_pt, tmp_out, 3, 1, 3);
	tmp_x = tmp_out[0]; tmp_y = tmp_out[1]; tmp_z = tmp_out[2];

	// Projection to std_pose plane
	Proj2Plane(vecz2, tmp_out, tmp_x, tmp_y);
	// Projection to std_pose plane

	tmp_pt[0] = tmp_x;	tmp_pt[1] = tmp_y; tmp_pt[2] = 0;
	MatrixMulti(R_1, tmp_pt, tmp_out, 3, 1, 3);
	// Projection to base_pose plane
	Proj2Plane(vecz1, tmp_out, x, y);
	// Projection to base_pose plane
	return;
}

bool window_semi_affine(BYTE* base, int col_b, int row_b, BYTE*& window, int win_w, int win_h,
	int feature_x, int feature_y, double R_1[9], double R_2[9], int rgb_flag, int f, double lambda)
{
	// unit vector of z_{match} axis in base coordinate 
	double vecz2[3];		double vecz1[3];
	vecz2[0]=R_2[2]*f;	vecz1[0]=0;//R_1[6]*f;
	vecz2[1]=R_2[5]*f;	vecz1[1]=0;//R_1[7]*f;
	vecz2[2]=R_2[8]*f;	vecz1[2]=f;//R_1[8]*f;
	
	double R_1T[9], R_2T[9];	double veczT[3], veczT2[3];
	Transpose(R_1, R_1T, 3, 3);
	Transpose(R_2, R_2T, 3, 3);
	veczT[0] = R_1T[2] * f;	veczT2[0] = 0;//R_2T[6]*f;
	veczT[1] = R_1T[5] * f;	veczT2[1] = 0;//R_2T[7]*f;
	veczT[2] = R_1T[8] * f;	veczT2[2] = f;//R_2T[8]*f;

	// compute view-changed window
	if (rgb_flag == 1)
		window = new BYTE[3 * win_w*win_h];// mode:RGB
	else if (rgb_flag == 0)
		window = new BYTE[win_w*win_h];// mode:Intensity
	int half_win_w = win_w / 2;
	int half_win_h = win_h / 2;
	int pos_res = 0;

	double center_x, center_y;
	GetFeaturePos(feature_x-col_b/2, row_b/2-feature_y, R_2T, R_1T, veczT, veczT2, center_x, center_y);
	// Sanity Check for GetFeaturePos parameter passing
		//for(int i=0;i<4;++i)
		//{
		//	double tmpxx,tmpyy;
		//	GetFeaturePos(i*col_b/4, i*row_b/4, R_2T, R_1T, veczT, veczT2, center_x, center_y);// point from base to match
		//	GetFeaturePos(center_x, center_y, R_1, R_2, vecz2, vecz1, tmpxx, tmpyy);// point from match to base
		//	if(int(tmpxx+0.5)==i*col_b/4&&int(tmpyy+0.5)==i*row_b/4)printf("Sanity Check Passed\n");
		//}

	double tag_y = center_y+half_win_h;
	for (int j = 0; j < win_h; ++j, --tag_y)
	{
		double tag_x = center_x-half_win_w;
		for (int i = 0; i < win_w; ++i, ++tag_x)
		{
			double tmp_x = 0;
			double tmp_y = 0;
			GetFeaturePos(tag_x, tag_y, R_1, R_2, vecz2, vecz1, tmp_x, tmp_y);
			tmp_x = col_b/2 + tmp_x;
			tmp_y = row_b/2 - tmp_y;
			if (rgb_flag == 1)
			{
				int pos_base = 3 * (tmp_x + tmp_y * col_b);
				window[pos_res] = base[pos_base]; pos_res++;// R
				window[pos_res] = base[pos_base + 1]; pos_res++;// G
				window[pos_res] = base[pos_base + 2]; pos_res++;// B
			}
			else if (rgb_flag == 0)
			{
				int fail = 0;
				Bilinear_Interp(tmp_x, tmp_y, base, row_b, col_b, window + pos_res, fail);
				pos_res++;// intensity
			}
		}
	}
	//over
	return true;
}