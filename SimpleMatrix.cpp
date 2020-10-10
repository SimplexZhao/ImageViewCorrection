#ifndef MAT_OP
#define MAT_OP

template<typename T>
void GetRotationMat(T* mat, T a1, T a2, T a3, int flag = 2)
{
	if (flag == 0)
	{
		T r = a1;
		T p = a2;
		T y = a3;
		mat[0] = cos(r)*cos(y);
		mat[1] = -cos(r)*sin(y);
		mat[2] = -sin(r);
		mat[3] = cos(p)*sin(y) + sin(r)*sin(p)*cos(y);
		mat[4] = cos(p)*cos(y) - sin(r)*sin(p)*sin(y);
		mat[5] = cos(r)*sin(p);
		mat[6] = sin(r)*cos(y)*cos(p) - sin(p)*sin(y);
		mat[7] = -cos(p)*sin(r)*sin(y) - sin(p)*cos(y);
		mat[8] = cos(r)*cos(p);
	}
	if (flag == 1)
	{
		T r = a1;
		T p = a2;
		T y = a3;
		mat[0] = cos(p)*cos(y);
		mat[1] = -cos(p)*sin(y);
		mat[2] = -sin(p);
		mat[3] = cos(r)*sin(y) + sin(p)*sin(r)*cos(y);
		mat[4] = cos(r)*cos(y) - sin(p)*sin(r)*sin(y);
		mat[5] = cos(p)*sin(r);
		mat[6] = sin(p)*cos(y)*cos(r) - sin(r)*sin(y);
		mat[7] = -cos(r)*sin(p)*sin(y) - sin(r)*cos(y);
		mat[8] = cos(p)*cos(r);
	}
	if (flag == 2)
	{
		T phi = a1;  // a1 a2 a3
		T omega = a2;// b1 b2 b3
		T kappa = a3;// c1 c2 c3
		mat[0] = cos(phi)*cos(kappa) - sin(phi)*sin(omega)*sin(kappa);
		mat[1] = -cos(phi)*sin(kappa) - sin(phi)*sin(omega)*cos(kappa);
		mat[2] = -sin(phi)*cos(omega);
		mat[3] = cos(omega)*sin(kappa);
		mat[4] = cos(omega)*cos(kappa);
		mat[5] = -sin(omega);
		mat[6] = sin(phi)*cos(kappa) + cos(phi)*sin(omega)*sin(kappa);
		mat[7] = -sin(phi)*sin(kappa) + cos(phi)*sin(omega)*cos(kappa);
		mat[8] = cos(phi)*cos(omega);
	}
}

///////////////////////////////////////////////
//设A为m x l阶矩阵，B为l x n阶矩阵，C为m x n阶矩阵，计算 C=A x B的子程序为：
template<typename T>
static bool MatrixMulti(T * A, T * B, T * C, int M, int N, int L)
{
	if (A == 0 || B == 0 || C == 0) return 0;
	int i, j, k;
	//Zero(C, M*N);
	memset(C, 0, M*N * sizeof(T));
	for (i = 0; i < M; i++)
	{
		for (j = 0; j < N; j++)
		{
			//			for (k=0;k<L;k++) C[i*N+j] += A[i*L+k]*B[k*N+j];
			for (k = 0; k < L; k++) *(C + i * N + j) += *(A + i * L + k)* *(B + k * N + j);	//等效！！！
		}
	}
	return 1;
}


template<typename T>
bool Transpose(T *src, T *dst, int m, int n)
{
	if (src == 0 || dst == 0)return 0;
	T *ps = src, *pd = dst;
	for (int i = 0; i < m; i++)
	{
		pd = dst + i;
		for (int j = 0; j < n; j++)
		{
			*pd = *ps;
			pd += m;
			++ps;
		}
	}
	pd = ps = 0; return 1;
}

#endif // !MAT_OP
