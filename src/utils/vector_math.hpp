#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <math.h>

void rotate2D(float* v, float* v_tf, float angle)
{
	v_tf[0] = (v[0] * cos(angle)) - (v[1] * sin(angle));
	v_tf[1] = (v[0] * sin(angle)) + (v[1] * cos(angle));
}

void rotate2D3D(float* v, float* v_tf, float angle)
{
	v_tf[0] = v[0];
	v_tf[1] = (v[1] * cos(angle)) - (v[2] * sin(angle));
	v_tf[2] = (v[1] * sin(angle)) + (v[2] * cos(angle));
}

float vectorProduct(float* a, float* b, int n)
{
	int product = 0;
	for (int i = 0; i < n; i++)
	{
		product += a[i] * b[i];
	}
	return product;
}

float crossProduct2D(float* a, float* b)
{
	return (a[0] * b[1]) - (a[1] * b[0]);
}

float magnitude(float* a, int n)
{
	float square_sum = 0;
	for (int i; i < n;i++)
	{
		square_sum += pow(a[i], 2);
	}
	return square_sum;
}

void weightedVectorAddition(float* a, float* b, float k1, float k2, int n, float* output)
{
	for (int i = 0; i < n; i++)
	{
		output[i] = (k1 * a[i]) + (k2 * b[i]);
	}
}

void nWeightedVectorAddition(float* a, float* b, float* k1, float* k2, int n, float* output)
{
	for (int i = 0; i < n; i++)
	{
		output[i] = (k1[i] * a[i]) + (k2[i] * b[i]);
	}
}

#endif // VECTOR_MATH_H