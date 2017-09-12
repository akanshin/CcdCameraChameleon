#include "BeamAnalyzer.h"
#include <cmath>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <iostream>

#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <fstream>

using namespace std;
using namespace cv;

typedef struct function_s {
	size_t size;
	double constCenter;
	double xDelta;
	double yDelta;
	double *x;
	double *y;
}function_s;

long long BeamAnalyzer::mtime() {
  struct timeval t;

  gettimeofday(&t, NULL);
  long long mt = (long long)(t.tv_sec * 1000 + t.tv_usec / 1000);
  return mt;
}

int BeamAnalyzer::getWidth() const {
	return width;
}

int BeamAnalyzer::getHeight() const {
	return height;
}

BeamAnalyzer::BeamAnalyzer(unsigned char *data, int width, int height, bool release) {
	this->data = data;
	this->release = release;
	this->width = width;
	this->height = height;
}
BeamAnalyzer::~BeamAnalyzer() {
	if (data != 0 && release)
		delete[] data;
	if (filtered != 0)
		delete[] filtered;
	if (distributionX != 0)
		delete[] distributionX;
	if (distributionY != 0)
		delete[] distributionY;

}

void BeamAnalyzer::beamCheckFunciton(bool *result) {
	*result = false;
	int av = getAverage(data, width * height) + 20;
	vector<vector<Point> > contours;
	Mat image(height, width, CV_8UC1, data);
	Mat bimage = image >= av;
	findContours(bimage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE );
	for (vector<Point> contour : contours) {
		if (contour.size() > 6) {
			*result = true;
			break;
		}
	}
}

bool BeamAnalyzer::analyze() {
	if (data == 0) {
		return false;
	}

	filtered = new unsigned char[width * height];
	memcpy(filtered, data, width * height);

	/** PLANE EVALUATING **/
	FuncPoint p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = (double)getPixel(filtered, width, height, (int)p1.x, (int)p1.y);
	p1.z += (double)getPixel(filtered, width, height, (int)p1.x + 1, (int)p1.y);
	p1.z += (double)getPixel(filtered, width, height, (int)p1.x, (int)p1.y + 1);
	p1.z += (double)getPixel(filtered, width, height, (int)p1.x + 1, (int)p1.y + 1);
	p1.z /= 4.0;
	FuncPoint p2;
	p2.x = width - 1;
	p2.y = 0;
	p2.z = (double)getPixel(filtered, width, height, (int)p2.x, (int)p2.y);
	p2.z += (double)getPixel(filtered, width, height, (int)p2.x - 1, (int)p2.y);
	p2.z += (double)getPixel(filtered, width, height, (int)p2.x, (int)p2.y + 1);
	p2.z += (double)getPixel(filtered, width, height, (int)p2.x - 1, (int)p2.y + 1);
	p2.z /= 4.0;
	FuncPoint p3;
	p3.x = 0;
	p3.y = height - 1;
	p3.z = (double)getPixel(filtered, width, height, (int)p3.x, (int)p3.y);
	p3.z += (double)getPixel(filtered, width, height, (int)p3.x + 1, (int)p3.y);
	p3.z += (double)getPixel(filtered, width, height, (int)p3.x, (int)p3.y - 1);
	p3.z += (double)getPixel(filtered, width, height, (int)p3.x + 1, (int)p3.y - 1);
	p3.z /= 4.0;
	FuncPoint p4;
	p4.x = width - 1;
	p4.y = height - 1;
	p4.z = (double)getPixel(filtered, width, height, (int)p4.x, (int)p4.y);
	p4.z += (double)getPixel(filtered, width, height, (int)p4.x - 1, (int)p4.y);
	p4.z += (double)getPixel(filtered, width, height, (int)p4.x, (int)p4.y - 1);
	p4.z += (double)getPixel(filtered, width, height, (int)p4.x - 1, (int)p4.y - 1);
	p4.z /= 4.0;

	FuncPoint planes[4];
	planes[0] = solvePlane(p1, p2, p3);
	planes[1] = solvePlane(p1, p2, p4);
	planes[2] = solvePlane(p1, p3, p4);
	planes[3] = solvePlane(p2, p3, p4);

	double A = 0, B = 0, C = 0;
	for (int i = 0; i < 4; i++) {
		A += planes[i].x;
		B += planes[i].y;
		C += planes[i].z;
	}
	A /= 4.0;
	B /= 4.0;
	C /= 4.0;

	constComponent = C;
	xLinearComponent = A;
	yLinearComponent = B;

	/** CENTER MASS **/
	distributionX = new int[width];
	distributionY = new int[height];

	memset(distributionX, 0, width * sizeof(int));
	memset(distributionY, 0, height * sizeof(int));

	for (int i = 0; i < height; ++i) {
		int shift = width * i;
		for (int j = 0; j < width; ++j) {

			int val = (int)filtered[shift + j]
					- (int)(constComponent
							+ xLinearComponent * j
							+ yLinearComponent * i);
			if (val < 0)
				val = 0;
			distributionX[j]+=val;
			distributionY[i]+=val;
		}
	}

	std::thread thread0_1(&BeamAnalyzer::subHalf, this, (int*)distributionX, width);
	subHalf((int*)distributionY, height);

	thread0_1.join();

	double sx, sy;

	std::thread thread1(&BeamAnalyzer::getCenterMass, this, (int*)distributionX, width, &gaussCenterX);
	std::thread thread2(&BeamAnalyzer::getCenterMass, this, (int*)distributionY, height, &gaussCenterY);
	std::thread thread3(&BeamAnalyzer::getSigma, this, (int*)distributionX, width, &sx);
	getSigma((int*)distributionY, height, &sy);

	thread1.join();
	thread2.join();
	thread3.join();

	double _amp = (double)((int)data[width * (int)gaussCenterY + (int)gaussCenterX]);
	double _back = constComponent + xLinearComponent * gaussCenterX
			+ yLinearComponent * gaussCenterY;
	if (fabs(_amp - _back) < 30) {
		gaussAmplitude = 0;
		gaussCenterX = 0;
		gaussCenterY = 0;
		gaussSigmaX = 0;
		gaussSigmaY = 0;
		gaussAngle = 0;
		return false;
	}

	/** GAUSS FITTING **/
	double sigma = std::fmax(4 * sx, 4 * sy);

	std::vector<double> x[4], y[4];
	bool begin[8] = {false};
	int c;
	int cc;
	c = getPixel(data, width, height, gaussCenterX, gaussCenterY);
	int cx = gaussCenterX;
	int cy = gaussCenterY;

	if (c < 255) {
		x[0].push_back(0);
		y[0].push_back(c);
		x[1].push_back(0);
		y[1].push_back(c);
		x[2].push_back(0);
		y[2].push_back(c);
		x[3].push_back(0);
		y[3].push_back(c);
	}
	double init_amp = (double)c;

	int diff_lim = 20;

	for (int i = 1; i < (int)sigma; i++) {
		c = (int)getPixel(data, width, height, cx + i, cy);
		if (!begin[0]) {
			cc = (int)getPixel(data, width, height, cx + i + diff_lim, cy);
			if ((int)(c - cc) > 0) {
				begin[0] = true;
			}
		}
		if (begin[0] && cx + i < width) {
			c += (int)getPixel(data, width, height, cx + i, cy+1);
			c += (int)getPixel(data, width, height, cx + i, cy-1);

			x[0].push_back((double)i);
			y[0].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx - i, cy);
		if (!begin[1]) {
			cc = (int)getPixel(data, width, height, cx - i - diff_lim, cy);
			if ((int)(c - cc) > 0) {
				begin[1] = true;
			}
		}
		if (begin[1] && cx - i >= 0) {
			c += (int)getPixel(data, width, height, cx - i, cy+1);
			c += (int)getPixel(data, width, height, cx - i, cy-1);

			x[0].push_back(-(double)i);
			y[0].push_back((double)c / 3.0);
		}
		c = (int)getPixel(data, width, height, cx, cy + i);
		if (!begin[2]) {
			cc = (int)getPixel(data, width, height, cx, cy + i + diff_lim);
			if ((int)(c - cc) > 0) {
				begin[2] = true;
			}
		}
		if (begin[2] && cy + i < height) {
			c += (int)getPixel(data, width, height, cx - 1, cy + i);
			c += (int)getPixel(data, width, height, cx + 1, cy + i);

			x[1].push_back((double)i);
			y[1].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx, cy - i);
		if (!begin[3]) {
			cc = (int)getPixel(data, width, height, cx, cy - i - diff_lim);
			if ((int)(c - cc) > 0) {
				begin[3] = true;
			}
		}
		if (begin[3] && cy - i >= 0) {
			c += (int)getPixel(data, width, height, cx - 1, cy - i);
			c += (int)getPixel(data, width, height, cx + 1, cy - i);

			x[1].push_back(-(double)i);
			y[1].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx + i, cy + i);
		if (!begin[4]) {
			cc = (int)getPixel(data, width, height, cx + i + diff_lim, cy + i + diff_lim);
			if ((int)(c - cc) > 0) {
				begin[4] = true;
			}
		}
		if (begin[4] && cx + i < width && cy + i < height) {
			c += (int)getPixel(data, width, height, cx + i - 1, cy + i + 1);
			c += (int)getPixel(data, width, height, cx + i + 1, cy + i - 1);

			x[2].push_back((double)i);
			y[2].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx - i, cy - i);
		if (!begin[5]) {
			cc = (int)getPixel(data, width, height, cx - i - diff_lim, cy - i - diff_lim);
			if ((int)(c - cc) > 0) {
				begin[5] = true;
			}
		}
		if (begin[5] && cx - i >= 0 && cy - i >= 0) {
			c += (int)getPixel(data, width, height, cx - i - 1, cy - i + 1);
			c += (int)getPixel(data, width, height, cx - i + 1, cy - i - 1);

			x[2].push_back(-(double)i);
			y[2].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx + i, cy - i);
		if (!begin[6]) {
			cc = (int)getPixel(data, width, height, cx + i + diff_lim, cy - i - diff_lim);
			if ((int)(c - cc) > 0) {
				begin[6] = true;
			}
		}
		if (begin[6] && cx + i < width && cy - i >= 0) {
			c += (int)getPixel(data, width, height, cx + i + 1, cy - i + 1);
			c += (int)getPixel(data, width, height, cx + i - 1, cy - i - 1);

			x[3].push_back((double)i);
			y[3].push_back((double)c / 3.0);
		}

		c = (int)getPixel(data, width, height, cx - i, cy + i);
		if (!begin[7]) {
			cc = (int)getPixel(data, width, height, cx - i - diff_lim, cy + i + diff_lim);
			if ((int)(c - cc) > 0) {
				begin[7] = true;
			}
		}
		if (begin[7] && cx - i >= 0 && cy + i < height) {
			c += (int)getPixel(data, width, height, cx - i + 1, cy + i + 1);
			c += (int)getPixel(data, width, height, cx - i - 1, cy + i - 1);

			x[3].push_back(-(double)i);
			y[3].push_back((double)c / 3.0);
		}
	}
	if (y[0].size() < 4 || y[1].size() < 4 || y[2].size() < 4 || y[3].size() < 4) {

		return false;
	}

	double cComp = constComponent + xLinearComponent * gaussCenterX + yLinearComponent * gaussCenterY;
	double xl = xLinearComponent;
	double yl = yLinearComponent;

	double init_sigma = pow(sigma / 4.0, 2.0);

	std::thread thread5(&BeamAnalyzer::solveGauss, this, x[0], y[0], cComp, xl, 0.0, init_amp, init_sigma, ampValues+0, sigmaValues+0, ampErrors+0, sigmaErrors+0);
	std::thread thread6(&BeamAnalyzer::solveGauss, this, x[1], y[1], cComp, 0.0, yl, init_amp, init_sigma, ampValues+1, sigmaValues+1, ampErrors+1, sigmaErrors+1);
	std::thread thread7(&BeamAnalyzer::solveGauss, this, x[2], y[2], cComp, xl, yl, init_amp, init_sigma, ampValues+2, sigmaValues+2, ampErrors+2, sigmaErrors+2);
	solveGauss(x[3], y[3], cComp, xl, -yl, init_amp, init_sigma, ampValues+3, sigmaValues+3, ampErrors+3, sigmaErrors+3);
	thread5.join();
	thread6.join();
	thread7.join();

	gaussAmplitude = ampValues[0];
	gaussAmplitude += ampValues[1];
	gaussAmplitude += ampValues[2];
	gaussAmplitude += ampValues[3];
	gaussAmplitude /= 4.0;


	/** FIT ELLIPSE **/
	std::vector<cv::Point> contour;
	cv::Point p[8];

	p[0].x = gaussCenterX - sigmaValues[0];
	p[0].y = gaussCenterY;

	p[1].x = gaussCenterX - sigmaValues[2];
	p[1].y = gaussCenterY - sigmaValues[2];

	p[2].x = gaussCenterX;
	p[2].y = gaussCenterY - sigmaValues[1];

	p[3].x = gaussCenterX + sigmaValues[3];
	p[3].y = gaussCenterY - sigmaValues[3];

	p[4].x = gaussCenterX + sigmaValues[0];
	p[4].y = gaussCenterY;

	p[5].x = gaussCenterX + sigmaValues[2];
	p[5].y = gaussCenterY + sigmaValues[2];

	p[6].x = gaussCenterX;
	p[6].y = gaussCenterY + sigmaValues[1];

	p[7].x = gaussCenterX - sigmaValues[3];
	p[7].y = gaussCenterY + sigmaValues[3];

	for (int i = 0; i < 8; i++)
		contour.push_back(p[i]);

	cv::RotatedRect box = cv::fitEllipse(cv::Mat(contour));

	gaussSigmaX = (double)box.size.width / 2.0;
	gaussSigmaY = (double)box.size.height / 2.0;
	gaussAngle = box.angle;

	return true;
}

BeamAnalyzer::FuncPoint BeamAnalyzer::solvePlane(BeamAnalyzer::FuncPoint p1,
												 BeamAnalyzer::FuncPoint p2,
												 BeamAnalyzer::FuncPoint p3) {
	double det = (p3.x - p2.x) * p1.y + (p1.x - p3.x) * p2.y + (p2.x - p1.x) * p3.y;

	FuncPoint result;
	result.x = ((p2.y - p3.y) * p1.z + (p3.y - p1.y) * p2.z + (-p2.y + p1.y) * p3.z) / det;
	result.y = ((p3.x - p2.x) * p1.z + (p1.x - p3.x) * p2.z + (p2.x - p1.x) * p3.z) / det;
	result.z = ((p2.x * p3.y - p3.x * p2.y) * p1.z + (p3.x * p1.y - p1.x * p3.y) * p2.z + (p1.x * p2.y - p2.x * p1.y) * p3.z) / det;

	return result;
}

unsigned char BeamAnalyzer::getPixel(unsigned char *data, int width, int height, int x, int y) {
	if (x < 0 || x > width - 1)
		return 0;
	else if (y < 0 || y > height - 1)
		return 0;
	else
		return data[width * y + x];
}

void BeamAnalyzer::getCenterMass(int* array, int size, double *result) {
	double a = 0;
	double b = 0;

	for (int i = 0; i < size; i++) {
		a += (double)(i * array[i]);
		b += (double) array[i];
	}

	*result =  a / b;
}

double BeamAnalyzer::getAverage(int* array, int size) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += (double) array[i];
	}

	return sum / (double)size;
}
double BeamAnalyzer::getAverage(unsigned char* array, int size) {
	int sum = 0;
	for (int i = 0; i < size; i++) {
		sum += (int) array[i];
	}

	return (double)sum / (double)size;
}

void BeamAnalyzer::getAverage_p(unsigned char* array, int size, double *result) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += (double) array[i];
	}

	*result = sum / (double)size;
}

void BeamAnalyzer::subAverage(int* array, int size) {
	int distribAverage = getAverage(array, size);

	for (int i = 0; i < size; i++) {
		array[i] = array[i] - distribAverage;
		if (array[i] < 0.0)
			array[i] = 0.0;
	}

}

bool BeamAnalyzer::subHalf(int* array, int size) {
	int min = array[0];
	int max = array[0];



	for (int i = 0; i < size; i++) {
		if (min > array[i])
			min = array[i];
		if (max < array[i])
			max = array[i];
	}


	int half = (min + max) / 2;

	for (int i = 0; i < size; i++) {
		array[i] -= half;
		if (array[i] < 0.0)
			array[i] = 0.0;
	}

	return true;
}

int BeamAnalyzer::getHalf(unsigned char* array, int size, int *min, int *max) {
	*min = array[0];
	*max = array[0];



	for (int i = 0; i < size; i++) {
		if (*min > array[i])
			*min = array[i];
		if (*max < array[i])
			*max = array[i];
	}

	int half = (*min + *max) / 2;

	return half;
}

double BeamAnalyzer::getAverageNum(int* array, int size) {
	double result = 0;
	double sum = 0;
	for (int i = 0; i < size; i++) {
		result += (double)array[i] * (double)i;
		sum += (double) array[i];
	}

	result /= sum;

	return result;
}

void BeamAnalyzer::getSigma(int* array, int size, double *result) {
	double average = getAverageNum(array, size);

	double sum = 0.0;

	double quad = 0.0;
	for (int i = 0; i < size; i++) {
		quad += (double) array[i] * ((double) i - average) * ((double) i - average);
		sum += array[i];
	}

	quad /= sum;

	*result = sqrt(quad);
}

int BeamAnalyzer::gauss_f (const gsl_vector * x, void *data, gsl_vector * f) {
	size_t n = ((function_s *)data)->size;
	double *d_x = ((function_s *)data)->x;
	double *d_y = ((function_s *)data)->y;
	double constCenter = ((function_s *)data)->constCenter;
	double xDelta = ((function_s *)data)->xDelta;
	double yDelta = ((function_s *)data)->yDelta;


	double amp = gsl_vector_get (x, 0);
	double sigma = gsl_vector_get (x, 1);

	size_t i;

	for (i = 0; i < n; i++) {
		double t = d_x[i];
		double Yi = constCenter + (xDelta + yDelta) * t + amp * exp(-t * t / 2.0 / fabs(sigma));
		gsl_vector_set (f, i, Yi - d_y[i]);
	}

	return GSL_SUCCESS;
}

int BeamAnalyzer::gauss_df(const gsl_vector * x, void *data, gsl_matrix *J) {
	size_t n = ((function_s *)data)->size;
	double *d_x = ((function_s *)data)->x;

	double amp = gsl_vector_get (x, 0);
	double sigma = gsl_vector_get (x, 1);

	size_t i;

	for (i = 0; i < n; i++)
	{
		double t = d_x[i];
		double e = exp(-t * t / 2.0 / fabs(sigma));
		double dg_dAmp = e;
		double dg_dSigma = e * t * t / 2.0 / sigma / sigma * amp;
		gsl_matrix_set (J, i, 0, dg_dAmp);
		gsl_matrix_set (J, i, 1, dg_dSigma);
	}

	return GSL_SUCCESS;
}

void BeamAnalyzer::solveGauss(std::vector<double> data_x, std::vector<double> data_y,
				double cc, double xl, double yl,
				double init_amp, double init_sigma,
				double *evaluated_amp, double *evaluated_sigma,
				double *amp_err, double *sigma_err) {

	const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
	gsl_multifit_nlinear_workspace *w;
	gsl_multifit_nlinear_fdf f;
	gsl_multifit_nlinear_parameters fdf_params =
			gsl_multifit_nlinear_default_parameters();
	fdf_params.solver = gsl_multifit_nlinear_solver_cholesky;

	int info;
	const size_t n =  data_x.size();
	const size_t p = 2;

	function_s func;
	func.x = data_x.data();
	func.y = data_y.data();
	func.size = data_x.size();
	func.constCenter = cc;
	func.xDelta = xl;
	func.yDelta = yl;

	gsl_matrix *covar = gsl_matrix_alloc (p, p);

	double x_init[p] = { init_amp, init_sigma };

	gsl_vector_view x = gsl_vector_view_array (x_init, p);
	const gsl_rng_type * type;
	gsl_rng * r;

	gsl_rng_env_setup();

	type = gsl_rng_default;
	r = gsl_rng_alloc (type);


	f.f = &BeamAnalyzer::gauss_f;
	f.df = &BeamAnalyzer::gauss_df;
	f.fvv = NULL;
	f.n = n;
	f.p = p;
	f.params = &func;

	w = gsl_multifit_nlinear_alloc (T, &fdf_params, n, p);
	gsl_multifit_nlinear_winit (&x.vector, NULL, &f, w);

	gsl_multifit_nlinear_driver(200, 1e-8, 1e-8, 0.0, NULL, NULL, &info, w);

	gsl_matrix *J = gsl_multifit_nlinear_jac(w);
	gsl_multifit_nlinear_covar (J, 0.0, covar);

	gsl_multifit_nlinear_covar (J, 0.0, covar);

#define FIT(i) gsl_vector_get(w->x, i)
#define ERR(i) sqrt(gsl_matrix_get(covar,i,i))

	*evaluated_amp = fabs(FIT(0));
	*evaluated_sigma = sqrt(fabs(FIT(1)));

	*amp_err = fabs(ERR(0));
	*sigma_err = sqrt(ERR(1)) / 2.0;

	gsl_multifit_nlinear_free (w);
	gsl_matrix_free (covar);
	gsl_rng_free (r);
}

unsigned char *BeamAnalyzer::getData() const {
	return data;
}

unsigned char *BeamAnalyzer::getFiltered() const {
	return filtered;
}

int BeamAnalyzer::getFilterRadius() const {
	return filterRadius;
}

void BeamAnalyzer::setFilterRadius(int value) {
	filterRadius = value;
}

double BeamAnalyzer::getConstComponent() const {
	return constComponent;
}

double BeamAnalyzer::getXLinearComponent() const {
	return xLinearComponent;
}

double BeamAnalyzer::getYLinearComponent() const {
	return yLinearComponent;
}

int *BeamAnalyzer::getDistributionX() const {
	return distributionX;
}

int *BeamAnalyzer::getDistributionY() const {
	return distributionY;
}

double BeamAnalyzer::getGaussAmplitude() const {
	return gaussAmplitude;
}

double BeamAnalyzer::getGaussCenterX() const {
	return gaussCenterX;
}

double BeamAnalyzer::getGaussCenterY() const {
	return gaussCenterY;
}

double BeamAnalyzer::getGaussSigmaX() const  {
	return gaussSigmaX;
}

double BeamAnalyzer::getGaussSigmaY() const  {
	return gaussSigmaY;
}

double BeamAnalyzer::getGaussAngle() const {
	return gaussAngle;
}

void BeamAnalyzer::resizeImage(double scale, unsigned char **res, int* res_width, int* res_height) {

	Mat src(height, width, CV_8U, data);
	Mat dst;
	resize(src, dst, Size(), scale, scale, INTER_LINEAR);

	*res_width = dst.cols;
	*res_height = dst.rows;
	int size = *res_width * *res_height;
	*res = new unsigned char[size];
	memcpy(*res, dst.data, size);
}
