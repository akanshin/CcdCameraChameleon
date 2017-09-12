#ifndef BEAMANALYZER_H
#define BEAMANALYZER_H

#include <vector>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <fstream>


class BeamAnalyzer
{
protected:
	int width = 0;
	int height = 0;
	unsigned char *data = 0;
	bool release = false;

	unsigned char *filtered = 0;

	int *distributionX = 0;
	int *distributionY = 0;

	double gaussAmplitude = 0;
	double gaussCenterX = 0;
	double gaussCenterY = 0;
	double gaussSigmaX = 0;
	double gaussSigmaY = 0;
	double gaussAngle = 0;

	int filterRadius = 10;

	double constComponent = 0;
	double xLinearComponent = 0;
	double yLinearComponent = 0;

	double ampValues[4];
	double ampErrors[4];
	double sigmaValues[4];
	double sigmaErrors[4];


public:
	BeamAnalyzer(unsigned char *data, int width, int height, bool release = false);
	~BeamAnalyzer();

	virtual bool analyze();

	unsigned char *getData() const;
	unsigned char *getFiltered() const;
	int getFilterRadius() const;
	void setFilterRadius(int value);
	double getConstComponent() const;
	double getXLinearComponent() const;
	double getYLinearComponent() const;
	int *getDistributionX() const;
	int *getDistributionY() const;
	double getGaussAmplitude() const;
	double getGaussCenterX() const;
	double getGaussCenterY() const;
	double getGaussSigmaX() const;
	double getGaussSigmaY() const;
	double getGaussAngle() const;

	int getWidth() const;
	int getHeight() const;

	void resizeImage(double scale, unsigned char **res, int* res_width, int* res_height);

protected:
	void beamCheckFunciton(bool *result);
	void getCenterMass(int* array, int size, double *result);
	double getAverage(int* array, int size);
	double getAverage(unsigned char* array, int size);
	void getAverage_p(unsigned char* array, int size, double *result);
	void subAverage(int* array, int size);
	bool subHalf(int* array, int size);
	int getHalf(unsigned char *array, int size, int *min, int *max);
	double getAverageNum(int* array, int size);
	void getSigma(int* array, int size, double *result);

	static int gauss_f(const gsl_vector * x, void *data, gsl_vector * f);
	static int gauss_df(const gsl_vector * x, void *data, gsl_matrix *J);


	void solveGauss(std::vector<double> data_x, std::vector<double> data_y, double cc, double xl, double yl,
					double init_amp, double init_sigma,
					double *evaluated_amp, double *evaluated_sigma,
					double *amp_err, double *sigma_err);
	unsigned char getPixel(unsigned char *data, int width, int height, int x, int y);


	typedef struct FuncPoint {
		double x, y, z;
	}FuncPoint;

	FuncPoint solvePlane(FuncPoint p1, FuncPoint p2, FuncPoint p3);

	long long mtime();
};

#endif // BEAMANALYZER_H
