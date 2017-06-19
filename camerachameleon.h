#ifndef CAMERACHAMELEON_H
#define CAMERACHAMELEON_H

#include <flycapture/FlyCapture2.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <string>

extern "C" {
#include <jpeglib.h>
}

#include <tango.h>
#include "CcdCameraChameleon.h"

using namespace FlyCapture2;
//using namespace CcdCameraChameleon_ns;

class CameraChameleon
{
private:
    /** Bus manager. Used for registering and unregistering callbacks.*/
    BusManager m_busMgr;

    /** Camera arrival callback handle. */
    CallbackHandle m_cbArrivalHandle;

    /** Camera removal callback handle. */
    CallbackHandle m_cbRemovalHandle;

    /** Bus reset callback handle. */
    CallbackHandle m_cbResetHandle;

    /** Camera object. */
    CameraBase* m_pCamera;

    /** Camera information for the camera. */
    CameraInfo m_camInfo;

    /** The raw image returned from the camera. */
    Image m_rawImage;
    bool newImage = false;




	/** Camera parameters **/
	double frameRate, frameRateMin, frameRateMax;
	bool frameRateAuto, frameRateOnOff;
	double brightness, brightnessMin, brightnessMax;
	double exposure, exposureMin, exposureMax;
	bool exposureAuto, exposureOnOff, exposureOnePush;
	double gamma, gammaMin, gammaMax;
	bool gammaOnOff;
	double gain, gainMin, gainMax;
	bool gainAuto, gainOnePush;
	double shutter, shutterMin, shutterMax;
	bool shutterAuto, shutterOnePush;

	/** The encoded data in JPG from image **/
    std::vector<unsigned char>* encodedImage;
    std::vector<unsigned char>* encodedImageExport;
    void imageProcessing();
	void encoding(unsigned char *in_data, int in_width, int in_height, std::vector<unsigned char> *out);
    int quality;
    std::mutex newImageMutex;

    std::string filename = "test";
    std::string format = ".jpg";

    bool b = true;

    std::mutex swapMutex;

    int imageWidthExport;
	int imageHeightExport;
	double AExport, BExport;
	double XExport, YExport;
	double PhiExport;


    /** The temporary image object. */
    Image m_tempImage;

    /** Converted image used for display. */
    Image m_convertedImage;

    /** Image statistics for the current image. */
    ImageStatistics m_imageStats;

    /** Image width. */
    unsigned int m_imageWidth;

    /** Image height. */
    unsigned int m_imageHeight;

    /** Received data size. */
    unsigned int m_receivedDataSize;

    /** Data size. */
    unsigned int m_dataSize;

    /** Bytes per pixel. */
    float m_bytesPerPixel;

    /** Whether the grab thread should keep running. */
    bool m_run;

    bool m_cameraPaused;
    std::mutex playMutex;

    Format7ImageSettings imageSettings;

    int serialNumber;

    std::thread* getImageThread;
    std::mutex* imageMutex;

	std::mutex analisysMutex;

	double A, B;
	double X, Y;
	double Phi;

	double amp;

	double scale = 1.0;

    int *distribX;
    int *distribY;

    TANGO_BASE_CLASS *tangoClass = NULL;

    std::vector<std::string> attrNames;

	/** Program Auto Exposure **/

	bool programAutoExposure = true;
	double autoExposureValue = 240.0;

	std::vector<double> ampValuesExposure;

	void autoExposure();

	std::mutex attrMutex;
	std::mutex propMutex;

public:
    /** Constructor. */
    CameraChameleon(int serialNumber);

    /** Destructor. */
    ~CameraChameleon();

    void setTangoDeviceClass(TANGO_BASE_CLASS *tc);
	void setDevicePipeBlobData(Tango::DevicePipeBlob &);
    /**
     * Start running with the specified serial number.
     *
     * @return Whether the function succeeded.
     */
	bool start(int serialNumber);
    bool reconnect();
	bool waitingConnect();
    /**
     * Stop image capture.
     *
     * @return Whether the function succeeded.
     */
	bool stop();
	void play();
	void pause();

	void setRunStatus(bool);
	bool getRunStatus();

    /** Register all relevant callbacks with the library. */
	void registerCallbacks();

    /** Unregister all relevant callbacks with the library. */
	void unregisterCallbacks();

	void forcePGRY16Mode();

	void run();

    void setQuality(int quality);
    int getQuality() const;

	void imageAnalisys(unsigned char *data, int width, int height);

    /** Get image from camera. */
	bool getImage(unsigned char **image, int *size);
	bool getBigImage(unsigned char *image, int size, int *width, int* height);
	void swapBuffers();
	int getImageWidth();
	int getImageHeight();

	double getA();
	double getB();
	double getX();
	double getY();
	double getPhi();

    double getScale() { return scale; }
	void setScale(double scale) { this->scale = scale; }

    void getDistribX(int** array, int *size);
    void getDistribY(int** array, int *size);

    /** FrameRate functions. */
	void setFrameRate(double frameRate);
	void setFrameRateAuto(bool a);
	void setFrameRateOnOff(bool onOff);
	double getFrameRate();
	double getFrameRateMin();
	double getFrameRateMax();
	bool getFrameRateAuto();
	bool getFrameRateOnOff();

    /** Brightness functions. */
	void setBrightness(double brightness);
	double getBrightness();
	double getBrightnessMin();
	double getBrightnessMax();

    /** Exposure functions. */
	void setExposure(double exposure);
	void setExposureAuto(bool a);
	void setExposureOnOff(bool onOff);
	void setExposureOnePush(bool onePush);
	double getExposure();
	double getExposureMin();
	double getExposureMax();
	bool getExposureAuto();
	bool getExposureOnOff();
	bool getExposureOnePush();

    /** Gamma functions. */
	void setGamma(double gamma);
	void setGammaOnOff(bool onOff);
	double getGamma();
	double getGammaMin();
	double getGammaMax();
	bool getGammaOnOff();


    /** Shutter functions. */
	void setShutter(double shutter);
	void setShutterAuto(bool a);
	void setShutterOnePush(bool onePush);
	double getShutter();
	double getShutterMin();
	double getShutterMax();
	bool getShutterAuto();
	bool getShutterOnePush();

    /** Gain functions. */
	void setGain(double gain);
	void setGainAuto(bool a);
	void setGainOnePush(bool onePush);
	double getGain();
	double getGainMin();
	double getGainMax();
	bool getGainOnePush();
	bool getGainAuto();

    /** Temperature function. */
	double getTemperature();

	/** Program Auto Exposure **/
	void setAutoExposure();

	double getAutoExposureValue();
	void setAutoExposureValue(int val);

/** Private functions **/
private:
	/** Get Property funciton **/

	Property getProperty(PropertyType type);

	/** Get PropInfo function **/
	PropertyInfo getPropertyInfo(PropertyType type);


	/** Read all properties and infos function **/
	void readProperties();

};

#endif // CAMERACHAMELEON_H
