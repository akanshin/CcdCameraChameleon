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

    float m_frameRate;

    /** The encoded data in PNG from image **/
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

	double scale = 1.0;

    int *distribX;
    int *distribY;

    TANGO_BASE_CLASS *tangoClass = NULL;

    std::vector<std::string> attrNames;

public:
    /** Constructor. */
    CameraChameleon(int serialNumber);

    /** Destructor. */
    ~CameraChameleon();

    void setTangoDeviceClass(TANGO_BASE_CLASS *tc);
    void setDevicePipeBlobData(Tango::DevicePipeBlob &);
    void destroyDevicePipeBlobData(Tango::DevicePipeBlob &);
    /**
     * Start running with the specified serial number.
     *
     * @return Whether the function succeeded.
     */
    bool Start(int serialNumber);
    bool reconnect();
	bool waitingConnect();
    /**
     * Stop image capture.
     *
     * @return Whether the function succeeded.
     */
    bool Stop();
    void Play();
    void Pause();

    void SetRunStatus(bool);
    bool GetRunStatus();

    /** Register all relevant callbacks with the library. */
    void RegisterCallbacks();

    /** Unregister all relevant callbacks with the library. */
    void UnregisterCallbacks();

    void ForcePGRY16Mode();

    void Run();

    void setQuality(int quality);
    int getQuality() const;

	void ImageAnalisys(unsigned char *data, int width, int height);

    /** Get image from camera. */
    bool GetImage(unsigned char **image, int *size);
	bool GetBigImage(unsigned char *image, int size, int *width, int* height);
	void swapBuffers();
    int GetImageWidth();
    int GetImageHeight();

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
    void SetFrameRate(double frameRate);
    void SetFrameRateAuto(bool a);
    void SetFrameRateOnOff(bool onOff);
    double GetFrameRate();
    bool GetFrameRateAuto();
    bool GetFrameRateOnOff();

    /** Brightness functions. */
    void SetBrightness(double brightness);
    double GetBrightness();

    /** Exposure functions. */
    void SetExposure(double exposure);
    void SetExposureAuto(bool a);
    void SetExposureOnOff(bool onOff);
    void SetExposureOnePush(bool onePush);
    double GetExposure();
    bool GetExposureAuto();
    bool GetExposureOnOff();
    bool GetExposureOnePush();

    /** Gamma functions. */
    void SetGamma(double gamma);
    void SetGammaOnOff(bool onOff);
    double GetGamma();
    bool GetGammaOnOff();


    /** Shutter functions. */
    void SetShutter(double shutter);
    void SetShutterAuto(bool a);
    void SetShutterOnePush(bool onePush);
    double GetShutter();
    bool GetShutterAuto();
    bool GetShutterOnePush();

    /** Gain functions. */
    void SetGain(double gain);
    void SetGainAuto(bool a);
    void SetGainOnePush(bool onePush);
    double GetGain();
    bool GetGainOnePush();
    bool GetGainAuto();

    /** Temperature function. */
    double GetTemperature();

};

#endif // CAMERACHAMELEON_H
