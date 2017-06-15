#include "camerachameleon.h"
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <sys/time.h>
#include "BeamAnalyzer.h"

using namespace std;
long long mtime()
{
  struct timeval t;

  gettimeofday(&t, NULL);
  long long mt = (long long)t.tv_sec * 1000 + t.tv_usec / 1000;
  return mt;
}

CameraChameleon::CameraChameleon(int serialNumber)
{
    m_pCamera = NULL;
    m_run = false;
    m_cameraPaused = false;

    m_imageWidth = 0;
    m_imageHeight = 0;
    m_bytesPerPixel = 0;
    m_receivedDataSize = 0;
    m_dataSize = 0;

    this->serialNumber = serialNumber;

	start(serialNumber);


    imageMutex = new mutex();

    encodedImage = new std::vector<unsigned char>();
    encodedImageExport = new std::vector<unsigned char>();

    newImageMutex.lock();

    quality = 75;

    distribX = new int[1280];
    distribY = new int[960];

	getImageThread = new thread(&CameraChameleon::run, this);
}

CameraChameleon::~CameraChameleon() {
    if (m_pCamera != NULL) {
        delete m_pCamera;
        m_pCamera = NULL;

        delete getImageThread;
        delete imageMutex;

        delete encodedImage;
        delete encodedImageExport;
    }
}

void CameraChameleon::forcePGRY16Mode() {
    Error error;
    const unsigned int k_imageDataFmtReg = 0x1048;
    unsigned int value = 0;
    error = m_pCamera->ReadRegister( k_imageDataFmtReg, &value );
    if ( error != PGRERROR_OK )
    {
        // Error
    }

    value &= ~(0x1 << 0);

    error = m_pCamera->WriteRegister( k_imageDataFmtReg, value );
    if ( error != PGRERROR_OK )
    {
        // Error
    }
}

bool CameraChameleon::start(int serialNumber) {

    Error error;
    this->serialNumber = serialNumber;
    if ( m_pCamera != NULL ) {
        CameraBase* tmp = m_pCamera;
        m_pCamera = NULL;
        delete tmp;
    }

    PGRGuid guid;

    error = m_busMgr.GetCameraFromSerialNumber(serialNumber, &guid);
	if ( error != PGRERROR_OK ) {
        return false;
    }
    InterfaceType ifType;
    error = m_busMgr.GetInterfaceTypeFromGuid( &guid, &ifType );
    if ( error != PGRERROR_OK ) {
        cout << "Failed to get interface for camera" << endl;
        return false;
    }
    if ( ifType == INTERFACE_GIGE ) {
        m_pCamera = new GigECamera();
    } else {
        m_pCamera = new Camera();
    }
    error = m_pCamera->Connect( &guid );
    if ( error != PGRERROR_OK ) {
        cout << "Failed to connect to camera" << endl;
        return false;
    }

    // Force the camera to PGR's Y16 endianness
	forcePGRY16Mode();

    // Get the camera info and print it out
    error = m_pCamera->GetCameraInfo( &m_camInfo );
    if ( error != PGRERROR_OK ) {
        cout << "Failed to get camera info from camera" << endl;
        return false;
    }

    FC2Version version;
    Utilities::GetLibraryVersion( &version );

    char title[512];
    sprintf(
            title,
            "FlyCap2 %u.%u.%u.%u - %s %s (%u)",
            version.major,
            version.minor,
            version.type,
            version.build,
            m_camInfo.vendorName,
            m_camInfo.modelName,
            m_camInfo.serialNumber );
    cout << endl << title << endl;

    error = m_pCamera->StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ) {
        cout << "Bandwidth exceeded" << endl;

        delete m_pCamera;
        m_pCamera = NULL;
        return false;
    }
    else if ( error != PGRERROR_OK ) {
        cout << "Failed to start image capture" << endl;

        delete m_pCamera;
        m_pCamera = NULL;
        return false;
    }

	setRunStatus(true);

    return true;
}

bool CameraChameleon::reconnect() {
	setRunStatus(false);
    imageMutex->lock();
    m_cameraPaused = true;
    imageMutex->unlock();
	std::cout << "Reconnecting";
    std::cout.flush();
    int i = 0;
	while (!start(serialNumber)) {
        sleep(1);
        std::cout << ".";
        std::cout.flush();
        i++;
    }
    imageMutex->lock();
    m_cameraPaused = false;
    imageMutex->unlock();
    std::cout << endl;
    return true;
}

bool CameraChameleon::waitingConnect() {
	setRunStatus(false);
	imageMutex->lock();
	m_cameraPaused = true;
	imageMutex->unlock();
	std::cout << "Waiting for connection\n";
	std::cout.flush();
	while (!start(serialNumber)) {
		sleep(1);
	}
	imageMutex->lock();
	m_cameraPaused = false;
	imageMutex->unlock();
	std::cout << endl;
	return true;
}

bool CameraChameleon::stop() {
    imageMutex->lock();
	if( getRunStatus() != true ) {
        return false;
    }

	setRunStatus(false);

    // Stop the image capture
    Error error;
    error = m_pCamera->StopCapture();
    if ( error != PGRERROR_OK ) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    m_pCamera->Disconnect();
    imageMutex->lock();

    return true;
}

void CameraChameleon::play() {
	if (!this->getRunStatus()) {
		this->start(this->serialNumber);
        playMutex.unlock();
        m_cameraPaused = false;
    }
}
void CameraChameleon::pause() {
	if (this->getRunStatus()) {
        playMutex.lock();
		this->stop();
        m_cameraPaused = true;
    }
}

void CameraChameleon::setRunStatus(bool runStatus) {
    m_run = runStatus;
}
bool CameraChameleon::getRunStatus() {
    return m_run;
}

bool CameraChameleon::getImage(unsigned char** image, int* size) {

    //newImageMutex.lock();

    swapMutex.lock();
    if(m_cameraPaused) {
        swapMutex.unlock();
        return false;
    }

    *size = encodedImageExport->size();
    *image = new unsigned char[*size];
    memcpy(*image, encodedImageExport->data(), *size);

    swapMutex.unlock();
    return true;
}

bool CameraChameleon::getBigImage(unsigned char *image, int size, int *width, int* height) {
	if (image == NULL || size <= 0)
		return false;
	imageMutex->lock();
	unsigned char *data = m_rawImage.GetData();
	*width = m_rawImage.GetCols();
	*height = m_rawImage.GetRows();
	memcpy(image, data, size);
	imageMutex->unlock();
	return true;
}

void CameraChameleon::swapBuffers() {
    swapMutex.lock();

    std::vector<unsigned char> *tmp = encodedImage;
    encodedImage = encodedImageExport;
    encodedImageExport = tmp;

    imageWidthExport = m_imageWidth;
	imageHeightExport = m_imageHeight;

    AExport = A;
    BExport = B;
    XExport = X;
    YExport = Y;
    PhiExport = Phi;

    swapMutex.unlock();
}

int CameraChameleon::getImageWidth() {
    int result;
    swapMutex.lock();
    if(m_cameraPaused) {
        swapMutex.unlock();
        return 0;
    }
    result = imageWidthExport;
    swapMutex.unlock();
    return result;
}

int CameraChameleon::getImageHeight() {
    int result;
    swapMutex.lock();
    if(m_cameraPaused) {
        swapMutex.unlock();
        return 0;
    }
    result = imageHeightExport;
    swapMutex.unlock();
    return result;
}



void CameraChameleon::run() {
//    long long t1, t2;
//    t1 = mtime();
//    t2 = t1;
    while (true) {
		playMutex.lock();

		if (m_pCamera == NULL) {
			waitingConnect();
		}

        std::thread processing(&CameraChameleon::imageProcessing, this);

		this->readProperties();

        Image tempImage;
        Error error = m_pCamera->RetrieveBuffer( &tempImage );
        if ( error != PGRERROR_OK ) {
            if (error == PGRERROR_IMAGE_CONSISTENCY_ERROR) {
                cout << "Failed to get image: PGRERROR_IMAGE_CONSISTENCY_ERROR" << endl;
                continue;
            } else if (error == PGRERROR_TIMEOUT) {
                cout << "Failed to get image: PGRERROR_TIMEOUT" << endl;
            } else {
                cout << "Failed to get image" << endl;
            }

            m_cameraPaused = true;
			processing.join();
            imageMutex->unlock();
            playMutex.unlock();

            reconnect();
            continue;
        }

		processing.join();

		if (programAutoExposure) {
			this->autoExposure();
		}

		imageMutex->lock();
        m_rawImage = tempImage;
        newImage = true;
        imageMutex->unlock();

        playMutex.unlock();

//        t1 = mtime();
//        float fps = 1000.0 / (float)(t1-t2);
//        t2 = t1;
    }
}

void CameraChameleon::setQuality(int quality) {
    imageMutex->lock();
    this->quality = quality;
    imageMutex->unlock();
}

int CameraChameleon::getQuality() const {
    return quality;
}

void CameraChameleon::imageProcessing() {

	if (m_rawImage.GetCols() == 0 || m_rawImage.GetRows() == 0) {
        return;
    }

    m_imageWidth = m_rawImage.GetCols();
    m_imageHeight = m_rawImage.GetRows();
    m_receivedDataSize = m_rawImage.GetReceivedDataSize();
    m_dataSize = m_rawImage.GetDataSize();
    m_bytesPerPixel = m_rawImage.GetBitsPerPixel() / 8.0f;

	unsigned char * copy = new unsigned char[m_rawImage.GetDataSize()];
	memcpy(copy, m_rawImage.GetData(), m_rawImage.GetDataSize());

	std::thread analyseThread(&CameraChameleon::imageAnalisys, this,
							  m_rawImage.GetData(),
							  m_rawImage.GetCols(),
							  m_rawImage.GetRows());
	std::thread encodingThread(&CameraChameleon::encoding, this,
							   copy,
							   m_rawImage.GetCols(),
							   m_rawImage.GetRows(),
							   encodedImage);

	analyseThread.join();
	encodingThread.join();

	delete[] copy;

    if (tangoClass != NULL) {
        try {

            tangoClass->push_data_ready_event("imageEncodedJpeg");

        } catch(Tango::DevFailed df) {
            cout << "Failed to push_data_ready_event(\"imageEncodedJpeg\")" << endl;
            for (size_t err = 0; err < df.errors.length(); err++) {
                cout << df.errors[err].desc.in() << endl;

            }
        }
    }

	swapBuffers();
    return;
}

void CameraChameleon::encoding(unsigned char *in_data, int in_width, int in_height, std::vector<unsigned char> *out) {

	BeamAnalyzer ba(in_data, in_width, in_height);
	unsigned char *scaled;
	int width, height;

	if (scale > 1.0)
		scale = 1.0;
	else if (scale < 0.01)
		scale = 0.01;

	ba.resizeImage(scale, &scaled, &width, &height);


    out->clear();

    unsigned char *result = NULL;
    unsigned long size = 0;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    jpeg_mem_dest(&cinfo, &result, &size);

	cinfo.image_width=width;
	cinfo.image_height=height;
    cinfo.input_components=1;
    cinfo.in_color_space=JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
	imageMutex->lock();
    jpeg_set_quality(&cinfo, quality, true);
	imageMutex->unlock();

    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    int row_stride;

    row_stride = cinfo.image_width * cinfo.input_components;

	unsigned char * src = scaled;

    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0]=(JSAMPLE *)(src+cinfo.next_scanline*row_stride);
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);


    jpeg_destroy_compress(&cinfo);

    out->insert(out->begin(), result, result+size);

	free(result);

	delete[] scaled;
}

void CameraChameleon::imageAnalisys(unsigned char *data, int width, int height){
//	cout << "-> ImageAnalisys()" << endl;
    analisysMutex.lock();

	A = 0;
	B = 0;
	X = 0;
	Y = 0;
	Phi = 0;
	amp = 0;

	BeamAnalyzer beam(data, width, height);
	bool result = beam.analyze();
//	beam.printTimeInfo();
	if (result) {


		A = beam.getGaussSigmaX();
		B = beam.getGaussSigmaY();
		X = beam.getGaussCenterX();
		Y = beam.getGaussCenterY();
		Phi = beam.getGaussAngle();
		amp = beam.getGaussAmplitude();
	}

    analisysMutex.unlock();
//	cout << "<- ImageAnalisys()" << endl;
}

void CameraChameleon::getDistribX(int** array, int *size) {
    analisysMutex.lock();
    *size = 1280;
    *array = new int[*size];
    memcpy(*array, distribX, sizeof(int) * *size);

    analisysMutex.unlock();
}

void CameraChameleon::getDistribY(int** array, int *size) {
    analisysMutex.lock();
    *size = 960;
    *array = new int[*size];
    memcpy(*array, distribY, sizeof(int) * *size);

    analisysMutex.unlock();
}

double CameraChameleon::getA() {
	double value;
    swapMutex.lock();
    value = AExport;
    swapMutex.unlock();
    return value;
}

double CameraChameleon::getB() {
	double value;
    swapMutex.lock();
    value = BExport;
    swapMutex.unlock();
    return value;
}

double CameraChameleon::getX() {
	double value;
    swapMutex.lock();
    value = XExport;
    swapMutex.unlock();
    return value;
}

double CameraChameleon::getY() {
	double value;
    swapMutex.lock();
    value = YExport;
    swapMutex.unlock();
    return value;
}

double CameraChameleon::getPhi() {
	double value;
    swapMutex.lock();
    value = PhiExport;
    swapMutex.unlock();
    return value;
}

/** FrameRate functions. */
void CameraChameleon::setFrameRate(double frameRate) {
	attrMutex.lock();
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
		attrMutex.unlock();
        return;
    }

    prop.absValue = (float)frameRate;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
	}
	attrMutex.unlock();
}


void CameraChameleon::setFrameRateAuto(bool a) {
	attrMutex.lock();
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
		attrMutex.unlock();
        return;
    }

    prop.autoManualMode = a;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
    }
	attrMutex.unlock();
}

void CameraChameleon::setFrameRateOnOff(bool onOff) {
	attrMutex.lock();
	Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
		attrMutex.unlock();
        return;
    }

    prop.onOff = onOff;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
	}
	attrMutex.unlock();
}

/** Brightness functions. */
void CameraChameleon::setBrightness(double brightness) {
	attrMutex.lock();
	Property prop;
    prop.type = BRIGHTNESS;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get brightness property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absValue = (float)brightness;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set brightness property" << endl;
	}
	attrMutex.unlock();
}

/** Exposure functions. */
void CameraChameleon::setExposure(double exposure) {
	attrMutex.unlock();
	Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absValue = (float)exposure;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setExposureAuto(bool a) {
	attrMutex.unlock();
	Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absControl = !a;
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setExposureOnOff(bool onOff) {
	attrMutex.unlock();
	Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.onOff = onOff;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setExposureOnePush(bool onePush) {
	attrMutex.lock();
	Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
	}
	attrMutex.unlock();
}

/** Gamma functions. */
void CameraChameleon::setGamma(double gamma) {
	attrMutex.lock();
	Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absValue = (float)gamma;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gamma property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setGammaOnOff(bool onOff) {
	attrMutex.lock();
	Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.onOff = onOff;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gamma property" << endl;
	}
	attrMutex.unlock();
}

/** Shutter functions. */
void CameraChameleon::setShutter(double shutter) {
	attrMutex.lock();
	Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absValue = (float)shutter;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setShutterAuto(bool a) {
	attrMutex.lock();
	Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.absControl = !a;
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setShutterOnePush(bool onePush) {
	attrMutex.lock();
	Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
	}
	attrMutex.unlock();
}

/** Gain functions. */
void CameraChameleon::setGain(double gain) {
	attrMutex.lock();
	Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
		attrMutex.unlock();
        return;
    }
	prop.absControl = true;
    prop.absValue = (float)gain;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setGainAuto(bool a) {
	attrMutex.lock();
	Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
		attrMutex.unlock();
        return;
	}
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
	}
	attrMutex.unlock();
}

void CameraChameleon::setGainOnePush(bool onePush) {
	attrMutex.lock();
	Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
		attrMutex.unlock();
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
	}
	attrMutex.unlock();
}

/** Temperature function. */
double CameraChameleon::getTemperature() {
    Property prop;
    prop.type = TEMPERATURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get temperature property" << endl;
        return 0;
	}
    return (double)prop.absValue;
}


double CameraChameleon::getFrameRate() {
	propMutex.lock();
	double val = frameRate;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getFrameRateAuto() {
	propMutex.lock();
	double val = frameRateAuto;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getFrameRateOnOff() {
	propMutex.lock();
	double val = frameRateOnOff;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getBrightness() {
	propMutex.lock();
	double val = brightness;
	propMutex.unlock();

	return val;
}

double CameraChameleon::getExposure() {
	propMutex.lock();
	double val = exposure;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getExposureAuto() {
	propMutex.lock();
	double val = exposureAuto;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getExposureOnOff() {
	propMutex.lock();
	double val = exposureOnOff;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getExposureOnePush() {
	propMutex.lock();
	double val = exposureOnePush;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGamma() {
	propMutex.lock();
	double val = gamma;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getGammaOnOff() {
	propMutex.lock();
	double val = gammaOnOff;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getShutter() {
	propMutex.lock();
	double val = shutter;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getShutterAuto() {
	propMutex.lock();
	double val = shutterAuto;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getShutterOnePush() {
	propMutex.lock();
	double val = shutterOnePush;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGain() {
	propMutex.lock();
	double val = gain;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getGainOnePush() {
	propMutex.lock();
	double val = gainOnePush;
	propMutex.unlock();
	return val;
}

bool CameraChameleon::getGainAuto() {
	propMutex.lock();
	double val = gainAuto;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getFrameRateMin() {
	propMutex.lock();
	double val = frameRateMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getFrameRateMax() {
	propMutex.lock();
	double val = frameRateMax;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getBrightnessMin() {
	propMutex.lock();
	double val = brightnessMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getBrightnessMax() {
	propMutex.lock();
	double val = brightnessMax;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getExposureMin() {
	propMutex.lock();
	double val = exposureMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getExposureMax() {
	propMutex.lock();
	double val = exposureMax;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGammaMin() {
	propMutex.lock();
	double val = gammaMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGammaMax() {
	propMutex.lock();
	double val = gammaMax;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getShutterMin() {
	propMutex.lock();
	double val = shutterMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getShutterMax() {
	propMutex.lock();
	double val = shutterMax;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGainMin() {
	propMutex.lock();
	double val = gainMin;
	propMutex.unlock();
	return val;
}

double CameraChameleon::getGainMax() {
	propMutex.lock();
	double val = gainMax;
	propMutex.unlock();
	return val;
}

Property CameraChameleon::getProperty(PropertyType type) {
	Property prop;
	prop.type = type;
	Error error = m_pCamera->GetProperty(&prop);
	if (error == PGRERROR_PROPERTY_FAILED) {
		cout << "Failed to get temperature property" << endl;
	}
	return prop;
}

PropertyInfo CameraChameleon::getPropertyInfo(PropertyType type) {
	PropertyInfo propInfo;
	propInfo.type = type;
	Error error = m_pCamera->GetPropertyInfo(&propInfo);
	if (error == PGRERROR_PROPERTY_FAILED) {
		cout << "Failed to get info of property with type " << type << endl;
	}
	return propInfo;
}

void CameraChameleon::readProperties() {
	Property prop;
	PropertyInfo propInfo;

	propMutex.lock();

	// Frame rate
	prop = getProperty(FRAME_RATE);
	propInfo = getPropertyInfo(FRAME_RATE);
	frameRate = prop.absValue;
	frameRateMin = propInfo.absMin;
	frameRateMax = propInfo.absMax;
	frameRateAuto = prop.autoManualMode;
	frameRateOnOff = prop.onOff;

	// Brightness
	prop = getProperty(BRIGHTNESS);
	propInfo = getPropertyInfo(BRIGHTNESS);
	brightness = prop.absValue;
	brightnessMin = propInfo.absMin;
	brightnessMax = propInfo.absMax;

	// Exposure
	prop = getProperty(AUTO_EXPOSURE);
	propInfo = getPropertyInfo(AUTO_EXPOSURE);
	exposure = prop.absValue;
	exposureMin = propInfo.absMin;
	exposureMax = propInfo.absMax;
	exposureAuto = prop.autoManualMode;
	exposureOnOff = prop.onOff;
	exposureOnePush = prop.onePush;

	// Gamma
	prop = getProperty(GAMMA);
	propInfo = getPropertyInfo(GAMMA);
	gamma = prop.absValue;
	gammaMin = propInfo.absMin;
	gammaMax = propInfo.absMax;
	gammaOnOff = prop.onOff;

	// Gain
	prop = getProperty(GAIN);
	propInfo = getPropertyInfo(GAIN);
	gain = prop.absValue;
	gainMin = propInfo.absMin;
	gainMax = propInfo.absMax;
	gainAuto = prop.autoManualMode;
	gainOnePush = prop.onePush;

	// Shutter
	prop = getProperty(SHUTTER);
	propInfo = getPropertyInfo(SHUTTER);
	shutter = prop.absValue;
	shutterMin = propInfo.absMin;
	shutterMax = propInfo.absMax;
	shutterAuto = prop.autoManualMode;
	shutterOnePush = prop.onePush;

	propMutex.unlock();
}

void CameraChameleon::setTangoDeviceClass(TANGO_BASE_CLASS *tc) {
    this->tangoClass = tc;
}

void CameraChameleon::autoExposure() {
	this->setExposureAuto(false);
	this->setExposureOnOff(true);
	this->setExposureOnePush(false);
	this->setGainAuto(false);
	this->setGainOnePush(false);
	this->setShutterAuto(false);
	this->setShutterOnePush(false);

	if (fabs(amp - autoExposureValue) < 5.0) {
		programAutoExposure = false;
	} else {
		double newGain = 0.0;
		double newShutter = shutter;

		double mult = autoExposureValue / amp;
		newShutter *= mult;
		if (newShutter < shutterMin) {
			double p1 = newShutter;
			double p0 = shutterMin;
			newShutter = shutterMin;
			newGain = 10.0 * log10(p1/p0);
			if (newGain < gainMin)
				newGain = gainMin;
		} else if (newShutter > shutterMax) {
			double p1 = newShutter;
			double p0 = shutterMax;
			newShutter = shutterMax;
			newGain = 10.0 * log10(p1/p0);
			if (newGain > gainMax)
				newGain = gainMax;
		}
		this->setShutter(newShutter);
		this->setGain(newGain);
	}
}

void CameraChameleon::setAutoExposure() {
	attrMutex.lock();
	programAutoExposure = true;
	attrMutex.unlock();
}

double CameraChameleon::getAutoExposureValue() {
	return autoExposureValue;
}

void CameraChameleon::setAutoExposureValue(int val) {
	attrMutex.lock();
	autoExposureValue = val;
	attrMutex.unlock();
}
