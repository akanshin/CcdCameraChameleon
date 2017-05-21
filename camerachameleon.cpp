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

    Start(serialNumber);


    imageMutex = new mutex();

    encodedImage = new std::vector<unsigned char>();
    encodedImageExport = new std::vector<unsigned char>();

    newImageMutex.lock();

    quality = 75;

    distribX = new int[1280];
    distribY = new int[960];

    getImageThread = new thread(&CameraChameleon::Run, this);
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

void CameraChameleon::ForcePGRY16Mode() {
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

bool CameraChameleon::Start(int serialNumber) {

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
    ForcePGRY16Mode();

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

    SetRunStatus(true);

    return true;
}

//bool CameraChameleon::Start(int serialNumber) {
//	PGRGuid guid;
//	error = m_busMgr.GetCameraFromSerialNumber(serialNumber, &guid);
//	if ( error != PGRERROR_OK ) {
//		return false;
//	}
//	m_pCamera = new Camera();
//	error = m_pCamera->Connect( &guid );
//	if ( error != PGRERROR_OK ) {
//		cout << "Failed to connect to camera" << endl;
//		return false;
//	}
//	error = m_pCamera->StartCapture();
//	if ( error != PGRERROR_OK ) {
//		cout << "Failed to start image capture" << endl;
//		Camera* tmp = m_pCamera;
//		m_pCamera = NULL;
//		delete tmp;
//		return false;
//	}
//	return true;
//}

bool CameraChameleon::reconnect() {
    SetRunStatus(false);
    imageMutex->lock();
    m_cameraPaused = true;
    imageMutex->unlock();
	std::cout << "Reconnecting";
    std::cout.flush();
    int i = 0;
    while (!Start(serialNumber)) {
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
	SetRunStatus(false);
	imageMutex->lock();
	m_cameraPaused = true;
	imageMutex->unlock();
	std::cout << "Waiting for connection\n";
	std::cout.flush();
	while (!Start(serialNumber)) {
		sleep(1);
	}
	imageMutex->lock();
	m_cameraPaused = false;
	imageMutex->unlock();
	std::cout << endl;
	return true;
}

bool CameraChameleon::Stop() {
    imageMutex->lock();
    if( GetRunStatus() != true ) {
        return false;
    }

    SetRunStatus(false);

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

//bool CameraChameleon::Stop() {
//	error = m_pCamera->StopCapture();
//	if ( error != PGRERROR_OK ) {
//		cout << "Failed to stop capture" << endl;
//	}
//	error = m_pCamera->Disconnect();
//	if ( error != PGRERROR_OK ) {
//		cout << "Failed to disconnect" << endl;
//	}
//	return true;
//}

void CameraChameleon::Play() {
    if (!this->GetRunStatus()) {
        this->Start(this->serialNumber);
        playMutex.unlock();
        m_cameraPaused = false;
    }
}
void CameraChameleon::Pause() {
    if (this->GetRunStatus()) {
        playMutex.lock();
        this->Stop();
        m_cameraPaused = true;
    }
}

void CameraChameleon::SetRunStatus(bool runStatus) {
    m_run = runStatus;
}
bool CameraChameleon::GetRunStatus() {
    return m_run;
}

bool CameraChameleon::GetImage(unsigned char** image, int* size) {

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

bool CameraChameleon::GetBigImage(unsigned char *image, int size, int *width, int* height) {
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

int CameraChameleon::GetImageWidth() {
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

int CameraChameleon::GetImageHeight() {
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



void CameraChameleon::Run() {
    long long t1, t2;
    t1 = mtime();
    t2 = t1;
    while (true) {
		playMutex.lock();

		if (m_pCamera == NULL) {
			waitingConnect();
		}

        std::thread processing(&CameraChameleon::imageProcessing, this);

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

		imageMutex->lock();
        m_rawImage = tempImage;
        newImage = true;
        imageMutex->unlock();

        playMutex.unlock();

        t1 = mtime();
        float fps = 1000.0 / (float)(t1-t2);
		//cout << fps << endl;
        t2 = t1;
    }
}

//void CameraChameleon::Run() {

//        Image tempImage;
//        Error error = m_pCamera->RetrieveBuffer( &tempImage );
//        if ( error != PGRERROR_OK ) {
//            cout << "Failed to get image" << endl;
//        }

//}

void CameraChameleon::setQuality(int quality) {
    imageMutex->lock();
    this->quality = quality;
    imageMutex->unlock();
}

int CameraChameleon::getQuality() const {
    return quality;
}


int decodeJpeg(unsigned char *src, unsigned long src_size, unsigned char **buffer, int *size, int *width, int *height) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    int pixel_size;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, src, src_size);

    (void)jpeg_read_header(&cinfo, TRUE);

    //cinfo.output_width = 1280;
    //cinfo.output_height = 960;
    //cinfo.output_components = 1;
    //cinfo.out_color_space = JCS_GRAYSCALE;


    jpeg_start_decompress(&cinfo);

    *width = cinfo.output_width;
    *height = cinfo.output_height;
    pixel_size = cinfo.output_components;

    *size = *width * *height * pixel_size;
    *buffer = new unsigned char[*size];

    int row_stride = cinfo.output_width * cinfo.output_components;

    while (cinfo.output_scanline < cinfo.output_height) {
        unsigned char *buffer_array[1];
        buffer_array[0] = *buffer + (cinfo.output_scanline) * row_stride;

        jpeg_read_scanlines(&cinfo, buffer_array, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    return 0;
}

void CameraChameleon::imageProcessing() {
//	cout << "-> imageProcessing()" << endl;

    if (m_rawImage.GetCols() == 0 || m_rawImage.GetRows() == 0) {
//		cout << "<- imageProcessing()" << endl;
        return;
    }

    m_imageWidth = m_rawImage.GetCols();
    m_imageHeight = m_rawImage.GetRows();
    m_receivedDataSize = m_rawImage.GetReceivedDataSize();
    m_dataSize = m_rawImage.GetDataSize();
    m_bytesPerPixel = m_rawImage.GetBitsPerPixel() / 8.0f;

	unsigned char * copy = new unsigned char[m_rawImage.GetDataSize()];
	memcpy(copy, m_rawImage.GetData(), m_rawImage.GetDataSize());

	std::thread analyseThread(&CameraChameleon::ImageAnalisys, this,
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

//	cout << "join()" << endl;

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
//	cout << "<- imageProcessing()" << endl;
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

void CameraChameleon::destroyDevicePipeBlobData(Tango::DevicePipeBlob & p_data) {

}

void CameraChameleon::ImageAnalisys(unsigned char *data, int width, int height){
//	cout << "-> ImageAnalisys()" << endl;
    analisysMutex.lock();

	A = 0;
	B = 0;
	X = 0;
	Y = 0;
	Phi = 0;

	BeamAnalyzer beam(data, width, height);
	bool result = beam.analyze();
//	beam.printTimeInfo();
	if (result) {


		A = beam.getGaussSigmaX();
		B = beam.getGaussSigmaY();
		X = beam.getGaussCenterX();
		Y = beam.getGaussCenterY();
		Phi = beam.getGaussAngle();
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
void CameraChameleon::SetFrameRate(double frameRate) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return;
    }

    prop.absValue = (float)frameRate;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
    }
}

//void CameraChameleon::SetFrameRate(double frameRate) {
//	Property prop;
//	prop.type = FRAME_RATE;
//	Error error = m_pCamera->GetProperty(&prop);
//	if (error == PGRERROR_PROPERTY_FAILED) {
//		cout << "Failed to get frame rate property" << endl;
//		return;
//	}
//	prop.absValue = (float)frameRate;
//	error = m_pCamera->SetProperty(&prop);
//	if (error == PGRERROR_PROPERTY_FAILED) {
//		cout << "Failed to set frame rate property" << endl;
//	}
//}

void CameraChameleon::SetFrameRateAuto(bool a) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return;
    }

    prop.absControl = !a;
    prop.autoManualMode = a;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
    }
}

//void CameraChameleon::SetFrameRateAuto(bool a) {
//	Property prop;
//	prop.type = FRAME_RATE;
//	Error error = m_pCamera->GetProperty(&prop);
//	if (error == PGRERROR_PROPERTY_FAILED) {
//		cout << "Failed to get frame rate property" << endl;
//		return;
//	}
//	prop.absControl = !a;
//	prop.autoManualMode = a;
//	error = m_pCamera->SetProperty(&prop);
//	if (error == PGRERROR_PROPERTY_FAILED) {
//		cout << "Failed to set frame rate property" << endl;
//	}
//}

void CameraChameleon::SetFrameRateOnOff(bool onOff) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return;
    }

    prop.onOff = onOff;

    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set frame rate property" << endl;
    }
}

double CameraChameleon::GetFrameRate() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return 0;

    }
    m_frameRate = prop.absValue;
    return (double)prop.absValue;
}

bool CameraChameleon::GetFrameRateAuto() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return 0;
    }
    return prop.autoManualMode;
}

bool CameraChameleon::GetFrameRateOnOff() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = FRAME_RATE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get frame rate property" << endl;
        return 0;
    }
    return prop.onOff;
}

/** Brightness functions. */
void CameraChameleon::SetBrightness(double brightness) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = BRIGHTNESS;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get brightness property" << endl;
        return;
    }
    prop.absValue = (float)brightness;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set brightness property" << endl;
    }
}

double CameraChameleon::GetBrightness() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = BRIGHTNESS;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get brightness property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

/** Exposure functions. */
void CameraChameleon::SetExposure(double exposure) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return;
    }
    prop.absValue = (float)exposure;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
    }
}

void CameraChameleon::SetExposureAuto(bool a) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return;
    }
    prop.absControl = !a;
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
    }
}

void CameraChameleon::SetExposureOnOff(bool onOff) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return;
    }
    prop.onOff = onOff;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
    }
}

void CameraChameleon::SetExposureOnePush(bool onePush) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set exposure property" << endl;
    }
}

double CameraChameleon::GetExposure() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

bool CameraChameleon::GetExposureAuto() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return 0;
    }
    return prop.autoManualMode;
}

bool CameraChameleon::GetExposureOnOff() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return 0;
    }
    return prop.onOff;
}

bool CameraChameleon::GetExposureOnePush() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get exposure property" << endl;
        return 0;
    }
    return prop.onePush;
}

/** Gamma functions. */
void CameraChameleon::SetGamma(double gamma) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
        return;
    }
    prop.absValue = (float)gamma;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gamma property" << endl;
    }
}

void CameraChameleon::SetGammaOnOff(bool onOff) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
        return;
    }
    prop.onOff = onOff;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gamma property" << endl;
    }
}

double CameraChameleon::GetGamma() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

bool CameraChameleon::GetGammaOnOff() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = GAMMA;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gamma property" << endl;
        return 0;
    }
    return prop.onOff;
}

/** Shutter functions. */
void CameraChameleon::SetShutter(double shutter) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return;
    }
    prop.absValue = (float)shutter;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
    }
}

void CameraChameleon::SetShutterAuto(bool a) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return;
    }
    prop.absControl = !a;
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
    }
}

void CameraChameleon::SetShutterOnePush(bool onePush) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set shutter property" << endl;
    }
}

double CameraChameleon::GetShutter() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

bool CameraChameleon::GetShutterAuto() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return 0;
    }
    return prop.autoManualMode;
}

bool CameraChameleon::GetShutterOnePush() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = SHUTTER;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get shutter property" << endl;
        return 0;
    }
    return prop.onePush;
}

/** Gain functions. */
void CameraChameleon::SetGain(double gain) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return;
    }
    prop.absValue = (float)gain;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
    }
}

void CameraChameleon::SetGainAuto(bool a) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return;
    }
    prop.absControl = !a;
    prop.autoManualMode = a;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
    }
}

void CameraChameleon::SetGainOnePush(bool onePush) {
    if (m_cameraPaused)
        return;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return;
    }
    prop.onePush = onePush;
    error = m_pCamera->SetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to set gain property" << endl;
    }
}

double CameraChameleon::GetGain() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

bool CameraChameleon::GetGainOnePush() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return 0;
    }
    return prop.onePush;
}

bool CameraChameleon::GetGainAuto() {
    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = GAIN;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get gain property" << endl;
        return 0;
    }
    return prop.autoManualMode;
}

/** Temperature function. */
double CameraChameleon::GetTemperature() {

    if (m_cameraPaused)
        return 0;
    Property prop;
    prop.type = TEMPERATURE;
    Error error = m_pCamera->GetProperty(&prop);
    if (error == PGRERROR_PROPERTY_FAILED) {
        cout << "Failed to get temperature property" << endl;
        return 0;
    }
    return (double)prop.absValue;
}

void CameraChameleon::setTangoDeviceClass(TANGO_BASE_CLASS *tc) {
    this->tangoClass = tc;
}
