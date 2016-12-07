/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads camera images using Point Grey's FlyCapture2 library.
 *
 * @author Ulrich Eck <ulrich.eck@tum.de>
 *
 * @todo
 *  - Camera selection
 */

#include <string>
#include <list>
#include <iostream>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/TracingProvider.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <opencv/cv.h>
#include <utVision/OpenCLManager.h>
#include <utUtil/TracingProvider.h>


//#include <Ubitrack/Util/CleanWindows.h>
//#ifndef _WIN32_WINNT
//	#define _WIN32_WINNT WINVER
//#endif
//#include <highgui.h>
#include <FlyCapture2.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.FlyCapture2Fmt7FrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;



namespace {

	class FlyCapturePixelFormatMap 
		: public std::map< std::string, FlyCapture2::PixelFormat >
	{
	public:
		FlyCapturePixelFormatMap()
		{
			using namespace FlyCapture2;
		(*this)["MONO8"] = PIXEL_FORMAT_MONO8;
		(*this)["411YUV8"] = PIXEL_FORMAT_411YUV8;
		(*this)["422YUV8"] = PIXEL_FORMAT_422YUV8;
		(*this)["444YUV8"] = PIXEL_FORMAT_444YUV8;
		(*this)["RGB8"] = PIXEL_FORMAT_RGB8;
		(*this)["MONO16"] = PIXEL_FORMAT_MONO16;
		(*this)["RGB16"] = PIXEL_FORMAT_RGB16;
		(*this)["S_MONO16"] = PIXEL_FORMAT_S_MONO16;
		(*this)["S_RGB16"] = PIXEL_FORMAT_S_RGB16;
		(*this)["RAW8"] = PIXEL_FORMAT_RAW8;
		(*this)["RAW16"] = PIXEL_FORMAT_RAW16;
		(*this)["MONO12"] = PIXEL_FORMAT_MONO12;
		(*this)["RAW12"] = PIXEL_FORMAT_RAW12;
		(*this)["BGR"] = PIXEL_FORMAT_BGR;
		(*this)["BGRU"] = PIXEL_FORMAT_BGRU;
		(*this)["RGB"] = PIXEL_FORMAT_RGB;
		(*this)["RGBU"] = PIXEL_FORMAT_RGBU;
		(*this)["BGR16"] = PIXEL_FORMAT_BGR16;
		(*this)["BGRU16"] = PIXEL_FORMAT_BGRU16;
		(*this)["422YUV8_JPEG"] = PIXEL_FORMAT_422YUV8_JPEG;
		}
	};
	static FlyCapturePixelFormatMap flyCapturePixelFormatMap;


	class FlyCaptureColorProcessingAlgorithmMap
		: public std::map< std::string, FlyCapture2::ColorProcessingAlgorithm >
	{
	public:
		FlyCaptureColorProcessingAlgorithmMap()
		{
			using namespace FlyCapture2;
			(*this)["DEFAULT"] = DEFAULT;
			(*this)["NO_COLOR_PROCESSING"] = NO_COLOR_PROCESSING;
			(*this)["NEAREST_NEIGHBOR"] = NEAREST_NEIGHBOR;
			(*this)["EDGE_SENSING"] = EDGE_SENSING;
			(*this)["HQ_LINEAR"] = HQ_LINEAR;
			(*this)["RIGOROUS"] = RIGOROUS;
			(*this)["IPP"] = IPP;
			(*this)["DIRECTIONAL_FILTER"] = DIRECTIONAL_FILTER;
		}
	};
	static FlyCaptureColorProcessingAlgorithmMap flyCaptureColorProcessingAlgorithmMap;


} // anonymous namespace

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Reads camera images using PGR FlyCapture
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Vision::ImageMeasurement.
 *
 * @par Configuration
 */
class FlyCapture2Fmt7FrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	FlyCapture2Fmt7FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

	/** destructor, waits until thread stops */
	~FlyCapture2Fmt7FrameGrabber();

    /** handler method for incoming pull requests */
	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for FlyCapture2Fmt7FrameGrabber" );
		}
	}

protected:
	// fly capture stuff
	FlyCapture2::PixelFormat m_pixelFormat;
	FlyCapture2::ColorProcessingAlgorithm m_colorProcessingAlgorithm;

	// the serial number
	int m_cameraSerialNumber;

	// the desired framerate
	float m_cameraFrameRate;
	
	// the desired image width
	unsigned int m_desiredWidth;

	// the desired image height
	unsigned int m_desiredHeight;

	// trigger flash
	bool m_triggerFlash;

	// gain
	double m_gainDB;

	// shutter
	double m_shutterMS;

	// shift timestamps (ms)
	int m_timeOffset;

	// automatic upload to GPU?
	bool m_autoGPUUpload;

	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;
};


FlyCapture2Fmt7FrameGrabber::FlyCapture2Fmt7FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_pixelFormat(FlyCapture2::PIXEL_FORMAT_RAW8)
	, m_colorProcessingAlgorithm( FlyCapture2::NEAREST_NEIGHBOR )
	, m_timeOffset( 0 )
	, m_cameraSerialNumber( -1 )
	, m_cameraFrameRate( 30.f )
	, m_desiredWidth( 0 )
	, m_desiredHeight( 0 )
	, m_gainDB( -1 ) // -1 is auto
	, m_shutterMS( -1 ) // -1 is auto
	, m_bStop( false )
	, m_triggerFlash( false )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &FlyCapture2Fmt7FrameGrabber::getIntrinsic, this, _1 ) )
	, m_autoGPUUpload(false)
{
	using namespace FlyCapture2;

	subgraph->m_DataflowAttributes.getAttributeData( "cameraSerialNumber", m_cameraSerialNumber );
	if (m_cameraSerialNumber == -1)
		LOG4CPP_WARN(logger, "No cameraSerialNumber specified - selecting first camera found on bus.." );


	// add more properties
	subgraph->m_DataflowAttributes.getAttributeData( "gainDB", m_gainDB );
	subgraph->m_DataflowAttributes.getAttributeData( "shutterMS", m_shutterMS );

	if ( subgraph->m_DataflowAttributes.getAttributeString( "triggerFlash" ) == "true")
	{
		m_triggerFlash = true;
	}

	if ( subgraph->m_DataflowAttributes.hasAttribute( "pixelFormat" ) )
	{
		std::string sPixelFormat = subgraph->m_DataflowAttributes.getAttributeString( "pixelFormat" );
		if ( flyCapturePixelFormatMap.find( sPixelFormat ) == flyCapturePixelFormatMap.end() )
			UBITRACK_THROW( "unknown pixel format: \"" + sPixelFormat + "\"" );
		m_pixelFormat = flyCapturePixelFormatMap[ sPixelFormat ];
	}

	if ( subgraph->m_DataflowAttributes.hasAttribute( "colorProcessingAlgorithm" ) )
	{
		std::string sColorProcessingAlgorithm = subgraph->m_DataflowAttributes.getAttributeString( "colorProcessingAlgorithm" );
		if ( flyCaptureColorProcessingAlgorithmMap.find( sColorProcessingAlgorithm ) == flyCaptureColorProcessingAlgorithmMap.end() )
			UBITRACK_THROW( "unknown color processing algorithm: \"" + sColorProcessingAlgorithm + "\"" );
		m_colorProcessingAlgorithm = flyCaptureColorProcessingAlgorithmMap[ sColorProcessingAlgorithm ];
	}


	subgraph->m_DataflowAttributes.getAttributeData( "frameRate", m_cameraFrameRate );
	subgraph->m_DataflowAttributes.getAttributeData("desiredWidth", m_desiredWidth);
	subgraph->m_DataflowAttributes.getAttributeData("desiredHeight", m_desiredHeight);


	subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );

	std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString( "intrinsicMatrixFile" );
	std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString( "distortionFile" );

	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isEnabled()) {
		if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
			m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
			LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
		}
		if (m_autoGPUUpload){
			oclManager.activate();
			LOG4CPP_INFO(logger, "Require OpenCLManager");
		}
	}

	m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));

}



void FlyCapture2Fmt7FrameGrabber::stop()
{
	LOG4CPP_TRACE(logger, "Stopping thread...");

	if (m_running)
	{
		LOG4CPP_TRACE(logger, "Thread was running");

		if (m_Thread)
		{
			m_bStop = true;
			m_Thread->join();
		}
		m_running = false;
	}
}


void FlyCapture2Fmt7FrameGrabber::start()
{
	LOG4CPP_TRACE(logger, "Starting thread...");

	if (!m_running)
	{
		m_bStop = false;
		// start thread
		m_Thread.reset(new boost::thread(boost::bind(&FlyCapture2Fmt7FrameGrabber::ThreadProc, this)));
		m_running = true;
	}
}


FlyCapture2Fmt7FrameGrabber::~FlyCapture2Fmt7FrameGrabber()
{
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}
}


void FlyCapture2Fmt7FrameGrabber::ThreadProc()
{
	using namespace FlyCapture2;
	
	LOG4CPP_DEBUG( logger, "Thread started" );

	// do we need to be able to configure the mode ??
	const Mode k_fmt7Mode = MODE_0;
	PixelFormat k_fmt7PixFmt = m_pixelFormat;

	// initialize FlyCapture
	BusManager busMgr;
	unsigned nCameras;
	if ( busMgr.GetNumOfCameras( &nCameras ) != PGRERROR_OK || nCameras == 0 )
	{
		LOG4CPP_ERROR( logger, "No PointGrey cameras found!" );
		return;
	}

	PGRGuid guid;

	if (m_cameraSerialNumber >= 0) {
		if ( busMgr.GetCameraFromSerialNumber((unsigned int) m_cameraSerialNumber, &guid) != PGRERROR_OK )
		{
			LOG4CPP_ERROR( logger, "Error in FlyCapture2::BusManager::GetCameraFromSerialNumber" );
			return;
		}
	}  else {
		if ( busMgr.GetCameraFromIndex( 0, &guid ) != PGRERROR_OK )
		{
			LOG4CPP_ERROR( logger, "Error in FlyCapture2::BusManager::GetCameraFromIndex" );
			return;
		}	
	}

	Camera cam;
	if ( cam.Connect( &guid ) != PGRERROR_OK )
	{
		LOG4CPP_ERROR( logger, "Error in FlyCapture2::Camera::Connect" );
		return;
	}

	// Get the camera information
	Error error;
	CameraInfo camInfo;
	error = cam.GetCameraInfo(&camInfo);
	if (error != PGRERROR_OK)
	{
		LOG4CPP_ERROR(logger, error.GetDescription());
		return;
	}

	LOG4CPP_INFO(logger, "*** CAMERA INFORMATION ***");
	LOG4CPP_INFO(logger, "Serial number -" << camInfo.serialNumber);
	LOG4CPP_INFO(logger, "Camera model - " << camInfo.modelName);
	LOG4CPP_INFO(logger, "Camera vendor - " << camInfo.vendorName);
	LOG4CPP_INFO(logger, "Sensor - " << camInfo.sensorInfo);
	LOG4CPP_INFO(logger, "Resolution - " << camInfo.sensorResolution);
	LOG4CPP_INFO(logger, "Firmware version - " << camInfo.firmwareVersion);
	LOG4CPP_INFO(logger, "Firmware build time - " << camInfo.firmwareBuildTime);


	// Query for available Format 7 modes
	Format7Info fmt7Info;
	bool supported;
	fmt7Info.mode = k_fmt7Mode;
	error = cam.GetFormat7Info( &fmt7Info, &supported );
	if (error != PGRERROR_OK)
	{
		LOG4CPP_ERROR( logger, error.GetDescription() );
		return;
	}

	LOG4CPP_INFO(logger, "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")");
	LOG4CPP_INFO(logger, "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")");
	LOG4CPP_INFO(logger, "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")");
	LOG4CPP_INFO(logger, "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField);

	if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
	{
		// Pixel format not supported!
		LOG4CPP_ERROR(logger, "Pixel format is not supported");
		return;
	}

	// set to maximum image size if not defined
	if (m_desiredWidth == 0) {
		m_desiredWidth = fmt7Info.maxWidth;
	}
	if (m_desiredHeight == 0) {
		m_desiredHeight = fmt7Info.maxHeight;
	}

	if ((fmt7Info.maxWidth < m_desiredWidth) || (fmt7Info.maxHeight < m_desiredHeight)) {
		LOG4CPP_ERROR(logger, "Camera does not support the requested image size: (" << m_desiredWidth << ", " << m_desiredHeight << ")");
		return;
	}

	// XXX todo: center image if width/height smaller than maxWidth/maxHeight

	Format7ImageSettings fmt7ImageSettings;
	fmt7ImageSettings.mode = k_fmt7Mode;
	fmt7ImageSettings.offsetX = 0;
	fmt7ImageSettings.offsetY = 0;
	fmt7ImageSettings.width = m_desiredWidth;
	fmt7ImageSettings.height = m_desiredHeight;
	fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

	bool valid;
	Format7PacketInfo fmt7PacketInfo;

	// Validate the settings to make sure that they are valid
	error = cam.ValidateFormat7Settings(
			&fmt7ImageSettings,
			&valid,
			&fmt7PacketInfo );
	if (error != PGRERROR_OK)
	{
		LOG4CPP_ERROR(logger, error.GetDescription());
		return;
	}

	if ( !valid )
	{
		// Settings are not valid
		LOG4CPP_ERROR(logger, "Format7 settings are not valid");
		return;
	}

	// Set the settings to the camera
	error = cam.SetFormat7Configuration(
			&fmt7ImageSettings,
			fmt7PacketInfo.recommendedBytesPerPacket );
	if (error != PGRERROR_OK)
	{
		LOG4CPP_ERROR(logger, error.GetDescription());
		return;
	}


	// if(m_triggerFlash) {
	// 	// set GPIO to output
	// 	cam.WriteRegister(0x11f8, 0xe0000000);
	// 	// set GPIO signal to delay 0 and duration 2 (last 6 bytes)
	// 	cam.WriteRegister(0x1500, 0x83000003);
	// } else {
	// 	// how to turn it off?!
	// 	//cam.WriteRegister(0x1500, 0x83000000);	
	// }

	// set gain
	if(m_gainDB < 0) {
		Property prop;
		prop.type = GAIN;
		prop.onePush = true;
		prop.onOff = true;
		prop.autoManualMode = true;
		prop.valueA = 0;
		prop.valueB = 0;
		if ( cam.SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting auto Gain." );
	} else {
		Property prop;
		prop.type = GAIN;
		prop.onePush = false;
		prop.onOff = true;
		prop.autoManualMode = false;
		prop.absControl = true;
		prop.absValue = (float)m_gainDB;
		if ( cam.SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting manual Gain." );
	}

	// set shutter
	if(m_shutterMS < 0) {
		Property prop;
		prop.type = SHUTTER;
		prop.onePush = true;
		prop.onOff = true;
		prop.autoManualMode = true;
		prop.valueA = 0;
		prop.valueB = 0;
		if ( cam.SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting auto Gain." );
	} else {
		Property prop;
		prop.type = SHUTTER;
		prop.onePush = false;
		prop.onOff = true;
		prop.autoManualMode = false;
		prop.absControl = true;
		prop.absValue = (float)m_shutterMS;
		if ( cam.SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting manual Gain." );
	}

	// set shutter
	if (m_cameraFrameRate > 0) {
		Property prop;
		prop.type = FRAME_RATE;
		prop.onePush = false;
		prop.onOff = true;
		prop.autoManualMode = false;
		prop.absControl = true;
		prop.absValue = (float)m_cameraFrameRate;
		if (cam.SetProperty(&prop) != PGRERROR_OK)
			LOG4CPP_ERROR(logger, "Error setting FrameRate.");
	}


	if ( cam.StartCapture() != PGRERROR_OK )
	{
		LOG4CPP_ERROR( logger, "Error in FlyCapture2::Camera::StartCapture" );
		return;
	}

	// Retrieve frame rate property
	Property frmRate;
	frmRate.type = FRAME_RATE;
	error = cam.GetProperty(&frmRate);
	if (error != PGRERROR_OK)
	{
		LOG4CPP_ERROR(logger, error.GetDescription());
		return;
	}

	LOG4CPP_INFO(logger, "Frame rate is " << std::fixed << std::setprecision(2) << frmRate.absValue << " fps");

	FlyCapture2::Image::SetDefaultColorProcessing(m_colorProcessingAlgorithm);

	while ( !m_bStop )
	{
		FlyCapture2::Image image;

		if ( cam.RetrieveBuffer( &image ) != PGRERROR_OK )
		{
			LOG4CPP_ERROR( logger, "Could not retrieve buffer" );
			return;
		}
		
		LOG4CPP_DEBUG( logger, "got image" );
		
		if ( !m_running )
			continue;

		boost::shared_ptr< Vision::Image > pColorImage;
		boost::shared_ptr< Vision::Image > pGreyImage;

		// TODO: real timestamps
		Measurement::Timestamp timeStamp = Measurement::now();

#ifdef ENABLE_EVENT_TRACING
		TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), timeStamp, getName().c_str(), "VideoCapture")
#endif

		if ( image.GetPixelFormat() == PIXEL_FORMAT_MONO8 )
		{
			pGreyImage.reset(new Vision::Image( image.GetCols(), image.GetRows(), 1, image.GetData() ) );
			pGreyImage->Mat().step = image.GetStride();
			pGreyImage->set_pixelFormat(Vision::Image::LUMINANCE);

			pGreyImage = m_undistorter->undistort( pGreyImage );

			// Flycapture Segfaults if GPU upload is done before undistortion
			// probably there is some issue with releasing/allocating memory
			if (m_autoGPUUpload){
				Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
				if (oclManager.isInitialized()) {
					//force upload to the GPU
					pGreyImage->uMat();
				}
			}

			// TODO: configureable downsampling for high-res cameras
			// LOG4CPP_DEBUG( logger, "downsampling" );
			// boost::shared_ptr< Vision::Image > pSmallImage( new Vision::Image( 320, 240, 1 ) );
			// cvResize( rawImage, *pSmallImage );
			// m_outPort.send( Vision::ImageMeasurement( timeStamp, pSmallImage ) );

			m_outPort.send( Measurement::ImageMeasurement( timeStamp, pGreyImage ) );
			
			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, pGreyImage->CvtColor( CV_GRAY2RGB, 3 ) ) );
		} 
		else if ( image.GetPixelFormat() == PIXEL_FORMAT_RGB8 )
		{
			pColorImage.reset(new Vision::Image( image.GetCols(), image.GetRows(), 3, image.GetData() ) );
			pColorImage->Mat().step = image.GetStride();
			pColorImage->set_pixelFormat(Vision::Image::RGB);

			pColorImage = m_undistorter->undistort( pColorImage );

			// Flycapture Segfaults if GPU upload is done before undistortion
			// probably there is some issue with releasing/allocating memory
			if (m_autoGPUUpload){
				Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
				if (oclManager.isInitialized()) {
					//force upload to the GPU
					pColorImage->uMat();
				}
			}

			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage ) );
			if ( m_outPort.isConnected() )
				m_outPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );
		} 
		else if ( image.GetPixelFormat() == PIXEL_FORMAT_RAW8 )
		{
			// convert RAW image to RGB8
			FlyCapture2::Image convertedImage;
			image.Convert(PIXEL_FORMAT_BGR, &convertedImage);

			pColorImage.reset(new Vision::Image( convertedImage.GetCols(), convertedImage.GetRows(), 3, convertedImage.GetData() ) );
			pColorImage->Mat().step = convertedImage.GetStride();
			pColorImage->set_pixelFormat(Vision::Image::BGR);

			pColorImage = m_undistorter->undistort( pColorImage );

			// Flycapture Segfaults if GPU upload is done before undistortion
			// probably there is some issue with releasing/allocating memory
			if (m_autoGPUUpload){
				Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
				if (oclManager.isInitialized()) {
					//force upload to the GPU
					pColorImage->uMat();
				}
			}

			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage ) );
			if ( m_outPort.isConnected() )
				m_outPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );
		} 
		else {
			LOG4CPP_DEBUG( logger, "UNKOWN PIXEL FORMAT: " << image.GetPixelFormat() );	
		}
	}

	cam.StopCapture();

	LOG4CPP_DEBUG( logger, "Thread stopped" );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::FlyCapture2Fmt7FrameGrabber > ( "FlyCapture2Fmt7FrameGrabber" );
}

