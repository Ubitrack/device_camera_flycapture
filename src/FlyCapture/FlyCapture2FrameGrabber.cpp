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
 * @author Daniel Pustka <daniel.pustka@ar-tracking.de>
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
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <opencv/cv.h>
#include <utVision/OpenCLManager.h>


//#include <Ubitrack/Util/CleanWindows.h>
//#ifndef _WIN32_WINNT
//	#define _WIN32_WINNT WINVER
//#endif
//#include <highgui.h>
#include <FlyCapture2.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.FlyCapture2FrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;



namespace {
	class FlyCaptureModeMap 
		: public std::map< std::string, FlyCapture2::VideoMode >
	{
	public:
		FlyCaptureModeMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "640x480RGB" ] = VIDEOMODE_640x480RGB;
			(*this)[ "640x480Y8" ] = VIDEOMODE_640x480Y8;
			(*this)[ "800x600RGB" ] = VIDEOMODE_800x600RGB;
			(*this)[ "800x600Y8" ] = VIDEOMODE_800x600Y8;
			(*this)[ "1024x768RGB" ] = VIDEOMODE_1024x768RGB;
			(*this)[ "1024x768Y8" ] = VIDEOMODE_1024x768Y8;
			(*this)[ "1280x960RGB" ] = VIDEOMODE_1280x960RGB;
			(*this)[ "1280x960Y8" ] = VIDEOMODE_1280x960Y8;
			(*this)[ "1600x1200RGB" ] = VIDEOMODE_1600x1200RGB;
			(*this)[ "1600x1200Y8" ] = VIDEOMODE_1600x1200Y8;
		}
	};
	static FlyCaptureModeMap flyCaptureModeMap;

	class FlyCapturePixelFormatMap 
		: public std::map< std::string, FlyCapture2::PixelFormat >
	{
	public:
		FlyCapturePixelFormatMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "MONO8" ] = PIXEL_FORMAT_MONO8;
			(*this)[ "411YUV8" ] = PIXEL_FORMAT_411YUV8;
			(*this)[ "422YUV8" ] = PIXEL_FORMAT_422YUV8;
			(*this)[ "444YUV8" ] = PIXEL_FORMAT_444YUV8;
			(*this)[ "RGB8" ] = PIXEL_FORMAT_RGB8;
			(*this)[ "MONO16" ] = PIXEL_FORMAT_MONO16;
			(*this)[ "RGB16" ] = PIXEL_FORMAT_RGB16;
			(*this)[ "S_MONO16" ] = PIXEL_FORMAT_S_MONO16;
			(*this)[ "S_RGB16" ] = PIXEL_FORMAT_S_RGB16;
			(*this)[ "RAW8" ] = PIXEL_FORMAT_RAW8;
			(*this)[ "RAW16" ] = PIXEL_FORMAT_RAW16;
			(*this)[ "MONO12" ] = PIXEL_FORMAT_MONO12;
			(*this)[ "RAW12" ] = PIXEL_FORMAT_RAW12;
			(*this)[ "BGR" ] = PIXEL_FORMAT_BGR;
			(*this)[ "BGRU" ] = PIXEL_FORMAT_BGRU;
			(*this)[ "RGB" ] = PIXEL_FORMAT_RGB;
			(*this)[ "RGBU" ] = PIXEL_FORMAT_RGBU;
		}
	};
	static FlyCapturePixelFormatMap flyCapturePixelFormatMap;

	class FlyCaptureFrameRateMap 
		: public std::map< std::string, FlyCapture2::FrameRate >
	{
	public:
		FlyCaptureFrameRateMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "1.875" ] = FRAMERATE_1_875;
			(*this)[ "3.75" ] = FRAMERATE_3_75;
			(*this)[ "7.5" ] = FRAMERATE_7_5;
			(*this)[ "15" ] = FRAMERATE_15;
			(*this)[ "30" ] = FRAMERATE_30;
			(*this)[ "60" ] = FRAMERATE_60;
			(*this)[ "120" ] = FRAMERATE_120;
			(*this)[ "240" ] = FRAMERATE_240;
		}
	};
	static FlyCaptureFrameRateMap flyCaptureFrameRateMap;

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
class FlyCapture2FrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	FlyCapture2FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~FlyCapture2FrameGrabber();

    /** handler method for incoming pull requests */
	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for FlyCapture2FrameGrabber" );
		}
	}

protected:
	// fly capture stuff
	FlyCapture2::VideoMode m_videoMode;
	FlyCapture2::FrameRate m_frameRate;
	FlyCapture2::PixelFormat m_pixelFormat;

	// the serial number
	int m_cameraSerialNumber;

	// or the bus index
	int m_cameraBusIndex;


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


FlyCapture2FrameGrabber::FlyCapture2FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_videoMode( FlyCapture2::NUM_VIDEOMODES )
	, m_frameRate( FlyCapture2::NUM_FRAMERATES )
	, m_pixelFormat( FlyCapture2::NUM_PIXEL_FORMATS )
	, m_timeOffset( 0 )
	, m_cameraSerialNumber( -1 )
	, m_cameraBusIndex( -1 )
	, m_gainDB( -1 ) // -1 is auto
	, m_shutterMS( -1 ) // -1 is auto
	, m_bStop( false )
	, m_triggerFlash( false )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &FlyCapture2FrameGrabber::getIntrinsic, this, _1 ) )
	, m_autoGPUUpload(false)
{
	using namespace FlyCapture2;

	// if you have problems with the ipp from flycapture
	// cvUseOptimized( 0 );

	subgraph->m_DataflowAttributes.getAttributeData( "cameraBusIndex", m_cameraBusIndex );
	subgraph->m_DataflowAttributes.getAttributeData( "cameraSerialNumber", m_cameraSerialNumber );
	if ((m_cameraBusIndex == -1) && (m_cameraSerialNumber == -1))
		UBITRACK_THROW( "Need to specify either cameraBusIndex OR cameraSerialNumber" );

	subgraph->m_DataflowAttributes.getAttributeData( "gainDB", m_gainDB );
	subgraph->m_DataflowAttributes.getAttributeData( "shutterMS", m_shutterMS );

	if ( subgraph->m_DataflowAttributes.getAttributeString( "triggerFlash" ) == "true")
	{
		m_triggerFlash = true;
	}


	if ( subgraph->m_DataflowAttributes.hasAttribute( "videoMode" ) )
	{
		std::string sVideoMode = subgraph->m_DataflowAttributes.getAttributeString( "videoMode" );
		if ( flyCaptureModeMap.find( sVideoMode ) == flyCaptureModeMap.end() )
			UBITRACK_THROW( "unknown video mode: \"" + sVideoMode + "\"" );
		m_videoMode = flyCaptureModeMap[ sVideoMode ];
	}

	//if ( subgraph->m_DataflowAttributes.hasAttribute( "pixelFormat" ) )
	//{
	//	std::string sPixelFormat = subgraph->m_DataflowAttributes.getAttributeString( "pixelFormat" );
	//	if ( flyCapturePixelFormatMap.find( sPixelFormat ) == flyCapturePixelFormatMap.end() )
	//		UBITRACK_THROW( "unknown pixel format: \"" + sPixelFormat + "\"" );
	//	m_pixelFormat = flyCapturePixelFormatMap[ sPixelFormat ];
	//}

	if ( subgraph->m_DataflowAttributes.hasAttribute( "frameRate" ) )
	{
		std::string sFrameRate = subgraph->m_DataflowAttributes.getAttributeString( "frameRate" );
		if ( flyCaptureFrameRateMap.find( sFrameRate ) == flyCaptureFrameRateMap.end() )
			UBITRACK_THROW( "unknown frame rate: \"" + sFrameRate + "\"" );
		m_frameRate = flyCaptureFrameRateMap[ sFrameRate ];
	}
	
	if ( ( m_frameRate != NUM_FRAMERATES ) != ( m_videoMode != NUM_VIDEOMODES ) )
		LOG4CPP_WARN( logger, "Both videoMode and frameRate must be set for any value to have an effect!" );

	subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );

	std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString( "intrinsicMatrixFile" );
	std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString( "distortionFile" );

	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isEnabled()) {
		if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
			m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
			LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
		}
	}


	m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));

	// start thread
	m_Thread.reset( new boost::thread( boost::bind( &FlyCapture2FrameGrabber::ThreadProc, this ) ) );
}


FlyCapture2FrameGrabber::~FlyCapture2FrameGrabber()
{
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}
}


void FlyCapture2FrameGrabber::ThreadProc()
{
	using namespace FlyCapture2;
	
	LOG4CPP_DEBUG( logger, "Thread started" );

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
	} else if (m_cameraBusIndex >= 0) {
		if ( busMgr.GetCameraFromIndex( (unsigned int) m_cameraBusIndex, &guid ) != PGRERROR_OK )
		{
			LOG4CPP_ERROR( logger, "Error in FlyCapture2::BusManager::GetCameraFromIndex" );
			return;
		}	
	} else {
		// should never happen ...
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
	
	if ( m_frameRate != NUM_FRAMERATES && m_videoMode != NUM_VIDEOMODES )
	{
		LOG4CPP_INFO( logger, "Setting framerate and videomode" );
		if ( cam.SetVideoModeAndFrameRate( m_videoMode, m_frameRate ) != PGRERROR_OK )
			LOG4CPP_WARN( logger, "Error in FlyCapture2::Camera::SetVideoModeAndFrameRate" );

	}

	if(m_triggerFlash) {
		// set GPIO to output
		cam.WriteRegister(0x11f8, 0xe0000000);
		// set GPIO signal to delay 0 and duration 2 (last 6 bytes)
		cam.WriteRegister(0x1500, 0x83000003);
	} else {
		// how to turn it off?!
		//cam.WriteRegister(0x1500, 0x83000000);	
	}

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



	if ( cam.StartCapture() != PGRERROR_OK )
	{
		LOG4CPP_ERROR( logger, "Error in FlyCapture2::Camera::StartCapture" );
		return;
	}
	
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
	cf->registerComponent< Ubitrack::Drivers::FlyCapture2FrameGrabber > ( "FlyCapture2FrameGrabber" );
}

