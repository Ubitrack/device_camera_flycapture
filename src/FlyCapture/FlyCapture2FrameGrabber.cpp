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
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/Image.h>
#include <opencv/cv.h>

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

protected:
	// fly capture stuff
	FlyCapture2::VideoMode m_videoMode;
	FlyCapture2::FrameRate m_frameRate;

	// shift timestamps (ms)
	int m_timeOffset;

	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
};


FlyCapture2FrameGrabber::FlyCapture2FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_videoMode( FlyCapture2::NUM_VIDEOMODES )
	, m_frameRate( FlyCapture2::NUM_FRAMERATES )
	, m_timeOffset( 0 )
	, m_bStop( false )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
{
	using namespace FlyCapture2;

	// if you have problems with the ipp from flycapture
	// cvUseOptimized( 0 );
	
	if ( subgraph->m_DataflowAttributes.hasAttribute( "videoMode" ) )
	{
		std::string sVideoMode = subgraph->m_DataflowAttributes.getAttributeString( "videoMode" );
		if ( flyCaptureModeMap.find( sVideoMode ) == flyCaptureModeMap.end() )
			UBITRACK_THROW( "unknown video mode: \"" + sVideoMode + "\"" );
		m_videoMode = flyCaptureModeMap[ sVideoMode ];
	}

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
	if ( busMgr.GetCameraFromIndex( 0, &guid ) != PGRERROR_OK )
	{
		LOG4CPP_ERROR( logger, "Error in FlyCapture2::BusManager::GetCameraFromIndex" );
		return;
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

		// TODO: real timestamps
		Measurement::Timestamp timeStamp = Measurement::now();

		if ( image.GetPixelFormat() == PIXEL_FORMAT_MONO8 )
		{
			Vision::Image rawImage( image.GetCols(), image.GetRows(), 1, image.GetData() );
			rawImage.widthStep = image.GetStride();

			// TODO: configureable downsampling for high-res cameras
			// LOG4CPP_DEBUG( logger, "downsampling" );
			// boost::shared_ptr< Vision::Image > pSmallImage( new Vision::Image( 320, 240, 1 ) );
			// cvResize( rawImage, *pSmallImage );
			// m_outPort.send( Vision::ImageMeasurement( timeStamp, pSmallImage ) );

			LOG4CPP_DEBUG( logger, "sending" );
			m_outPort.send( Measurement::ImageMeasurement( timeStamp, rawImage.Clone() ) );
			
			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, rawImage.CvtColor( CV_GRAY2RGB, 3 ) ) );
		} 
		else if ( image.GetPixelFormat() == PIXEL_FORMAT_RGB8 )
		{
			Vision::Image rawImage( image.GetCols(), image.GetRows(), 3, image.GetData() );
			rawImage.widthStep = image.GetStride();

			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, rawImage.Clone() ) );
			if ( m_outPort.isConnected() )
				m_outPort.send( Measurement::ImageMeasurement( timeStamp, rawImage.CvtColor( CV_RGB2GRAY, 1 ) ) );
		}
	}

	cam.StopCapture();

	LOG4CPP_DEBUG( logger, "Thread stopped" );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::FlyCapture2FrameGrabber > ( "FlyCapture2FrameGrabber" );
}

