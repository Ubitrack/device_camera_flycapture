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
 * Reads camera images using Point Grey's FlyCapture library.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
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

#include <Ubitrack/Util/CleanWindows.h>
#ifndef _WIN32_WINNT
	#define _WIN32_WINNT WINVER
#endif
#include <highgui.h>
#include <PGRFlyCapture.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.FlyCaptureFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;

namespace {
	class FlyCaptureModeMap 
		: public std::map< std::string, FlyCaptureVideoMode >
	{
	public:
		FlyCaptureModeMap()
		{
			(*this)[ "640x480RGB" ] = FLYCAPTURE_VIDEOMODE_640x480RGB;
			(*this)[ "640x480Y8" ] = FLYCAPTURE_VIDEOMODE_640x480Y8;
			(*this)[ "800x600RGB" ] = FLYCAPTURE_VIDEOMODE_800x600RGB;
			(*this)[ "800x600Y8" ] = FLYCAPTURE_VIDEOMODE_800x600Y8;
			(*this)[ "1024x768RGB" ] = FLYCAPTURE_VIDEOMODE_1024x768RGB;
			(*this)[ "1024x768Y8" ] = FLYCAPTURE_VIDEOMODE_1024x768Y8;
			(*this)[ "1280x960RGB" ] = FLYCAPTURE_VIDEOMODE_1280x960RGB;
			(*this)[ "1280x960Y8" ] = FLYCAPTURE_VIDEOMODE_1280x960Y8;
			(*this)[ "1600x1200RGB" ] = FLYCAPTURE_VIDEOMODE_1600x1200RGB;
			(*this)[ "1600x1200Y8" ] = FLYCAPTURE_VIDEOMODE_1600x1200Y8;
			(*this)[ "ANY" ] = FLYCAPTURE_VIDEOMODE_ANY;
		}
	};
	static FlyCaptureModeMap flyCaptureModeMap;

	class FlyCaptureFrameRateMap 
		: public std::map< std::string, FlyCaptureFrameRate >
	{
	public:
		FlyCaptureFrameRateMap()
		{
			(*this)[ "1.875" ] = FLYCAPTURE_FRAMERATE_1_875;
			(*this)[ "3.75" ] = FLYCAPTURE_FRAMERATE_3_75;
			(*this)[ "7.5" ] = FLYCAPTURE_FRAMERATE_7_5;
			(*this)[ "15" ] = FLYCAPTURE_FRAMERATE_15;
			(*this)[ "30" ] = FLYCAPTURE_FRAMERATE_30;
			(*this)[ "50" ] = FLYCAPTURE_FRAMERATE_50;
			(*this)[ "60" ] = FLYCAPTURE_FRAMERATE_60;
			(*this)[ "120" ] = FLYCAPTURE_FRAMERATE_120;
			(*this)[ "240" ] = FLYCAPTURE_FRAMERATE_240;
			(*this)[ "ANY" ] = FLYCAPTURE_FRAMERATE_ANY;
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
class FlyCaptureFrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	FlyCaptureFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~FlyCaptureFrameGrabber();

protected:
	// fly capture stuff
	FlyCaptureContext m_context;

	FlyCaptureVideoMode m_videoMode;
	FlyCaptureFrameRate m_frameRate;

	// shift timestamps (ms)
	int m_timeOffset;

	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< ImageMeasurement > m_colorOutPort;
};


FlyCaptureFrameGrabber::FlyCaptureFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_videoMode( FLYCAPTURE_VIDEOMODE_ANY )
	, m_frameRate( FLYCAPTURE_FRAMERATE_ANY )
	, m_timeOffset( 0 )
	, m_bStop( false )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
{
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

	subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );

	// start thread
	m_Thread.reset( new boost::thread( boost::bind( &FlyCaptureFrameGrabber::ThreadProc, this ) ) );
}


FlyCaptureFrameGrabber::~FlyCaptureFrameGrabber()
{
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}
}


void FlyCaptureFrameGrabber::ThreadProc()
{
	LOG4CPP_DEBUG( logger, "Thread started" );

	// initialize FlyCapture
	if ( flycaptureCreateContext( &m_context ) != FLYCAPTURE_OK )
	{
		LOG4CPP_ERROR( logger, "Error in flycaptureCreateContext" );
		return;
	}

	if ( flycaptureInitialize( m_context, 0 ) != FLYCAPTURE_OK )
	{
		LOG4CPP_ERROR( logger, "Error in flycaptureInitialize" );
		return;
	}

	if ( flycaptureStart( m_context, m_videoMode, m_frameRate ) != FLYCAPTURE_OK )
	{
		LOG4CPP_ERROR( logger, "Error in flycaptureStart" );
		return;
	}

	while ( !m_bStop )
	{
		FlyCaptureImage image;
		if ( flycaptureGrabImage2( m_context, &image ) != FLYCAPTURE_OK )
		{
			LOG4CPP_ERROR( logger, "Error in flycaptureGrabImage2" );
			break;
		}
		LOG4CPP_DEBUG( logger, "got image" );

		if ( !m_running )
			continue;

		// TODO: real timestamps
		Measurement::Timestamp timeStamp = Measurement::now();

		if ( image.pixelFormat == FLYCAPTURE_MONO8 )
		{
			Vision::Image rawImage( image.iCols, image.iRows, 1, image.pData );
			rawImage.widthStep = image.iRowInc;

			// TODO: configureable downsampling for high-res cameras
			// LOG4CPP_DEBUG( logger, "downsampling" );
			// boost::shared_ptr< Vision::Image > pSmallImage( new Vision::Image( 320, 240, 1 ) );
			// cvResize( rawImage, *pSmallImage );
			// m_outPort.send( Vision::ImageMeasurement( timeStamp, pSmallImage ) );

			LOG4CPP_DEBUG( logger, "sending" );
			m_outPort.send( Ubitrack::Vision::ImageMeasurement( timeStamp, rawImage.Clone() ) );
		} 
		else if ( image.pixelFormat == FLYCAPTURE_RGB8 )
		{
			Vision::Image rawImage( image.iCols, image.iRows, 3, image.pData );
			rawImage.widthStep = image.iRowInc;

			if ( m_colorOutPort.isConnected() )
				m_colorOutPort.send( Vision::ImageMeasurement( timeStamp, rawImage.Clone() ) );
			if ( m_outPort.isConnected() )
				m_outPort.send( Vision::ImageMeasurement( timeStamp, rawImage.CvtColor( CV_RGB2GRAY, 1 ) ) );
		}
	}

	flycaptureStop( m_context );

	LOG4CPP_DEBUG( logger, "Thread stopped" );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::FlyCaptureFrameGrabber > ( "FlyCaptureFrameGrabber" );
}

