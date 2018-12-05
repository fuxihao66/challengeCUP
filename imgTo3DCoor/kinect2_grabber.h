// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <boost/asio.hpp>
#include <winsock2.h> 
#pragma comment(lib,"ws2_32.lib")  
#include <Windows.h>
#include <Kinect.h>
#include "myMath.h"
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>



namespace pcl
{
    struct pcl::PointXYZ;
    struct pcl::PointXYZI;
    struct pcl::PointXYZRGB;
    struct pcl::PointXYZRGBA;
    template <typename T> class pcl::PointCloud;

    template<class Interface>
    inline void SafeRelease( Interface *& IRelease )
    {
        if( IRelease != NULL ){
            IRelease->Release();
            IRelease = NULL;
        }
    }

    class Kinect2Grabber : public pcl::Grabber
    {
        public:
            Kinect2Grabber();
            virtual ~Kinect2Grabber() throw ();
            virtual void start();
            virtual void stop();
            virtual bool isRunning() const;
            virtual std::string getName() const;
            virtual float getFramesPerSecond() const;

			typedef void (signal_Kinect2_Image) (boost::shared_ptr<myMath::myImage>&);


        protected:
            

			boost::signals2::signal< signal_Kinect2_Image> * signal_image;

			boost::shared_ptr<myMath::myImage> convertRGBDepthToImage(RGBQUAD* colorBuffer, UINT16* depthBuffer);


            boost::thread thread;
            mutable boost::mutex mutex;

            void threadFunction();

            bool quit;
            bool running;

            HRESULT result;
            IKinectSensor* sensor;
            ICoordinateMapper* mapper;
            IColorFrameSource* colorSource;
            IColorFrameReader* colorReader;
            IDepthFrameSource* depthSource;
            IDepthFrameReader* depthReader;
            IInfraredFrameSource* infraredSource;
            IInfraredFrameReader* infraredReader;

            int colorWidth;
            int colorHeight;
            std::vector<RGBQUAD> colorBuffer;

            int depthWidth;
            int depthHeight;
            std::vector<UINT16> depthBuffer;

            int infraredWidth;
            int infraredHeight;
            std::vector<UINT16> infraredBuffer;
    };

    pcl::Kinect2Grabber::Kinect2Grabber()
        : sensor( nullptr )
        , mapper( nullptr )
        , colorSource( nullptr )
        , colorReader( nullptr )
        , depthSource( nullptr )
        , depthReader( nullptr )
        , infraredSource( nullptr )
        , infraredReader( nullptr )
        , result( S_OK )
        , colorWidth( 1920 )
        , colorHeight( 1080 )
        , colorBuffer()
        , depthWidth( 512 )
        , depthHeight( 424 )
        , depthBuffer()
        , infraredWidth( 512 )
        , infraredHeight( 424 )
        , infraredBuffer()
        , running( false )
        , quit( false )
    {
		//imageData = boost::make_shared<std::vector<myMath::myPoint>>();

        // Create Sensor Instance
        result = GetDefaultKinectSensor( &sensor );
        if( FAILED( result ) ){
            throw std::exception( "Exception : GetDefaultKinectSensor()" );
        }

        // Open Sensor
        result = sensor->Open();
        if( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::Open()" );
        }

        // Retrieved Coordinate Mapper
        result = sensor->get_CoordinateMapper( &mapper );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::get_CoordinateMapper()" );
        }

        // Retrieved Color Frame Source
        result = sensor->get_ColorFrameSource( &colorSource );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::get_ColorFrameSource()" );
        }

        // Retrieved Depth Frame Source
        result = sensor->get_DepthFrameSource( &depthSource );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::get_DepthFrameSource()" );
        }

        // Retrieved Infrared Frame Source
        result = sensor->get_InfraredFrameSource( &infraredSource );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::get_InfraredFrameSource()" );
        }

        // Retrieved Color Frame Size
        IFrameDescription* colorDescription;
        result = colorSource->get_FrameDescription( &colorDescription );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IColorFrameSource::get_FrameDescription()" );
        }

        result = colorDescription->get_Width( &colorWidth ); // 1920
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Width()" );
        }

        result = colorDescription->get_Height( &colorHeight ); // 1080
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Height()" );
        }

        SafeRelease( colorDescription );

        // To Reserve Color Frame Buffer
        colorBuffer.resize( colorWidth * colorHeight );

        // Retrieved Depth Frame Size
        IFrameDescription* depthDescription;
        result = depthSource->get_FrameDescription( &depthDescription );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IDepthFrameSource::get_FrameDescription()" );
        }

        result = depthDescription->get_Width( &depthWidth ); // 512
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Width()" );
        }

        result = depthDescription->get_Height( &depthHeight ); // 424
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Height()" );
        }

        SafeRelease( depthDescription );

        // To Reserve Depth Frame Buffer
        depthBuffer.resize( depthWidth * depthHeight );

        // Retrieved Infrared Frame Size
        IFrameDescription* infraredDescription;
        result = infraredSource->get_FrameDescription( &infraredDescription );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IInfraredFrameSource::get_FrameDescription()" );
        }

        result = infraredDescription->get_Width( &infraredWidth ); // 512
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Width()" );
        }

        result = infraredDescription->get_Height( &infraredHeight ); // 424
        if( FAILED( result ) ){
            throw std::exception( "Exception : IFrameDescription::get_Height()" );
        }

        SafeRelease( infraredDescription );

        // To Reserve Infrared Frame Buffer
        infraredBuffer.resize( infraredWidth * infraredHeight );

		signal_image = createSignal<signal_Kinect2_Image>();
    }




    pcl::Kinect2Grabber::~Kinect2Grabber() throw()
    {
        stop();

		disconnect_all_slots< signal_Kinect2_Image>();


        thread.join();

        // End Processing
        if( sensor ){
            sensor->Close();
        }
        SafeRelease( sensor );
        SafeRelease( mapper );
        SafeRelease( colorSource );
        SafeRelease( colorReader );
        SafeRelease( depthSource );
        SafeRelease( depthReader );
        SafeRelease( infraredSource );
        SafeRelease( infraredReader );
    }

    void pcl::Kinect2Grabber::start()
    {
        // Open Color Frame Reader
        result = colorSource->OpenReader( &colorReader );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IColorFrameSource::OpenReader()" );
        }

        // Open Depth Frame Reader
        result = depthSource->OpenReader( &depthReader );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IDepthFrameSource::OpenReader()" );
        }

        // Open Infrared Frame Reader
        result = infraredSource->OpenReader( &infraredReader );
        if( FAILED( result ) ){
            throw std::exception( "Exception : IInfraredFrameSource::OpenReader()" );
        }

        running = true;

        thread = boost::thread( &Kinect2Grabber::threadFunction, this );
    }

    void pcl::Kinect2Grabber::stop()
    {
        boost::unique_lock<boost::mutex> lock( mutex );

        quit = true;
        running = false;

        lock.unlock();
    }

    bool pcl::Kinect2Grabber::isRunning() const
    {
        boost::unique_lock<boost::mutex> lock( mutex );

        return running;

        lock.unlock();
    }

    std::string pcl::Kinect2Grabber::getName() const
    {
        return std::string( "Kinect2Grabber" );
    }

    float pcl::Kinect2Grabber::getFramesPerSecond() const
    {
        return 30.0f;
    }



	/*boost::signals2::connection Kinect2Grabber::RegisterCallback(const boost::function<void(signal_Kinect2_Image)> & callback) {
		boost::signals2::connection ret = signal_image->connect(callback);
	}*/


    void pcl::Kinect2Grabber::threadFunction()
    {
        while( !quit ){
            boost::unique_lock<boost::mutex> lock( mutex );

            // Acquire Latest Color Frame
            IColorFrame* colorFrame = nullptr;
            result = colorReader->AcquireLatestFrame( &colorFrame );
            if( SUCCEEDED( result ) ){
                // Retrieved Color Data
                result = colorFrame->CopyConvertedFrameDataToArray( colorBuffer.size() * sizeof( RGBQUAD ), reinterpret_cast<BYTE*>( &colorBuffer[0] ), ColorImageFormat::ColorImageFormat_Bgra );
                if( FAILED( result ) ){
                    throw std::exception( "Exception : IColorFrame::CopyConvertedFrameDataToArray()" );
                }
            }
            SafeRelease( colorFrame );

            // Acquire Latest Depth Frame
            IDepthFrame* depthFrame = nullptr;
            result = depthReader->AcquireLatestFrame( &depthFrame );
            if( SUCCEEDED( result ) ){
                // Retrieved Depth Data
                result = depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] );
                if( FAILED( result ) ){
                    throw std::exception( "Exception : IDepthFrame::CopyFrameDataToArray()" );
                }
            }
            SafeRelease( depthFrame );

            // Acquire Latest Infrared Frame
            IInfraredFrame* infraredFrame = nullptr;
            result = infraredReader->AcquireLatestFrame( &infraredFrame );
            if( SUCCEEDED( result ) ){
                // Retrieved Infrared Data
                result = infraredFrame->CopyFrameDataToArray( infraredBuffer.size(), &infraredBuffer[0] );
                if( FAILED( result ) ){
                    throw std::exception( "Exception : IInfraredFrame::CopyFrameDataToArray()" );
                }
            }
            SafeRelease( infraredFrame );

            lock.unlock();

            if( signal_image->num_slots() > 0 ){
				signal_image->operator()(convertRGBDepthToImage(&colorBuffer[0], &depthBuffer[0]));
            }

        }
    }


	boost::shared_ptr<myMath::myImage> pcl::Kinect2Grabber::convertRGBDepthToImage(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{

		int width = static_cast<uint32_t>(depthWidth);
		int height = static_cast<uint32_t>(depthHeight);

		boost::shared_ptr<myMath::myImage> img = boost::make_shared< myMath::myImage >();
		img->resize(width,height);

		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++) {
				pcl::PointXYZRGB point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				img->points[y*depthWidth + x].copyFromPCLPoint(point);
				//std::cout << img->points[y*depthWidth + x].r << std::endl;
			}
		}

		return img;
	}

}

#endif KINECT2_GRABBER

