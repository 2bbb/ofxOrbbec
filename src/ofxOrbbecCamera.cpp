
#include "ofxOrbbecCamera.h"
#include <libobsensor/hpp/Utils.hpp>

#if USE_GLOBAL_CONTEXT
static ob::Context context;
#endif

void ofxOrbbecCamera::printInfo() {
#if USE_GLOBAL_CONTEXT
    auto tCtx = &context;
#else
    auto tCtx = make_shared<ob::Context>(); //ofxOrbbecCamera::getContext();
#endif
    // Query the list of connected devices
    auto devList = tCtx->queryDeviceList();
    
    uint32_t devCount = devList->deviceCount();

    const int DEFAULT_WIDTH = 640;
    const int DEFAULT_HEIGHT = 480;

    // traverse the device list and create a pipe
    for(uint32_t deviceIndex = 0; deviceIndex < devCount; deviceIndex++) {
        bool g_isIRUnique;
        std::shared_ptr<ob::Context> m_context = std::make_shared<ob::Context>();
        std::shared_ptr<ob::Pipeline> m_pipeline = std::make_shared<ob::Pipeline>();
        std::shared_ptr<ob::Config> m_config;
        std::shared_ptr<ob::DeviceList> m_deviceList;
        std::shared_ptr<ob::Device> m_device = nullptr;
        std::vector<std::string> m_deviceStringList;
        std::vector<std::string> m_deviceDepthModeStringList;
        int m_curDeviceDepthMode;

        OBCameraParam m_curCameraParams;

        std::shared_ptr<ob::StreamProfileList> m_colorStreamProfileList;
        std::shared_ptr<ob::StreamProfileList> m_depthStreamProfileList;
        std::shared_ptr<ob::StreamProfileList> m_irStreamProfileList;

        std::shared_ptr<ob::VideoStreamProfile> m_colorStreamProfile;
        std::shared_ptr<ob::VideoStreamProfile> m_depthStreamProfile;
        std::shared_ptr<ob::VideoStreamProfile> m_irStreamProfile;

        std::shared_ptr<ob::Frame> m_curColorFrame;
        std::shared_ptr<ob::Frame> m_curDepthFrame;
        std::shared_ptr<ob::Frame> m_curIRFrame;

        std::shared_ptr<ob::FrameSet> m_curFrameSet;
        
        bool m_bIsD2CAlignmentOn = false;
        bool m_bIsSWD2C = false;
        bool m_bisFrameSyncOn = false;

        bool m_bIsDepthOn = false;
        bool m_bIsColorOn = false;
        bool m_bIsIROn = false;
        bool m_bIsPointCloudOn = false;
        //ob::PointCloudFilter m_pointCloud;
        std::vector<OBColorPoint> m_points;
        
        m_device = m_deviceList->getDevice(deviceIndex);
        m_pipeline = std::make_shared<ob::Pipeline>(m_device);
        // Configure which streams to enable or disable for the Pipeline by creating a Config
        m_config = std::make_shared<ob::Config>();
        
        // Retrieve Depth work mode list
        if (m_device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
            try {
                // Get the current Depth work mode
                auto curDepthMode = m_device->getCurrentDepthWorkMode();
                // Obtain the work mode list for Depth camera
                auto depthModeList = m_device->getDepthWorkModeList();
                for (uint32_t i = 0; i < depthModeList->count(); i++) {
                    m_deviceDepthModeStringList.push_back((*depthModeList)[i].name);
                    ofLogNotice() << m_deviceDepthModeStringList.back();
                    if (strcmp(curDepthMode.name, (*depthModeList)[i].name) == 0) {
                        m_curDeviceDepthMode = i;
                    }
                }
                m_device->switchDepthWorkMode(m_deviceDepthModeStringList[0].c_str());
            }
            catch (ob::Error& e) {
                std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
            }
        }
        
        // Obtain all Stream Profiles for Color camera, including resolution, frame rate, and format
        m_colorStreamProfileList = m_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        m_colorStreamProfile = nullptr;
        try {
            // According to the desired configurations to find the corresponding Profile, preference to RGB888 format
            m_colorStreamProfile = m_colorStreamProfileList->getVideoStreamProfile(1920, 1080, OB_FORMAT_RGB888, 30);
        }
        catch (ob::Error& e) {
            // Open default Profile if cannot find the corresponding format
            m_colorStreamProfile = std::const_pointer_cast<ob::StreamProfile>(m_colorStreamProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        
        
        // Obtain all Stream Profiles for Depth camera, including resolution, frame rate, and format
        m_depthStreamProfileList = m_pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        m_depthStreamProfile = nullptr;
        try {
            // According to the desired configurations to find the corresponding Profile, preference to Y16 format
            m_depthStreamProfile = m_depthStreamProfileList->getVideoStreamProfile(1920, 0, OB_FORMAT_Y16, 30);
        }
        catch (ob::Error& e) {
            // Open default Profile if cannot find the corresponding format
            m_depthStreamProfile = std::const_pointer_cast<ob::StreamProfile>(m_depthStreamProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        
        // Try find supported depth to color align hardware mode profile
        m_depthStreamProfileList = m_pipeline->getD2CDepthProfileList(m_colorStreamProfile, ALIGN_D2C_HW_MODE);
        if (m_depthStreamProfileList->count() > 0) {
            m_bIsSWD2C = false;
        }
        else {
            // Try find supported depth to color align software mode profile
            m_depthStreamProfileList = m_pipeline->getD2CDepthProfileList(m_colorStreamProfile, ALIGN_D2C_SW_MODE);
            if (m_depthStreamProfileList->count() > 0) {
                m_bIsSWD2C = true;
            }
        }
        
        // Obtain all Stream Profiles for IR camera, including resolution, frame rate, and format
        try {
            m_irStreamProfileList = m_pipeline->getStreamProfileList(OB_SENSOR_IR);
            m_irStreamProfile = nullptr;
            try {
                // According to the desired configurations to find the corresponding Profile, preference to Y16 format
                m_irStreamProfile = m_irStreamProfileList->getVideoStreamProfile(1920, 1080, OB_FORMAT_Y16, 30);
            }
            catch (ob::Error& e) {
                // Open default Profile if cannot find the corresponding format
                m_irStreamProfile = std::const_pointer_cast<ob::StreamProfile>(m_irStreamProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
            }
        }
        catch (ob::Error& e) {
            g_isIRUnique = false;
            // Dual IR, open with IR Left in default
            m_irStreamProfileList = m_pipeline->getStreamProfileList(OB_SENSOR_IR_LEFT);
            m_irStreamProfile = nullptr;
            try {
                // According to the desired configurations to find the corresponding Profile, preference to Y16 format
                m_irStreamProfile = m_irStreamProfileList->getVideoStreamProfile(DEFAULT_WIDTH, DEFAULT_HEIGHT, OB_FORMAT_Y16, 30);
            }
            catch (ob::Error& e) {
                // Open default Profile if cannot find the corresponding format
                m_irStreamProfile = std::const_pointer_cast<ob::StreamProfile>(m_irStreamProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
            }
        }
        
        // TO OBTAIN A DEFAULT CAMERA PARAMETER
        try {
            m_config->enableStream(m_depthStreamProfile);
            m_pipeline->start(m_config);
            m_curCameraParams = m_pipeline->getCameraParam();
            m_pipeline->stop();
            m_config->disableStream(OB_STREAM_DEPTH);
        }
        catch (ob::Error& e) {
            std::cerr << "startCurDepth: " << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
        }
    }
}

std::vector<std::shared_ptr<ob::DeviceInfo>> ofxOrbbecCamera::getDeviceList() {
    std::vector<std::shared_ptr<ob::DeviceInfo>> dInfo;

#if USE_GLOBAL_CONTEXT
    auto tCtx = &context;
#else
    auto tCtx = make_shared<ob::Context>(); //ofxOrbbecCamera::getContext();
#endif
    // Query the list of connected devices
    auto devList = tCtx->queryDeviceList();
    
    // Get the number of connected devices
    uint32_t devCount = devList->deviceCount();

    // traverse the device list and create a pipe
    for(uint32_t i = 0; i < devCount; i++) {
        auto dev  = devList->getDevice(i);
        auto info = dev->getDeviceInfo();

        ofLogNotice("ofxOrbbecCamera::getDeviceList()") << "["<< i <<"] device is " << info->name() << " serial: " << info->serialNumber();

        dInfo.push_back(info);
    }

    return dInfo; 
}

//Class functions 
ofxOrbbecCamera::~ofxOrbbecCamera(){
    close();
#ifdef OFXORBBEC_DECODE_H264_H265
    if(bInitOneTime){
        avcodec_close(codecContext264);
        av_free(codecContext264);

        avcodec_close(codecContext265);
        av_free(codecContext265);
        bInitOneTime = false;
    }
#endif
}

void ofxOrbbecCamera::close(){
    clear();
}

void ofxOrbbecCamera::clear(){

    if(isThreadRunning()) {
        waitForThread(true, 2000);
    } try {
		if(mPipe) {
			mPipe->stop();
			mPipe.reset();
			pointCloud.reset();
        }
    } catch(ob::Error &e) {
        ofLogError("ofxOrbbecCamera::clear") << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType();
    }

    mCurrentSettings = ofxOrbbec::Settings();
    bNewFrameColor = bNewFrameDepth = bNewFrameIR = false;
    mInternalColorFrameNo = mInternalDepthFrameNo = 0;
    mExtColorFrameNo = mExtDepthFrameNo = 0;
    mPipe.reset();
#if !USE_GLOBAL_CONTEXT
	ctxLocal.reset();
#endif
}

bool ofxOrbbecCamera::open(ofxOrbbec::Settings aSettings){
    clear(); 
	
	ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "log.txt");
	ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_INFO);

#if USE_GLOBAL_CONTEXT
    auto tCtx = &context;
#else
	ctxLocal = make_shared<ob::Context>();
    auto tCtx = ctxLocal;
#endif
    
    std::shared_ptr<ob::Device> device;

    //need depth frames for point cloud
    if( aSettings.bPointCloud && !aSettings.bDepth ){
        aSettings.bDepth = true; 
    }

    mCurrentSettings = aSettings; 

    if( aSettings.ip != ""){
        try{
            device = tCtx->createNetDevice(aSettings.ip.c_str(), 8090);
        }catch(ob::Error &e) {
            ofLogError("ofxOrbbecCamera::open") << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType();
        }
        if(!device){
            return false; 
        }
    }else{

        // Query the list of connected devices
        auto devList = tCtx->queryDeviceList();
        
        // Get the number of connected devices
        int devCount = devList->deviceCount();

        bool openWithSerial = aSettings.deviceSerial != "";

        // traverse the device list and create a pipe
        for(int i = 0; i < devCount; i++) {
            // Get the device and create the pipeline
            auto dev  = devList->getDevice(i);
            auto info = dev->getDeviceInfo();

            std::cout << "[" << i << "] device is " << info->name() << " serial: " << info->serialNumber() << std::endl;

            if( openWithSerial ){
                std::string serialStr(info->serialNumber());
                if( aSettings.deviceSerial == serialStr ){
                    device = dev;
                    break; 
                }
            }else{
                if( aSettings.deviceID == i ){
                    device = dev; 
                    break; 
                }
            }
        }
    }

    if( device ){
        // pass in device to create pipeline
        mPipe = std::make_shared<ob::Pipeline>(device);
    
        if( mPipe ){

             // Create Config for configuring Pipeline work
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

            std::shared_ptr<ob::StreamProfile> depthProfile;
            std::shared_ptr<ob::StreamProfile> colorProfile;
            std::shared_ptr<ob::StreamProfile> irProfile;

            if( aSettings.bDepth ){
                // Get the depth camera configuration list
                auto depthProfileList = mPipe->getStreamProfileList(OB_SENSOR_DEPTH);

                if(0 < mCurrentSettings.depthFrameSize.requestWidth){
                    try {
                        auto requestType = mCurrentSettings.depthFrameSize;

                        // Find the corresponding profile according to the specified format
                        depthProfile = depthProfileList->getVideoStreamProfile(requestType.requestWidth,
                                                                               requestType.requestHeight,
                                                                               requestType.format,
                                                                               requestType.frameRate);
                    }
                    catch(ob::Error &e) {
                        ofLogWarning("ofxOrbbecCamera::open") << " couldn't open depth with requested dimensions / format - using default "; 
                        depthProfile = depthProfileList->getProfile(0);
                    }

                }else{  
                    depthProfile = depthProfileList->getProfile(0);
                }
                
                // enable depth stream
                config->enableStream(depthProfile);
            }

            if( aSettings.bColor ){
                // Get the color camera configuration list
                auto colorProfileList = mPipe->getStreamProfileList(OB_SENSOR_COLOR);

                if(0 < mCurrentSettings.colorFrameSize.requestWidth){
                    try {
                        auto requestType = mCurrentSettings.colorFrameSize;

                        // Find the corresponding profile according to the specified format
                        colorProfile = colorProfileList->getVideoStreamProfile(requestType.requestWidth, requestType.requestHeight, requestType.format, requestType.frameRate);
                    }
                    catch(ob::Error &e) {
                        ofLogWarning("ofxOrbbecCamera::open") << " couldn't open color with requested dimensions / format - using default "; 
                        colorProfile = colorProfileList->getProfile(0);
                    }

                }else{  
                    colorProfile = colorProfileList->getProfile(0);
                }
                
                // enable color stream
                config->enableStream(colorProfile);
            }
            
            if( aSettings.bIR ) {
                auto irProfileList = mPipe->getStreamProfileList(OB_SENSOR_IR);
                
                if(0 < irProfileList->count()) {
                    if(0 < mCurrentSettings.irFrameSize.requestWidth) {
                        try {
                            auto requestType = mCurrentSettings.irFrameSize;
                            
                            irProfile = irProfileList->getVideoStreamProfile(requestType.requestWidth,
                                                                             requestType.requestHeight,
                                                                             requestType.format,
                                                                             requestType.frameRate);
                        }
                        catch(ob::Error &e) {
                            ofLogWarning("ofxOrbbecCamera::open") << "couldn't open depth with requested dimensions / format - using default";
                            irProfile = irProfileList->getProfile(0);
                        }
                    } else {
                        irProfile = irProfileList->getProfile(0);
                    }
                } else {
                    irProfileList = mPipe->getStreamProfileList(OB_SENSOR_IR_LEFT);
                    if(0 < irProfileList->count()) {
                        if(0 < mCurrentSettings.irFrameSize.requestWidth) {
                            try {
                                auto requestType = mCurrentSettings.irFrameSize;
                                
                                irProfile = irProfileList->getVideoStreamProfile(requestType.requestWidth,
                                                                                 requestType.requestHeight,
                                                                                 requestType.format,
                                                                                 requestType.frameRate);
                            }
                            catch(ob::Error &e) {
                                ofLogWarning("ofxOrbbecCamera::open") << "couldn't open depth with requested dimensions / format - using default";
                                irProfile = irProfileList->getProfile(0);
                            }
                        } else {
                            irProfile = irProfileList->getProfile(0);
                        }
                    } else {
                        ofLogError("ofxOrbbecCamera::open") << "no IR...?";
                    }
                }
            }
            
            if( aSettings.bPointCloud ){
                if( aSettings.bColor && aSettings.bPointCloudRGB ){
                    
					// Try find supported depth to color align hardware mode profile
					auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
					if(depthProfileList->count() > 0) {
						config->setAlignMode(ALIGN_D2C_HW_MODE);
					}
					else {
						// Try find supported depth to color align software mode profile
						auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
						if(depthProfileList->count() > 0) {
							config->setAlignMode(ALIGN_D2C_SW_MODE);
						}else{
							config->setAlignMode(ALIGN_DISABLE);
						}
					}
                    
                }else{
                    config->setAlignMode(ALIGN_DISABLE);
				}
            }
            
            // Pass in the configuration and start the pipeline
            mPipe->start(config);

            if( aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                auto cameraParam = mPipe->getCameraParam();

                pointCloud = std::make_shared<ob::PointCloudFilter>();
                pointCloud->setCameraParam(cameraParam);
                
                if( aSettings.bPointCloudRGB ){
                    pointCloud->setCreatePointFormat(OB_FORMAT_RGB_POINT);
					auto param = mPipe->getCalibrationParam(config);
										
					auto     vsp            = colorProfile->as<ob::VideoStreamProfile>();
					uint32_t colorWidth     = vsp->width();
					uint32_t colorHeight    = vsp->height();
					uint32_t tableSize = colorWidth * colorHeight * 2 * sizeof(float);
					xyTableData.resize(tableSize);
					
					if(!ob::CoordinateTransformHelper::transformationInitXYTables(param, OB_SENSOR_COLOR, &xyTableData[0], &tableSize, &xyTables)) {
						ofLogError("ofxOrbbecCamera") << " couldn't init xyTables for depth";
					}
					
                }else{
                    pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
                    
					auto param = mPipe->getCalibrationParam(config);
					auto     vsp            = depthProfile->as<ob::VideoStreamProfile>();
					uint32_t depthWidth     = vsp->width();
					uint32_t depthHeight    = vsp->height();
					uint32_t tableSize = depthWidth * depthHeight * 2 * sizeof(float);
					xyTableData.resize(tableSize);
					
					if(!ob::CoordinateTransformHelper::transformationInitXYTables(param, OB_SENSOR_DEPTH, &xyTableData[0], &tableSize, &xyTables)) {
						ofLogError("ofxOrbbecCamera") << " couldn't init xyTables for depth";
					}

                }
            }

            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);

            startThread();

        } else {
            return false;
        }

    }

    return true; 
}

bool ofxOrbbecCamera::isConnected(){
	if( mPipe && mPipe->getDevice() ){
		return true;
	}
	return false;
}

const ofPixels &ofxOrbbecCamera::getDepthPixels() {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mDepthPixels;
}

const ofShortPixels &ofxOrbbecCamera::getDepthPixelsS() {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mDepthPixelsS;
}

const ofFloatPixels &ofxOrbbecCamera::getDepthPixelsF() {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mDepthPixelsF;
} 

const ofPixels &ofxOrbbecCamera::getColorPixels() {
    mExtColorFrameNo = mInternalColorFrameNo;
    return mColorPixels;
}

const ofPixels &ofxOrbbecCamera::getIRPixels() {
    mExtIRFrameNo = mInternalIRFrameNo;
    return mIRPixels;
}

const ofShortPixels &ofxOrbbecCamera::getIRPixelsS() {
    mExtIRFrameNo = mInternalIRFrameNo;
    return mIRPixelsS;
}

std::vector <glm::vec3> ofxOrbbecCamera::getPointCloud(){
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mPointCloudPtsLocal;
} 

ofMesh ofxOrbbecCamera::getPointCloudMesh(){
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mPointCloudMeshLocal;
}

void ofxOrbbecCamera::update(){
    if( mPipe ){
        bNewFrameDepth = mExtDepthFrameNo < mInternalDepthFrameNo;
        bNewFrameColor = mExtColorFrameNo < mInternalColorFrameNo;
        bNewFrameIR = mExtIRFrameNo < mInternalIRFrameNo;
    }
}

void ofxOrbbecCamera::threadedFunction(){
    while(isThreadRunning()){
        if( mPipe ) {
            auto frameSet = mPipe->waitForFrames(20);
            if(frameSet) {
                
                if( mCurrentSettings.bDepth ){
                    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                    if(depthFrame) {
                        mDepthPixels = processFrame(depthFrame);

                        if( mCurrentSettings.bPointCloud && !mCurrentSettings.bPointCloudRGB ){
                            try {
                                std::shared_ptr<ob::Frame> pointCloudFrame = pointCloud->process(frameSet);
                                pointCloudToMesh(frameSet->depthFrame());
                            }
                            catch(std::exception &e) {
                                ofLogError("ofxOrbbecCamera") << "Get point cloud failed";
                            };
                        } else {
                            mInternalDepthFrameNo++;
                        }
                    }
                }

                if(mCurrentSettings.bColor) {
                    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                    if(colorFrame) {
                        mColorPixels = processFrame(colorFrame);

                        if( mCurrentSettings.bPointCloudRGB ){
                            if(frameSet != nullptr && frameSet->depthFrame() != nullptr && frameSet->colorFrame() != nullptr) {
                                // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                                // millimeter)
                                auto depthValueScale = frameSet->depthFrame()->getValueScale();
                                pointCloud->setPositionDataScaled(depthValueScale);
                                try {
                                    std::shared_ptr<ob::Frame> pointCloudFrame = pointCloud->process(frameSet);
                                    pointCloudToMesh(frameSet->depthFrame(), frameSet->colorFrame());
                                }
                                catch(std::exception &e) {
                                    ofLogError("ofxOrbbecCamera") << "Get point cloud failed";
                                }
                            }
                        }else{
                            //In case h264 and we can't decode - pixels will be empty 
                            if(mColorPixels.getWidth()){
                                mInternalColorFrameNo++; 
                            }
                        }

                

                    }
                }
                
                if(mCurrentSettings.bIR) {
                    auto irFrame = frameSet->irFrame();
                    if(irFrame) {
                        mIRPixelsS = processFrameShortPixels(irFrame);
                        mInternalIRFrameNo++;
                    } else {
                        ofLogError() << "ir frame is null";
                    }
                }
            }
        }

        ofSleepMillis(2);
    }
}
        
bool ofxOrbbecCamera::isFrameNew() const {
    return bNewFrameColor || bNewFrameDepth || bNewFrameIR;
}

bool ofxOrbbecCamera::isFrameNewDepth() const {
    return bNewFrameDepth;
}

bool ofxOrbbecCamera::isFrameNewColor() const {
    return bNewFrameColor;
}

bool ofxOrbbecCamera::isFrameNewIR() const {
    return bNewFrameIR;
}


#ifdef OFXORBBEC_DECODE_H264_H265

void ofxOrbbecCamera::initH26XCodecs(){
    if( !bInitOneTime ){
        // Initialize FFmpeg libraries
        av_register_all();
        avcodec_register_all();
        bInitOneTime = true; 

        // Allocate an AVCodecContext and set its codec
        codec264 = avcodec_find_decoder(AV_CODEC_ID_H264);
        codecContext264 = avcodec_alloc_context3(codec264);
        avcodec_open2(codecContext264, codec264, NULL);

        // // Allocate an AVCodecContext and set its codec
        codec265 = avcodec_find_decoder(AV_CODEC_ID_H265);
        codecContext265 = avcodec_alloc_context3(codec265);
        avcodec_open2(codecContext265, codec265, NULL);
    }
}

ofPixels ofxOrbbecCamera::decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264){
    initH26XCodecs();
    
    AVPacket packet; 
    av_init_packet(&packet);
    packet.data = myData;
    packet.size = dataSize;

    // Allocate an AVFrame for decoded data
    AVFrame* frame = av_frame_alloc();
    
    ofPixels pix;

    auto codecContext = codecContext264;
    if( !bH264 ){
        codecContext = codecContext265; 
    }

    int ret = avcodec_send_packet(codecContext, &packet);
    if (ret < 0) {
        cout << "Error sending a packet for decoding" << endl; 
        return pix; 
    }
 
    int frameDecoded = avcodec_receive_frame(codecContext, frame);

    if( frameDecoded == 0 ){

        // Allocate an AVFrame for RGB data
        AVFrame* rgbFrame = av_frame_alloc();

        rgbFrame->format = AV_PIX_FMT_RGB24; 
        rgbFrame->width = codecContext->width; 
        rgbFrame->height = codecContext->height; 

        av_frame_get_buffer(rgbFrame, 0);

        // Create a sws context for RGB conversion
        swsContext = sws_getCachedContext(swsContext, codecContext->width, codecContext->height, codecContext->pix_fmt,
            codecContext->width, codecContext->height, (AVPixelFormat)rgbFrame->format, SWS_BILINEAR, NULL, NULL, NULL);
        
        // Convert the decoded frame to RGB
        int result = sws_scale(swsContext, frame->data, frame->linesize, 0, frame->height, rgbFrame->data, rgbFrame->linesize);
        pix.setFromPixels((unsigned char * )rgbFrame->data[0], codecContext->width, codecContext->height, 3); 

        av_frame_free(&rgbFrame);
    }

    // Clean up and free allocated memory
    av_frame_free(&frame);

    return pix; 
}

#endif 

ofFloatPixels ofxOrbbecCamera::processFrameFloatPixels(std::shared_ptr<ob::Frame> frame) {
    ofFloatPixels pix;
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        if( !frame ){
            return pix;
        }

        if(frame->type() == OB_FRAME_DEPTH) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                std::vector<float> raw_pixels;
                raw_pixels.resize(videoFrame->width() * videoFrame->height());
                float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();
                std::for_each(raw_pixels.begin(),
                              raw_pixels.end(),
                              [scale](float &x) { x *= scale; });
                std::memcpy(raw_pixels.data(), videoFrame->data(), sizeof(float) * videoFrame->width() * videoFrame->height());
                
                pix.setFromPixels(raw_pixels.data(), videoFrame->width(), videoFrame->height(), 1);
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrame") << " OB_FORMAT not supported " << std::endl;
    }
    return pix;
}

ofShortPixels ofxOrbbecCamera::processFrameShortPixels(std::shared_ptr<ob::Frame> frame) {
    ofShortPixels pix;
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        if( !frame ){
            return pix;
        }

        if(frame->type() == OB_FRAME_DEPTH
           || frame->type() == OB_FRAME_IR
           || frame->type() == OB_FRAME_IR_LEFT
           || frame->type() == OB_FRAME_IR_RIGHT)
        {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                pix.setFromPixels((unsigned short *)videoFrame->data(), videoFrame->width(), videoFrame->height(), 1);
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrame") << " OB_FORMAT not supported " << std::endl;
    }
    return pix;
}

ofPixels ofxOrbbecCamera::processFrame(std::shared_ptr<ob::Frame> frame){
    ofPixels pix;
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        
        if( !frame ){
            return pix; 
        }

        if(frame->type() == OB_FRAME_COLOR) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            switch(videoFrame->format()) {
            case OB_FORMAT_H264:

                #ifdef OFXORBBEC_DECODE_H264_H265 
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), true);
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB ";
                #endif  

            break; 
            case OB_FORMAT_H265:

                #ifdef OFXORBBEC_DECODE_H264_H265 
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), false);
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB ";
                #endif 

            break; 
            case OB_FORMAT_MJPG: {

#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                cv::cvtColor(rstMat, rstMat, cv::COLOR_BGR2RGB);
#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set color format to OB_FORMAT_RGB";
#endif

            } break;
            case OB_FORMAT_NV21: {
                cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_NV21);
            } break;
            case OB_FORMAT_YUYV:
            case OB_FORMAT_YUY2: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB);
            } break;
            case OB_FORMAT_RGB: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
                rstMat = rawMat;
            } break;
            case OB_FORMAT_UYVY: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_UYVY);
            } break;
            default:
                break;
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_DEPTH) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                // depth frame pixel value multiply scale to get distance in millimeter
                float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();

                // threshold to 5.46m
                cv::threshold(rawMat, cvtMat, 5460.0f / scale, 0, cv::THRESH_TRUNC);
                cvtMat.convertTo(cvtMat, CV_8UC1, scale * 0.05);
                rstMat = cvtMat;//cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT || frame->type() == OB_FRAME_IR_RIGHT) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
                rstMat = rawMat;//cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_Y8) {
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_8UC1, videoFrame->data());
                rstMat = rawMat;//cv::cvtColor(rawMat * 2, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_MJPG) {
#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                rstMat = rawMat;//cv::cvtColor(rstMat * 2, rstMat, cv::COLOR_GRAY2RGB);
#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set IR format to OB_FORMAT_Y16 or OB_FORMAT_Y8";
#endif
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrame") << " OB_FORMAT not supported " << std::endl; 
    }
    return pix; 
}

void ofxOrbbecCamera::pointCloudToMesh(std::shared_ptr<ob::DepthFrame> depthFrame,
                                       std::shared_ptr<ob::ColorFrame> colorFrame)
{
    if( depthFrame ){
    
		bool bRGB = false;
		if(colorFrame){
			bRGB = true;
		}

        int numPoints = 0;
		uint32_t pointcloudSize = 0;
		
		 if(bRGB){
			numPoints = colorFrame->width() * colorFrame->height();
            pointcloudSize = numPoints * sizeof(OBColorPoint);
        }else{
			numPoints = depthFrame->width() * depthFrame->height();
            pointcloudSize = numPoints * sizeof(OBPoint);
        }
		
		std::vector <uint8_t> pointcloudData;
		if( mPointcloudData.size() != pointcloudSize){
			mPointcloudData.resize(pointcloudSize);
		}

        mPointCloudMesh = ofMesh();
        mPointCloudPts.clear();
        mPointCloudPts.reserve(numPoints);
        mPointCloudMesh.setMode(OF_PRIMITIVE_POINTS);

        if( bRGB ){
			OBColorPoint *point = (OBColorPoint *)&mPointcloudData[0];
			ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(&xyTables, depthFrame->data(), colorFrame->data(), point);

			point = (OBColorPoint *)&mPointcloudData[0];

            std::vector <ofFloatColor> tColors;
            tColors.reserve(numPoints);

            for(int i = 0; i < numPoints; i++) {
                auto pt = glm::vec3(point->x, -point->y, -point->z);

                mPointCloudPts.push_back(pt);
                tColors.push_back(ofColor((int)point->r, (int)point->g, (int)point->b, 255));

                point++;
            }
            mPointCloudMesh.clear();
            mPointCloudMesh.addColors(tColors);

        }else{
			OBPoint *point = (OBPoint *)&mPointcloudData[0];
			ob::CoordinateTransformHelper::transformationDepthToPointCloud(&xyTables, depthFrame->data(), point);

			point = (OBPoint *)&mPointcloudData[0];

            for(int i = 0; i < numPoints; i++) {
                auto pt = glm::vec3(point->x, -point->y, -point->z);

                mPointCloudPts.push_back(pt);
                point++;
            }
            mPointCloudMesh.clear();
        }

        mPointCloudMesh.addVertices(mPointCloudPts);
        mPointCloudMesh.setupIndicesAuto(); 

        if( lock() ){
            mPointCloudMeshLocal = mPointCloudMesh; 
            mPointCloudPtsLocal = mPointCloudPts;
            if( bRGB ){
                mInternalColorFrameNo++;
            }else{
                mInternalDepthFrameNo++;
            }
            unlock();
        }
    }
}
