// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"

#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <android/log.h>
#include <android/native_window.h>
#include "media/NdkMediaCodec.h"
#include "media/NdkMediaMuxer.h"
#include "media/NdkMediaExtractor.h"
#include "media/NdkMediaFormat.h"

#define INPUT_TIMEOUT_MS 2000

#define COLOR_FormatYUV420Planar 19
#define COLOR_FormatYUV420SemiPlanar 21
#define COLOR_FormatSurface 0x7f000789 //See https://developer.android.com/reference/android/media/MediaCodecInfo.CodecCapabilities for codes

using namespace cv;

#define TAG "NativeCodec"
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)


static inline void deleter_AMediaExtractor(AMediaExtractor *extractor) {
    AMediaExtractor_delete(extractor);
}

static inline void deleter_AMediaCodec(AMediaCodec *codec) {
    AMediaCodec_stop(codec);
    AMediaCodec_delete(codec);
}

static inline void deleter_AMediaFormat(AMediaFormat *format) {
    AMediaFormat_delete(format);
}

class AndroidMediaNdkCapture : public IVideoCapture
{
    struct YUV2RGBCoef {
        YUV2RGBCoef()
        {
            reset();
        }

        void reset()
        {
            red.yAdd = 0.0;
            red.yMul = 0.0;
            red.uAdd = 0.0;
            red.uMul = 0.0;
            red.vAdd = 0.0;
            red.vMul = 0.0;
            red.add = 0.0;

            green.yAdd = 0.0;
            green.yMul = 0.0;
            green.uAdd = 0.0;
            green.uMul = 0.0;
            green.vAdd = 0.0;
            green.vMul = 0.0;
            green.add = 0.0;

            blue.yAdd = 0.0;
            blue.yMul = 0.0;
            blue.uAdd = 0.0;
            blue.uMul = 0.0;
            blue.vAdd = 0.0;
            blue.vMul = 0.0;
            blue.add = 0.0;
        }

        struct {
            double yAdd;
            double yMul;
            double uAdd;
            double uMul;
            double vAdd;
            double vMul;
            double add;
        } red, green, blue;
    };

    static void calculateYUV2RGBCoef(YUV2RGBCoef& coef, double KR, double KB, int range) {
        /*
            REF: http://avisynth.nl/index.php/Color_conversions
        */

        double KG = 1 - KR - KB;

        coef.reset();

        if (2 == range) {
            /*
            r = (255/219)*y + (255/112)*v*(1-Kr) - (255*16/219 + 255*128/112*(1-Kr))
            g = (255/219)*y - (255/112)*u*(1-Kb)*Kb/Kg - (255/112)*v*(1-Kr)*Kr/Kg
                - (255*16/219 - 255/112*128*(1-Kb)*Kb/Kg - 255/112*128*(1-Kr)*Kr/Kg)
            b = (255/219)*y + (255/112)*u*(1-Kb) - (255*16/219 + 255*128/112*(1-Kb))
            */

            coef.red.yMul = 255.0 / 219.0;
            coef.red.vMul = (255.0 / 112.0) * (1.0 - KR);
            coef.red.add = - (255.0 * 16.0 / 219.0 + (255.0 * 128.0 / 112.0) * (1.0 - KR));

            coef.green.yMul = 255.0 / 219.0;
            coef.green.uMul = - (255.0 / 112.0) * (1.0 - KB) * KB / KG;
            coef.green.vMul = - (255.0 / 112.0) * (1.0 - KR) * KR / KG;
            coef.green.add = - (255.0 * 16.0 / 219.0 - 255.0 / 112.0 * 128.0 * (1.0 - KB) * KB / KG - 255.0 / 112.0 * 128.0 * (1.0 - KR) * KR / KG);

            coef.blue.yMul = 255.0 / 219.0;
            coef.blue.uMul = (255.0 / 112.0) * (1.0 - KB);
            coef.blue.add = - (255.0 * 16.0 / 219.0 + 255.0 * 128.0 / 112.0 * (1.0 - KB));

            return;
        }

        /*
        r = y + 2*(v-128)*(1-Kr)
        g = y - 2*(u-128)*(1-Kb)*Kb/Kg - 2*(v-128)*(1-Kr)*Kr/Kg
        b = y + 2*(u-128)*(1-Kb)
        */

        coef.red.yMul = 1.0;
        coef.red.vAdd = -128.0;
        coef.red.vMul = 2.0 * (1.0 - KR);

        coef.green.yMul = 1.0;
        coef.green.uAdd = -128.0;
        coef.green.uMul = -2.0 * (1 - KB) * KB / KG;
        coef.green.vAdd = -128.0;
        coef.green.vMul = -2.0 * (1 - KR) * KR / KG;

        coef.blue.yMul = 1.0;
        coef.blue.uAdd = -128.0;
        coef.blue.uMul = 2.0 * (1.0 - KB);
    }

    YUV2RGBCoef yuv2rgbCoef;

    void updateFormat(std::shared_ptr<AMediaFormat>& format) {
        int32_t colorRange = 2, colorStandard = 1;

        AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_COLOR_RANGE, &colorRange);
        AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_COLOR_STANDARD, &colorStandard);
        AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_BIT_RATE, &bitrate);

        if (0 == colorRange) colorRange = 2;
        if (0 == colorStandard) colorStandard = 1;

        LOGE("color range: %d", colorRange);
        LOGE("color standard: %d", colorStandard);
        LOGE("bitrate: %d", bitrate);

        double KR = 0.2126, KB = 0.0722; //COLOR_STANDARD_BT709

        switch (colorStandard)
        {
        case 2: //COLOR_STANDARD_BT601_PAL
            KR = 0.299;
            KB = 0.114;
            break;

        case 4: //COLOR_STANDARD_BT601_NTSC
            KR = 0.299;
            KB = 0.114;
            break;

        case 6: //COLOR_STANDARD_BT2020
            KR = 0.2627;
            KB = 0.0593;
        }

        calculateYUV2RGBCoef(yuv2rgbCoef, KR, KB, colorRange);
    }

    void yuv2rgb(int uOffset, int vOffset, int uvStep, int uvStride, int uvScale) {
        LOGE("buffer size: %d, uOffset: %d, vOffset: %d, uvStep: %d, uvStride: %d, uvScale: %d", (int)buffer.size(), uOffset, vOffset, uvStep, uvStride, uvScale);

        frame.create(videoHeight, videoWidth, CV_8UC3);

        frame.forEach<Point3_<uint8_t>>(
            [this, uOffset, vOffset, uvStep, uvStride, uvScale](Point3_<uint8_t>& pixel, const int position[]) -> void {
                const int row = position[0];
                const int column = position[1];

                const int y = buffer[row * frameStride + column];
                const int uvDelta = (row / uvScale) * uvStride + (column / uvScale) * uvStep;
                const int u = buffer[uOffset + uvDelta];
                const int v = buffer[vOffset + uvDelta];

                const int r = int(
                                (y + yuv2rgbCoef.red.yAdd) * yuv2rgbCoef.red.yMul +
                                (u + yuv2rgbCoef.red.uAdd) * yuv2rgbCoef.red.uMul +
                                (v + yuv2rgbCoef.red.vAdd) * yuv2rgbCoef.red.vMul +
                                yuv2rgbCoef.red.add
                            );

                const int g = int(
                                (y + yuv2rgbCoef.green.yAdd) * yuv2rgbCoef.green.yMul +
                                (u + yuv2rgbCoef.green.uAdd) * yuv2rgbCoef.green.uMul +
                                (v + yuv2rgbCoef.green.vAdd) * yuv2rgbCoef.green.vMul +
                                yuv2rgbCoef.green.add
                            );

                const int b = int(
                                (y + yuv2rgbCoef.blue.yAdd) * yuv2rgbCoef.blue.yMul +
                                (u + yuv2rgbCoef.blue.uAdd) * yuv2rgbCoef.blue.uMul +
                                (v + yuv2rgbCoef.blue.vAdd) * yuv2rgbCoef.blue.vMul +
                                yuv2rgbCoef.blue.add
                            );

                pixel.x = (r <= 0) ? 0 : ((r >= 255) ? 255 : r);
                pixel.y = (g <= 0) ? 0 : ((g >= 255) ? 255 : g);
                pixel.z = (b <= 0) ? 0 : ((b >= 255) ? 255 : b);
            }
        );
    }

public:
    AndroidMediaNdkCapture():
        sawInputEOS(false), sawOutputEOS(false),
        frameStride(0), frameWidth(0), frameHeight(0), colorFormat(0),
        videoWidth(0), videoHeight(0),
        videoFrameCount(0),
        videoRotation(0), videoRotationCode(-1),
        videoOrientationAuto(false),
        bitrate(0) {}

    std::shared_ptr<AMediaExtractor> mediaExtractor;
    std::shared_ptr<AMediaCodec> mediaCodec;
    bool sawInputEOS;
    bool sawOutputEOS;
    int32_t frameStride;
    int32_t frameWidth;
    int32_t frameHeight;
    int32_t colorFormat;
    int32_t videoWidth;
    int32_t videoHeight;
    float videoFrameRate;
    int32_t videoFrameCount;
    int32_t videoRotation;
    int32_t videoRotationCode;
    bool videoOrientationAuto;
    int32_t bitrate;
    std::vector<uint8_t> buffer;
    Mat frame;

    ~AndroidMediaNdkCapture() { cleanUp(); }

    bool decodeFrame() {
        while (!sawInputEOS || !sawOutputEOS) {
            if (!sawInputEOS) {
                auto bufferIndex = AMediaCodec_dequeueInputBuffer(mediaCodec.get(), INPUT_TIMEOUT_MS);
                LOGV("input buffer %zd", bufferIndex);
                if (bufferIndex >= 0) {
                    size_t bufferSize;
                    auto inputBuffer = AMediaCodec_getInputBuffer(mediaCodec.get(), bufferIndex, &bufferSize);
                    auto sampleSize = AMediaExtractor_readSampleData(mediaExtractor.get(), inputBuffer, bufferSize);
                    if (sampleSize < 0) {
                        sampleSize = 0;
                        sawInputEOS = true;
                        LOGV("EOS");
                    }
                    auto presentationTimeUs = AMediaExtractor_getSampleTime(mediaExtractor.get());

                    AMediaCodec_queueInputBuffer(mediaCodec.get(), bufferIndex, 0, sampleSize,
                        presentationTimeUs, sawInputEOS ? AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM : 0);
                    AMediaExtractor_advance(mediaExtractor.get());
                }
            }

            if (!sawOutputEOS) {
                AMediaCodecBufferInfo info;
                auto bufferIndex = AMediaCodec_dequeueOutputBuffer(mediaCodec.get(), &info, 0);
                if (bufferIndex >= 0) {
                    size_t bufferSize = 0;
                    auto mediaFormat = std::shared_ptr<AMediaFormat>(AMediaCodec_getOutputFormat(mediaCodec.get()), deleter_AMediaFormat);
                    AMediaFormat_getInt32(mediaFormat.get(), AMEDIAFORMAT_KEY_WIDTH, &frameWidth);
                    AMediaFormat_getInt32(mediaFormat.get(), AMEDIAFORMAT_KEY_STRIDE, &frameStride);
                    AMediaFormat_getInt32(mediaFormat.get(), AMEDIAFORMAT_KEY_HEIGHT, &frameHeight);
                    AMediaFormat_getInt32(mediaFormat.get(), AMEDIAFORMAT_KEY_COLOR_FORMAT, &colorFormat);
                    uint8_t* codecBuffer = AMediaCodec_getOutputBuffer(mediaCodec.get(), bufferIndex, &bufferSize);
                    buffer = std::vector<uint8_t>(codecBuffer, codecBuffer + bufferSize);
                    LOGV("colorFormat: %d", colorFormat);
                    LOGV("buffer size: %zu", bufferSize);
                    LOGV("width (frame): %d", frameWidth);
                    LOGV("stride (frame): %d", frameStride);
                    LOGV("height (frame): %d", frameHeight);

                    updateFormat(mediaFormat);

                    if (frameStride < frameWidth) frameStride = frameWidth;

                    if (info.flags & AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM)
                    {
                        LOGV("output EOS");
                        sawOutputEOS = true;
                    }

                    AMediaCodec_releaseOutputBuffer(mediaCodec.get(), bufferIndex, info.size != 0);
                    return true;
                } else if (bufferIndex == AMEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED) {
                    LOGV("output buffers changed");
                } else if (bufferIndex == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
                    auto format = AMediaCodec_getOutputFormat(mediaCodec.get());
                    LOGV("format changed to: %s", AMediaFormat_toString(format));
                    AMediaFormat_delete(format);
                } else if (bufferIndex == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
                    LOGV("no output buffer right now");
                } else {
                    LOGV("unexpected info code: %zd", bufferIndex);
                }
            }
        }
        return false;
    }

    bool isOpened() const CV_OVERRIDE { return mediaCodec.get() != nullptr; }

    int getCaptureDomain() CV_OVERRIDE { return CAP_ANDROID; }

    bool grabFrame() CV_OVERRIDE
    {
        // clear the previous frame
        buffer.clear();
        return decodeFrame();
    }

    bool retrieveFrame(int, OutputArray out) CV_OVERRIDE
    {
        if (buffer.empty()) {
            return false;
        }

        if (colorFormat == COLOR_FormatYUV420Planar) {
            int uOffset = frameHeight * frameStride;
            int uvStride = frameStride / 2;
            yuv2rgb(
                uOffset,
                uOffset + (frameHeight / 2) * uvStride,
                1,
                uvStride,
                2
            );
        } else if (colorFormat == COLOR_FormatYUV420SemiPlanar) {
            int uOffset = frameHeight * frameStride;
            yuv2rgb(
                uOffset,
                uOffset + 1,
                2,
                frameStride,
                2
            );
        } else {
            LOGE("Unsupported video format: %d", colorFormat);
            return false;
        }

        out.assign(frame);

        if (videoOrientationAuto && -1 != videoRotationCode) {
            cv::rotate(out, out, videoRotationCode);
        }

        return true;
    }

    double getProperty(int property_id) const CV_OVERRIDE
    {
        switch (property_id)
        {
            case CV_CAP_PROP_FRAME_WIDTH:
                return (( videoOrientationAuto &&
                         (cv::ROTATE_90_CLOCKWISE == videoRotationCode || cv::ROTATE_90_COUNTERCLOCKWISE == videoRotationCode))
                        ? videoHeight : videoWidth);
            case CV_CAP_PROP_FRAME_HEIGHT:
                return (( videoOrientationAuto &&
                         (cv::ROTATE_90_CLOCKWISE == videoRotationCode || cv::ROTATE_90_COUNTERCLOCKWISE == videoRotationCode))
                        ? videoWidth : videoHeight);
            case CV_CAP_PROP_FPS: return videoFrameRate;
            case CV_CAP_PROP_FRAME_COUNT: return videoFrameCount;
            case CAP_PROP_ORIENTATION_META: return videoRotation;
            case CAP_PROP_ORIENTATION_AUTO: return videoOrientationAuto ? 1 : 0;
            case CAP_PROP_BITRATE: return bitrate;
        }
        return 0;
    }

    bool setProperty(int property_id, double value) CV_OVERRIDE
    {
        switch (property_id)
        {
            case CAP_PROP_ORIENTATION_AUTO: {
                videoOrientationAuto = value != 0 ? true : false;
                return true;
            }
        }

        return false;
    }

    bool initCapture(const char * filename)
    {
        mediaExtractor = std::shared_ptr<AMediaExtractor>(AMediaExtractor_new(), deleter_AMediaExtractor);
        if (!mediaExtractor) {
            return false;
        }

        media_status_t err;
        int fd;
        long fileSize = 0x7ffffffffffffffL;

        if (':' == filename[0]) {
            fd = atoi(filename + 1);
            LOGE("received fd %d", fd);
        } else {
            struct stat statBuffer;
            if (stat(filename, &statBuffer) != 0) {
                LOGE("failed to stat file: %s (%s)", filename, strerror(errno));
                return false;
            }

            fileSize = statBuffer.st_size;
            fd = open(filename, O_RDONLY);
        }

        if (fd < 0) {
            LOGE("failed to open file: %s %d (%s)", filename, fd, strerror(errno));
            return false;
        }

        err = AMediaExtractor_setDataSourceFd(mediaExtractor.get(), fd, 0, fileSize);
        close(fd);

        if (err != AMEDIA_OK) {
            LOGV("setDataSource error: %d", err);
            return false;
        }

        int numtracks = AMediaExtractor_getTrackCount(mediaExtractor.get());

        LOGV("input has %d tracks", numtracks);
        for (int i = 0; i < numtracks; i++) {
            auto format = std::shared_ptr<AMediaFormat>(AMediaExtractor_getTrackFormat(mediaExtractor.get(), i), deleter_AMediaFormat);
            if (!format) {
                continue;
            }
            const char *s = AMediaFormat_toString(format.get());
            LOGV("track %d format: %s", i, s);
            const char *mime;
            if (!AMediaFormat_getString(format.get(), AMEDIAFORMAT_KEY_MIME, &mime)) {
                LOGV("no mime type");
            } else if (!strncmp(mime, "video/", 6)) {
                int32_t trackWidth, trackHeight, fps, frameCount = 0, rotation = 0;
                AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_WIDTH, &trackWidth);
                AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_HEIGHT, &trackHeight);
                AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_FRAME_RATE, &fps);

                #if __ANDROID_API__ >= 28
                    AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_ROTATION, &rotation);
                    LOGV("rotation (track): %d", rotation);
                #endif

                #if __ANDROID_API__ >= 29
                    AMediaFormat_getInt32(format.get(), AMEDIAFORMAT_KEY_FRAME_COUNT, &frameCount);
                #endif

                updateFormat(format);

                LOGV("width (track): %d", trackWidth);
                LOGV("height (track): %d", trackHeight);
                if (AMediaExtractor_selectTrack(mediaExtractor.get(), i) != AMEDIA_OK) {
                    continue;
                }
                mediaCodec = std::shared_ptr<AMediaCodec>(AMediaCodec_createDecoderByType(mime), deleter_AMediaCodec);
                if (!mediaCodec) {
                    continue;
                }
                if (AMediaCodec_configure(mediaCodec.get(), format.get(), NULL, NULL, 0) != AMEDIA_OK) {
                    continue;
                }
                sawInputEOS = false;
                sawOutputEOS = false;
                if (AMediaCodec_start(mediaCodec.get()) != AMEDIA_OK) {
                    continue;
                }

                videoWidth = trackWidth;
                videoHeight = trackHeight;
                videoFrameRate = fps;
                videoFrameCount = frameCount;
                videoRotation = rotation;

                switch(videoRotation) {
                    case 90:
                        videoRotationCode = cv::ROTATE_90_CLOCKWISE;
                        break;

                    case 180:
                        videoRotationCode = cv::ROTATE_180;
                        break;

                    case 270:
                        videoRotationCode = cv::ROTATE_90_COUNTERCLOCKWISE;
                        break;

                    default:
                        videoRotationCode = -1;
                        break;
                }

                return true;
            }
        }

        return false;
    }

    void cleanUp() {
        sawInputEOS = true;
        sawOutputEOS = true;
        frameStride = 0;
        frameWidth = 0;
        frameHeight = 0;
        colorFormat = 0;
        videoWidth = 0;
        videoHeight = 0;
        videoFrameRate = 0;
        videoFrameCount = 0;
        videoRotation = 0;
        videoRotationCode = -1;
    }
};



class AndroidMediaNdkVideoWriter CV_FINAL :
    public cv::IVideoWriter
{
    typedef struct {
        int fourcc;
        const char* mime;
        OutputFormat muxerFormat;
    }
    FourCCInfo;

    static const int64_t TIMEOUT = 1000L;
    static const FourCCInfo FOURCC_INFO[];

    static const FourCCInfo* findInfo(int fourcc) {
        for( const FourCCInfo *it = FOURCC_INFO; NULL != it->mime; it++ ) {
            if (fourcc == it->fourcc) return it;
        }
        return NULL;
    }

    AMediaFormat* format;
    AMediaCodec* encoder;
    AMediaMuxer* muxer;

    #if __ANDROID_API__ >= 26
    ANativeWindow* surface;
    #endif

    long frameIndex;
    int width;
    int height;
    double frameRate;
    ssize_t videoTrackIndex;
    int fd;

    void drainEncoder(bool end) {
        if (end) {
            #if __ANDROID_API__ >= 26
            AMediaCodec_signalEndOfInputStream(encoder);
            #else
            writeBytes(NULL, 0);
            #endif
        }

        AMediaCodecBufferInfo bufferInfo;
        ssize_t bufferIndex;
        size_t  bufferSize;
        uint8_t *buffer;

        while (true) {
            bufferIndex = AMediaCodec_dequeueOutputBuffer(encoder, &bufferInfo, TIMEOUT);
            if (bufferIndex >= 0) {
                buffer = AMediaCodec_getOutputBuffer(encoder, (size_t)bufferIndex, &bufferSize);

                if (NULL == buffer || 0 == bufferSize){
                    LOGE("Can't get output buffer");
                    break;
                }

                if (videoTrackIndex >= 0) {
                    bufferInfo.presentationTimeUs = frameIndex * 1000000L / frameRate;
                    LOGV("Muxer write to track %d: %d byte(s)", (int)videoTrackIndex, (int)bufferInfo.size);
                    AMediaMuxer_writeSampleData(muxer, (size_t)videoTrackIndex, buffer, &bufferInfo);
                } else {
                    LOGE("Invalid video track !");
                }

                AMediaCodec_releaseOutputBuffer(encoder, (size_t)bufferIndex, false);
                if (bufferInfo.flags & AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM) break;
            } else if (AMEDIACODEC_INFO_TRY_AGAIN_LATER == bufferIndex) {
                if (!end) break;
            } else if (AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED == bufferIndex) {
                videoTrackIndex = AMediaMuxer_addTrack(muxer, AMediaCodec_getOutputFormat(encoder));
                if (videoTrackIndex >= 0) {
                    AMediaMuxer_start(muxer);
                }
                LOGV("New videoTrackIndex: %d", (int)videoTrackIndex);
            }
        }
    }

    #if __ANDROID_API__ < 26
    void writeBytes( uint8_t* inputBuffer, size_t inputBufferSize ) {
        LOGV("[writeBytes] inputBufferSize=%u", (unsigned int)inputBufferSize);

        ssize_t bufferIndex;
        size_t  bufferSize;
        uint8_t* buffer;
        size_t  partialSize;
        bool firstCall = true;
        uint32_t flags;

        while(inputBufferSize > 0 || firstCall) {
            bufferIndex = AMediaCodec_dequeueInputBuffer(encoder, TIMEOUT);

            if (bufferIndex >= 0) {
                firstCall = false;
                buffer = AMediaCodec_getInputBuffer(encoder, (size_t)bufferIndex, &bufferSize);
                if (NULL == buffer || 0 == bufferSize) break;

                flags = 0;
                partialSize = (inputBufferSize > bufferSize) ? bufferSize : inputBufferSize;
                if (partialSize > 0) {
                    memcpy(buffer, inputBuffer, partialSize);
                    inputBuffer += partialSize;
                    inputBufferSize -= partialSize;
                    if (inputBufferSize > 0) {
                        flags = AMEDIACODEC_BUFFER_FLAG_PARTIAL_FRAME;
                    }
                } else {
                    flags = AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM;
                }

                LOGV(
                    "[writeBytes] partial - bufferIndex=%d, bufferSize=%u, partialSize=%u, remaining inputBufferSize=%u",
                    (int)bufferIndex, (unsigned int)bufferSize, (unsigned int)partialSize, (unsigned int)inputBufferSize
                );

                AMediaCodec_queueInputBuffer(encoder, (size_t)bufferIndex, 0, partialSize, frameIndex * 1000000L / frameRate, flags);
                if (NULL != inputBuffer) drainEncoder(false);
            }
        }
    }
    #endif

public:
    AndroidMediaNdkVideoWriter(const cv::String& filename, int fourcc, double fps, cv::Size frameSize, const VideoWriterParameters& params)
        : format(NULL),
          encoder(NULL),
          muxer(NULL),
          #if __ANDROID_API__ >= 26
          surface(NULL),
          #endif
          frameIndex(0),
          width(0),
          height(0),
          frameRate(0.),
          videoTrackIndex(-1),
          fd(-1) {
        open(filename, fourcc, fps, frameSize, params);
    }
    virtual ~AndroidMediaNdkVideoWriter() { close(); }

    virtual int getCaptureDomain() const CV_OVERRIDE { return cv::CAP_ANDROID; }

    virtual void write(cv::InputArray image_ ) CV_OVERRIDE
    {
        if (!image_.isMat()) {
            LOGE("Support only Mat input");
            return;
        }

        Mat image = image_.getMat();
        if (CV_8UC3 != image.type() || image.cols > width || image.rows > height) {
            LOGE(
                "Expected input to be a mat of maximum %d x %d of type CV_8UC3 (%d), but received %d x %d of type: %d",
                width, height, CV_8UC3,
                image.cols, image.rows, image.type()
            );
            return;
        }

        #if __ANDROID_API__ >= 26
        ANativeWindow_Buffer buffer;
        if (0 != ANativeWindow_lock(surface, &buffer, NULL)) {
            LOGE("Failed to lock the surface");
        } else {
            if (AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM == buffer.format) {
                Mat bufferMat(image.rows, image.cols, CV_8UC4, buffer.bits, buffer.stride * 4);
                cvtColor(image, bufferMat, CV_BGR2RGBA);
            } else {
                LOGE("Unknow surface buffer format: %u", buffer.format);
            }

            ANativeWindow_unlockAndPost(surface);
        }
        #else
        LOGV("[write] image: %d  x %d", image.cols, image.rows);

        //OpenCV don't support RGB to NV12 so we need to connvert to YV12 and then manually changed it to NV12
        Mat imageYV12;
        cvtColor(image, imageYV12, CV_BGR2YUV_YV12);

        //convert from YV12 to NV12
        size_t yPlaneSize = width * height;
        size_t vPlaneSize = yPlaneSize / 4;

        Mat channels[2] = {
            Mat( vPlaneSize, 1, CV_8UC1, imageYV12.ptr() + yPlaneSize ).clone(),
            Mat( vPlaneSize, 1, CV_8UC1, imageYV12.ptr() + yPlaneSize + vPlaneSize ).clone()
        };
        Mat vuMat( vPlaneSize, 1, CV_8UC2, imageYV12.ptr() + yPlaneSize );
        merge(channels, 2, vuMat);

        writeBytes( imageYV12.ptr(), imageYV12.rows * imageYV12.cols );
        #endif

        drainEncoder(false);

        frameIndex++;
    }

    virtual bool open( const cv::String& filename, int fourcc, double fps, cv::Size frameSize, const VideoWriterParameters& params )
    {
        media_status_t status;

        close();

        const FourCCInfo* info = findInfo(fourcc);
        if (NULL == info) {
            LOGE("ERROR: findInfo");
            return false;
        }

        format = AMediaFormat_new();
        if (NULL == format) {
            LOGE("ERROR: AMediaFormat_new");
            goto error;
        }

        LOGV("mime: %s, width: %d, height: %d, fps: %f", info->mime, frameSize.width, frameSize.height, fps);

        AMediaFormat_setString(format, AMEDIAFORMAT_KEY_MIME, info->mime);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_WIDTH, frameSize.width);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_HEIGHT, frameSize.height);
        AMediaFormat_setFloat(format, AMEDIAFORMAT_KEY_FRAME_RATE, (float)fps);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_I_FRAME_INTERVAL, 5);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_BIT_RATE, frameSize.width * frameSize.height * 5);
        AMediaFormat_setInt32(
            format, AMEDIAFORMAT_KEY_COLOR_FORMAT,
            #if __ANDROID_API__ >= 26
            COLOR_FormatSurface
            #else
            COLOR_FormatYUV420SemiPlanar
            #endif
        );

        encoder = AMediaCodec_createEncoderByType(info->mime);
        if (NULL == encoder) {
            LOGE("ERROR: AMediaCodec_createEncoderByType");
            goto error;
        }

        status = AMediaCodec_configure(encoder, format, NULL, NULL, AMEDIACODEC_CONFIGURE_FLAG_ENCODE);
        if (AMEDIA_OK != status) {
            LOGE("ERROR: AMediaCodec_configure (%d)", status);
            goto error;
        }

        #if __ANDROID_API__ >= 26
        status = AMediaCodec_createInputSurface(encoder, &surface);
        if (AMEDIA_OK != status || NULL == surface) {
            LOGE("ERROR: AMediaCodec_createInputSurface (%d)", status);
            goto error;
        }
        #endif

        AMediaCodec_start(encoder);

        fd = ::open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (fd < 0) {
            LOGE("ERROR: open");
            goto error;
        }

        muxer = AMediaMuxer_new(fd, info->muxerFormat);
        if (NULL == muxer) {
            LOGE("ERROR: AMediaMuxer_new");
            goto error;
        }

        AMediaMuxer_setOrientationHint(muxer, params.get(CAP_PROP_ORIENTATION_META, 0));

        frameIndex = 0;
        width = frameSize.width;
        height = frameSize.height;
        frameRate = fps;
        videoTrackIndex = -1;

        return true;

    error:
        close();
        return false;
    }

    virtual void close()
    {
        if (videoTrackIndex >= 0 && NULL != muxer) {
            drainEncoder(true);
            AMediaMuxer_stop(muxer);
        }

        if (NULL != encoder) AMediaCodec_delete(encoder);
        if (NULL != muxer) AMediaMuxer_delete(muxer);

        #if __ANDROID_API__ >= 26
        if (NULL != surface) ANativeWindow_release(surface);
        #endif

        if (fd >= 0) ::close(fd);
        if (NULL != format) AMediaFormat_delete(format);

        format = NULL;
        encoder = NULL;
        muxer = NULL;
        #if __ANDROID_API__ >= 26
        surface = NULL;
        #endif
        frameIndex = 0;
        width = 0;
        height = 0;
        frameRate = 0.;
        videoTrackIndex = -1;
        fd = -1;
    }

    virtual double getProperty(int) const CV_OVERRIDE { return 0.; }
    virtual bool setProperty(int, double) CV_OVERRIDE { return false; }
    virtual bool isOpened() const CV_OVERRIDE { return NULL != encoder; }
};


const AndroidMediaNdkVideoWriter::FourCCInfo AndroidMediaNdkVideoWriter::FOURCC_INFO[] = {
    { CV_FOURCC('H', '2', '6', '4'), "video/avc", AMEDIAMUXER_OUTPUT_FORMAT_MPEG_4 },
    { CV_FOURCC('H', '2', '6', '5'), "video/hevc", AMEDIAMUXER_OUTPUT_FORMAT_MPEG_4 },
    { CV_FOURCC('H', '2', '6', '3'), "video/3gpp", AMEDIAMUXER_OUTPUT_FORMAT_MPEG_4 },
    { CV_FOURCC('M', 'P', '4', 'V'), "video/mp4v-es", AMEDIAMUXER_OUTPUT_FORMAT_MPEG_4 },
    { 0, NULL },
};



/****************** Implementation of interface functions ********************/

Ptr<IVideoCapture> cv::createAndroidCapture_file(const std::string &filename) {
    Ptr<AndroidMediaNdkCapture> res = makePtr<AndroidMediaNdkCapture>();
    if (res && res->initCapture(filename.c_str()))
        return res;
    return Ptr<IVideoCapture>();
}


Ptr<IVideoWriter> cv::createAndroidVideoWriter(
    const std::string& filename, int fourcc,
    double fps, const cv::Size& frameSize,
    const VideoWriterParameters& params) {
    Ptr<AndroidMediaNdkVideoWriter> writer = makePtr<AndroidMediaNdkVideoWriter>(filename, fourcc, fps, frameSize, params);
    if (writer && writer->isOpened())
        return writer;
    return Ptr<IVideoWriter>();
}
