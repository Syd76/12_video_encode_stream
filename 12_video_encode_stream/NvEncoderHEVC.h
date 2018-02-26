

#ifndef __NV_ENCODER_HEVC_H__
#define __NV_ENCODER_HEVC_H__

// #include <queue>
// 
// // #include "IMp4Writer.h"
// // #include "Mutex.h"
// 
// #include <mutex>
// 
// // #include <PvMp4Writer.h>
// #include <NvVideoEncoder.h>
// 
// // #include <nvbuf_utils.h>
// // #include <NvBufferColorFormat.h>
// 
// #include <NvUtils.h>
// // #include "NvCudaProc.h"
// #include <nvbuf_utils.h>
// 
// #include <sstream>

#include <NvUtils.h>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <sstream>
#include <string.h>

#include <thread>
#include <mutex>
#include <list>


#include "type_encode.h"

#define VERBOSE_DEBUG 7
#define VERBOSE_INFO 5
#define VERBOSE_WARNING 3
#define VERBOSE_ERROR 2
#define VERBOSE_FATAL 0


#define V4L2_BUFFERS_NUM    4

#define DEBUG(fmt, ...) \
    do { \
    if (m_ContextDevice->enable_verbose <= VERBOSE_DEBUG ) \
        printf("INFO: %s(): (line:%d) " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)
    
#define INFO(fmt, ...) \
    do { \
    if (m_ContextDevice->enable_verbose <= VERBOSE_INFO ) \
        printf("INFO: %s(): (line:%d) " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define WARN(fmt, ...) \
    do { \
    if (m_ContextDevice->enable_verbose <= VERBOSE_WARNING ) \
        printf("WARN: %s(): (line:%d) " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define CHECK_ERROR(cond, fmt, ...) \
    do { \
            if (!cond) { \
                printf("CHECK ERROR: %s(): (line:%d) " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            } \
    } while(0)

#define ERROR_RETURN(fmt, ...) \
    do { \
        printf("ERROR: %s(): (line:%d) " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)

    
//         return false; \

class NvEncoderHEVC // : public IMp4Writer
{
    
public:
    
     
    typedef struct {
        uint32_t size;
        char *buffer;
    } DataHEVC ;
    


    NvEncoderHEVC();
    virtual ~NvEncoderHEVC();

    // -----------------------------------------------
    // Public 
    // -----------------------------------------------
    
    
    bool getStreamEOS( void ) ;
    void setStreamEOS( bool _stat );
    
    static NvEncoderHEVC::DataHEVC * popBufferHEVC( void );
    
    void pushBufferImage( Image_T *_data) ;
    
    
    
    void run( void );

    uint32_t SizeListBufferImage( void );
    
    uint32_t setInitialized( bool _stat ) ;
    bool isInitialized( void );
    
    // -----------------------------------------------
    // Public heritage
    // -----------------------------------------------
//     bool IsAvailable() const { return true; }
//     bool IsOpened() { return mInitialized; }
//     uint32_t GetAvgBitrate() const { return 0; }
//     void SetAvgBitrate( uint32_t aValue ) { /*mAvgBitrate = aValue;*/ setContextBitRate(aValue); }
//     
//     uint32_t GetFPS() const { return 0; }
//     void SetFPS( uint32_t aValue ) { /*mRecordFps = aValue;*/ setContextFPS(aValue);}
// 
//     
//     void GetLastError( std::string &aString ) { aString = ""; };
//     void ResetLastError() {};
//     
//     //! Open  :: creat mp4 file with first buffer image for video parameters
//     bool Open( const std::string &aFilename, char  *aImage );
//     //! close opened MP4 file
//     void Close( void );
//     //! WriteFrame on opened MP4 file
//      bool WriteFrame( char *aImage, uint32_t *aFileSizeDelta );
//     //! elapse time on this recording MP4.
//     uint32_t getVideoTimestamp( void ) ;
    
    

protected :
    
    // -----------------------------------------------
    // LIST ENCODE HEVC
    // -----------------------------------------------
    std::list< Image_T *> mlistBufferImage;
    static std::list< NvEncoderHEVC::DataHEVC *> mlistBufferHEVC;
    
    
    std::mutex mMuxListImage;
    static std::mutex mMuxListHEVC;
    
    static void pushBufferHEVC( NvEncoderHEVC::DataHEVC *_data) ;
    Image_T * popBufferImage( void );
    
    // -----------------------------------------------
    // CONTEXT 
    // -----------------------------------------------
    
    std::mutex mMuxCurrentImage;
    
    std::thread *mh264CaptureLoop;
    char *mCurrentImage ;
    unsigned long long m_frame_count;
    
    
    // -----------------------------------------------
    // EOS 
    // -----------------------------------------------
    
    std::mutex m_mutex_eos;
    bool m_eos_stream ;
    Context_T *m_ContextDevice;
    
    

    // -----------------------------------------------
    // ENCODE THREAD 
    // -----------------------------------------------
    
    std::thread *mLoopCompressHEVC;
    
//     void LoopCompressFrame( void );
    
    
    
    
    
    bool mInitialized;
    bool camera_initialize(Context_T * ctx , char *aimage);
    bool encoder_initialize(Context_T * ctx);
//     static NvEncoderHEVC::NV_Color_Fmt *nv_color_fmt nvcolor_fmt ;
    
    // Static function
//     static NvBufferColorFormat get_nvbuff_color_fmt(unsigned int v4l2_pixfmt) ;
    
    // Init function
    void setContextDevName( char *_name );
    void setOutFilePath( const char *_file ) ;
    
    void setContextOutputResolution( unsigned int _w , unsigned int _h );
    bool setContextOutputFormatPixel( char *_fmtpix );
    bool setContextOutputFormatPixel( uint32_t _fmtpix );
    bool setContextFPS( uint32_t _fps , uint32_t _den = 1);
    bool setContextBitRate( uint32_t _br ) ;
    bool setContextROI( char * _roi );
    void setContextIFrameInterval( uint32_t _ifi );
    void setContextIdrInterval( uint32_t _idri );
    void setContextInsertSpsPps( bool _sps );
    void setContextRateControl( v4l2_mpeg_video_bitrate_mode _rc );
    void setContextRuntimeParams( char * _rp );
    bool setContextProfile( v4l2_mpeg_video_h264_profile _profile ) ;
    bool setContextTemporalTradeoff( v4l2_enc_temporal_tradeoff_level_type _tt );
    void setContextSliceLengthType( v4l2_enc_slice_length_type _slt );
    void setContextSliceLength( uint32_t _sl );
    void setContextVirtualBuffer( uint32_t _vb );
    void setContextBFrame( uint32_t _bf );
    void setContextRFrame( uint32_t _rf );
    void setContextSliceIntrarefreshInterval( uint32_t _sir );
    void setContextReportMetadata( bool _rm );
    void setContextDumpMv( bool _dm );    
    
	void setdefaults( void );
	void setDefaultSimpleContext( void );
    int setDefaultsContext( void );
    void setEncodePixFormat (uint32_t _pixfmt );
    bool setContextLevel( v4l2_mpeg_video_h264_level _stat );
    
//     bool setContextEnableCuda( int _stat ) ;
    bool openContextStreamFile( void );
    bool openContextROIParamfile( void );
    
    
    char *getPvFrame( void ) ;
    
    void populate_roi_Param(std::ifstream * stream, v4l2_enc_frame_ROI_params *VEnc_ROI_params);
    
    
//     void loopEncodeFrame ( void ) ;    
//     int read_video_frame( NvBuffer & buffer) ;
//     int read_video_frame(std::ifstream * stream, NvBuffer & buffer);
//     int read_video_frame( NvBuffer & buffer) ;
//     int read_video_frame(  char *aImage, NvBuffer & buffer);
    
    uint32_t cleanup( void );
//     static void abort( Context_T *ctx);
    
//     static int writeEncoderOutputFrame(std::ofstream * stream, NvBuffer * buffer);

//     int set_runtime_params(Context_T  *ctx);
//     int get_next_parsed_pair(Context_T  *ctx, char *id, uint32_t *value);
//     int get_next_runtime_param_change_frame(Context_T *ctx);
    
    // Private function
//     void drawClock(AVFrame *pic, uint64_t _now , const char *_format) ;
//     void drawtext(AVFrame *pic, int x, int y, const char *txt, int o);

	void LoopCompressFrame( void ) ;
	void compressYFrame(Image_T *_data) ;
	void compressYUV420Frame(uint8_t *yuv_bytes);
	int encodeFrame(uint8_t *yuv_bytes) ;
	void copyYuvToBuffer(uint8_t *yuv_bytes, NvBuffer &buffer) ;
	int get_next_runtime_param_change_frame(Context_T *ctx);
	void abort( Context_T *ctx);
	void ClearListHEVC( void ) ;
// 	int setDefaultsContext( void );
	int set_runtime_params(Context_T  *ctx);
	
	void Close( void );
	bool WriteFrame( char *aImage, uint32_t *aFileSizeDelta );
	
	static int write_encoder_output_frame(Context_T*, NvBuffer*);
    static bool encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer, NvBuffer * shared_buffer, void *arg);
	int get_next_parsed_pair(Context_T  *ctx, char *id, uint32_t *value);
// 	int NvEncoderHEVC::write_encoder_output_frame(Context_T *ctx, NvBuffer * buffer);
	
    
    void addBufferImage( Image_T *_data );
};

#endif // __NV_ENCODER_HEVC_H__



/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



