#include <linux/videodev2.h>
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

#include <stdio.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <poll.h>


// #include <PvString.h>
// #include <char.h>

// #include "NvVideoConverter.h"
// #include "NvEglRenderer.h"
// #include "NvUtils.h"
// #include "NvCudaProc.h"
// #include "nvbuf_utils.h"
// 
// #include "camera_v4l2_cuda.h"
#include <v4l2_nv_extensions.h>


#include "NvEncoderHEVC.h"

#include "Crc_Encode.h"

// NvEncoderHEVC::NV_Color_Fmt *NvEncoderHEVC::nvcolor_fmt =
// {
//     // TODO add more pixel format mapping
//     {V4L2_PIX_FMT_UYVY, NvBufferColorFormat_UYVY},
//     {V4L2_PIX_FMT_VYUY, NvBufferColorFormat_VYUY},
//     {V4L2_PIX_FMT_YUYV, NvBufferColorFormat_YUYV},
//     {V4L2_PIX_FMT_YVYU, NvBufferColorFormat_YVYU},
// };

#define IS_DIGIT(c) (c >= '0' && c <= '9')

#define BUFFER_LENGTH 40

std::list< NvEncoderHEVC::DataHEVC *> NvEncoderHEVC::mlistBufferHEVC;
std::mutex NvEncoderHEVC::mMuxListHEVC;


NvEncoderHEVC::NvEncoderHEVC( ) {
    
    setdefaults();
    setStreamEOS( false );
    setInitialized( false );
    m_frame_count = 0;
}


NvEncoderHEVC::~NvEncoderHEVC()
{
//     Close();
}


/**
 * @fn NvBufferColorFormat NvEncoderHEVC::get_nvbuff_color_fmt(unsigned int v4l2_pixfmt) 
 * @brief acces to NVidia color format
 * 
 */
// NvBufferColorFormat NvEncoderHEVC::get_nvbuff_color_fmt(unsigned int v4l2_pixfmt) {
//     
//     NvBufferColorFormat lret = NvBufferColorFormat_Invalid;
//   for (unsigned i = 0; i < sizeof(NvEncoderHEVC::nvcolor_fmt); i++)
//   {
//     if (v4l2_pixfmt == NvEncoderHEVC::nvcolor_fmt[i].v4l2_pixfmt)
//       return lret = NvEncoderHEVC::nvcolor_fmt[i].nvbuff_color;
//   }
// 
//   return lret ; // NvBufferColorFormat_Invalid;
// }


bool NvEncoderHEVC::getStreamEOS( void ) {
    std::lock_guard<std::mutex> lguard( m_mutex_eos );
    return m_eos_stream;
}

void NvEncoderHEVC::setStreamEOS( bool _stat ) {
    std::lock_guard<std::mutex> lguard( m_mutex_eos );
    m_eos_stream = _stat;
}
    
    
    
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//					SETTINGS CONTEXT
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// END 595

void NvEncoderHEVC::setdefaults( void )
{
	
    m_ContextDevice = new Context_T;
    memset(m_ContextDevice, 0, sizeof(Context_T));
	m_ContextDevice->encoder_pixfmt = V4L2_PIX_FMT_H264;
    m_ContextDevice->bitrate = 4 * 1024 * 1024;
    m_ContextDevice->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
    m_ContextDevice->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    m_ContextDevice->iframe_interval = 30;
    m_ContextDevice->idr_interval = 256;
    m_ContextDevice->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
    m_ContextDevice->fps_n = 5;
    m_ContextDevice->fps_d = 1;
    m_ContextDevice->num_b_frames = (uint32_t) -1;
	m_ContextDevice->use_gold_crc = false;
    m_ContextDevice->ROI_Param_file_path = nullptr;
    m_ContextDevice->Recon_Ref_file_path = nullptr;
    m_ContextDevice->RPS_Param_file_path = nullptr;
    m_ContextDevice->hints_Param_file_path = nullptr;
    m_ContextDevice->GDR_Param_file_path = nullptr;
    m_ContextDevice->GDR_out_file_path = nullptr;
	
    m_ContextDevice->nMinQpI = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMinQpP = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMinQpB = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
}


/**
 * @fn void NvEncoderHEVC::setContextDevName( char *_name )
 * @brief Set V4l2 video device node
 * 
 */
void NvEncoderHEVC::setContextDevName( char *_name )
{
//     m_ContextDevice->in_file_path = strdup(_name) ;
    
    
    int llen = strlen(_name)+1;
    
    m_ContextDevice->in_file_path = new char [llen];
    
    memset( m_ContextDevice->in_file_path , 0 , llen );
    memcpy(m_ContextDevice->in_file_path  , _name , llen - 1);
    
}

/**
 * @fn void NvEncoderHEVC::setContextOutputResolution( unsigned int _w , unsigned int _h )
 * @brief Set output resolution of video device
 * 
 */
void NvEncoderHEVC::setContextOutputResolution( unsigned int _w , unsigned int _h )
{
    
    m_ContextDevice->width = _w;
    m_ContextDevice->height = _h;
        
}



/**
 * @fn bool NvEncoderHEVC::setContextOutputFormatPixel( char *_fmtpix ) 
 * @brief Set output pixel format of video device (supports only YUYV/YVYU/UYVY/VYUY)
 * @see videodev2.h 
 */
bool NvEncoderHEVC::setContextOutputFormatPixel( char *_fmtpix ) {
    
    bool lret = true;
    
    if (strcmp(_fmtpix, "YUYV") == 0) {
        m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_YUYV;
    }
    else {
        if (strcmp(_fmtpix, "YVYU") == 0) {
            m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_YVYU;
        }
        else {
            if (strcmp(_fmtpix, "VYUY") == 0) {
                m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_VYUY;
            }
            else {
                if (strcmp(_fmtpix, "UYVY") == 0) {
                    m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_UYVY;
                }
                else
                {
                    lret = false;
                }
            }
        }
    }
    
//      sprintf(m_ContextDevice->cam_file, "camera.%s", _fmtpix);
    // FALSE => unsupport format
    return lret;
}

/**
 * @fn bool NvEncoderHEVC::setContextOutputFormatPixel( unsigned int _fmtpix ) 
 * @brief Set output pixel format of video device (supports only YUYV/YVYU/UYVY/VYUY)
 * @see videodev2.h 
 */
bool NvEncoderHEVC::setContextOutputFormatPixel( uint32_t _fmtpix ) {
    bool lret = true;
    m_ContextDevice->cam_pixfmt = _fmtpix;
    return lret;
}
            
/**
 * @fn bool NvEncoderHEVC::setContextOutputFormatPixel( unsigned int _fps ) 
 * @brief Set renderer frame rate (30 fps by default)
 */
bool NvEncoderHEVC::setContextFPS( uint32_t _fps , uint32_t _den ) {
    m_ContextDevice->fps_n = _fps;
    m_ContextDevice->fps_d = ( 0 < _den ? _den : 1 ) ;
    
}

bool NvEncoderHEVC::setContextBitRate( uint32_t _br ) {
    m_ContextDevice->bitrate = _br;
    return ( 0 < _br );
}

bool NvEncoderHEVC::setContextROI( char * _roi ) {
    m_ContextDevice->ROI_Param_file_path = strdup( _roi );
    return ( m_ContextDevice->ROI_Param_file_path != nullptr );
}
 

/**
 * @fn void NvEncoderHEVC::setContextNthVIC( unsigned int _nfps ) 
 * @brief Save the n-th frame before VIC processing , (0 fps by default)
 */
// void NvEncoderHEVC::setContextNthVIC( int _nfps ) {
//     m_ContextDevice->save_n_frame = _nfps;
// }

/**
 * @fn void NvEncoderHEVC::setContextIFrameInterval( uint32_t _ifi ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 */
void NvEncoderHEVC::setContextIFrameInterval( uint32_t _ifi ) {
    m_ContextDevice->iframe_interval = _ifi;
}

/**
 * @fn bool NvEncoderHEVC::setContextIdrInterval( uint32_t _idri ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 */
void NvEncoderHEVC::setContextIdrInterval( uint32_t _idri ) {
    m_ContextDevice->iframe_interval = _idri;
}

/**
 * @fn void NvEncoderHEVC::setContextInsertSpsPps( bool _sps )
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 */
void NvEncoderHEVC::setContextInsertSpsPps( bool _sps ) {
    m_ContextDevice->insert_sps_pps_at_idr = _sps;
}

/**
 * @fn void NvEncoderHEVC::setContextRateControl( v4l2_mpeg_video_bitrate_mode _rc )
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextRateControl( v4l2_mpeg_video_bitrate_mode _rc ) {
    m_ContextDevice->ratecontrol = _rc;
}

/**
 * @fn void NvEncoderHEVC::setContextRuntimeParams( char * _rp )
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextRuntimeParams( char * _rp ) {
//     m_ContextDevice->runtime_params_str = new std::stringstream( _rp);
}


/**
 * @fn bool NvEncoderHEVC::setContextProfile( v4l2_mpeg_video_h264_profile _profile )
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
bool NvEncoderHEVC::setContextProfile( v4l2_mpeg_video_h264_profile _profile ) {
    
    v4l2_mpeg_video_h264_profile lpro = (v4l2_mpeg_video_h264_profile)m_ContextDevice->profile;
    if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        switch(_profile ) {        
                case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE :
                case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH :
                case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN : {
                    m_ContextDevice->profile = _profile;
                    break;
                };
                default : {
                    ERROR_RETURN("Unsupported value for profile on H.264");
                }
        }
    }
    else if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H265 )
    {
        switch(_profile ) {
                case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN : {
                    m_ContextDevice->profile = _profile;
                    break;
                }
                default : {
                    ERROR_RETURN("Unsupported value for profile on H.265");
                }
        }
    }    
    return (m_ContextDevice->profile == _profile );
}


/**
 * @fn bool NvEncoderHEVC::setContextTemporalTradeoff( v4l2_enc_temporal_tradeoff_level_type * _tt )
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
bool NvEncoderHEVC::setContextTemporalTradeoff( v4l2_enc_temporal_tradeoff_level_type  _tt ) {
    
    bool lret = true;
    m_ContextDevice->temporal_tradeoff_level = _tt;
    
    if ( m_ContextDevice->temporal_tradeoff_level < V4L2_ENC_TEMPORAL_TRADEOFF_LEVEL_DROPNONE 
            || m_ContextDevice->temporal_tradeoff_level > V4L2_ENC_TEMPORAL_TRADEOFF_LEVEL_DROP2IN3 ) {
                ERROR_RETURN ( "Unsupported value for temporal tradeoff" );
                lret = false;
        }
    return lret;
}


/**
 * @fn void NvEncoderHEVC::setContextSliceLengthType( v4l2_enc_slice_length_type _slt ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextSliceLengthType( v4l2_enc_slice_length_type _slt ) {
    m_ContextDevice->slice_length_type = _slt;
    
    if( _slt != V4L2_ENC_SLICE_LENGTH_TYPE_MBLK || _slt != V4L2_ENC_SLICE_LENGTH_TYPE_BITS ) {
        ERROR_RETURN ( "Unsupported value for slice length type" );
        
    }
}



/**
 * @fn void NvEncoderHEVC::setContextSliceLength( v4l2_enc_slice_length_type _slt ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextSliceLength( uint32_t _sl ) {
    m_ContextDevice->slice_length = _sl;
    if( _sl == 0 ) {
        ERROR_RETURN ( "Slice length should be > 0" );
    }
}



/**
 * @fn void NvEncoderHEVC::setContextVirtualBuffer( uint32_t _vb ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextVirtualBuffer( uint32_t _vb ) {
    m_ContextDevice->virtual_buffer_size = _vb;
    if( _vb == 0 ) {
        ERROR_RETURN ( "Virtual buffer size should be > 0" );
    }
}

/**
 * @fn void NvEncoderHEVC::setContextBFrame( uint32_t _bf ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextBFrame( uint32_t _bf ) {
    m_ContextDevice->num_b_frames = _bf;
//     if( _bf == 0 ) {
//         ERROR_RETURN ( "num B-frames should be > 0" );
//     }
}

/**
 * @fn void NvEncoderHEVC::setContextRFrame( uint32_t _rf ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextRFrame( uint32_t _rf ) {
    m_ContextDevice->num_reference_frames = _rf;
    if( _rf == 0 ) {
        ERROR_RETURN ( "Num reference frames should be > 0" );
    }
}

/**
 * @fn void NvEncoderHEVC::setContextSliceIntrarefreshInterval( uint32_t _sir ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextSliceIntrarefreshInterval( uint32_t _sir ) {
    m_ContextDevice->slice_intrarefresh_interval = _sir;
    if( _sir == 0 ) {
        ERROR_RETURN ( "Slice intrarefresh interval should be > 0" );
    }
}

/**
 * @fn void NvEncoderHEVC::setContextReportMetadata( bool _rm ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextReportMetadata( bool _rm ) {
    m_ContextDevice->report_metadata = _rm;
}


/**
 * @fn void NvEncoderHEVC::setContextDumpMv( bool _dm ) 
 * @brief Enable CUDA aglorithm (draw a black box in the upper left corner)
 * @see V4L2_MPEG_VIDEO_BITRATE_MODE_CBR V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
 */
void NvEncoderHEVC::setContextDumpMv( bool _dm ) {
    m_ContextDevice->dump_mv = _dm;
}


/**
 * @fn bool NvEncoderHEVC::setContextEnableVerboseLog( v4l2_mpeg_video_h264_level _stat ) 
 * @brief Enable verbose message
 */
bool NvEncoderHEVC::setContextLevel( v4l2_mpeg_video_h264_level _stat ) {
    m_ContextDevice->level = _stat;
}


bool NvEncoderHEVC::openContextStreamFile( void )
{
    bool lret = false;
    
    if (m_ContextDevice->out_file_path != nullptr ) {
        m_ContextDevice->out_file = new std::ofstream(m_ContextDevice->out_file_path);
        
        lret = m_ContextDevice->out_file->is_open() ;
        
        if( !lret ) {
            ERROR_RETURN("Could not open output file %s" , m_ContextDevice->out_file_path );
        }
    }
    return lret;
}

    

bool NvEncoderHEVC::openContextROIParamfile( void )
{
    bool lret = false;
    
    if (m_ContextDevice->ROI_Param_file_path != nullptr ) {
        m_ContextDevice->roi_Param_file = new std::ifstream(m_ContextDevice->ROI_Param_file_path);
        
        lret = m_ContextDevice->roi_Param_file->is_open() ;
        
        if( !lret ) {
            ERROR_RETURN("Could not open output file %s" , m_ContextDevice->ROI_Param_file_path );
        }
    }
    return lret;
}

    
    
    
/*
void NvEncoderHEVC::setDefaultsContext(Context_T * ctx) {
    
    m_ContextDevice = new Context_T;
    memset(m_ContextDevice, 0, sizeof(Context_T));

    m_ContextDevice->cam_devname = "/dev/video0";
    m_ContextDevice->cam_fd = -1;
    m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_YUYV;
    m_ContextDevice->cam_w = 640;
    m_ContextDevice->cam_h = 480;
    m_ContextDevice->frame = 0;
    m_ContextDevice->save_n_frame = 0;

    m_ContextDevice->conv = NULL;
    m_ContextDevice->vic_pixfmt = V4L2_PIX_FMT_YUV420M;
    m_ContextDevice->vic_flip = (enum v4l2_flip_method) -1;
    m_ContextDevice->vic_interpolation = (enum v4l2_interpolation_method) -1;
    m_ContextDevice->vic_tnr = (enum v4l2_tnr_algorithm) -1;

    m_ContextDevice->g_buff = NULL;
    m_ContextDevice->renderer = NULL;
    m_ContextDevice->got_error = false;
    m_ContextDevice->fps = 30;

    m_ContextDevice->conv_output_plane_buf_queue = new queue < nv_buffer * >;
    pthread_mutex_init(&m_ContextDevice->queue_lock, NULL);
    pthread_cond_init(&m_ContextDevice->queue_cond, NULL);

    m_ContextDevice->enable_cuda = false;
    m_ContextDevice->egl_image = NULL;
    m_ContextDevice->egl_display = EGL_NO_DISPLAY;

    m_ContextDevice->enable_verbose = false;
}
*/
void NvEncoderHEVC::setDefaultSimpleContext( void )
{
    
    m_ContextDevice = new Context_T;
    memset(m_ContextDevice, 0, sizeof(Context_T));

    m_ContextDevice->cam_pixfmt = V4L2_PIX_FMT_YUYV;
    
    m_ContextDevice->encoder_pixfmt = V4L2_PIX_FMT_H264 ;
    m_ContextDevice->bitrate = 4 * 1024 * 1024;
    m_ContextDevice->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
    m_ContextDevice->width = 640;
    m_ContextDevice->height = 480;
    
    m_ContextDevice->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR; // V4L2_MPEG_VIDEO_BITRATE_MODE_VBR
    m_ContextDevice->iframe_interval = 30;
    m_ContextDevice->idr_interval = 256;
    m_ContextDevice->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
    m_ContextDevice->fps_n = 30;
    m_ContextDevice->fps_d = 1;
    m_ContextDevice->num_b_frames = (uint32_t) -1;
    m_ContextDevice->nMinQpI = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMinQpP = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMinQpB = (uint32_t)QP_RETAIN_VAL;
    m_ContextDevice->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
    
    // option
    m_ContextDevice->report_metadata = false;
    m_ContextDevice->dump_mv = false;
    m_ContextDevice->enable_verbose = VERBOSE_DEBUG;
}

/*
void NvEncoderHEVC::setDefaultsContext( void )
{
    memset(m_ContextDevice, 0, sizeof(Context_T));

//     m_ContextDevice->bitrate = 4 * 1024 * 1024;
//     m_ContextDevice->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
//     m_ContextDevice->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
//     m_ContextDevice->iframe_interval = 30;
//     m_ContextDevice->externalRPS = false;
//     m_ContextDevice->enableGDR = false;
//     m_ContextDevice->enableROI = false;
//     m_ContextDevice->bnoIframe = false;
//     m_ContextDevice->bGapsInFrameNumAllowed = false;
//     m_ContextDevice->bReconCrc = false;
//     m_ContextDevice->nH264FrameNumBits = 0;
//     m_ContextDevice->nH265PocLsbBits = 0;
//     m_ContextDevice->idr_interval = 256;
//     m_ContextDevice->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
//     m_ContextDevice->fps_n = 30;
//     m_ContextDevice->fps_d = 1;
//     m_ContextDevice->gdr_start_frame_number = 0xffffffff;
//     m_ContextDevice->gdr_num_frames = 0xffffffff;
//     m_ContextDevice->gdr_out_frame_number = 0xffffffff;
//     m_ContextDevice->num_b_frames = (uint32_t) -1;
	
	
	
    
//     m_ContextDevice->nMinQpI = (uint32_t)QP_RETAIN_VAL;
//     m_ContextDevice->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
//     m_ContextDevice->nMinQpP = (uint32_t)QP_RETAIN_VAL;
//     m_ContextDevice->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
//     m_ContextDevice->nMinQpB = (uint32_t)QP_RETAIN_VAL;
//     m_ContextDevice->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
     

    //close to lossless encoding with these settings
    // https://devtalk.nvidia.com/default/topic/1026493/jetson-tx1/lossless-h-264-encoding/post/5220785/#5220785
    m_ContextDevice->nMinQpI = 0;
    m_ContextDevice->nMaxQpI = 1;
    m_ContextDevice->nMinQpP = 0;
    m_ContextDevice->nMaxQpP = 1;
    m_ContextDevice->nMinQpB = 0;
    m_ContextDevice->nMaxQpB = 1;

//     m_ContextDevice->use_gold_crc = false;
//     m_ContextDevice->pBitStreamCrc = nullptr;
//     m_ContextDevice->externalRCHints = false;
//     m_ContextDevice->input_metadata = false;
//     m_ContextDevice->sMaxQp = 51;
}
*/

int NvEncoderHEVC::setDefaultsContext( void ) {
    int ret = 0;
  
    
    m_ContextDevice->enable_verbose = VERBOSE_DEBUG;
    m_ContextDevice->buffer_length = BUFFER_LENGTH;

	if (m_ContextDevice->runtime_params_str)
    {
        get_next_runtime_param_change_frame(m_ContextDevice);
    }
    
    if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H265) {
//         CHECK_ERROR(m_ContextDevice->width < 144 || m_ContextDevice->height < 144, "Height/Width should be > 144 for H.265");
        if(m_ContextDevice->width < 144 || m_ContextDevice->height < 144)
            ERROR_RETURN("Height/Width should be > 144 for H.265");
    }

    if (m_ContextDevice->use_gold_crc) {
        m_ContextDevice->pBitStreamCrc = Crc_Encode::InitCrc(CRC32_POLYNOMIAL);
        CHECK_ERROR(!m_ContextDevice->pBitStreamCrc, "InitCrc failed");
    }

// FILE    m_ContextDevice->out_file = new ofstream(m_ContextDevice->out_file_path);
//     CHECK_ERROR(!m_ContextDevice->out_file->is_open(), "Could not open output file");

	DEBUG("TRY NvVideoEncoder::createVideoEncoder \n");
    m_ContextDevice->enc = NvVideoEncoder::createVideoEncoder("enc0");
	
	
    CHECK_ERROR(!m_ContextDevice->enc, "Could not create encoder");

	
	
	
	
	DEBUG("TRY NvVideoEncoder::createVideoEncoder setCapturePlaneFormat \n");
    // It is necessary that Capture Plane format be set before Output Plane
    // format.
    // Set encoder capture plane format. It is necessary to set width and
    // height on thr capture plane as well
    // https://devtalk.nvidia.com/default/topic/1027631/formatting-images-to-feed-into-nvvideoencoder-tegra-multimedia-api-/
    ret = m_ContextDevice->enc->setCapturePlaneFormat(
				m_ContextDevice->encoder_pixfmt, 
				m_ContextDevice->width,
				m_ContextDevice->height, 
				(m_ContextDevice->width * m_ContextDevice->height * 3) / 2
			);
    
// 	ret = m_ContextDevice->enc->setCapturePlaneFormat(
// 				m_ContextDevice->encoder_pixfmt, 
// 				m_ContextDevice->width,
// 				m_ContextDevice->height, 
// 				2 * 1024 * 1024
// 			);
// 		(m_ContextDevice->width * m_ContextDevice->height * 
	CHECK_ERROR(ret < 0, "Could not set output plane format");

	
	DEBUG("TRY NvVideoEncoder::createVideoEncoder setOutputPlaneFormat \n");
    // Set encoder output plane format
//     ret = m_ContextDevice->enc->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, m_ContextDevice->width, m_ContextDevice->height);
	ret = m_ContextDevice->enc->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, m_ContextDevice->width, m_ContextDevice->height);

	
	CHECK_ERROR(ret < 0, "Could not set output plane format");

	DEBUG("TRY NvVideoEncoder::createVideoEncoder setBitrate \n");
    ret = m_ContextDevice->enc->setBitrate(m_ContextDevice->bitrate);
    CHECK_ERROR(ret < 0, "Could not set encoder bitrate");

	DEBUG("TRY NvVideoEncoder::createVideoEncoder setProfile \n");
    ret = m_ContextDevice->enc->setProfile(m_ContextDevice->profile);
    CHECK_ERROR(ret < 0, "Could not set encoder profile");

	DEBUG("TRY NvVideoEncoder::createVideoEncoder setLevel \n");
    if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H264) {
        ret = m_ContextDevice->enc->setLevel(m_ContextDevice->level);
        CHECK_ERROR(ret < 0, "Could not set encoder level");
    }

	DEBUG("TRY NvVideoEncoder::createVideoEncoder setRateControlMode \n");
    ret = m_ContextDevice->enc->setRateControlMode(m_ContextDevice->ratecontrol);
    CHECK_ERROR(ret < 0, "Could not set encoder rate control mode");

    ret = m_ContextDevice->enc->setIDRInterval(m_ContextDevice->idr_interval);
    CHECK_ERROR(ret < 0, "Could not set encoder IDR interval");

    ret = m_ContextDevice->enc->setIFrameInterval(m_ContextDevice->iframe_interval);
    CHECK_ERROR(ret < 0, "Could not set encoder I-Frame interval");

    ret = m_ContextDevice->enc->setFrameRate(m_ContextDevice->fps_n, m_ContextDevice->fps_d);
    CHECK_ERROR(ret < 0, "Could not set framerate");

    if (m_ContextDevice->temporal_tradeoff_level) {
        ret = m_ContextDevice->enc->setTemporalTradeoff(m_ContextDevice->temporal_tradeoff_level);
        CHECK_ERROR(ret < 0, "Could not set temporal tradeoff level");
    }

    if (m_ContextDevice->slice_length) {
        ret = m_ContextDevice->enc->setSliceLength(m_ContextDevice->slice_length_type,
                                      m_ContextDevice->slice_length);
        CHECK_ERROR(ret < 0, "Could not set slice length params");
    }

    
	ERROR_RETURN("TRY NvVideoEncoder::createVideoEncoder DONE \n");
// TX2
//     if (m_ContextDevice->hw_preset_type) {
//         ret = m_ContextDevice->enc->setHWPresetType(m_ContextDevice->hw_preset_type);
//         CHECK_ERROR(ret < 0, "Could not set encoder HW Preset Type");
//     }

    if (m_ContextDevice->virtual_buffer_size) {
        ret = m_ContextDevice->enc->setVirtualBufferSize(m_ContextDevice->virtual_buffer_size);
        CHECK_ERROR(ret < 0, "Could not set virtual buffer size");
    }

    if (m_ContextDevice->num_reference_frames) {
        ret = m_ContextDevice->enc->setNumReferenceFrames(m_ContextDevice->num_reference_frames);
        CHECK_ERROR(ret < 0, "Could not set num reference frames");
    }

    if (m_ContextDevice->slice_intrarefresh_interval) {
        ret = m_ContextDevice->enc->setSliceIntrarefresh(m_ContextDevice->slice_intrarefresh_interval);
        CHECK_ERROR(ret < 0, "Could not set slice intrarefresh interval");
    }

    if (m_ContextDevice->insert_sps_pps_at_idr) {
        ret = m_ContextDevice->enc->setInsertSpsPpsAtIdrEnabled(true);
        CHECK_ERROR(ret < 0, "Could not set insertSPSPPSAtIDR");
    }

    if (m_ContextDevice->num_b_frames != (uint32_t) -1) {
        ret = m_ContextDevice->enc->setNumBFrames(m_ContextDevice->num_b_frames);
        CHECK_ERROR(ret < 0, "Could not set number of B Frames");
    }

    if ((m_ContextDevice->nMinQpI != (uint32_t) QP_RETAIN_VAL) ||
        (m_ContextDevice->nMaxQpI != (uint32_t) QP_RETAIN_VAL) ||
        (m_ContextDevice->nMinQpP != (uint32_t) QP_RETAIN_VAL) ||
        (m_ContextDevice->nMaxQpP != (uint32_t) QP_RETAIN_VAL) ||
        (m_ContextDevice->nMinQpB != (uint32_t) QP_RETAIN_VAL) ||
        (m_ContextDevice->nMaxQpB != (uint32_t) QP_RETAIN_VAL)) {
        ret = m_ContextDevice->enc->setQpRange(m_ContextDevice->nMinQpI, m_ContextDevice->nMaxQpI, m_ContextDevice->nMinQpP,
                                  m_ContextDevice->nMaxQpP, m_ContextDevice->nMinQpB, m_ContextDevice->nMaxQpB);
        CHECK_ERROR(ret < 0, "Could not set quantization parameters");
    }

    if (m_ContextDevice->dump_mv) {
        ret = m_ContextDevice->enc->enableMotionVectorReporting();
        CHECK_ERROR(ret < 0, "Could not enable motion vector reporting");
    }

    if (m_ContextDevice->bnoIframe) {
        m_ContextDevice->iframe_interval = ((1 << 31) + 1); /* TODO: how can we do this properly */
        ret = m_ContextDevice->enc->setIFrameInterval(m_ContextDevice->iframe_interval);
        CHECK_ERROR(ret < 0, "Could not set encoder I-Frame interval");
    }

    
// TX2 28.2
//     if (m_ContextDevice->enableROI) {
//         v4l2_enc_enable_roi_param VEnc_enable_ext_roi_ctrl;
// 
//         VEnc_enable_ext_roi_ctrl.bEnableROI = m_ContextDevice->enableROI;
//         ret = m_ContextDevice->enc->enableROI(VEnc_enable_ext_roi_ctrl);
//         CHECK_ERROR(ret < 0, "Could not enable ROI");
//     }
// 
//     if (m_ContextDevice->bReconCrc) {
//         v4l2_enc_enable_reconcrc_param VEnc_enable_recon_crc_ctrl;
// 
//         VEnc_enable_recon_crc_ctrl.bEnableReconCRC = m_ContextDevice->bReconCrc;
//         ret = m_ContextDevice->enc->enableReconCRC(VEnc_enable_recon_crc_ctrl);
//         CHECK_ERROR(ret < 0, "Could not enable Recon CRC");
//     }

//     if (m_ContextDevice->externalRPS) {
//         v4l2_enc_enable_ext_rps_ctr VEnc_enable_ext_rps_ctrl;
// 
//         VEnc_enable_ext_rps_ctrl.bEnableExternalRPS = m_ContextDevice->externalRPS;
//         if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H264) {
//             VEnc_enable_ext_rps_ctrl.bGapsInFrameNumAllowed = m_ContextDevice->bGapsInFrameNumAllowed;
//             VEnc_enable_ext_rps_ctrl.nH264FrameNumBits = m_ContextDevice->nH264FrameNumBits;
//         }
//         if (m_ContextDevice->encoder_pixfmt == V4L2_PIX_FMT_H265) {
//             VEnc_enable_ext_rps_ctrl.nH265PocLsbBits = m_ContextDevice->nH265PocLsbBits;
//         }
// //         ret = m_ContextDevice->enc->enableExternalRPS(VEnc_enable_ext_rps_ctrl); // TX2
//         CHECK_ERROR(ret < 0, "Could not enable external RPS");
//     }

//     if (m_ContextDevice->externalRCHints) {
//         v4l2_enc_enable_ext_rate_ctr VEnc_enable_ext_rate_ctrl;
// 
//         VEnc_enable_ext_rate_ctrl.bEnableExternalPictureRC = m_ContextDevice->externalRCHints;
//         VEnc_enable_ext_rate_ctrl.nsessionMaxQP = m_ContextDevice->sMaxQp;
// 
// //         ret = m_ContextDevice->enc->enableExternalRC(VEnc_enable_ext_rate_ctrl); // TX2
//         CHECK_ERROR(ret < 0, "Could not enable external RC");
//     }

    // Query, Export and Map the output plane buffers so that we can read
    // raw data into the buffers
    ret = m_ContextDevice->enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
    CHECK_ERROR(ret < 0, "Could not setup output plane");

    // Query, Export and Map the output plane buffers so that we can write
    // encoded data from the buffers
    ret = m_ContextDevice->enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
    CHECK_ERROR(ret < 0, "Could not setup capture plane");

    // output plane STREAMON
    ret = m_ContextDevice->enc->output_plane.setStreamStatus(true);
    CHECK_ERROR(ret < 0, "Error in output plane streamon");

    // capture plane STREAMON
    ret = m_ContextDevice->enc->capture_plane.setStreamStatus(true);
    CHECK_ERROR(ret < 0, "Error in capture plane streamon");

    m_ContextDevice->enc->capture_plane.
            setDQThreadCallback( NvEncoderHEVC::encoder_capture_plane_dq_callback );

    // startDQThread starts a thread internally which calls the
    // encoder_capture_plane_dq_callback whenever a buffer is dequeued
    // on the plane
    m_ContextDevice->enc->capture_plane.startDQThread(m_ContextDevice);

    // Enqueue all the empty capture plane buffers
    for (uint32_t i = 0; i < m_ContextDevice->enc->capture_plane.getNumBuffers(); i++) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = m_ContextDevice->enc->capture_plane.qBuffer(v4l2_buf, nullptr);
        if (ret < 0) {
            std::cout << "Error while queueing buffer at capture plane" << std::endl;
            abort(m_ContextDevice);
            //goto cleanup;
            return cleanup( );
//             return cleanup(m_ContextDevice, 1);
        }
    }
    
        ERROR_RETURN( "capture_plane done ");
    /*
    // Read video frame and queue all the output plane buffers
    for (uint32_t i = 0; i < m_ContextDevice->enc->output_plane.getNumBuffers(); i++)
    {
		
            std::cout << "Read video frame and queue all the output plane buffers :: " << i << std::endl;
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer = m_ContextDevice->enc->output_plane.getNthBuffer(i);

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        if (read_video_frame(m_ContextDevice->in_file, *buffer) < 0)
        {
            std::cout << "Could not read complete frame from input file" << std::endl;
            v4l2_buf.m.planes[0].bytesused = 0;
        }

        if (m_ContextDevice->runtime_params_str &&
            (m_ContextDevice->enc->output_plane.getTotalQueuedBuffers() ==
                m_ContextDevice->next_param_change_frame))
        {
            set_runtime_params(m_ContextDevice);
            if (m_ContextDevice->runtime_params_str)
                get_next_runtime_param_change_frame(m_ContextDevice);
        }

        if (m_ContextDevice->ROI_Param_file_path)
        {
            v4l2_enc_frame_ROI_params VEnc_ROI_params;

            populate_roi_Param(m_ContextDevice->roi_Param_file, &VEnc_ROI_params);

            m_ContextDevice->enc->setROIParams(v4l2_buf.index, VEnc_ROI_params);
            v4l2_buf.reserved2 = v4l2_buf.index;
        }

        ret = m_ContextDevice->enc->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            std::cout << "Error while queueing buffer at output plane" << std::endl;
            abort(m_ContextDevice);
//             goto cleanup;
        }

        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            std::cout << "File read complete." << std::endl;
//             eos = true;
//             break;
        }
    }
    */
    return ret;
}


void NvEncoderHEVC::setEncodePixFormat (uint32_t _pixfmt )
{
    
    if (m_ContextDevice->width < 144 || m_ContextDevice->height < 144 ) {
        _pixfmt = V4L2_PIX_FMT_H264;
        ERROR_RETURN( "Height/Width should be > 144 for H.265 , encode will become on H.264");
    }
    
    m_ContextDevice->encoder_pixfmt = _pixfmt ;
}

void NvEncoderHEVC::setOutFilePath( const char *_file ) {
    m_ContextDevice->out_file_path = strdup(_file);

}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//					SETTINGS CONTEXT END
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


/**
 * @fn bool camera_initialize(Context_T * ctx)
 * @brief   init size format fror video
 * @warning V4L2_PIX_FMT_UYVY format UYVY CbCr
 * @see
 */
bool NvEncoderHEVC::camera_initialize(Context_T * ctx , char *aimage) {

//     ctx->width = aimage->GetWidth();
//     ctx->height = aimage->GetHeight();
//     ctx->pvsize = aimage->GetImageSize();
    
    ctx->cam_pixfmt = V4L2_PIX_FMT_UYVY;
//     ctx->cam_pixfmt = aimage->GetPixelType();        //< @todo PV FORMAT , convert to V4L2
    
//     prepare_buffers( ctx );
    
    
    return true;
}


/**
 * @fn bool NvEncoderHEVC::vic_initialize(Context_T * ctx)
 * @brief init convert BGRa to YUV420M , need camera_initialize for fix pix and size formats
 * @see NvEncoderHEVC::camera_initialize 
 */
/*
bool NvEncoderHEVC::vic_initialize(Context_T * ctx)
{
    // Create VIC (VIdeo Converter) instance
    ctx->conv = NvVideoConverter::createVideoConverter("conv");
    if (ctx->conv == NULL)
        ERROR_RETURN("Failed to create video converter");

    if (ctx->vic_flip != -1 && ctx->conv->setFlipMethod(ctx->vic_flip) < 0)
        ERROR_RETURN("Failed to set flip method");

    if (ctx->vic_interpolation != -1 &&
            ctx->conv->setInterpolationMethod(ctx->vic_interpolation) < 0)
            ERROR_RETURN("Failed to set interpolation method");

    if (ctx->vic_tnr != -1 && ctx->conv->setTnrAlgorithm(ctx->vic_tnr) < 0)
            ERROR_RETURN("Failed to set tnr algorithm");

    // Set up VIC output plane format
    if (ctx->conv->setOutputPlaneFormat(ctx->cam_pixfmt, ctx->cam_w,
                ctx->cam_h, V4L2_NV_BUFFER_LAYOUT_PITCH) < 0)
        ERROR_RETURN("Failed to set up VIC output plane format");

    // Set up VIC capture plane format
    // The target format can be reconfigured from set_defaults()
    if (ctx->conv->setCapturePlaneFormat(ctx->vic_pixfmt, ctx->cam_w,
                ctx->cam_h, V4L2_NV_BUFFER_LAYOUT_PITCH) < 0)
        ERROR_RETURN("Failed to set up VIC capture plane format");

    // Allocate VIC output plane
    if (ctx->conv->output_plane.setupPlane(V4L2_MEMORY_DMABUF,
                V4L2_BUFFERS_NUM, false, false) < 0)
        ERROR_RETURN("Failed to allocate VIC output plane");

    // Allocate VIC capture plane
    if (ctx->conv->capture_plane.setupPlane(V4L2_MEMORY_MMAP,
                V4L2_BUFFERS_NUM, true, false) < 0)
        ERROR_RETURN("Failed to allocate VIC capture plane");

    return true;
}
*/

int NvEncoderHEVC::set_runtime_params(Context_T  *ctx)
{
    bool lberr =false;
    char charval;
    uint32_t intval;
    int ret;

    std::cout << "Frame " << ctx->next_param_change_frame <<
        ": Changing parameters" << std::endl;
    while (! (ctx->runtime_params_str->eof()) )
    {
        ret = get_next_parsed_pair(ctx, &charval, &intval);
        lberr = lberr || (ret < 0 );
        switch (charval)
        {
            case 'b':
                ctx->enc->setBitrate(intval);
                std::cout << "Bitrate = " << intval << std::endl;
                break;
            case 'r':
            {
                int fps_num = intval;
                lberr = lberr || ( ret != '/');

                ctx->runtime_params_str->seekg(-1, std::ios::cur);
                ret = get_next_parsed_pair(ctx, &charval, &intval);
                lberr = lberr || ( ret < 0);

                std::cout << "Framerate = " << fps_num << "/"  << intval << std::endl;

                ctx->enc->setFrameRate(fps_num, intval);
                break;
            }
            case 'i':
                if (intval > 0)
                {
                    ctx->enc->forceIDR();
                    std::cout << "Forcing IDR" << std::endl;
                }
                break;
            default:
                lberr = true;
                    std::cout << "error IDR" << std::endl;
        }
        switch (ret)
        {
            case 0:
                delete ctx->runtime_params_str;
                ctx->runtime_params_str = NULL;
                return 0;
            case ';':
                return 0;
            case ',':
                break;
            default:
                break;
        }
    }
//     return 0;
    if ( lberr ) {
        std::cout << "Skipping further runtime parameter changes" << std::endl;
        delete ctx->runtime_params_str;
        ctx->runtime_params_str = NULL;
    }
    return (lberr ? -1 : 0 );
}


int NvEncoderHEVC::get_next_parsed_pair(Context_T  *ctx, char *id, uint32_t *value)
{
    char charval;

    *ctx->runtime_params_str >> *id;
    if (ctx->runtime_params_str->eof())
    {
        return -1;
    }

    charval = ctx->runtime_params_str->peek();
    if (!IS_DIGIT(charval))
    {
        return -1;
    }

    *ctx->runtime_params_str >> *value;

    *ctx->runtime_params_str >> charval;
    if (ctx->runtime_params_str->eof())
    {
        return 0;
    }

    return charval;
}


void NvEncoderHEVC::populate_roi_Param(std::ifstream * stream, v4l2_enc_frame_ROI_params *VEnc_ROI_params)
{
    unsigned int ROIIndex = 0;

    if (!stream->eof()) {
        *stream >> VEnc_ROI_params->num_ROI_regions;
        while (ROIIndex < VEnc_ROI_params->num_ROI_regions)
        {
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].QPdelta;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.left;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.top;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.width;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.height;
            ROIIndex++;
        }
    } else {
        std::cout << "EOF of ROI_param_file & rewind" << std::endl;
        stream->clear();
        stream->seekg(0);
    }
}


/**
 * @fn
 * @brief read_video_frame(  char *aImage, NvBuffer & buffer)
 * 
 * DATA =  4 Y + 2 U + 2 V => 4Y + 4(U+V) => seek :: 4 + 4
 */

/*
int NvEncoderHEVC::read_video_frame(  char *aImage, NvBuffer & buffer)
{
    uint32_t i, j;
    char *data = nullptr ;
    uint32_t gcount = 0;
     const uint8_t *lData ; //= aImage->GetDataPointer();
    
    
    for (i = 0; i < buffer.n_planes; i++)
    {
        NvBuffer::NvBufferPlane &plane = buffer.planes[i];
//         std::streamsize bytes_to_read = plane.fmt.bytesperpixel * plane.fmt.width;
        uint32_t bytes_to_read = plane.fmt.bytesperpixel * plane.fmt.width;
        
        
        data = (char *) plane.data;
        plane.bytesused = 0;
        for (j = 0; j < plane.fmt.height; j++)
        {
            if (i == 0) { // Y
//                 stream->read(data, bytes_to_read);
                memcpy( data , (char *)lData , bytes_to_read * sizeof( char ));
                
                
            } else { // U and V
//                 stream->seekg(bytes_to_read, stream->cur);
                lData = lData + bytes_to_read ;
//                 gcount += bytes_to_read;
                memset(data, 0x80, bytes_to_read);
            }
            
//             if (gcount  < bytes_to_read)
//                 return -1;
            
            data += plane.fmt.stride;
        }
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }
    return 0;
}

*/


//! close opened MP4 file
void NvEncoderHEVC::Close( void ) {
    
    uint32_t empty;
    // remove last image
    WriteFrame( nullptr , &empty);
    
    // Wait EOS
    if( mh264CaptureLoop->joinable() ) {
        mh264CaptureLoop->join();
    }
    
    cleanup();
}
    
    
//! WriteFrame on opened MP4 file
bool NvEncoderHEVC::WriteFrame( char *aImage, uint32_t *aFileSizeDelta ) {
    bool lret = true;
    
    std::lock_guard<std::mutex> lguard( mMuxCurrentImage );
    mCurrentImage = aImage;
    
    return lret;
}


char *NvEncoderHEVC::getPvFrame( void ) {

        std::lock_guard<std::mutex> lguard( mMuxCurrentImage );
    return mCurrentImage ;
}
    
    
uint32_t NvEncoderHEVC::cleanup( void ) {
    
    uint32_t error = 0 ;
    if (m_ContextDevice->enc && m_ContextDevice->enc->isInError())
    {
        ERROR_RETURN("Encoder is in error" );
        error = 1;
    }
    if (m_ContextDevice->got_error)
    {
        error = 1;
    }

    delete m_ContextDevice->enc;
    delete m_ContextDevice->in_file;
    delete m_ContextDevice->out_file;
    delete m_ContextDevice->roi_Param_file;

    free(m_ContextDevice->in_file_path);
    free(m_ContextDevice->out_file_path);
    free(m_ContextDevice->ROI_Param_file_path);
    delete m_ContextDevice->runtime_params_str;


    if (error)
        ERROR_RETURN("App run failed\n");
    else
        INFO("App run was successful\n");

        DEBUG("encodeFrame : cleanup = %lli \n" , m_frame_count );
    return -error;
    
}



// -------------------------------------------
// TX2 CODE
// -------------------------------------------

/*
int encode_frames(Context_T& ctx) {
    int ret = 0;

    // Keep reading input till EOS is reached
    while (!ctx.got_error && !ctx.enc->isInError() && !g_eos)
    {
        
        // INI BUFFER
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

        
        // READ SOURCE
        if (ctx.enc->output_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 10) < 0)
        {
            cerr << "ERROR while DQing buffer at output plane" << endl;
            abort(&ctx);
            //goto cleanup;
            return cleanup(ctx, 1);
        }
        
        // RUNTIME API :: send FRAME to HEVC DEVICE

        if (ctx.runtime_params_str &&
            (ctx.enc->output_plane.getTotalQueuedBuffers() ==
             ctx.next_param_change_frame))
        {
            set_runtime_params(&ctx);
            if (ctx.runtime_params_str)
                get_next_runtime_param_change_frame(&ctx);
        }
        
        // READ HEVC DEVICE BUFFER
        if (read_video_frame(ctx.in_file, *buffer) < 0)
        {
            cerr << "Could not read complete frame from input file" << endl;
            v4l2_buf.m.planes[0].bytesused = 0;
        }

        
        
        // 
        if (ctx.input_metadata)
        {
            v4l2_ctrl_videoenc_input_metadata VEnc_imeta_param;
            v4l2_enc_frame_ROI_params VEnc_ROI_params;
            v4l2_enc_frame_ReconCRC_params VEnc_ReconCRC_params;
            v4l2_enc_frame_ext_rps_ctrl_params VEnc_ext_rps_ctrl_params;
            v4l2_enc_frame_ext_rate_ctrl_params VEnc_ext_rate_ctrl_params;
            v4l2_enc_gdr_params VEnc_gdr_params;
            VEnc_imeta_param.flag = 0;

            if (ctx.ROI_Param_file_path)
            {
                if (ctx.enableROI) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_ROI_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncROIParams = &VEnc_ROI_params;
                    populate_roi_Param(ctx.roi_Param_file, VEnc_imeta_param.VideoEncROIParams);
                }
            }

            if (ctx.bReconCrc)
            {
                VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RECONCRC_PARAM_FLAG;

                VEnc_ReconCRC_params.ReconCRCRect.left = ctx.rl;
                VEnc_ReconCRC_params.ReconCRCRect.top = ctx.rt;
                VEnc_ReconCRC_params.ReconCRCRect.width = ctx.rw;
                VEnc_ReconCRC_params.ReconCRCRect.height = ctx.rh;

                VEnc_imeta_param.VideoReconCRCParams = &VEnc_ReconCRC_params;
            }

            if (ctx.RPS_Param_file_path)
            {
                if (ctx.externalRPS) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RPS_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncRPSParams = &VEnc_ext_rps_ctrl_params;
                    populate_ext_rps_ctrl_Param(ctx.rps_Param_file, VEnc_imeta_param.VideoEncRPSParams);
                }
            }

            if (ctx.GDR_Param_file_path)
            {
                if (ctx.enableGDR)
                {
                    if (ctx.gdr_start_frame_number == 0xFFFFFFFF)
                        populate_gdr_Param(ctx.gdr_Param_file, &ctx.gdr_start_frame_number,
                                           &ctx.gdr_num_frames);
                    if (g_input_frames_queued_count == ctx.gdr_start_frame_number)
                    {
                        ctx.gdr_out_frame_number = ctx.gdr_start_frame_number;
                        VEnc_gdr_params.nGDRFrames = ctx.gdr_num_frames;
                        VEnc_imeta_param.flag |= V4L2_ENC_INPUT_GDR_PARAM_FLAG;
                        VEnc_imeta_param.VideoEncGDRParams = &VEnc_gdr_params;
                    }
                }
            }

            if (ctx.hints_Param_file_path)
            {
                if (ctx.externalRCHints) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RC_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncExtRCParams = &VEnc_ext_rate_ctrl_params;

                    populate_ext_rate_ctrl_Param(ctx.hints_Param_file, VEnc_imeta_param.VideoEncExtRCParams);

                }
            }

            if (VEnc_imeta_param.flag)
            {
                ctx.enc->SetInputMetaParams(v4l2_buf.index, VEnc_imeta_param);
                v4l2_buf.reserved2 = v4l2_buf.index;
            }
        }

        ret = ctx.enc->output_plane.qBuffer(v4l2_buf, nullptr);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at output plane" << endl;
            abort(&ctx);
            //goto cleanup;
            return cleanup(ctx, 1);
        }

        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            cerr << "File read complete." << endl;
            g_eos = getStreamEOS();
            
        }
        g_input_frames_queued_count++;
    }

    return ret;
}

*/

// NvBufferDestroy



void NvEncoderHEVC::ClearListHEVC( void ) {
    while(!mlistBufferHEVC.empty()) {
//         ldata =  mlistBufferHEVC.front();
        mlistBufferHEVC.pop_front();
    }
}


// ------------------------------------------------------------------------


NvEncoderHEVC::DataHEVC * NvEncoderHEVC::popBufferHEVC( void ) {
    
    std::lock_guard<std::mutex> lck(mMuxListHEVC);
//     std::cout << ' ' << mlistBufferHEVC.front();
    NvEncoderHEVC::DataHEVC *ldata = nullptr;
    if(!mlistBufferHEVC.empty()) {
        
        ldata =  mlistBufferHEVC.front();
        
        mlistBufferHEVC.pop_front();
    }
    
    int lret = (mlistBufferHEVC.size()) ;
    ERROR_RETURN("popBufferHEVC h264 to Apps %d \n" , lret );
    return ldata;
}


void NvEncoderHEVC::pushBufferHEVC( NvEncoderHEVC::DataHEVC *_data) {
    
    std::lock_guard<std::mutex> lck(mMuxListHEVC);
    if( BUFFER_LENGTH < mlistBufferHEVC.size() ) {
        NvEncoderHEVC::DataHEVC *ldata = popBufferHEVC();
        
        if( ldata != nullptr ){
            delete ldata;
            ldata = nullptr;
        }
        ERROR_RETURN( "pushBufferHEVC :: LOST and DROP HEVC FRAME ");
    } 
    
    mlistBufferHEVC.push_back (_data);
    
    int lret = (mlistBufferHEVC.size()) ;
    ERROR_RETURN("pushBufferHEVC frame h264  : %d \n" , lret );
}




Image_T * NvEncoderHEVC::popBufferImage( void ) {
    
    std::lock_guard<std::mutex> lck(mMuxListImage);
//     std::cout << ' ' << mlistBufferHEVC.front();
    Image_T *ldata = nullptr;
    if(!mlistBufferImage.empty()) {
        
        ldata =  mlistBufferImage.front();
        
        mlistBufferImage.pop_front();
    }
    
    int lret = mlistBufferImage.size() ;
    printf("pop image : pop_front %d done \n" , lret );
    return ldata;
}



void NvEncoderHEVC::pushBufferImage( Image_T *_data) {
    
    
    if ( !( isInitialized() ) ) {
        
        printf("pushBufferImage First image : %d x %d \n" , _data->width ,_data->height );
        m_ContextDevice->height = _data->height;
        m_ContextDevice->width = _data->width;
        
        printf("pushBufferImage First image : setDefaultsContext \n"   );
        setDefaultsContext();
        
        printf("pushBufferImage First image : V4L2_PIX_FMT_H264 \n");
        setEncodePixFormat( V4L2_PIX_FMT_H264 );
        
//             m_ContextDevice->out_file_path = strdup(output_fname);
        setInitialized( true );
        
        printf("pushBufferImage First image : mInitialized done \n" );
    }
    
    if( m_ContextDevice->buffer_length < SizeListBufferImage() ) {
        Image_T *ldata = popBufferImage();
        
        if( ldata != nullptr ){
            delete ldata;
            ldata = nullptr;
        }
        ERROR_RETURN( "pushBufferImage :: DROP FRAME IMAGE");
    } 
    
    addBufferImage(_data);    
    printf("add image : push_back %d done \n" , SizeListBufferImage());
}



bool NvEncoderHEVC::isInitialized( void ) {
    std::lock_guard<std::mutex> lck(mMuxListImage);
    return mInitialized;    
}
uint32_t NvEncoderHEVC::setInitialized( bool _stat ) {
    std::lock_guard<std::mutex> lck(mMuxListImage);
    mInitialized = _stat;    
}
void NvEncoderHEVC::addBufferImage( Image_T *_data ) {
    
    std::lock_guard<std::mutex> lck(mMuxListImage);
    mlistBufferImage.push_back (_data);    
    
}
uint32_t NvEncoderHEVC::SizeListBufferImage( void ) {
    std::lock_guard<std::mutex> lck(mMuxListImage);
    return mlistBufferImage.size ();    
}
// THREAD 


void NvEncoderHEVC::run( void ) {

        mLoopCompressHEVC = new std::thread(&NvEncoderHEVC::LoopCompressFrame, this);

        mLoopCompressHEVC->detach();
    
}



void NvEncoderHEVC::LoopCompressFrame( void ) {
    
    bool leos = false;
    while  ( !leos ) {
        
        Image_T * ldata = popBufferImage();
        
        if( ldata != nullptr ) {
            if( ldata->imageData != nullptr ) {
                
                DEBUG("LoopCompressFrame image : %d x %d \n" , ldata->width ,ldata->height );
                compressYFrame( ldata );
            }
            else{
                DEBUG("LoopCompressFrame wait end \n"  );
                compressYFrame( nullptr );
//                 setStreamEOS(true);
            }
        }
        else{
            
                DEBUG("LoopCompressFrame wait image \n"  );
            std::this_thread::sleep_for(std::chrono::milliseconds(500)) ;
        }
        
        leos = getStreamEOS();
    }
    
    
}

/*
a YUV buffer is 1.5x the size of a raw Y buffer
https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-yuv420m.html
*/
void NvEncoderHEVC::compressYFrame(Image_T *_data) {
    
    uint8_t *y_bytes = _data->imageData;
    if (y_bytes) {
        
         DEBUG("compress image : %d x %d \n" , _data->width ,_data->height );
        
        uint8_t *yuv_bytes = (uint8_t *) malloc((m_ContextDevice->height * m_ContextDevice->width * 3) / 2);
        //copy the raw luminance values into the new buffer
        memcpy(yuv_bytes, y_bytes, m_ContextDevice->height * m_ContextDevice->width);
        int offset = m_ContextDevice->height * m_ContextDevice->width;
        uint8_t blank_uv = 0x80;
        //fill in the U and V fields with blank chrominance values
        memset(yuv_bytes + offset, blank_uv, (m_ContextDevice->height * m_ContextDevice->width) / 2);
		
		
         DEBUG("compress to compressYUV420Frame : %d x %d \n" , _data->width ,_data->height );
        compressYUV420Frame(yuv_bytes);
//         free(yuv_bytes);
//         free(_data);
    } else {
        compressYUV420Frame(nullptr);
    }
}



/*
a YUV buffer is 1.5x the size of a raw Y buffer
https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-yuv420m.html
*/
// void NvEncoderHEVC::compressYFrame(uint8_t *y_bytes) {
//     if (y_bytes) {
//         
//         uint8_t *yuv_bytes = (uint8_t *) malloc((m_ContextDevice->height * m_ContextDevice->width * 3) / 2);
//         //copy the raw luminance values into the new buffer
//         memcpy(yuv_bytes, y_bytes, m_ContextDevice->height * m_ContextDevice->width);
//         int offset = m_ContextDevice->height * m_ContextDevice->width;
//         uint8_t blank_uv = 0x80;
//         //fill in the U and V fields with blank chrominance values
//         memset(yuv_bytes + offset, blank_uv, (m_ContextDevice->height * m_ContextDevice->width) / 2);
//         compressYUV420Frame(yuv_bytes);
//         free(yuv_bytes);
//     } else {
//         compressYUV420Frame(nullptr);
//     }
// }


/*
a YUV buffer is 1.5x the size of a raw Y buffer
https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-yuv420m.html
*/
void NvEncoderHEVC::compressYUV420Frame(uint8_t *yuv_bytes) {
    encodeFrame(yuv_bytes);
}



void NvEncoderHEVC::copyYuvToBuffer(uint8_t *yuv_bytes, NvBuffer &buffer) {

    uint32_t i, j, k;
    uint8_t *src = yuv_bytes;
    uint8_t *dst;


    for (i = 0; i < buffer.n_planes; i++) {

        NvBuffer::NvBufferPlane &plane = buffer.planes[i];
        uint32_t bytes_per_row = plane.fmt.bytesperpixel * plane.fmt.width;
        dst = plane.data;
        plane.bytesused = 0;

        for (j = 0; j < plane.fmt.height; j++) {
            for (k = 0; k < bytes_per_row; k++) {
                *dst = *src;
                dst++;
                src++;
            }
            dst += (plane.fmt.stride - bytes_per_row);
        }
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }
}


int NvEncoderHEVC::get_next_runtime_param_change_frame(Context_T *ctx)
{
    char charval;
    int ret;
    bool lberr = false;
    ret = get_next_parsed_pair(ctx, &charval, &ctx->next_param_change_frame);
    if(ret == 0)
    {
        return 0;
    }

    lberr = (ret != ';' && ret != ',') || charval != 'f' ;


    if( lberr ) {
        std::cout << "Skipping further runtime parameter changes" << std::endl;
        delete ctx->runtime_params_str;
        ctx->runtime_params_str = NULL;
    }
    return (lberr ? -1 : 0 );
}

void NvEncoderHEVC::abort( Context_T *ctx)
{
    ctx->got_error = true;
    ctx->enc->abort();
}


/*
bool NvEncoderHEVC::encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer,
                                  NvBuffer * shared_buffer, void *arg)
{
    Context_T *ctx = (Context_T *) arg;
    NvVideoEncoder *enc = ctx->enc;
    uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;
    uint32_t ReconRef_Y_CRC = 0;
    uint32_t ReconRef_U_CRC = 0;
    uint32_t ReconRef_V_CRC = 0;
    static uint32_t num_encoded_frames = 1;

    if (v4l2_buf == nullptr)
    {
        std::cout << "Error while dequeing buffer from output plane" << std::endl;
        abort(ctx);
        return false;
    }

    // GOT EOS from encoder. Stop dqthread.
    if (buffer->planes[0].bytesused == 0)
    {
        return false;
    }

    // Computing CRC with each frame
    if(ctx->pBitStreamCrc)
        Crc_Encode::CalculateCrc (ctx->pBitStreamCrc, buffer->planes[0].data, buffer->planes[0].bytesused);

    write_encoder_output_frame(ctx, buffer);
    num_encoded_frames++;

    // Accounting for the first frame as it is only sps+pps
    if (ctx->gdr_out_frame_number != 0xFFFFFFFF)
        if ( (ctx->enableGDR) && (ctx->GDR_out_file_path) && (num_encoded_frames >= ctx->gdr_out_frame_number+1))
            write_encoder_output_frame(ctx, buffer);

    if (ctx->report_metadata)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
        if (ctx->enc->getMetadata(v4l2_buf->index, enc_metadata) == 0)
        {
            if (ctx->bReconCrc && enc_metadata.bValidReconCRC) {
                // CRC for Recon frame
                std::cout << "Frame: " << frame_num << std::endl;
                std::cout << "ReconFrame_Y_CRC " << enc_metadata.ReconFrame_Y_CRC <<
                     " ReconFrame_U_CRC " << enc_metadata.ReconFrame_U_CRC <<
                     " ReconFrame_V_CRC " << enc_metadata.ReconFrame_V_CRC <<
                     std::endl;

                if (!ctx->recon_Ref_file->eof()) {
                    *ctx->recon_Ref_file >> ReconRef_Y_CRC;
                    *ctx->recon_Ref_file >> ReconRef_U_CRC;
                    *ctx->recon_Ref_file >> ReconRef_V_CRC;
                }

                if ((ReconRef_Y_CRC != enc_metadata.ReconFrame_Y_CRC) ||
                    (ReconRef_U_CRC != enc_metadata.ReconFrame_U_CRC) ||
                    (ReconRef_V_CRC != enc_metadata.ReconFrame_V_CRC))
                {
                    std::cout << "Recon CRC FAIL" << std::endl;
                    std::cout << "ReconRef_Y_CRC " << ReconRef_Y_CRC <<
                         " ReconRef_U_CRC " << ReconRef_U_CRC <<
                         " ReconRef_V_CRC " << ReconRef_V_CRC <<
                         std::endl;
                    abort(ctx);
                    return false;
                }
                std::cout << "Recon CRC PASS for frame : " << frame_num << std::endl;
            } else if (ctx->externalRPS && enc_metadata.bRPSFeedback_status) {
                // RPS Feedback
                std::cout << "Frame: " << frame_num << std::endl;
                std::cout << "nCurrentRefFrameId " << enc_metadata.nCurrentRefFrameId <<
                     " nActiveRefFrames " << enc_metadata.nActiveRefFrames << std::endl;

                for (uint32_t i = 0; i < enc_metadata.nActiveRefFrames; i++)
                {
                    std::cout << "FrameId " << enc_metadata.RPSList[i].nFrameId <<
                         " IdrFrame " <<  (int) enc_metadata.RPSList[i].bIdrFrame <<
                         " LTRefFrame " <<  (int) enc_metadata.RPSList[i].bLTRefFrame <<
                         " PictureOrderCnt " << enc_metadata.RPSList[i].nPictureOrderCnt <<
                         " FrameNum " << enc_metadata.RPSList[i].nFrameNum <<
                         " LTFrameIdx " <<  enc_metadata.RPSList[i].nLTRFrameIdx << std::endl;
                }
            } else if (ctx->externalRCHints) {
                // Rate Control Feedback
                std::cout << "Frame: " << frame_num << std::endl;
                std::cout << "EncodedBits " << enc_metadata.EncodedFrameBits <<
                     " MinQP " << enc_metadata.FrameMinQP <<
                     " MaxQP " << enc_metadata.FrameMaxQP <<
                     std::endl;
            } else {
                std::cout << "Frame " << frame_num <<
                     ": isKeyFrame=" << (int) enc_metadata.KeyFrame <<
                     " AvgQP=" << enc_metadata.AvgQP <<
                     " MinQP=" << enc_metadata.FrameMinQP <<
                     " MaxQP=" << enc_metadata.FrameMaxQP <<
                     " EncodedBits=" << enc_metadata.EncodedFrameBits <<
                     std::endl;
            }
        }
    }
    if (ctx->dump_mv)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata_MV enc_mv_metadata;
        if (ctx->enc->getMotionVectors(v4l2_buf->index, enc_mv_metadata) == 0)
        {
            uint32_t numMVs = enc_mv_metadata.bufSize / sizeof(MVInfo);
            MVInfo *pInfo = enc_mv_metadata.pMVInfo;

            std::cout << "Frame " << frame_num << ": Num MVs=" << numMVs << std::endl;

            for (uint32_t i = 0; i < numMVs; i++, pInfo++)
            {
                std::cout << i << ": mv_x=" << pInfo->mv_x <<
                     " mv_y=" << pInfo->mv_y <<
                     " weight=" << pInfo->weight <<
                     std::endl;
            }
        }
    }

    if (enc->capture_plane.qBuffer(*v4l2_buf, nullptr) < 0)
    {
        std::cout << "Error while Qing buffer at capture plane" << std::endl;
        abort(ctx);
        return false;
    }

    return true;
}
*/



int NvEncoderHEVC::write_encoder_output_frame(Context_T *ctx, NvBuffer * buffer)
{
//     stream->write((char *) buffer->planes[0].data, buffer->planes[0].bytesused);
    
    
    DataHEVC *ldata = new DataHEVC ;
    
//     ldata->width = 
    ldata->size = buffer->planes[0].bytesused;
    
    ldata->buffer = (char *) malloc( ldata->size *sizeof( char )  ) ;
    memcpy(  ldata->buffer , buffer->planes[0].data  , ldata->size); 
    
        std::cout << "write_encoder_output_frame ::pushBufferHEVC  " << std::endl;
       
    pushBufferHEVC( ldata );
    
    
    return 0;
}


bool NvEncoderHEVC::encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer, NvBuffer * shared_buffer, void *arg)
{
    
    
    Context_T  *ctx = static_cast<Context_T *>(arg); // (Context_T *)
    printf(" encoder_capture_plane_dq_callback :: tro to capture %s !!!\n" , ( ctx != nullptr ? "Context find" : "No context ") );
    NvVideoEncoder *enc = ctx->enc;
    uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;

    if (v4l2_buf == NULL)
    {
        ERROR_RETURN("Error while dequeing buffer from output plane" );
//         abort(ctx);
        return false;
    }

    // GOT EOS from encoder. Stop dqthread.
    if (buffer->planes[0].bytesused == 0)
    {
        return false;
    }

    write_encoder_output_frame(ctx, buffer);

    if (ctx->report_metadata)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
        if (ctx->enc->getMetadata(v4l2_buf->index, enc_metadata) == 0)
        {
            std::cout << "Frame " << frame_num <<
                ": isKeyFrame=" << (int) enc_metadata.KeyFrame <<
                " AvgQP=" << enc_metadata.AvgQP <<
                " MinQP=" << enc_metadata.FrameMinQP <<
                " MaxQP=" << enc_metadata.FrameMaxQP <<
                " EncodedBits=" << enc_metadata.EncodedFrameBits <<
            std::endl;
        }
    }
    if (ctx->dump_mv)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata_MV enc_mv_metadata;
        if (ctx->enc->getMotionVectors(v4l2_buf->index, enc_mv_metadata) == 0)
        {
            uint32_t numMVs = enc_mv_metadata.bufSize / sizeof(MVInfo);
            MVInfo *pInfo = enc_mv_metadata.pMVInfo;

            std::cout << "Frame " << frame_num << ": Num MVs=" << numMVs << std::endl;

            for (uint32_t i = 0; i < numMVs; i++, pInfo++)
            {
                std::cout << i << ": mv_x=" << pInfo->mv_x << " mv_y=" << pInfo->mv_y << " weight=" << pInfo->weight << std::endl;
            }
        }
    }

    if (enc->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
    {
        ERROR_RETURN( "Error while Qing buffer at capture plane" );
//         abort(ctx);
        return false;
    }

    return true;
}

/*
int NvEncoderHEVC::encodeFrame(uint8_t *yuv_bytes) {
	
		int ret = 0;
		int error = 0;
    // Keep reading input till EOS is reached
//     while (!m_ContextDevice->got_error && !m_ContextDevice->enc->isInError() && !eos)
//     {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
		NvBuffer *buffer = m_ContextDevice->enc->output_plane.getNthBuffer(m_frame_count);

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

         DEBUG("encodeFrame to dqBuffer : %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
        if (m_ContextDevice->enc->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0)
        {
            std::cout << "ERROR while DQing buffer at output plane" << std::endl;
//             abort(m_ContextDevice);
			setStreamEOS(true);
//             goto cleanup;
        }

		
         DEBUG("encodeFrame to runtime_params_str : %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
        if (m_ContextDevice->runtime_params_str &&
            (m_ContextDevice->enc->output_plane.getTotalQueuedBuffers() ==
                m_ContextDevice->next_param_change_frame))
        {
            set_runtime_params(m_ContextDevice);
            if (m_ContextDevice->runtime_params_str)
                get_next_runtime_param_change_frame(m_ContextDevice);
        }
        
        
//          DEBUG("encodeFrame to read_video_frame : %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
//         if (read_video_frame(m_ContextDevice->in_file, *buffer) < 0)
//         {
//             std::cout << "Could not read complete frame from input file" << std::endl;
//             v4l2_buf.m.planes[0].bytesused = 0;
//         }


        
         DEBUG("encodeFrame to copyYuvToBuffer : %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
		if(yuv_bytes)
            copyYuvToBuffer(yuv_bytes, *buffer);
        else
            v4l2_buf.m.planes[0].bytesused = 0;

		

        if (m_ContextDevice->ROI_Param_file_path)
        {
            v4l2_enc_frame_ROI_params VEnc_ROI_params;

            populate_roi_Param(m_ContextDevice->roi_Param_file, &VEnc_ROI_params);

            m_ContextDevice->enc->setROIParams(v4l2_buf.index, VEnc_ROI_params);
            v4l2_buf.reserved2 = v4l2_buf.index;
        }

        ret = m_ContextDevice->enc->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            std::cout << "Error while queueing buffer at output plane" << std::endl;
            abort(m_ContextDevice);
            setStreamEOS(true);
//             return cleanup( );
        }

        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            std::cout << "File read complete." << std::endl;
            abort(m_ContextDevice);
            setStreamEOS(true);
//             return cleanup( );
        }
    

    // Wait till capture plane DQ Thread finishes
    // i.e. all the capture plane buffers are dequeued
    m_ContextDevice->enc->capture_plane.waitForDQThread(-1);


    if (m_ContextDevice->got_error)
    {
        error = 1;
    }

//     delete m_ContextDevice->enc;
//     delete m_ContextDevice->in_file;
//     delete m_ContextDevice->out_file;
//     delete m_ContextDevice->roi_Param_file;
// 
//     free(m_ContextDevice->in_file_path);
//     free(m_ContextDevice->out_file_path);
//     free(m_ContextDevice->ROI_Param_file_path);
//     delete m_ContextDevice->runtime_params_str;

    if (error)
    {
        std::cout << "App run failed" << std::endl;
    }
    else
    {
        std::cout << "App run was successful" << std::endl;
		m_frame_count++;
    }
    
    
    
    DEBUG("encodeFrame : frame_count = %lli \n" , m_frame_count );
    return ret;
	
}
*/

//  ---------------------------------------------------------------------------
int NvEncoderHEVC::encodeFrame(uint8_t *yuv_bytes) {
    int ret = 0;
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *buffer = m_ContextDevice->enc->output_plane.getNthBuffer(m_frame_count);

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

    v4l2_buf.m.planes = planes;

         DEBUG("encodeFrame : ContextDevice = %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
         
    if (m_frame_count < m_ContextDevice->enc->output_plane.getNumBuffers()) {
        DEBUG("encodeFrame : index = %lli  \n" , m_frame_count );
        v4l2_buf.index = m_frame_count;

        if (yuv_bytes)
            copyYuvToBuffer(yuv_bytes, *buffer);
        else
            v4l2_buf.m.planes[0].bytesused = 0;

    } else {
        
        DEBUG("encodeFrame : few frame < %lli  \n" , m_frame_count );
        
        if (m_ContextDevice->enc->output_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 10) < 0) {
            std::cout << "ERROR while DQing buffer at output plane" << std::endl;
            abort(m_ContextDevice);
            setStreamEOS(true);
            return cleanup( );
//             return cleanup(m_ContextDevice, 1);
        }
        else{
            
            DEBUG("encodeFrame : DQing buffer at output plane = %lli  \n" , m_frame_count );
        }
    }

    
         DEBUG("encodeFrame : runtime_params_str = %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
    if (m_ContextDevice->runtime_params_str &&
        (m_ContextDevice->enc->output_plane.getTotalQueuedBuffers() ==
         m_ContextDevice->next_param_change_frame)) {
        set_runtime_params(m_ContextDevice);
        if (m_ContextDevice->runtime_params_str)
            get_next_runtime_param_change_frame(m_ContextDevice);
    }
    else{
        ERROR_RETURN("encodeFrame : runtime_params_str on %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
    }

    if (m_frame_count >= m_ContextDevice->enc->output_plane.getNumBuffers()) {
        if (yuv_bytes)
            copyYuvToBuffer(yuv_bytes, *buffer);
        else
            v4l2_buf.m.planes[0].bytesused = 0;
    }

    
    DEBUG("encodeFrame : output_plane = %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
    ret = m_ContextDevice->enc->output_plane.qBuffer(v4l2_buf, nullptr);
    if (ret < 0) {
        std::cout << "Error while queueing buffer at output plane" << std::endl;
        abort(m_ContextDevice);
        setStreamEOS(true);
            return cleanup( );
//             return cleanup(m_ContextDevice, 1);
    }
    else{
        
         DEBUG("encodeFrame : queueing buffer at output plane for %d x %d \n" , m_ContextDevice->width ,m_ContextDevice->height );
    }


    m_frame_count++;
    
    DEBUG("encodeFrame : frame_count = %lli \n" , m_frame_count );
    return ret;
}




