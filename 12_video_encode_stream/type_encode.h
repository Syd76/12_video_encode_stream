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



#ifndef _TYPE_ENCODE_H_
#define _TYPE_ENCODE_H_


// #include "video_encode.h"



#include <NvVideoEncoder.h>

#include <fstream>
#include <sstream>
#include <cstdint>

#include "Crc_Encode.h"

/*          
typedef struct {
	unsigned height;
	unsigned width;
	unsigned pixel_bytes;
	char* data;
*/

typedef struct _image_t {
    
    
        uint8_t * imageData;       /* Pointer to aligned image data.         */  

        uint32_t  nChannels;         /* Most of OpenCV functions support 1,2,3 or 4 channels */
        uint32_t  depth;             /* Pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S, IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported.  */
        uint32_t  width;             /* Image width in pixels.                           */
        uint32_t  height;            /* Image height in pixels.                          */
        uint32_t pixel_bytes;
        uint32_t  widthStep;
        uint32_t  imageSize;         /* Image data size in bytes (==image->height*image->widthStep in case of interleaved data)*/                          

} Image_T;






typedef struct
{
    NvVideoEncoder *enc;
    uint32_t encoder_pixfmt;

    char *in_file_path;
    std::ifstream *in_file;
	uint32_t cam_pixfmt;
    uint32_t width;
    uint32_t height;

    uint32_t buffer_length;
    uint16_t enable_verbose;
    
    char *out_file_path;
    std::ofstream *out_file;

    char *ROI_Param_file_path;
    char *Recon_Ref_file_path;
    char *RPS_Param_file_path;
    char *hints_Param_file_path;
    char *GDR_Param_file_path;
    char *GDR_out_file_path;
    std::ifstream *roi_Param_file;
    std::ifstream *recon_Ref_file;
    std::ifstream *rps_Param_file;
    std::ifstream *hints_Param_file;
    std::ifstream *gdr_Param_file;
    std::ofstream *gdr_out_file;

    uint32_t bitrate;
    uint32_t profile;
    enum v4l2_mpeg_video_bitrate_mode ratecontrol;
    uint32_t iframe_interval;
    uint32_t idr_interval;
    enum v4l2_mpeg_video_h264_level level;
    uint32_t fps_n;
    uint32_t fps_d;
    uint32_t gdr_start_frame_number; /* Frame number where GDR has to be started */
    uint32_t gdr_num_frames; /* Number of frames where GDR to be applied */
    uint32_t gdr_out_frame_number; /* Frames number from where encoded buffers are to be dumped */

    uint32_t slice_length;
    uint32_t virtual_buffer_size;
    uint32_t num_reference_frames;
    uint32_t slice_intrarefresh_interval;
    uint32_t num_b_frames;
    uint32_t nMinQpI;              /* Minimum QP value to use for index frames */
    uint32_t nMaxQpI;              /* Maximum QP value to use for index frames */
    uint32_t nMinQpP;              /* Minimum QP value to use for P frames */
    uint32_t nMaxQpP;              /* Maximum QP value to use for P frames */
    uint32_t nMinQpB;              /* Minimum QP value to use for B frames */
    uint32_t nMaxQpB;              /* Maximum QP value to use for B frames */
    uint32_t sMaxQp;               /* Session Maximum QP value */
    bool insert_sps_pps_at_idr;
    bool insert_vui;
    bool insert_aud;

    bool report_metadata;
    bool input_metadata;
    bool dump_mv;
    bool externalRPS;
    bool enableGDR;
    bool bGapsInFrameNumAllowed;
    bool bnoIframe;
    uint32_t nH264FrameNumBits;
    uint32_t nH265PocLsbBits;
    bool externalRCHints;
    bool enableROI;

    bool use_gold_crc;
    char gold_crc[20];
    Crc *pBitStreamCrc;

    bool bReconCrc;
    uint32_t rl;                   /* Reconstructed surface Left cordinate */
    uint32_t rt;                   /* Reconstructed surface Top cordinate */
    uint32_t rw;                   /* Reconstructed surface width */
    uint32_t rh;                   /* Reconstructed surface height */

    std::stringstream *runtime_params_str;
    uint32_t next_param_change_frame;
    bool got_error;
	
// TX2	
    enum v4l2_enc_temporal_tradeoff_level_type temporal_tradeoff_level;
//     enum v4l2_enc_hw_preset_type hw_preset_type;
    v4l2_enc_slice_length_type slice_length_type;
	
} Context_T;

// int parse_csv_args(Context_T * ctx, int argc, char *argv[]);


#endif // _TYPE_ENCODE_H_
