




#ifndef _NV_INTERFACE_HEVC_H_
#define _NV_INTERFACE_HEVC_H_


#include "type_encode.h"

#define CALLBACK_BUFFER_H264(buff)    (char *)(buff->planes[0].data, buffer->planes[0].bytesused)



#include "NvEncoderHEVC.h"

#include <fstream>

class NvInterfaceHEVC {
    
public :
    NvInterfaceHEVC() ;
    ~NvInterfaceHEVC();
    NvEncoderHEVC::DataHEVC * getBufferHEVC( void );
    void addBufferImage( Image_T * _data);
    uint32_t SizeListBufferImage( void ) ;
    // CRC TOOLS
//     static void abort(Context_T *ctx);

	bool isEOS( void ) ;
    bool CloseHEVC( void ) ;
    void runEncoderHEVC( void ) ;
	bool slotDisposeHEVC( Image_T * _data ) ;

	
	void writeHEVC( void );
	bool openFileOutput( char  * _out );
	
protected :
    
	std::mutex m_mux_HEVC;
    NvEncoderHEVC *m_Encoder_HEVC;
    
	std::thread *mLoopWriteFrame;
//     char *out_file_path;
    std::ofstream *m_out_file;

	void LoopWriteFrame( void );
	int write_encoder_output_frame( NvEncoderHEVC::DataHEVC * _hrvc);
    
    
};

#endif 

