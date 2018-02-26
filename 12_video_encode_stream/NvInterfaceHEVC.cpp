

#include "NvInterfaceHEVC.h"


#include "NvEncoderHEVC.h"





NvInterfaceHEVC::NvInterfaceHEVC() {
    m_Encoder_HEVC = new NvEncoderHEVC();
	mLoopWriteFrame = nullptr;
}


NvInterfaceHEVC::~NvInterfaceHEVC() {
    
}
  


uint32_t NvInterfaceHEVC::SizeListBufferImage( void ) {
        
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
        return m_Encoder_HEVC->SizeListBufferImage();
}
  
bool NvInterfaceHEVC::slotDisposeHEVC( Image_T * _data ) {
    
    if ( _data != nullptr) {
        
        
		std::lock_guard<std::mutex> lck(m_mux_HEVC);
        m_Encoder_HEVC->pushBufferImage(_data);
//         m_Encoder_HEVC->compressYFrame(_data);
//         free(image.data);
    }
    
    return false;
}
  
  
bool NvInterfaceHEVC::CloseHEVC( void ) {
//     cout << "Cleaning Up" << endl;
    
    Image_T * _data = new Image_T;
    
    _data->imageData = nullptr;
	
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
    m_Encoder_HEVC->pushBufferImage(_data);
    return false;
}
  
 
bool NvInterfaceHEVC::isEOS( void ) {

	
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
	return m_Encoder_HEVC->getStreamEOS();
}

NvEncoderHEVC::DataHEVC * NvInterfaceHEVC::getBufferHEVC( void ) {
    
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
    return  m_Encoder_HEVC->popBufferHEVC();
    
}

void NvInterfaceHEVC::addBufferImage( Image_T * _data) {
	
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
	
    m_Encoder_HEVC->pushBufferImage( _data );
    
}


// EXTRA CODE 
void NvInterfaceHEVC::runEncoderHEVC( void ) {
      
    std::lock_guard<std::mutex> lck(m_mux_HEVC);
      m_Encoder_HEVC->run();
}

// WRITE 
int NvInterfaceHEVC::write_encoder_output_frame( NvEncoderHEVC::DataHEVC * _hrvc)
{
	if( m_out_file->is_open() ) {
		m_out_file->write((char *) _hrvc->buffer , _hrvc->size);
	}
	return 0;
}

bool NvInterfaceHEVC::openFileOutput( char  * _out ) {
    m_out_file = new std::ofstream(_out);
	
	return m_out_file->is_open() ;
}



void NvInterfaceHEVC::writeHEVC( void ) {

        mLoopWriteFrame = new std::thread(&NvInterfaceHEVC::LoopWriteFrame, this);

        mLoopWriteFrame->detach();
    
}


void NvInterfaceHEVC::LoopWriteFrame( void ) {
	
	NvEncoderHEVC::DataHEVC *ldata = nullptr; 
	bool eos = false;
	while ( !eos ) {
		
			ldata = getBufferHEVC();
			if( ldata != nullptr ) {
				write_encoder_output_frame ( ldata ) ;
			}
			else{
				std::this_thread::sleep_for(std::chrono::milliseconds(25)) ;
			}
			
			eos = isEOS();
	}

}


/*

void NvEncoderHEVC::run( void ) {

        mLoopCompressHEVC = new std::thread(&NvEncoderHEVC::LoopCompressFrame, this);

        mLoopCompressHEVC->detach();
    
}

void NvEncoderHEVC::record( void ) {
    
    bool leos = false;
    while  ( !leos ) {
        
        Image_T * ldata = getBufferHEVC();
        
        if( ldata != nullptr ) {
            if( ldata->imageData != nullptr ) {
                
                printf("LoopCompressFrame image : %d x %d \n" , ldata->width ,ldata->height );
                compressYFrame( ldata );
            }
            else{
                compressYFrame( nullptr );
            }
        }
        else{
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500)) ;
        }
        
        leos = getStreamEOS();
    }
    
    
}
*/
// FILE    m_ContextDevice->out_file = new ofstream(m_ContextDevice->out_file_path);
