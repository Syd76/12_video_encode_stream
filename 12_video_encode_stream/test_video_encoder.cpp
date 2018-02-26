//
// Created by gwall on 1/10/18.
//
#include "png_helpers.h"

#include "type_encode.h"
#include "NvInterfaceHEVC.h"
#include <string>
#include <vector>

#define PNG_FOLDER "/tmp/projets/png/"
#define OUTPUT "test.h264"

double getUnixTime(void) {
    struct timespec tv;

    if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

int main(int argc, char *argv[]) {
    double t0 = getUnixTime();
    double t1, t2;

    NvInterfaceHEVC *linterface = new NvInterfaceHEVC();
    
    Image_T *tmp ;
    
    
    
    vector<Image_T> images = vector<Image_T>();
    printf("Reading Images \n");
    
    get_images(std::string(PNG_FOLDER), images, 25);

//     VideoEncoder encoder(images[0].height, images[0].width, OUTPUT);
    printf("Starting Compression \n");
    
    t1 = getUnixTime();
    for (auto &image : images) {
        
        linterface->slotDisposeHEVC( &image );
        
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            printf("read Images done \n");
//         encoder.compressYFrame((uint8_t *) image.data);
//         free(image.data);
    }
    

    
    
    linterface->runEncoderHEVC();
    linterface->openFileOutput("/tmp/test.h264");
	linterface->writeHEVC();
    
    
	
    
        bool lret = true ;
    while ( lret ) {
        uint32_t lsize = linterface->SizeListBufferImage();
        printf("wait %d Images done \n", lsize );
        lret = 0 <  lsize ;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    printf("Cleaning Up \n");
    linterface->CloseHEVC(   );
        
//     encoder.compressYFrame(nullptr);

    t2 = getUnixTime();

    double diff = ( t2 - t1 )  ;
    printf( "Encoder Time: %lf Total Time: %lf , Per Frame Time : %lf \n" , diff , (t2 - t0) ,  diff / images.size() );


    return 0;
}

