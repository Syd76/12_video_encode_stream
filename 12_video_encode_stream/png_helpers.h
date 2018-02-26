#ifndef PNG_HELPERS_H
#define PNG_HELPERS_H

#include <vector>
#include <string>

#include "type_encode.h"


using namespace::std;


// typedef struct {
// 	unsigned height;
// 	unsigned width;
// 	unsigned pixel_bytes;
// 	char* data;
// 
// } Image_T;

enum PngSaveMode { YUV420, RAW };

void read_png_file(char* file_name, Image_T &image);

bool hasEnding(std::string const &fullString, std::string const &ending);

int get_images(string dir, vector<Image_T> &images, unsigned max_image_count);

int free_images(vector<Image_T> &images);

int save_images_to_file(string inputDir, string outputFname);

#endif
