


#include <png.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <malloc.h>
#include <string>
#include <dirent.h>
#include <errno.h>
#include <iostream>
#include <algorithm>
#include "png_helpers.h"



#define PNG_DEBUG 3

using namespace ::std;

void png_abort(const char * s, ...) {
	va_list args;
	va_start(args, s);
	vfprintf(stderr, s, args);
	fprintf(stderr, "\n");
	va_end(args);
	abort();
}

void read_png_file(char* file_name, Image_T &image) {

	unsigned x, y;

	unsigned width, height;
	png_byte color_type;
	png_byte bit_depth;

	png_structp png_ptr;
	png_infop info_ptr;
	int number_of_passes;
	png_bytep * row_pointers;

	unsigned char header[8];    // 8 is the maximum size that can be checked

	/* open file and test for it being a png */
	FILE *fp = fopen(file_name, "rb");
	if (!fp)
		png_abort("[read_png_file] File %s could not be opened for reading",
				file_name);
	fread(header, 1, 8, fp);
	if (png_sig_cmp(header, 0, 8))
		png_abort("[read_png_file] File %s is not recognized as a PNG file",
				file_name);

	/* initialize stuff */
	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!png_ptr)
		png_abort("[read_png_file] png_create_read_struct failed");

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		png_abort("[read_png_file] png_create_info_struct failed");

	if (setjmp(png_jmpbuf(png_ptr)))
		png_abort("[read_png_file] Error during init_io");

	png_init_io(png_ptr, fp);
	png_set_sig_bytes(png_ptr, 8);

	png_read_info(png_ptr, info_ptr);

	width = png_get_image_width(png_ptr, info_ptr);
	height = png_get_image_height(png_ptr, info_ptr);
	color_type = png_get_color_type(png_ptr, info_ptr);
	bit_depth = png_get_bit_depth(png_ptr, info_ptr);



	int color_type_i = color_type == PNG_COLOR_TYPE_GRAY;
	int bit_depth_i = bit_depth;

	cout << width << " " << height << " " << color_type_i << " " << bit_depth_i
			<< endl;

	number_of_passes = png_set_interlace_handling(png_ptr);
	png_read_update_info(png_ptr, info_ptr);

	/* read file */
	if (setjmp(png_jmpbuf(png_ptr)))
		png_abort("[read_png_file] Error during read_image");

	row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
	for (y = 0; y < height; y++)
		row_pointers[y] = (png_byte*) malloc(
				png_get_rowbytes(png_ptr, info_ptr));

	png_read_image(png_ptr, row_pointers);

	image.width = width;
	image.height = height;
	image.pixel_bytes = 1;
	image.imageData = (uint8_t *)malloc(sizeof(uint8_t) * height * width);

	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			image.imageData[x + y*width] = row_pointers[y][2*x];
		}
        free(row_pointers[y]);
	}
    free(row_pointers);

	png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);

	fclose(fp);
}

bool hasEnding(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0
				== fullString.compare(fullString.length() - ending.length(),
						ending.length(), ending));
	} else {
		return false;
	}
}

/*function... might want it in some class?*/
int get_images(std::string dir, vector<Image_T> &images, unsigned max_image_count) {

	if (max_image_count < 1)
        return 0;

	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << dir << endl;
		return errno;
	}

	vector<string> files = vector<string>();

	while ((dirp = readdir(dp)) != NULL) {
		if (hasEnding(string(dirp->d_name), string(".png"))) {
			files.push_back(string(dirp->d_name));
			//cout << dirp->d_name << endl;
		}
	}
	closedir(dp);

	sort(files.begin(), files.end());
	for (vector<string>::iterator i = files.begin(); i != files.end(); ++i) {
		//std::cout << *i << std::endl;
		string filename = *i;
		string fullpath = dir + filename;
		char *cstr = (char*) fullpath.c_str();
		Image_T image;
		read_png_file(cstr, image);
		images.push_back(image);

        if (images.size() == max_image_count) {
            break;
        }
	}

	return 0;
}

int free_images(vector<Image_T> &images) {

    for (vector<Image_T>::iterator img = images.begin(); img != images.end(); ++img) {
       free(img->imageData);
    }

    return 0;
}

int save_images_to_file(string inputDir, string outputFname) {
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(inputDir.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << inputDir << endl;
		return errno;
	}

	vector<string> files = vector<string>();

	while ((dirp = readdir(dp)) != NULL) {
		if (hasEnding(string(dirp->d_name), string(".png"))) {
			files.push_back(string(dirp->d_name));
			//cout << dirp->d_name << endl;
		}
	}
	closedir(dp);


	//ofstream myFile (outputFname, ios::out | ios::binary);


	sort(files.begin(), files.end());

	std::ofstream outfile (outputFname.c_str(), std::ofstream::binary);
    //std::ofstream outfile ("new.txt", std::ofstream::binary);
	char emptyUV = 0x80;

	for (vector<string>::iterator i = files.begin(); i != files.end(); ++i) {
		//std::cout << *i << std::endl;
		string filename = *i;
		string fullpath = inputDir + filename;
		char *cstr = (char*) fullpath.c_str();
		Image_T image;
		read_png_file(cstr, image);
		//write Y
		outfile.write( (char *)image.imageData, image.height*image.width*image.pixel_bytes);
		//write U,V each at quarter of the full image resolution
		int uvRez = (image.height*image.width)/2;
		for (int i = 0; i < uvRez; i++) {
			outfile.write(&emptyUV, 1);
		}
	}

	outfile.close();

	return 0;
}


