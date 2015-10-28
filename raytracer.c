#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vecmat.h"
#include "msg.h"
#include <stdio.h>
#include <string.h>

const unsigned int width = 512;
const unsigned int height = 512;
unsigned char * ImageArray;

typedef struct {
	float vector[3];
	float position[3];
	int numReflections = 10;
} Ray;

typedef struct {
	float cameraPos[3];
	float distanceToScreen;
	const unsigned int widthWorld = 512;
	const unsigned int widthPixels = 512;
} Perspective;

int main(int argc, char* argv[]){
  if(argc<2||argc>3){
    fprintf (stderr, "Invalid number of arguments: Expected 1 got %d\n", argc-1);
    exit (EXIT_FAILURE);
  }

  char* filename;

  if(strcmp( argv[2], "reference" )) {
    filename = "reference.png";
  } else if(strcmp( argv[2], "custom")) {
    filename = "custom.png";
  } else {
    fprintf (stderr, "Invalid output arument: Expected either \"reference\" or \"custom\"; got %s\n", argv[2]);
    exit (EXIT_FAILURE);
  }

  ImageArray = malloc(width*height*sizeof(unsigned char));

	for (int x=0; x<512; x++) {
		for (int y=0; y<512; y++) {
			getRay();
			// Calculate and set the color of the pixel.
		}
	}

  stbi_write_png(filename, width, height, 3, ImageArray, width*3);

  free(ImageArray);

  exit(EXIT_SUCCESS);
}

void getRay(Perspective p, vec2 screenCoord, Ray *ray) { 
	
}
