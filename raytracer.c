#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vecmat.h"
#include "msg.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define debug 1
#define trace if (debug) write

char buffer[256];
const unsigned int width = 512;
const unsigned int height = 512;
unsigned char ImageArray[height][width];

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
    sprintf(buffer, "Invalid number of arguments: Expected 1 got %d\n", argc-1);
  	write(2, buffer, strlen(buffer));
    return -1;
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

  int x;
  int y;
  for(x=0; x<height; x++){
    for(y=0; y<width; y++) {
      ImageArray[x][y]=100;
    }
  }

	for (int x=0; x<512; x++) {
		for (int y=0; y<512; y++) {
			getRay();
			// Calculate and set the color of the pixel.
		}
	}
  sprintf(buffer, "stbi_write_png(%s, %d, %d, %d, ImageArray, %lu)\n",filename, width, height, 3, sizeof ImageArray[0]);
  trace(1, buffer, strlen(buffer));

  stbi_write_png(filename, width, height, 3, ImageArray, sizeof ImageArray[0]);

  return 1;
}

void getRay(Perspective p, vec2 screenCoord, Ray *ray) { 
	
}
