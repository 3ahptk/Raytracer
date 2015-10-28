#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define debug 1
#define trace if (debug) write

char buffer[256];
const unsigned int width = 800;
const unsigned int height = 600;
unsigned char * ImageArray;

int main(int argc, char* argv[]){
  if(argc<2||argc>3){
    sprintf(buffer, "Invalid number of arguments: Expected 1 got %d\n", argc-1);
  	write(2, buffer, strlen(buffer));
    return -1;
  }

  char filename[20];
  if(strcmp( argv[1], "reference" )){
    strcpy (filename,"reference.png");
  }else if(strcmp( argv[1], "custom")){
    strcpy (filename,"custom.png");
  }else{
    sprintf(buffer, "Invalid output arument: Expected either \"reference\" or \"custom\"; got %s\n", argv[2]);
  	write(2, buffer, strlen(buffer));
    return -1;
  }

  ImageArray = malloc(width*height*sizeof(unsigned char));

  stbi_write_png(filename, width, height, 3, ImageArray, width*3);

  free(ImageArray);

  exit(EXIT_SUCCESS);
}
