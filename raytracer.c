#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <stdio.h>
#include <string.h>

const unsigned int width = 800;
const unsigned int height = 600;
unsigned char * ImageArray;

int main(int argc, char* argv[]){
  if(argc<2||argc>3){
    fprintf (stderr, "Invalid number of arguments: Expected 1 got %d\n", argc-1);
    exit (EXIT_FAILURE);
  }

  char* filename;

  if(strcmp( argv[2], "reference" )){
    filename = "reference.png";
  }else if(strcmp( argv[2], "custom")){
    filename = "custom.png";
  }else{
    fprintf (stderr, "Invalid output arument: Expected either \"reference\" or \"custom\"; got %s\n", argv[2]);
    exit (EXIT_FAILURE);
  }

  ImageArray = malloc(width*height*sizeof(unsigned char));

  stbi_write_png(filename, width, height, 3, ImageArray, width*3);

  free(ImageArray);

  exit(EXIT_SUCCESS);
}
