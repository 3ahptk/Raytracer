#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define debug 1
#define trace if (debug) write

char buffer[256];
const unsigned int width = 512;
const unsigned int height = 512;
unsigned char ImageArray[height][width];

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

  int x;
  int y;
  for(x=0; x<height; x++){
    for(y=0; y<width; y++) {
      ImageArray[x][y]=100;
    }
  }

  sprintf(buffer, "stbi_write_png(%s, %d, %d, %d, ImageArray, %lu)\n",filename, width, height, 3, sizeof ImageArray[0]);
  trace(1, buffer, strlen(buffer));

  stbi_write_png(filename, width, height, 3, ImageArray, sizeof ImageArray[0]);

  return 1;
}
