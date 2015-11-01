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
unsigned char ImageArray[height*width*3];
int colorred[3] = {255,0,0};
int colorgreen[3] = {0,255,0};
int colorblue[3] = {0,0,255};

enum material {
    reflective = 0,
		red = 1,
    green = 2,
    blue = 3
};

typedef struct {
	float vector[3];
	float position[3];
	int numReflections;//=10;
} Ray;

typedef struct {
	const	float cameraPos[3];// = {0.0,0.0,0.0};
	const float distanceToScreen;// = 2;
	const unsigned int widthWorld;// = 2;
	const unsigned int widthPixels;// = 512;
} Perspective;

typedef struct {
  float pos[3];
  float radius;
  int mat;
} Sphere;

typedef struct {
	float t;
	Ray ray;
	int objectCode;
	struct Sphere * sph;
	struct Triangle * tri;
} RayHit;

Perspective * perspective = NULL;

// void getRay(Perspective p, float screenCoord[2], Ray * ray) {
// 	ray->vector = normalize((screenCoord, p.distanceToScreen) - &p.cameraPos);
// 	ray->position = (screenCoord, p.distanceToScreen);
// }

int RaySphereIntersect(Ray * ray, Sphere * sph){
  float e[3];
  float d[3];
  float c[3];

	memcpy(e,ray->vector,sizeof(float[3]));
	memcpy(d,ray->position,sizeof(float[3]));
	memcpy(c,sph->pos,sizeof(float[3]));

  float r = sph->radius;
  float eminc = 0;
  vec3f_sub_new(&eminc,e,c);
  float discriminant = (vec3f_dot(e,c)*vec3f_dot(e,c))-vec3f_dot(d,d) * vec3f_dot(&eminc,&eminc)-(r*r);
  float t = 0;
  float tpos = 0;
  float tneg = 0;

  if(discriminant>0){//which one do I take?
    t = (-vec3f_dot(e,c))/ vec3f_dot(d,d);
  }else if(discriminant==0){
    tpos = (-vec3f_dot(e,c) + sqrt( discriminant ) )/ vec3f_dot(d,d);
    tneg = (-vec3f_dot(e,c) - sqrt( discriminant ) )/ vec3f_dot(d,d);

    if(tpos<tneg){
      t = tpos;
    }else{
      t = tneg;
    }
  }else{
    t = 0;
  }
  return t;
}


int main(int argc, char* argv[]){
  if(argc!=2){
    sprintf(buffer, "Invalid number of arguments: Expected 1, got %d\n", argc-1);
  	write(2, buffer, strlen(buffer));
    return -1;
  }
  char * filename;



  if(strcmp( argv[1], "reference" ) == 0) {
		filename = "reference.png";
  } else if(strcmp( argv[1], "custom") == 0) {
    filename = "custom.png";
  } else {
    fprintf (stderr, "Invalid output arument: Expected either \"reference\" or \"custom\"; got %s\n", argv[2]);
    return -1;
  }

	//  Sphere testSphere = {
	// 		{1,0,1},1,1
	//  };

  unsigned int x;
  unsigned int y;

	for (x=0; x<height; x++) {
		for (y=0; y<width; y++) {
			//getRay(perspective, float screenCoord[2], Ray * ray);
			// Calculate and set the color of the pixel.
			int pos = (x * width + y) * 3;
      ImageArray[pos]=255;//blue
			ImageArray[pos+1]=255;//green
			ImageArray[pos+2]=255;//red

			sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos, height*width*3);
			trace(1, buffer, strlen(buffer));
			sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+1, height*width*3);
			trace(1, buffer, strlen(buffer));
			sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+2, height*width*3);
			trace(1, buffer, strlen(buffer));
		}
	}
  sprintf(buffer, "stbi_write_png(%s, %d, %d, %d, ImageArray, %u)\n",filename, width, height, 3, width*3);
  trace(1, buffer, strlen(buffer));

  stbi_write_png(filename, width, height, 3, ImageArray, width*3);

  return 1;
}
