#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vecmat.h"
#include "msg.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define WIDTH 512
#define HEIGHT 512
#define ARRAYSIZE WIDTH*HEIGHT*3

#define debug 1
#define trace if (debug) write

// Set up the header for all of the methods.
void getRay(Perspective p, float screenCoord[2], Ray *ray);

char buffer[256];
unsigned char ImageArray[ARRAYSIZE];
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
	int numReflections; //10
} Ray;

typedef struct {
	const	float cameraPos[3]; //{0.0,0.0,0.0}
	const float distanceToScreen; //2
	const unsigned int widthWorld; //2
	const unsigned int widthPixels; //512
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

typedef struct {
  float pos[3];
  int radius;
  int mat;
} Sphere;

typedef struct {
	float pos[3];
} Triangle;

void getRay(Perspective p, float screenCoord[2], Ray *ray) {
	ray.vector = normalize((screenCoord, p.distanceToScreen) - p.cameraPos);
	ray.position = (screenCoord, p.distanceToScreen);
}

int RaySphereIntersect(Ray *ray, Sphere *sph){
  float e[3] = ray.vector;
  float d[3] = ray.position;
  float c[3] = sph.pos;
  float r = sph.radius;
  float eminc = 0;
  vec3f_sub_new(eminc,e,c);
  float discriminant = (vec3f_dot(e,c)*vec3f_dot(e,c))-vec3f_dot(d,d) * vec3f_dot(eminc,eminc)-(r*r);
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

int RayTriangleIntersect(Ray *ray, Triangle *tri) {

}

static inline void progress(int x, int n)
{
		if ( x % (n/100 +1) != 0 ) return;
    float r = x/(float)n;
    int c = r * 80;
    printf("[%3d%%] ", (int)(r*100) );

    for (int x=0; x<c; x++){
       printf("░");
		}
    for (int x=c; x<80; x++){
       printf(" ");
		}
    printf("\n\033[F\033[J");
}

int main(int argc, char* argv[]){
  if(argc!=2){
    sprintf(buffer, "Invalid number of arguments: Expected 1, got %d\n", argc-1);
  	write(2, buffer, strlen(buffer));
    return -1;
  }
  char * filename;

	time_t t;
	srand((unsigned) time(&t));//seed the random

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

	for (x=0; x<HEIGHT; x++) {
		for (y=0; y<WIDTH; y++) {
			//getRay(perspective, float screenCoord[2], Ray * ray);
			// Calculate and set the color of the pixel.
			int pos = (x * WIDTH + y) * 3;
      ImageArray[pos]=rand() % 255;//blue channel
			ImageArray[pos+1]=rand() % 255;//green channel
			ImageArray[pos+2]=rand() % 255;//red channel
			progress(pos, ARRAYSIZE);
			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+1, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+2, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
		}
	}
	//finalize the progress bar
	printf("[%3d%%] ", 100);
	for (int x=0; x<80; x++){
		 printf("█");
	}
	printf("\n");

  stbi_write_png(filename, WIDTH, HEIGHT, 3, ImageArray, WIDTH*3);

  return 1;
}
