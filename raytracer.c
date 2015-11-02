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

char buffer[256];
unsigned char ImageArray[ARRAYSIZE];
int colorred[3] = {255,0,0};
int colorgreen[3] = {0,255,0};
int colorblue[3] = {0,0,255};
int colorblack[3] = {0,0,0};
int colorwhite[3] = {255,255,255};

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
	float t;
	Ray ray;
	int objectCode;
	struct Sphere * sph;
	struct Triangle * tri;
} RayHit;

typedef struct {
  float pos[3];
  float radius;
  int mat;
} Sphere;

typedef struct {
	float pos[3];
} Triangle;

void getRay(Perspective * p, float screenCoord[2], Ray * ray) {
	float posPixel[3] = {screenCoord[0], screenCoord[1], p->distanceToScreen};
	float srcVec[3];
	vec3f_sub_new(srcVec, posPixel, p->cameraPos);
	memcpy(ray->vector, srcVec, sizeof(float[3]));
	vec3f_normalize(ray->vector);
	memcpy(ray->position, posPixel, sizeof(float[3]));
}

/*
int RayTriangleIntersect(Ray *ray, Triangle *tri) {

}
*/

int hit = 0;
int miss = 0;

float RaySphereIntersect(Ray * ray, Sphere * sph){
// 	// The line passes through p1 and p2:
// float[3] p1 = ray->position;
// float[3] p2 = ray->vector;
//
// // Sphere center is p3, radius is r:
// float[3] p3 = sph->pos;
// float r = sph->radius;
//
// float x1 = p1[0]; float y1 = p1[1]; float z1 = p1[2];
// float x2 = p2[0]; float y2 = p2[1]; float z2 = p2[2];
// float x3 = p3[0]; float y3 = p3[1]; float z3 = p3[2];
//
// float dx = x2 - x1;
// float dy = y2 - y1;
// float dz = z2 - z1;
//
// float a = dx*dx + dy*dy + dz*dz;
// float b = 2.0 * (dx * (x1 - x3) + dy * (y1 - y3) + dz * (z1 - z3));
// float c = x3*x3 + y3*y3 + z3*z3 + x1*x1 + y1*y1 + z1*z1 - 2.0 * (x3*x1 + y3*y1 + z3*z1) - r*r;
//
// float test = b*b - 4.0*a*c;
// float[3] hitp = {0,0,0};
//
// 	if (test >= 0.0) {
// 	  // Hit (according to Treebeard, "a fine hit").
// 	  float u = (-b - sqrt(test)) / (2.0 * a);
//
// 		float eminc[3] = {0,0,0};
// 		vec3f_sub_new(eminc,e,c);
//
// 	  float[3] hitp = p1 + u * (p2 - p1);
// 	  // Now use hitp.
// 	}
// }
  float e[3];
  float d[3];
  float c[3];

	memcpy(e,ray->vector,sizeof(float[3]));
	memcpy(d,ray->position,sizeof(float[3]));
	memcpy(c,sph->pos,sizeof(float[3]));

	// printf("e = {%f,%f,%f}\n",e[0],e[1],e[2]);
	// printf("d = {%f,%f,%f}\n",d[0],d[1],d[2]);
  // printf("c = {%f,%f,%f}\n",c[0],c[1],c[2]);

  float r = 1.0;
  float eminc[3] = {0,0,0};
  vec3f_sub_new(eminc,e,c);
	// printf("eminc= {%f,%f,%f}\n",eminc[0],eminc[1],eminc[2]);
	// printf("vec3f_dot(d,eminc)=%f,vec3f_dot(d,d)=%f,vec3f_dot(&eminc,&eminc)=%f,(r*r)=%f\n",vec3f_dot(d,eminc),vec3f_dot(d,d),vec3f_dot(&eminc,&eminc),(r*r));

	//((d⋅(e−c))^2−(d⋅d)((e−c)⋅(e−c)−R^2))
	float discriminant = (vec3f_dot(d,eminc)*vec3f_dot(d,eminc)) - vec3f_dot(d,d) *((vec3f_dot(eminc,eminc)-(r*r)));
	// printf("discriminant:%f\n",discriminant);

  float t = 0;
  float tpos = 0;
  float tneg = 0;

  if(discriminant>0){//which one do I take?
    t = (-vec3f_dot(e,c) + sqrt( discriminant ) )/ vec3f_dot(d,d);
		hit++;
  }else if(discriminant==0){
    tpos = (-vec3f_dot(e,c) + sqrt( discriminant ) )/ vec3f_dot(d,d);
    tneg = (-vec3f_dot(e,c) - sqrt( discriminant ) )/ vec3f_dot(d,d);

    if(tpos<tneg){
      t = tpos;
			hit++;
    }else{
      t = tneg;
    }
  }else{
    t = 0;
		miss++;
  }
  return t;
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

	Sphere testSphere = {
		{0,0,-16},2,1
	};

	Ray ray = {
		{0,0,0}, {0,0,2}, 10.0
	};

	Perspective p = {
		{0.0,0.0,0.0}, 2, 2, 512
	};

	float r = 0;
  unsigned int x;
  unsigned int y;
	float screenCoord[2];

	for (x=0; x<HEIGHT; x++) {
		for (y=0; y<WIDTH; y++) {
			screenCoord[0] = x;
			screenCoord[1] = y;
			// printf("BEFORE[%d,%d] Ray = {%f,%f,%f},{%f,%f,%f},%f\n",x,y,ray.vector[0], ray.vector[1], ray.vector[2],ray.position[0], ray.position[1], ray.position[2],ray.numReflections);

			getRay(&p, screenCoord, &ray);

			// printf("AFTER[%d,%d] Ray = {%f,%f,%f},{%f,%f,%f},%f\n",x,y,ray.vector[0], ray.vector[1], ray.vector[2],ray.position[0], ray.position[1], ray.position[2],ray.numReflections);

			//{0,0,0}, {0,0,2}, 10
			// printf("testSphere = {%f,%f,%f},%f,%f\n",testSphere.pos[0],testSphere.pos[1],testSphere.pos[2],testSphere.radius,testSphere.mat);
			// Calculate and set the color of the pixel.

			int pos = (x * WIDTH + y) * 3;
			r = RaySphereIntersect(&ray, &testSphere);

			if (r == 0) {
				ImageArray[pos] = 0;//blue channel
				ImageArray[pos+1] = 0;//green channel
				ImageArray[pos+2] = 0;//red channel
			} else {
				ImageArray[pos] = 255;//blue channel
				ImageArray[pos+1] = 255;//green channel
				ImageArray[pos+2] = 255;//red channel
			}

			// progress(pos, ARRAYSIZE);

			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+1, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
			// sprintf(buffer, "OUT(X=%d, Y=%d, POS=%d, SIZE=%d)\n", x, y, pos+2, ARRAYSIZE);
			// trace(1, buffer, strlen(buffer));
		}
	}
	//finalize the progress bar
	// printf("[%3d%%] ", 100);
	// for (int x=0; x<80; x++){
	// 	 printf("█");
	// }
	// printf("\n");

	printf("hit:%d,miss:%d\n",hit,miss);

  stbi_write_png(filename, WIDTH, HEIGHT, 3, ImageArray, WIDTH*3);

  return 1;
}
