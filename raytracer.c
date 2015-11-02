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
	const	float cameraPos[3];
	const float distanceToScreen;
	const unsigned int widthWorld;
	const unsigned int widthPixels; 
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
  int radius;
  int mat;
} Sphere;

typedef struct {
	float posa[3];
	float posb[3];
	float posc[3];
	int mat;
} Triangle;

void getRay(Perspective * p, float screenCoord[2], Ray * ray) { 
	float posPixel[3] = {screenCoord[0], screenCoord[1], p->distanceToScreen};
	float srcVec[3];
	vec3f_sub_new(srcVec, posPixel, p->cameraPos);
	memcpy(ray->vector, srcVec, sizeof(float[3]));
	//printf("Ray Vector Before normalization: %f, %f, %f \n", ray->vector[0], ray->vector[1], ray->vector[2]);
	vec3f_normalize(ray->vector);
	//printf("Ray Vector After normalization: %f, %f, %f\n", ray->vector[0], ray->vector[1], ray->vector[2]);
	memcpy(ray->position, posPixel, sizeof(float[3]));
	//printf("Ray Position: %f, %f, %f\n\n", ray->position[0], ray->position[1], ray->position[2]);
}

int RayTriangleIntersect(Ray * ray, Triangle * tri) {
	float verta[3], vertb[3], vertc[3];
	float vecray[3], posray[3];
	float xa, xb, xc, xd, xe, ya, yb, yc, yd, ye, za, zb, zc, zd, ze;	
	float m, beta, gamma, t;
	float a, b, c, d, e, f, g, h, i, j, k, l;
	float closestHit = 100;
	
	// Copy the three positions from the triangle class into the x, y, z arrays.
	memcpy(verta, tri->posa, sizeof(float[3]));
	memcpy(vertb, tri->posb, sizeof(float[3]));
	memcpy(vertc, tri->posc, sizeof(float[3]));
	memcpy(vecray, ray->vector, sizeof(float[3]));
	memcpy(posray, ray->position, sizeof(float[3]));
	//printf("Vertex A: %f, %f, %f. Vertex B: %f, %f, %f. Vertex C: %f, %f, %f. \n Ray Vector: %f, %f, %f. Ray Position: %f, %f, %f \n",
	//	verta[0], verta[1], verta[2], vertb[0], vertb[1], vertb[2], vertc[0], vertc[1], vertc[2], vecray[0], vecray[1], vecray[2], 
	//	posray[0], posray[1], posray[2]);  
	
	// Set up the a,b, and c components of the 3 vertices.
	xa = verta[0];
	xb = vertb[0];
	xc = vertc[0];
	xd = vecray[0];
	xe = posray[0];
	ya = verta[1];
	yb = vertb[1];
	yc = vertc[1];
	yd = vecray[1];
	ye = posray[1];
	za = verta[2];
	zb = vertb[2];
	zc = vertc[2];
	zd = vecray[2];
	ze = posray[2];

	// Set up the components.
	a = xa - xb;
	b = ya - yb;
	c = za - zb;
	d = xa - xc;
	e = ya - yc;
	f = za - zc;
	g = xd;
	h = yd;
	i = zd;
	j = xa - xe;
	k = ya - ye;
	l = za - ze;

	// Calculate M, Beta, Gamma, and t.
	m = (a * ((e * i) - (h * f))) + (b * ((g * f) - (d * i))) + (c * ((d * h) - (e * g)));
	t = (-(((f * ((a * k) - (j * b))) + (e * ((j * c) - (a * l))) + (d * ((b * l) - (k * c))))) / m);

	if (t > 0) 
		//printf("Result of t: %f\n", t);
	
	// Return the result of the intersection.
	if (t < 0) 
		return 0;
	gamma = (((i * ((a * k) - (j * b))) + (h * ((j * c) - (a * l))) + (g * ((b * l) - (k * c)))) / m);
	if ((gamma < 0) || (gamma > 1)) 
		return 0; 
	beta = ((j * ((e * i) - (h * f)) + (k * ((g * f) - (d * i))) + (l * ((d * h) - (e * g)))) / m);
	if ((beta < 0) || (beta > (1-gamma)))
		return 0;
	printf("Reached");
	return t;
}

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
		{0,0,3},1,1
	};

	Ray ray = {
		{0,0,0},{0,0,2}, 10
	};
	
	Perspective p = {
		{0.0,0.0,0.0}, -2, 2, 512
	};	

	Triangle back1 = {
		{-8,-2,-20},{8,-2,-20},{8,10,-20}, 1
	};	

  unsigned int x;
  unsigned int y;
	float screenCoord[2];

	for (x=0; x<HEIGHT; x++) {
		for (y=0; y<WIDTH; y++) {
			screenCoord[0] = x;
			screenCoord[1] = y;
			getRay(&p, screenCoord, &ray);			

			// Calculate and set the color of the pixel.

			int pos = (x * WIDTH + y) * 3;
			//int r = RaySphereIntersect(&ray, &testSphere);
			int r = RayTriangleIntersect(&ray, &back1);
			if (r == 0) {
				ImageArray[pos] = 0;//blue channel
				ImageArray[pos+1] = 0;//green channel
				ImageArray[pos+2] = 0;//red channel
			} else {
				ImageArray[pos] = 255;//blue channel
				ImageArray[pos+1] = 255;//green channel
				ImageArray[pos+2] = 255;//red channel
			}
      
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
