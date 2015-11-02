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

#define RED   {255,0,0}
#define GREEN {0,255,0}
#define BLUE  {0,0,255}
#define BLACK {0,0,0}
#define WHITE {255,255,255}

#define debug 1
#define trace if (debug) write

char buffer[256];
unsigned char ImageArray[ARRAYSIZE];
float pixellength = 2.0/512.0;

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
	float hit;
  float color[3];
} RayHit;

typedef struct {
  float pos[3];
  float radius;
  float color[3];
  int reflect;
} Sphere;

typedef struct {
	float posa[3];
	float posb[3];
	float posc[3];
	float color[3];
	int reflect;
} Triangle;

void getRay(Perspective * p, float screenCoord[2], Ray * ray) {
  float camPos[3] = {0,0,0};
  float posPixel[3] = {-1 + screenCoord[0]*pixellength, 1 - screenCoord[1]*pixellength, -2};
	float srcVec[3];
	vec3f_sub_new(srcVec, posPixel, camPos);
	memcpy(ray->vector, srcVec, sizeof(float[3]));
	vec3f_normalize(ray->vector);
	memcpy(ray->position, camPos, sizeof(float[3]));
}

int hit = 0;
int miss = 0;

RayHit RayTriangleIntersect(Ray * ray, Triangle * tri) {
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
	
	/*
	printf("Vertex A: %f, %f, %f. Vertex B: %f, %f, %f. Vertex C: %f, %f, %f. \n Ray Vector: %f, %f, %f. Ray Position: %f, %f, %f \n",
		verta[0], verta[1], verta[2], vertb[0], vertb[1], vertb[2], vertc[0], vertc[1], vertc[2], vecray[0], vecray[1], vecray[2], 
		posray[0], posray[1], posray[2]);  
	*/
	
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
	
	// Return the result of the intersection.
	if (t < 0) {
		miss++;
		t = 0;
	}
	gamma = (((i * ((a * k) - (j * b))) + (h * ((j * c) - (a * l))) + (g * ((b * l) - (k * c)))) / m);
	if ((gamma < 0) || (gamma > 1)) {
		miss++;
		t = 0;
	}
	beta = ((j * ((e * i) - (h * f)) + (k * ((g * f) - (d * i))) + (l * ((d * h) - (e * g)))) / m);
	if ((beta < 0) || (beta > (1-gamma))) {
		miss++;
		t = 0;
	}	

	float hitPostion[3] = {0,0,0};
  vec4f_scalarMult(vecray, t);
  vec3f_add_new(hitPostion, posray, vecray);

	/*
  float normal[3] = {0,0,0};
  vec3f_sub_new(normal,hitPostion, m);
  vec3f_normalize(normal);
	*/

  float lightPos[3] = {-5.0,-3.0,-15.0};
  vec3f_sub_new(lightPos,lightPos,hitPostion);
  vec3f_normalize(lightPos);

  float diffuse = 1;

  float outcolor[3];
  memcpy(outcolor,tri->color,sizeof(float[3]));
  vec4f_scalarMult(outcolor, diffuse);

  RayHit rayHit = {0,{0,0,0}};
  if(t != 0){
    rayHit.hit = 1;
    rayHit.color[0] = outcolor[0];
    rayHit.color[1] = outcolor[1];
    rayHit.color[2] = outcolor[2];
    //printf("rayHit = {%f,%f,%f},%f\n",rayHit.color[0],rayHit.color[1],rayHit.color[2],rayHit.hit);
  }

  return rayHit;
}

RayHit RaySphereIntersect(Ray * ray, Sphere * sph){
  float e[3];
  float d[3];
  float c[3];

	memcpy(e,ray->position,sizeof(float[3]));
	memcpy(d,ray->vector,sizeof(float[3]));
	memcpy(c,sph->pos,sizeof(float[3]));

	// printf("e = {%f,%f,%f}\n",e[0],e[1],e[2]);
	// printf("d = {%f,%f,%f}\n",d[0],d[1],d[2]);
  // printf("c = {%f,%f,%f}\n",c[0],c[1],c[2]);

  float r = sph->radius;
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

  if(discriminant==0){//which one do I take?
    t = (- vec3f_dot(d,eminc) + sqrt( discriminant ) )/ vec3f_dot(d,d);
		hit++;
  }else if(discriminant>0){
    tpos = (-vec3f_dot(d,eminc) + sqrt( discriminant ) )/ vec3f_dot(d,d);
    tneg = (-vec3f_dot(d,eminc) - sqrt( discriminant ) )/ vec3f_dot(d,d);
		hit++;
    if(tpos<tneg){
      t = tpos;
    }else{
      t = tneg;
    }
  }else{
    t = 0;
		miss++;
  }

  float hitPostion[3] = {0,0,0};
  vec4f_scalarMult(d, t);
  vec3f_add_new(hitPostion, e, d);

  float normal[3] = {0,0,0};
  vec3f_sub_new(normal,hitPostion,c);
  vec3f_normalize(normal);

  float lightPos[3] = {-5.0,-3.0,-15.0};
  vec3f_sub_new(lightPos,lightPos,hitPostion);
  vec3f_normalize(lightPos);

  float diffuse = 1;//vec3f_dot(normal,lightPos);

  float outcolor[3];
  memcpy(outcolor,sph->color,sizeof(float[3]));
  vec4f_scalarMult(outcolor, diffuse);

  // printf("sph->color = {%f,%f,%f}\n",sph->color[0],sph->color[1],sph->color[2]);

  RayHit rayHit = {0,{0,0,0}};
  if(t!=0){
    rayHit.hit = 1;
    rayHit.color[0] = outcolor[0];
    rayHit.color[1] = outcolor[1];
    rayHit.color[2] = outcolor[2];
    //printf("rayHit = {%f,%f,%f},%f\n",rayHit.color[0],rayHit.color[1],rayHit.color[2],rayHit.hit);
  }

  // printf("e = {%f,%f,%f}\n",e[0],e[1],e[2]);
  // printf("d = {%f,%f,%f}\n",d[0],d[1],d[2]);
  // printf("t=%f\n",t);

  // if(t>0){
  //     printf("p(t) = e + t*d:{%f,%f,%f}\n", hitPostion[0], hitPostion[1], hitPostion[2]);
  // }

  //normal vector = normalize(hit pos - center pos of sphere)
  //difuse shading = dot(normalize(normal vector) • normalize(light pos - hit pos))

  //when reflection hit add a small number + direction the ray would go

  return rayHit;
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

	Sphere sph1 = {
		{0,0,-16},2,RED,0
	};

  Sphere sph2 = {
    {3,-1,-14},1,GREEN,0//x and y are swaped and -()
  };

  Sphere sph3 = {
    {-3,-1,-14},1,BLUE,0//x and y are swaped and -()
  };

	Ray ray = {
		{0,0,0}, {0,0,-2}, 10.0
	};

	Perspective p = {
		{0.0,0.0,0.0}, 2, 2, 512
	};	

	Triangle back1 = {
		{-8,-2,-20},{8,-2,-20},{8,10,-20}, BLUE, 0
	};	
		
	Triangle back2 = {
		{-8,-2,-20},{8,10,-20},{-8,10,-20}, BLUE, 0
	};	

	Triangle bot1 = {
		{8,-2,-20},{8,-2,-10},{-8,-2,-20}, WHITE, 0
	};	

	Triangle bot2 = {
		{-8,-2,-20},{-8,-2,-10},{8,-2,-10}, WHITE, 0
	};	

	Triangle right = {
		{8,-2,-20},{8,-2,-10},{8,10,-20}, RED, 0	
	};	

  unsigned int x;
  unsigned int y;
	float screenCoord[2];

	for (x=0; x<WIDTH; x++) {
		for (y=0; y<HEIGHT; y++) {
  // for (x=255; x<256; x++) {
  //   for (y=255; y<256; y++) {
			screenCoord[0] = y;
			screenCoord[1] = x;
			// printf("BEFORE[%d,%d] Ray = {%f,%f,%f},{%f,%f,%f},%f\n",x,y,ray.vector[0], ray.vector[1], ray.vector[2],ray.position[0], ray.position[1], ray.position[2],ray.numReflections);

			// printf("AFTER[%d,%d] Ray = {%f,%f,%f},{%f,%f,%f},%f\n",x,y,ray.vector[0], ray.vector[1], ray.vector[2],ray.position[0], ray.position[1], ray.position[2],ray.numReflections);

			//{0,0,0}, {0,0,2}, 10
			// printf("testSphere = {%f,%f,%f},%f,%f\n",testSphere.pos[0],testSphere.pos[1],testSphere.pos[2],testSphere.radius,testSphere.mat);
			// Calculate and set the color of the pixel.

			int pos = (x * WIDTH + y) * 3;
      getRay(&p, screenCoord, &ray);
			RayHit r1 = RaySphereIntersect(&ray, &sph1);
      RayHit r2 = RaySphereIntersect(&ray, &sph2);
      RayHit r3 = RaySphereIntersect(&ray, &sph3);
			RayHit b1 = RayTriangleIntersect(&ray, &back1);
			RayHit b2 = RayTriangleIntersect(&ray, &back2);
			RayHit f1 = RayTriangleIntersect(&ray, &bot1);
			RayHit f2 = RayTriangleIntersect(&ray, &bot2);
			RayHit rt = RayTriangleIntersect(&ray, &right);

			if (b1.hit != 0) {
				ImageArray[pos] = b1.color[0];//blue channel
				ImageArray[pos+1] = b1.color[1];//green channel
				ImageArray[pos+2] = b1.color[2];//red channel
			}	else {
				ImageArray[pos] = 0;//blue channel
				ImageArray[pos+1] = 0;//green channel
				ImageArray[pos+2] = 0;//red channel
			}	
		
			if (b2.hit != 0) {
				ImageArray[pos] = b2.color[0];//blue channel
				ImageArray[pos+1] = b2.color[1];//green channel
				ImageArray[pos+2] = b2.color[2];//red channel
			}		

			if (f1.hit != 0) {
				ImageArray[pos] = f1.color[0];//blue channel
				ImageArray[pos+1] = f1.color[1];//green channel
				ImageArray[pos+2] = f1.color[2];//red channel
			}		

			if (f2.hit != 0) {
				ImageArray[pos] = f2.color[0];//blue channel
				ImageArray[pos+1] = f2.color[1];//green channel
				ImageArray[pos+2] = f2.color[2];//red channel
			}	

			if (rt.hit != 0) {
				ImageArray[pos] = rt.color[0];//blue channel
				ImageArray[pos+1] = rt.color[1];//green channel
				ImageArray[pos+2] = rt.color[2];//red channel
			}		

      if (r1.hit != 0) {
				ImageArray[pos] = r1.color[0];//blue channel
				ImageArray[pos+1] = r1.color[1];//green channel
				ImageArray[pos+2] = r1.color[2];//red channel
			} 
			
      if (r2.hit != 0) {
				ImageArray[pos] = r2.color[0];//blue channel
				ImageArray[pos+1] = r2.color[1];//green channel
				ImageArray[pos+2] = r2.color[2];//red channel
			}

			if (r3.hit != 0) {
				ImageArray[pos] = r3.color[0];//blue channel
				ImageArray[pos+1] = r3.color[1];//green channel
				ImageArray[pos+2] = r3.color[2];//red channel
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
