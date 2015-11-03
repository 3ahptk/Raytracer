/** @file This program demonstrates the process of Ray-Tracing.
 *				It shoots out a series of rays for each of the pixels
 *				in order to calculate the intersection of the objects,
 *				either a triangle or a sphere. It then calculates the
 *				color of the objects based on the ray hit result. The
 *				ray hit result tells the color of the object to be drawn.
 *				This program demonstrates two different scenes done with
 * 				ray-tracing: reference and custom.
 *
 * @author Charles Kralapp
 * @author Nik Koshcheyev
 */

// Import helper classes to use vec3f functions.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vecmat.h"
#include "msg.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Set up the width, height, and arraysize for the image.
#define WIDTH 512
#define HEIGHT 512
#define ARRAYSIZE WIDTH*HEIGHT*3

// Set up the RGB values for each of the colors used in this program.
#define RED   {255,0,0}
#define GREEN {0,255,0}
#define BLUE  {0,0,255}
#define BLACK {0,0,0}
#define WHITE {255,255,255}

#define debug 1
#define trace if (debug) write
#define MAX(a,b) (((a)>(b))?(a):(b))

char buffer[256];
unsigned char ImageArray[ARRAYSIZE];
float pixellength = 2.0/512.0;
char * filename;
int hit = 0;
int miss = 0;

// Create the Ray struct that will calculate the rays being shot.
typedef struct {
	float vector[3];
	float position[3];
	int numReflections; //10
} Ray;

// Set up the perspective of the camera.
typedef struct {
	const	float cameraPos[3];
	const float distanceToScreen;
	const unsigned int widthWorld;
	const unsigned int widthPixels;
} Perspective;

// This is the sphere struct.
typedef struct {
  float pos[3];
  float radius;
  float color[3];
  int reflect;
} Sphere;

// This is the triangle struct.
typedef struct {
	float posa[3];
	float posb[3];
	float posc[3];
	float color[3];
	int reflect;
} Triangle;

// This is the struct that represent the pixel hit by a ray.
typedef struct {
	int hittype;
	Sphere sph;
	Triangle tri;
	float normal[3];
	float lightpos[3];
	float hit;
  float color[3];
} RayHit;

/* This is called every time you shoot out a ray at each of the pixels
	 in the imageArray. This calculates the ray attributes that will be
	 used to calculate the color of the scene objects. */
void getRay(Perspective * p, float screenCoord[2], Ray * ray) {
  float camPos[3] = {0,0,0};
  memcpy(camPos,p->cameraPos,sizeof(float[3]));		// Set up the position of the camera.
  float posPixel[3] = {-1 + screenCoord[0]*pixellength, 1 - screenCoord[1]*pixellength, -2}; // Map the location of pixel on screen
	float srcVec[3];
	vec3f_sub_new(srcVec, posPixel, camPos);	// Subtract the cameraPosition from the pixel position.
	memcpy(ray->vector, srcVec, sizeof(float[3])); 
	vec3f_normalize(ray->vector);	// Normalize the ray vector.
	memcpy(ray->position, camPos, sizeof(float[3]));	
}

/* This is called every time you shoot out a ray at each of the pixels
	 in the imageArray. This calculates the ray attributes that will be
	 used to calculate the color of the scene objects. */
RayHit RayTriangleIntersect(Ray * ray, Triangle * tri) {
	// The three triangle vertices.
	float verta[3], vertb[3], vertc[3];
	float vecray[3], posray[3];
	float n[3] = {0.0, 0.0, 0.0};	// Normal vector
	float xa, xb, xc, xd, xe, ya, yb, yc, yd, ye, za, zb, zc, zd, ze;	// The components to determine t.	
	float m, beta, gamma, t;	// Components to determine the Barycentric coordinates.
	float a, b, c, d, e, f, g, h, i, j, k, l;	
	
	// Copy the three positions from the triangle class into the x, y, z arrays.
	memcpy(verta, tri->posa, sizeof(float[3]));
	memcpy(vertb, tri->posb, sizeof(float[3]));
	memcpy(vertc, tri->posc, sizeof(float[3]));
	memcpy(vecray, ray->vector, sizeof(float[3]));
	memcpy(posray, ray->position, sizeof(float[3]));

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
	
	// Calculate the normal of the triangle
	float u[3] = {0,0,0};
	float v[3] = {0,0,0};
	vec3f_sub_new(u, vertb, verta);
	vec3f_sub_new(v, vertc, verta);
	vec3f_cross_new(n, v, u);
	hit++;

	// Determine the position that the vector hits the object.
	float hitPostion[3] = {0,0,0};
  vec3f_scalarMult(vecray, t);
  vec3f_add_new(hitPostion, posray, vecray);

	// Set up the normal of the triangle in regards to the hit position.
  float normal[3] = {0,0,0};
  vec3f_sub_new(normal, hitPostion, n);
  vec3f_normalize(normal);
  
  // Set up the light position based on the user selected scene.
  float lightPos[3] = {0,0,0};
	if (strcmp(filename, "reference.png" ) == 0) {
  	float refLightPos[3] = {3.0, 5.0,-15.0};
  	memcpy(lightPos,refLightPos,sizeof(float[3]));
  } else {
  	float cusLightPos[3] = {0.0, 7.0, -15.0};
  	memcpy(lightPos,cusLightPos,sizeof(float[3]));
  }
  vec3f_sub_new(lightPos,lightPos,hitPostion);
  vec3f_normalize(lightPos);

	// Calculate the diffuse shading.
  float diffuse = vec3f_dot(normal,lightPos);
  diffuse = MAX(0,diffuse);
  float outcolor[3];
  memcpy(outcolor,tri->color,sizeof(float[3]));
  vec3f_scalarMult(outcolor, diffuse);
	
	// Set up the rayHit object to return.
  RayHit rayHit = {0,{{0,0,0},0,BLACK,0},{{0,0,0},{0,0,0},{0,0,0}, BLACK, 0},{0,0,0},{0,0,0},0,{0,0,0}};
  if(t != 0){
		rayHit.hittype=1;//zero for spheres, one for triangles
		memcpy(&rayHit.tri,tri,sizeof(Triangle));

		rayHit.normal[0] = normal[0];
    rayHit.normal[1] = normal[1];
    rayHit.normal[2] = normal[2];
		rayHit.lightpos[0] = lightPos[0];
		rayHit.lightpos[1] = lightPos[1];
		rayHit.lightpos[2] = lightPos[2];

    rayHit.hit = 1;
    rayHit.color[0] = outcolor[0];
    rayHit.color[1] = outcolor[1];
    rayHit.color[2] = outcolor[2];
  }

  return rayHit;
}

RayHit RaySphereIntersect(Ray * ray, Sphere * sph){
  float e[3];//e = starting position of ray
  float d[3];//d = vector representing ray
  float c[3];//c=(xc, yc, zc) ; Center of sphere

	memcpy(e,ray->position,sizeof(float[3]));
	memcpy(d,ray->vector,sizeof(float[3]));
	memcpy(c,sph->pos,sizeof(float[3]));

  float r = sph->radius;
  float eminc[3] = {0,0,0};
  vec3f_sub_new(eminc,e,c);

	float discriminant = (vec3f_dot(d,eminc)*vec3f_dot(d,eminc)) - vec3f_dot(d,d) *((vec3f_dot(eminc,eminc)-(r*r)));

  float t = 0;//t = “time”/distance
  float tpos = 0;//find the time for the 2 intersection path
  float tneg = 0;//find the time for the 2 intersection path

  if(discriminant==0){//1 solution
    t = (-vec3f_dot(d,eminc) + sqrt( discriminant ) )/ vec3f_dot(d,d);
		hit++;
  }else if(discriminant>0){//2 solutions
    tpos = (-vec3f_dot(d,eminc) + sqrt( discriminant ) )/ vec3f_dot(d,d);
    tneg = (-vec3f_dot(d,eminc) - sqrt( discriminant ) )/ vec3f_dot(d,d);
		hit++;
    if(tpos<tneg){
      t = tpos;
    }else{
      t = tneg;
    }
  }else{//didnt hit
    t = 0;
		miss++;
  }

  float hitPostion[3] = {0,0,0};//calculate where the ray hit
  vec3f_scalarMult(d, t);
  vec3f_add_new(hitPostion, e, d);

  float normal[3] = {0,0,0};//calculate the normal vector of the hit position
  vec3f_sub_new(normal,hitPostion,c);
  vec3f_normalize(normal);

	float lightPos[3] = {0,0,0};//calculate where the light is
	if (strcmp(filename, "reference.png" ) == 0) {
  	float refLightPos[3] = {3.0, 5.0,-15.0};
  	memcpy(lightPos,refLightPos,sizeof(float[3]));
  } else {
  	float cusLightPos[3] = {0.0, 7.0, -15.0};
  	memcpy(lightPos,cusLightPos,sizeof(float[3]));
  }
  vec3f_sub_new(lightPos,lightPos,hitPostion);
  vec3f_normalize(lightPos);

  float diffuse = vec3f_dot(normal,lightPos);//calculate diffuse shading
  diffuse = MAX(0,diffuse)/2+.2;

  float outcolor[3];//calculate the color
  memcpy(outcolor,sph->color,sizeof(float[3]));
  vec3f_scalarMult(outcolor, diffuse);

  RayHit rayHit = {0,{{0,0,0},0,BLACK,0},{{0,0,0},{0,0,0},{0,0,0}, BLACK, 0},{0,0,0},{0,0,0},0,{0,0,0}};
  if(t!=0){
		rayHit.hittype=0;//zero for spheres, one for triangles
		memcpy(&rayHit.sph,sph,sizeof(Sphere));

		rayHit.normal[0] = normal[0];
    rayHit.normal[1] = normal[1];
    rayHit.normal[2] = normal[2];
		rayHit.lightpos[0] = lightPos[0];
		rayHit.lightpos[1] = lightPos[1];
		rayHit.lightpos[2] = lightPos[2];

    rayHit.hit = 1;
    rayHit.color[0] = outcolor[0];
    rayHit.color[1] = outcolor[1];
    rayHit.color[2] = outcolor[2];
  }
  return rayHit;
}

RayHit GenerateShadows(RayHit r, RayHit a){

}

static inline void progress(int x, int n){
		if ( x % (n/10 +1) != 0 ) return;
    float r = x/(float)n;
    int c = r * 80;
    printf("█ %3d%% ", (int)(r*100) );
    for (int x=0; x<c; x++){printf("█");}
    for (int x=c; x<80; x++){printf("░");}
    printf("\n\033[F\033[J");
}

int main(int argc, char* argv[]){
  if(argc!=2){//check for arguments
    sprintf(buffer, "Invalid number of arguments: Expected 1, got %d\n", argc-1);
  	write(2, buffer, strlen(buffer));
    return -1;
  }

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

	Ray ray = {
		{0,0,0}, {0,0,-2}, 10.0
	};

	Perspective p = {
		{0.0,0.0,0.0}, 2, 2, 512
	};

	if (strcmp(filename, "reference.png") == 0) {//setup for the reference scene
		Sphere sph1 = {
			{0,0,-16},2,BLUE,0
		};

		Sphere sph2 = {
		  {3,-1,-14},1,GREEN,0
		};

		Sphere sph3 = {
		  {-3,-1,-14},1,RED,0
		};

		Triangle back1 = {
			{-8,-2,-20},{8,-2,-20},{8,10,-20}, BLUE, 0
		};

		Triangle back2 = {
			{-8,-2,-20},{8,10,-20},{-8,10,-20}, BLUE, 0
		};

		Triangle bot1 = {
			{-8,-2,-20},{8,-2,-10},{8,-2,-20}, WHITE, 0
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

		for (x=0; x<WIDTH; x++) {//interate through each pixel on the screen
			for (y=0; y<HEIGHT; y++) {//interate through each pixel on the screen
				screenCoord[0] = y;
				screenCoord[1] = x;

				int pos = (x * WIDTH + y) * 3;//map from a 2d array to a 1d array
		    getRay(&p, screenCoord, &ray);
				RayHit r1 = RaySphereIntersect(&ray, &sph1);    // get the intersection between the ray and each object in the scene
		    RayHit r2 = RaySphereIntersect(&ray, &sph2);    // get the intersection between the ray and each object in the scene
		    RayHit r3 = RaySphereIntersect(&ray, &sph3);    // get the intersection between the ray and each object in the scene
				RayHit b1 = RayTriangleIntersect(&ray, &back1); // get the intersection between the ray and each object in the scene
				RayHit b2 = RayTriangleIntersect(&ray, &back2); // get the intersection between the ray and each object in the scene
				RayHit f1 = RayTriangleIntersect(&ray, &bot1);  // get the intersection between the ray and each object in the scene
				RayHit f2 = RayTriangleIntersect(&ray, &bot2);  // get the intersection between the ray and each object in the scene
				RayHit rt = RayTriangleIntersect(&ray, &right); // get the intersection between the ray and each object in the scene

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
			}
		}
	} else {//we are in the custom scene
		Triangle back1 = {
			{-7,-7,-20},{7,-7,-20},{7,7,-20}, BLUE, 0
		};

		Triangle back2 = {
			{-7,-7,-20},{7,7,-20},{-7,7,-20}, BLUE, 0
		};

		Triangle bot1 = {
			{-7,-7,-20},{7,-7,-10},{7,-7,-20}, WHITE, 0
		};

		Triangle bot2 = {
			{-7,-7,-20},{-7,-7,-10},{7,-7,-10}, WHITE, 0
		};

		Triangle right1 = {
			{7,-7,-20},{7,-7,-10},{7,7,-20}, RED, 0
		};

		Triangle right2 = {
			{7,-7,-10},{7,7,-10},{7,7,-20}, RED, 0
		};

		Triangle left1 = {
			{-7,7,-20},{-7,-7,-10},{-7,-7,-20}, GREEN, 0
		};

		Triangle left2 = {
			{-7,7,-20},{-7,7,-10},{-7,-7,-10}, GREEN, 0
		};

		Sphere sph1 = {
			{0,0,-14},2,WHITE,0
		};

		unsigned int x;
		unsigned int y;
		float screenCoord[2];

		for (x=0; x<WIDTH; x++) {//iterate through each pixel on the screen
			for (y=0; y<HEIGHT; y++) {//iterate through each pixel on the screen
				screenCoord[0] = y;
				screenCoord[1] = x;

				int pos = (x * WIDTH + y) * 3;//map the 2d array to a 1d array
		    getRay(&p, screenCoord, &ray);
				RayHit b1 = RayTriangleIntersect(&ray, &back1); //get the intersection between the ray and each object in the scene
				RayHit b2 = RayTriangleIntersect(&ray, &back2); //get the intersection between the ray and each object in the scene
				RayHit f1 = RayTriangleIntersect(&ray, &bot1);  //get the intersection between the ray and each object in the scene
				RayHit f2 = RayTriangleIntersect(&ray, &bot2);  //get the intersection between the ray and each object in the scene
				RayHit r1 = RayTriangleIntersect(&ray, &right1);//get the intersection between the ray and each object in the scene
				RayHit r2 = RayTriangleIntersect(&ray, &right2);//get the intersection between the ray and each object in the scene
				RayHit l1 = RayTriangleIntersect(&ray, &left1); //get the intersection between the ray and each object in the scene
				RayHit l2 = RayTriangleIntersect(&ray, &left2); //get the intersection between the ray and each object in the scene
				RayHit s = RaySphereIntersect(&ray, &sph1);     //get the intersection between the ray and each object in the scene

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

				if (l1.hit != 0) {
					ImageArray[pos] = l1.color[0];//blue channel
					ImageArray[pos+1] = l1.color[1];//green channel
					ImageArray[pos+2] = l1.color[2];//red channel
				}

				if (l2.hit != 0) {
					ImageArray[pos] = l2.color[0];//blue channel
					ImageArray[pos+1] = l2.color[1];//green channel
					ImageArray[pos+2] = l2.color[2];//red channel
				}

				if (s.hit != 0) {
					ImageArray[pos] = s.color[0];//blue channel
					ImageArray[pos+1] = s.color[1];//green channel
					ImageArray[pos+2] = s.color[2];//red channel
				}
			}
		}
	}

<<<<<<< HEAD
	printf("hit:%d,miss:%d\n",hit,miss);

  //////finalize the progress bar///////
  printf("█ %3d%% ", 100);
  for (int x=0; x<80; x++){printf("█");}
  printf("\n");
  //////////////////////////////////////

  stbi_write_png(filename, WIDTH, HEIGHT, 3, ImageArray, WIDTH*3);
=======
  stbi_write_png(filename, WIDTH, HEIGHT, 3, ImageArray, WIDTH*3);//write the array of pixels to the image
>>>>>>> 27f96d4da3ddc76275a50b100ee7f1aa061039f1

  return 1;
}
