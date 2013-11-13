/*
Team members:
G. Karthik Reddy 2010A7PS140H
Sai Aditya Chitturu	 2010A7PS063H
*/


#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdlib.h>
#include<stdio.h>
#include <unistd.h>
#include <math.h>

class Point3{

	public:
		float x,y,z;
		void set(float dx,float dy, float dz){
			x=dx;
			y=dz;
			z=dz;
		}
		void set(Point3& p){
			x=p.x;
			y=p.y;
			z=p.z;
		}
		Point3(float xx, float yy, float zz){
			x=xx;
			y=yy;
			z=zz;
		}
		Point3(){
			x=y=z=0;
		}
		void build4Tuple(float v[]){
			v[3]=1.0f;
			v[0]=x;
			v[1]=y;
			v[2]=z;
		}

};

class Vector3{

	public:
		float x,y,z;

		void set(float dx,float dy, float dz){
			x=dx;
			y=dy;
			z=dz;
		}

		void set(Vector3& v){
			x = v.x; y = v.y; z = v.z;
		}

		void setDiff(Point3& a, Point3& b){
			x=a.x-b.x;
			y=a.y-b.y;
			z=a.z-b.z;
		}
		void normalize();//adjust vector to unit length

		Vector3(float xx,float yy, float zz){
			x=xx;
			y=yy;
			z=zz;
		}
		Vector3(Vector3& v){
			x=v.x;
			y=v.y;
			z=v.z;

		}

		Vector3(){
			x=y=z=0;
		}

		Vector3 cross(Vector3 v);
		float dot(Vector3 b);
};

float Vector3::dot(Vector3 b){
	float x=x*b.x;
	float y=y*b.y;
	float z=z*b.z;
	return (x+y+z);

}

void Vector3::normalize(){

	float magnitude=sqrt(x*x+y*y+z*z);
	x=x/magnitude;
	y=y/magnitude;
	z=z/magnitude;
}

class Camera{
	private:
		Point3 eye;
		Vector3 u,v,n;
		//double viewAngle, aspect,nearDist,farDist;
		void setModelViewMatrix();
	public:
		Camera(){
			;
		}
		void set(Point3 eye, Point3 look, Vector3 up);
		void roll(float angle);
		void pitch(float angle);
		void yaw(float yaw);

		void slide(float delU, float delV, float delN);
		//void setShape(float vAng, float asp, float nearD, float farD);
		//void getShape(float &vAng, float &asp, float &nearD, float &farD);
};

void Camera::setModelViewMatrix(){
	float m[16];
	Vector3 eVec(eye.x,eye.y,eye.z);//vector of eye
	m[0]=u.x;m[4]=u.y;m[8]=u.z;m[12]=-eVec.dot(u);
	m[1]=v.x;m[5]=v.y;m[9]=v.z;m[13]=-eVec.dot(v);
	m[2]=n.x;m[6]=n.y;m[10]=n.z;m[14]=-eVec.dot(n);
	m[3]=0;m[7]=0;m[11]=0;m[15]=1.0;
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(m);//load modelview matrix

}

Vector3 Vector3::cross(Vector3 v){
	Vector3 t(0,0,0);
	t.x=y*v.z - z*v.y;
	t.y=-x*v.z + z*v.x;
	t.z=x*v.y - y*v.x;
	return t;
}

void Camera::set(Point3 eye, Point3 look, Vector3 up){
	eye.set(eye);//store given eye position
	n.set(eye.x-look.x,eye.y-look.y,eye.z-look.z);//make n
	//Vector3 t( up.cross(n));
	//u.set(t.x,t.y,t.z);//u=n cross up
	u.set(up.cross(n).x,up.cross(n).y,up.cross(n).z);
	n.normalize();
	u.normalize();//make unit vectors
	//Vector3 t1(n.cross(u));
	//v.set(t1.x,t1.y,t1.z);//v=n cross u
	v.set(n.cross(u).x,n.cross(u).y,n.cross(u).z);
	setModelViewMatrix(); //tell openGL

}

void Camera::slide(float delU, float delV, float delN){
	eye.x += delU * u.x + delV * v.x + delN * n.x;
	eye.y += delU * u.y + delV * v.y + delN * n.y;
	eye.z += delU * u.z + delV * v.z + delN * n.z;
	setModelViewMatrix();
}

void Camera::roll(float angle){
	float cs=cos(3.141/180 * angle);
	float sn=sin(3.141/180 * angle);

	Vector3 t(u);

	u.set(cs*t.x - sn*v.x, cs*t.y - sn*v.y, cs*t.z-sn*v.z);
	v.set(sn*t.x+cs*v.x, sn*t.y+cs*v.y,sn*t.z+cs*v.z);
	setModelViewMatrix();
}


//Camera cam=Camera();

//glulookat variables

static GLdouble xview=.1, yview=.1, zview=.1;

// range kutta variables
// k is spring constant and is set to 1.5
// b is damping constant(friction coefficient) and is set to 0.9
static GLdouble x0=0,x1,v0=0.5,v1,a1,b1,c1,d1,a2,b2,c2,d2,k=1.5,m=1,b=0.9,h=0.05,t,y; //mass 1
static GLdouble x00=0,x11,v00=0.5,v11,a11,b11,c11,d11,a22,b22,c22,d22; //mass 2
static GLdouble x000=0,x111,v000=0.5,v111,a111,b111,c111,d111,a222,b222,c222,d222; //mass 3
//base variables used to store position and velocity of masses
static GLdouble x0base=0,x00base=-0.5, x000base=-1;
//done varibale is used to synchronize the masses as mentioned in documentation
static int done=0,i=0;

//variables used to create cylinders for lamp
static GLUquadric *myQuad;
static GLUquadric *myQuad1;


static GLfloat spin = 0.0,spin2=0.0,x=2.0;
//static int shoulder=0,elbow=0;



void init(void)
{
	glClearColor (1, 218/255.0, 218/255.0, 1.0);//glClearColor() is used to set the background color
	//glShadeModel (GL_FLAT);
	// Enable lighting
	glShadeModel (GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	//position light
	GLfloat lightpos[] = {0, 2, 2, 1};
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
	//set light color as white and of diffusive type
	GLfloat lightColor0[] = {1.f, 1.f, 1.f, 1.0f}; //Color (0.5, 0.5, 0.5)
   	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);



    /*
    GLfloat lightpos1[] = {3.5, 1, 0, 1};
	glLightfv(GL_LIGHT1, GL_POSITION, lightpos1);
	GLfloat lightColor1[] = {1.f, 1.f, 1.f, 1.0f}; //Color (0.5, 0.5, 0.5)
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor1);
    //GLfloat spot_direction[] = { -1.0, 0.0, 0.0 };
  	//glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spot_direction);
	*/
}


void legs(){
	glPushMatrix();

}

void drawers(){
	glPushMatrix();
	glTranslated(-3/2.0,0,0);
	glTranslated(0.67/2,-1.2,0);
	glScaled(0.67,2.5,1.5);
	glutSolidCube (1);
//	glScaled(0.67-0.01,1-0.01,1.5-0.01);
//	glutSolidCube (1);
	glPopMatrix();

}

void drawTable(){

	glPushMatrix();
		glTranslated(2.5,0,0);
		//drawing table drawers
		glPushMatrix();
			GLfloat colorDrawers[] = {220/255.0f,201/255.0f,120/255.0f, 1.f};
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorDrawers);
			drawers();
			glRotated(180,0,1,0);
			drawers();
		glPopMatrix();
		//creating a table top
		glPushMatrix();
			GLfloat colorTop[] = {250/255.0f,250/255.0f,205/255.0f, 1.f};
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, colorTop);
			glTranslated(0,0.125,0);
			glScaled(3,0.25,2);
			glutSolidCube (1);
		glPopMatrix();
	glPopMatrix();
}

void drawLegs(){
//draw bed legs
	glPushMatrix();
		glScaled(0.2,2,0.2);
		glutSolidCube(1);
	glPopMatrix();
}

void drawBed(){

	GLfloat color[] = {150/255.0f,32/255.0f,32/255.0f, 1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glPushMatrix();
		glTranslated(-1.5,0,0);
		glPushMatrix();
		//bed base
			glTranslated(0,-1.25,0);
			glScaled(3,0.25,4.5);
			glutSolidCube(1);
		glPopMatrix();
		glPushMatrix();
			glRotated(180,0,1,0);
			glTranslated(1.5-0.1,-1.25-0.125,2.25-0.1);
			drawLegs();
		glPopMatrix();
		// bed mattress
		GLfloat colorMatress[] = {.0f,0.9f,.9f, 1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorMatress);
		glPushMatrix();
			glColor3d(1,0,1);
			glTranslated(0,-0.925,0);
			glScaled(2.5,0.5,4);
			glutSolidCube(1);
		glPopMatrix();
		//draw bedLegs
		GLfloat colorBedLegs[] = {150/255.0f,32/255.0f,32/255.0f, 1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorBedLegs);
		glPushMatrix();
			glNormal3d(0,0,1);
			glTranslated(-1.5+0.1,-1.25-0.125,2.25-0.1);
			drawLegs();
		glPopMatrix();
		glPushMatrix();
			glNormal3d(0,0,1);
			glTranslated(1.5-0.1,-1.25-0.125,2.25-0.1);
			drawLegs();
		glPopMatrix();
		glPushMatrix();
			glRotated(180,0,1,0);
//			glNormal3d(0,0,-1);
			glPushMatrix();
				glTranslated(-1.5+0.1,-1.25-0.125,2.25-0.1);
				drawLegs();
			glPopMatrix();
		glPopMatrix();
		glPushMatrix();
			glTranslated(0,-0.5,-2.25+0.1);
			glScaled(3,0.4,0.2);
			glutSolidCube(1);
		glPopMatrix();
		glPushMatrix();
			glRotated(180,0,1,0);
			glTranslated(0,-0.5,-2.25+0.1);
			glScaled(3,0.4,0.2);
			glutSolidCube(1);
		glPopMatrix();
	glPopMatrix();

}

void drawLine(){
//draw a line of length 20 units
	   	glBegin(GL_LINES);
	   		glVertex3d(0,0,0);
	   		glVertex3d(0,0,20);
	   	glEnd();
}

void drawUpperLedge(){
	glPushMatrix();
		glColor3d(189/255.0,137/255.0,103/255.0);
		glTranslated(-3.5,0,0);
		glScaled(1,0.2,3);
		glutSolidCube(1);
	glPopMatrix();
}

void drawSideLedge(){
	glPushMatrix();
		glRotated(-90,1,0,0);
		glTranslated(-3.5,1.5-0.1,1.5);
		glScaled(1,0.2,3);
		glutSolidCube(1);
	glPopMatrix();
}

void drawLamp(){

	GLfloat color[] = {205/255.0,104/255.0,57/255.0,1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);

	//lamp base
	glutSolidSphere(0.5,15,6);
	//lamp neck
	glTranslated(0,0.60,0);
	glScaled(0.2,0.3,0.2);
	glutSolidCube(1);
	glRotated(-90,1,0,0);
	glTranslated(0,0,0.5);
	myQuad1=gluNewQuadric();
	//lamp cover cone
	GLfloat color1[] = {176/255.0,224/255.0,230/255.0,1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color1);
	gluCylinder(myQuad1, 3, 1, 3, 15, 5);
}

//ignore
void drawBlade(){
		glBegin(GL_TRIANGLES );
			glVertex3d(0,0,0);
			glVertex3d(0.71,0,0);
			glVertex3d(0.5,0.5,0);
		glEnd();
}

void drawChairLegs(){
	//draw chai legs
	glPushMatrix();
		glTranslated(-(1.5*0.5)+0.1,-1.5*0.5,(1.5*0.5)-0.1);
		GLfloat colorBase[] = {105/255.0,105/255.0,105/255.0,1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorBase);
		glScaled(0.2,1.5,0.2);
		glutSolidCube(1);
	glPopMatrix();
}

void drawChairBack(){
	//draws chair back support
	glPushMatrix();
		glTranslated(-(1.5*0.5)+0.1,1.5*0.5,(1.5*0.5)-0.1);
		GLfloat colorBase[] = {139/255.0,139/255.0,131/255.0,1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorBase);
		glScaled(0.2,1.5,0.2);
		glutSolidCube(1);
	glPopMatrix();

}

void drawChair(){
	glPushMatrix();
		//draw 4 chair legs at corners of chair base
		glTranslated(2,-1,2);
		drawChairLegs();
		glRotated(90,0,1,0);
		drawChairLegs();
		glRotated(90,0,1,0);
		drawChairLegs();
		glRotated(90,0,1,0);
		drawChairLegs();
		glPushMatrix();
			GLfloat colorBase[] = {176/255.0,224/255.0,230/255.0,1.f};
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorBase);
			glScaled(1.5,0.2,1.5);
			glutSolidCube(1);
		glPopMatrix();
		glRotated(90,0,1,0);
		//draw chair back support which consists of 4 verical cuboids
		drawChairBack();
		glRotated(90,0,1,0);
		drawChairBack();
		glRotated(-90,0,1,0);
		glPushMatrix();
			glTranslated(-0.2,1.5*0.5,0.75-0.1);
			glScaled(0.1,1.5,0.1);
			glutSolidCube(1);
		glPopMatrix();
		//draw chair base
		glPushMatrix();
			glTranslated(0.2,1.5*0.5,0.75-0.1);
			glScaled(0.1,1.5,0.1);
			glutSolidCube(1);
		glPopMatrix();
		//draw chair back support upper cuboid
		glPushMatrix();
			glTranslated(0,1.5,0.75-0.1);
			glScaled(1.5,0.2,0.2);
			glutSolidCube(1);
		glPopMatrix();

	glPopMatrix();
}

void display(void)
{


	glClear(GL_COLOR_BUFFER_BIT);//used to reset the background to the color specified in the init method.

	glColor3d(0,0,0);
	//draw lines on the floor
	for(i=0;i<10;){
		glPushMatrix();
			glTranslated(i,0,0);
			drawLine();

		glPopMatrix();
		glPushMatrix();
			glTranslated(0,0,i-1);
			glBegin(GL_LINES);
	   			glVertex3d(-2.0,0,0);
	   			glVertex3d(20,0,0);
	   		glEnd();

		glPopMatrix();
		i+=2;
	}
	//ignore
	glPushMatrix();
		//glScaled(4.99,4.99,4.99);
	   	//glutWireCube(1);
	   	glColor3d(0,0,0);
	   	glTranslated(-4,-2.5,-2.5);
		drawLine();
		glRotated(90,0,1,0);
	   	drawLine();
	   	glRotated(-90,1,0,0);
	   	drawLine();
	glPopMatrix();
	//draw bed
	glPushMatrix();
		glTranslated(0.1,0,0.5);
		drawBed();
	glPopMatrix();
	//draw table
	drawTable();
	//draw cupboard
	GLfloat color[] = {139/255.0,105/255.0,20/255.0,1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glPushMatrix();
		drawSideLedge();
		glTranslated(0,0,3-0.2);
		//draw side cuboids of the cupboard
		drawSideLedge();
	glPopMatrix();
	glPushMatrix();
		drawUpperLedge();
		glTranslated(0,3,0);
		//draw upper and bottom cuboids of the cupboard
		drawUpperLedge();
	glPopMatrix();
	glPushMatrix();
		glTranslated(3.5,0.7,-0.5);
		//draw lamp
		drawLamp();
	glPopMatrix();
	glPushMatrix();
		//draw teapot
		GLfloat colorTeapot[] = {255/255.0,222/255.0,173/255.0,1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorTeapot);
		//glColor3d(138/255.0,138/255.0,52/255.0); 255	222	173
		glTranslated(-3.5,0.5,0);
		glPushMatrix();
			glRotated(90,0,1,0);
			glutSolidTeapot(0.5);
		glPopMatrix();

	glPopMatrix();


	// draw worm - spheres and cylinders
	glPushMatrix();
		glTranslated(2,0.325,0);
		glPushMatrix();
			//mass1 sphere
			glColor3d(142/255.0,136/255.0,136/255.0);
			GLfloat colorSphere[] = {142/255.0f,136/255.0f,136/255.0f,1.f};
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorSphere);
			glTranslated(x0base+x0,0,0);
			glutSolidSphere(0.130,10,6);
		glPopMatrix();
		//cylinder body
		glColor3d(231/255.0,171/255.0,171/255.0);
		GLfloat colorCylinder[] = {231/255.0,171/255.0,171/255.0,1.f};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorCylinder);
		myQuad=gluNewQuadric();
		glPushMatrix();
		glTranslated(x00base+x00,0,0);
		glRotated(90,0,1,0);
		gluCylinder( myQuad, 0.125, 0.125,(x0base+x0)-(x00base+x00) ,10, 10 );
		glPopMatrix();
		//end cylinder body
		glPushMatrix();
			//mass2 sphere
			glColor3d(142/255.0,136/255.0,136/255.0);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorSphere);
			glTranslated(x00base+x00,0,0);
			glutSolidSphere(0.130,10,6);
		glPopMatrix();
		//cylinder body
		glColor3d(231/255.0,171/255.0,171/255.0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorCylinder);
		myQuad=gluNewQuadric();
		glPushMatrix();
		glTranslated(x000base+x000,0,0);
		glRotated(90,0,1,0);
		gluCylinder( myQuad, 0.125, 0.125,(x00base+x00)-(x000base+x000) ,10, 10 );
		glPopMatrix();
		//end cylinder body
		glPushMatrix();
			//mass sphere 3
			glColor3d(142/255.0,136/255.0,136/255.0);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorSphere);
			glTranslated(x000base+x000,0,0);
			glutSolidSphere(0.130,10,6);
		glPopMatrix();
	glPopMatrix();
	/*//table fan
	glPushMatrix();
		glTranslated(1,1.5,-0.5);
		glRotated(spin2,0,1,0);
		glRotated(spin,0,0,1);
			drawBlade();
		glRotated(120,0,0,1);
			drawBlade();
		glRotated(120,0,0,1);
			drawBlade();
	glPopMatrix();
	//table fan end
		*/
	//draw chair
	drawChair();
	glFlush();
	glutSwapBuffers();
}

void spinDisplay(void)
{
if(done==0){
		//mass 1
		if((x1-x0)>=0.0 & v0>0.0){
		a1=v0;
		b1=v0+(h/2.0)*a1;
		c1=v0+(h/2.0)*b1;
		d1=v0+h*c1;

		t=-1*(k/m);
		y=-1*(b/m);

		a2=t*x0+y*v0;
		b2=t*(x0+(h/2.0)*a2)+y*(v0+(h/2.0)*a2);
		c2=t*(x0+(h/2.0)*b2)+y*(v0+(h/2.0)*b2);
		d2=t*(x0+h*c2)+y*(v0+h*c2);

		x1=x0+(h/6.0)*(a1+2*b1+2*c1+d1);
		v1=v0+(h/6.0)*(a2+2*b2+2*c2+d2);

		x0=x1;
		v0=v1;
		//printf("%e %e\n",x0,v0);
		}
		else{
			//glTranslated(x0,0,0);
			v0=0.5;
			x0base+=x0;
			x0=0;
			done=1;
		}
	}
	if(done==1){
		//mass 2
		if((x11-x00)>=0.0 & v00>0.0){
			a11=v00;
			b11=v00+(h/2.0)*a11;
			c11=v00+(h/2.0)*b11;
			d11=v00+h*c11;


			a22=t*x00+y*v00;
			b22=t*(x00+(h/2.0)*a22)+y*(v00+(h/2.0)*a22);
			c22=t*(x00+(h/2.0)*b22)+y*(v00+(h/2.0)*b22);
			d22=t*(x00+h*c22)+y*(v00+h*c22);

			x11=x00+(h/6.0)*(a11+2*b11+2*c11+d11);
			v11=v00+(h/6.0)*(a22+2*b22+2*c22+d22);

			x00=x11;
			v00=v11;
			//printf("%e %e\n",x00,v00);
		}
		else{
			//glTranslated(x00,0,0);
			done=2;
			v00=0.5;
			x00base+=x00;
			x00=0;
		}
	}
	if(done==2){
		//mass 3
		if((x111-x000)>=0.0 & v000>0.0){
			a111=v000;
			b111=v000+(h/2.0)*a111;
			c111=v000+(h/2.0)*b111;
			d111=v000+h*c111;


			a222=t*x000+y*v000;
			b222=t*(x000+(h/2.0)*a222)+y*(v000+(h/2.0)*a222);
			c222=t*(x000+(h/2.0)*b222)+y*(v000+(h/2.0)*b222);
			d222=t*(x000+h*c222)+y*(v000+h*c222);

			x111=x000+(h/6.0)*(a111+2*b111+2*c111+d111);
			v111=v000+(h/6.0)*(a222+2*b222+2*c222+d222);

			x000=x111;
			v000=v111;
			//printf("%e %e\n",x000,v000);
		}
		else{
			//glTranslated(x00,0,0);
			done=0;
			v000=0.5;
			x000base+=x000;
			x000=0;
		}
	}
//table fan code
//ignore
	spin+=5.0;
	spin2+=x;
	if(spin2>=90){
		x=-2.0;
	}
	else if(spin2<=-10)
		x=2.0;
	//printf("%f\n",spin2);

//table fan code end
	glutPostRedisplay();

}

void reshape(int w, int h)
{
	//define viewport parameters
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
	//define orthogonal projection parallelopiped
  glOrtho(-5.0, 5.0, -5.0, 5.0, 5.0, -10.0);
//	 gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 5.0, 125.0);
//   gluPerspective(65.0, (GLfloat) w/(GLfloat) h, .0, 10.0);
//   gluOrtho2D(-50.0, 50.0, -50.0, 50.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   //looking from (1,1,1) towards (0,0,0)
//  gluLookAt(xview,yview,zview,0,0,0,0,1,0);
//      gluLookAt(1,0,0,0,0,0,0,1,0);
//   gluLookAt(5,5,5,0,0,0,0,1,0);
	Point3 eye(.1,.1,0.1);	
	Point3 look(0,0,0);
	Vector3 up(0,1,0); 	
	Camera cam;
	cam.set(eye,look,up);
	glutPostRedisplay();

}

void mouse(int button, int state, int x, int y)
{

//when left mouse button clicked, animate the worm
   switch (button) {
      case GLUT_LEFT_BUTTON:
         if (state == GLUT_DOWN)
            glutIdleFunc(spinDisplay);
         break;
      case GLUT_MIDDLE_BUTTON:
         if (state == GLUT_DOWN)
            glutIdleFunc(NULL);
         break;
      default:
         break;
   }
}

void myKeyBoardFunc(unsigned char key, int x, int y){
	switch(key){
		case 'x':	xview+=0.1;
					printf("x\n");
					break;
		case 'y':	yview+=0.1;
					printf("y\n");
					break;
		case 'z':	zview+=0.1;
					printf("z\n");
					break;

	}
	glLoadIdentity();
	//looking from (1,1,1) towards (0,0,0)
	//gluLookAt(xview,yview,zview,0,0,0,0,1,0);
	//Camera cam;
	Point3 eye(1,1,1);	
	Point3 look(0,0,0);
	Vector3 up(0,1,0); 	
	Camera cam;
	cam.set(eye,look,up);
	glutPostRedisplay();
}



int main(int argc, char** argv)
{
   glutInit(&argc, argv);
	glutInitDisplayMode ( GLUT_DOUBLE | GLUT_RGBA);
	//glEnable(GL_DEPTH_TEST);
	//define window size
   glutInitWindowSize (640, 640);
   //define window position on the monitor
   glutInitWindowPosition (100, 100);
   //set title of window
   glutCreateWindow ("worm");
   //initialize the system
   init();
   // display is the callback function to draw on the screen
   glutDisplayFunc(display);
   // reshape funcion defines the projection and modelview matrices
   glutReshapeFunc(reshape);
   // mouse is the mouse event callback function
   glutMouseFunc(mouse);
   glutKeyboardFunc(myKeyBoardFunc);
   glutMainLoop();
   return 0;
}

/*
Team members:
G. Karthik Reddy 2010A7PS140H
Sai Aditya Chitturu	 2010A7PS063H
*/

