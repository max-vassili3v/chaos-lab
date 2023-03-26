#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")

#include <Windows.h>
#include "iostream"
#include <GLFW/glfw3.h>
#include <gl/GLU.h>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#define _USE_MATH_DEFINES
#include<math.h>
#include <fstream>

std::fstream logData;

GLFWwindow* w;
GLUquadricObj* quad;
btDynamicsWorld* world;
btDispatcher* d;
btCollisionConfiguration* colConfig;
btBroadphaseInterface* brph;
btConstraintSolver* slv;

btVector3 cylDims(0.25, 1, 0.25);
btVector3 spokeDims(0.1, 1, 0.1);

float bucketRad = 0.1;
float bucketThick = 0.025;

float dropVelocity = 0;

std::vector<btRigidBody*> entities;
std::vector<btRigidBody*> buckets;

int rateperframe = 5;
float piperadius = 0.2;

float startAngle = 0.0001;

void setup() {
	glfwInit();
	glEnable(GL_DEPTH_TEST);
}

btRigidBody* makeSphere(btVector3 vec3, float mass, float radius) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(vec3);

	btVector3 inertia(0, 0, 0);

	btSphereShape* sph = new btSphereShape(radius);
	sph->calculateLocalInertia(mass, inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo inf(mass, motion, sph, inertia);
	btRigidBody* rb = new btRigidBody(inf);

	world->addRigidBody(rb);
	entities.push_back(rb);
	return rb;
}

btCylinderShape* makeCylinder(float mass, float radius, float width) {
	
	btCylinderShapeZ* cy = new btCylinderShapeZ(btVector3(radius,width,radius));
	
	return cy;
}

btBoxShape* box(float x, float y, float z) {
	btBoxShape* bs = new btBoxShape(btVector3(x, y, z));
	return bs;
}

btRigidBody* makeBucket(btVector3 pos, float mass) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0,0,bucketRad));

	btVector3 inertia(0, 0, 0);

	btCompoundShape* bucket = new btCompoundShape();
	bucket->addChildShape(t,box(bucketRad,bucketRad,bucketThick));

	t.setOrigin(btVector3(0, 0, -bucketRad));
	bucket->addChildShape(t, box(bucketRad, bucketRad, bucketThick));

	t.setOrigin(btVector3(-bucketRad, 0, 0));
	bucket->addChildShape(t, box(bucketThick, bucketRad, bucketRad));

	t.setOrigin(btVector3(bucketRad, 0, 0));
	bucket->addChildShape(t, box(bucketThick, bucketRad, bucketRad));

	t.setOrigin(btVector3(0, -bucketRad, 0));
	bucket->addChildShape(t, box(bucketRad / 1.5, bucketThick, bucketRad));

	bucket->calculateLocalInertia(mass, inertia);

	t.setIdentity();
	t.setOrigin(pos);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo inf(mass, motion, bucket, inertia);
	btRigidBody* bucketBody = new btRigidBody(inf);

	world->addRigidBody(bucketBody);

	buckets.push_back(bucketBody);
	return bucketBody;
}

btBoxShape* makeSpokes(float mass, float sq, float height) {
	btBoxShape* bs = new btBoxShape(btVector3(sq, height, sq));
	return bs;
}

btRigidBody* makeWheel(btVector3 vec3, float mass) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0,0,0));

	btVector3 inertia(0, 0, 0);

	btCompoundShape* wheel = new btCompoundShape();
	wheel->addChildShape(t, makeCylinder(1,cylDims.x(),cylDims.y()));

	t.setIdentity();
	t.setOrigin(btVector3(0, 0, 0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));

	t.setIdentity();
	t.setOrigin(btVector3(0, 0, -0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));

	t.setRotation(btQuaternion(btVector3(0,0,1),M_PI/3));
	t.setOrigin(btVector3(0, 0, 0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));

	t.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI / 3));
	t.setOrigin(btVector3(0, 0, -0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));

	t.setRotation(btQuaternion(btVector3(0, 0, 1), 2*M_PI / 3));
	t.setOrigin(btVector3(0, 0, 0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));

	t.setRotation(btQuaternion(btVector3(0, 0, 1), 2*M_PI / 3));
	t.setOrigin(btVector3(0, 0, -0.5));
	wheel->addChildShape(t, makeSpokes(1, spokeDims.x(), spokeDims.y()));


	wheel->calculateLocalInertia(mass, inertia);

	t.setRotation(btQuaternion(0, 0, startAngle));
	t.setOrigin(vec3);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo inf(mass, motion, wheel, inertia);
	btRigidBody* wheelBody = new btRigidBody(inf);
	wheelBody->setLinearFactor(btVector3(0, 0, 0));
	wheelBody->setAngularFactor(btVector3(0, 0, 1));

	world->addRigidBody(wheelBody);
	entities.push_back(wheelBody);
	return wheelBody;
}

void renderSphere(btRigidBody* sphere) {
	if (sphere->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE) {
		return;
	}
	else {
		glColor3f(0, 1, 1);
		float rad = ((btSphereShape*)sphere->getCollisionShape())->getRadius();
		btTransform t;
		sphere->getMotionState()->getWorldTransform(t);
		float mat[16];
		t.getOpenGLMatrix(mat);
		glPushMatrix();
		glMultMatrixf(mat);
		gluSphere(quad, rad, 4, 4);
		glPopMatrix();
		
	}
}

void renderDaBaby(btRigidBody* cs) {
	if (cs->getCollisionShape()->getShapeType() != COMPOUND_SHAPE_PROXYTYPE) {
		return;
	}

	btCompoundShape* bcs = (btCompoundShape*)cs->getCollisionShape();
	bcs->getNumChildShapes();
	for (int j = 0; j < bcs->getNumChildShapes(); j++) {
		float mat[16];
		float mat2[16];
		bcs->getChildList()[j].m_transform.getOpenGLMatrix(mat);
		

		btTransform t;
		cs->getMotionState()->getWorldTransform(t);
		t.getOpenGLMatrix(mat2);

		if (bcs->getChildList()[j].m_childShapeType == CYLINDER_SHAPE_PROXYTYPE) {
			
			glPushMatrix();
			glMultMatrixf(mat2);
			glMultMatrixf(mat);
			glColor3f(1, 1, 0);
			gluCylinder(quad, cylDims.x(), cylDims.z(), cylDims.y(), 10, 1);
			glPopMatrix();
		}else if (bcs->getChildList()[j].m_childShapeType == BOX_SHAPE_PROXYTYPE) {

			glPushMatrix();
			glMultMatrixf(mat2);
			glMultMatrixf(mat);
			glRotatef(-90, 1, 0, 0);
			glTranslatef(0, 0, -spokeDims.y());
			if (bcs->getChildList()[j].m_transform.getOrigin().z() < 0) {
				glColor3f(0, 1, 0);
			}
			else {
				glColor3f(1, 0, 0);
			}
			gluCylinder(quad,0.05,0.05,1*2,4,1);
			glPopMatrix();
		}

		
	}
}

double renderBuckets(btRigidBody* bck) {

	float mat2[16];
		btTransform t;
		bck->getMotionState()->getWorldTransform(t);
		
		t.getOpenGLMatrix(mat2);

			glPushMatrix();
			glMultMatrixf(mat2);
			glRotatef(-90, 1, 0, 0);
			glColor3f(1, 0, 1);
			gluCylinder(quad, bucketRad, bucketRad, bucketRad*2, 4, 1);
			glPopMatrix();
		
			return atan(t.getOrigin().x() / t.getOrigin().y());
}


void physicsInit() {
	colConfig = new btDefaultCollisionConfiguration();
	d = new btCollisionDispatcher(colConfig);
	brph = new btDbvtBroadphase();
	slv = new btSequentialImpulseConstraintSolver();

	world = new btDiscreteDynamicsWorld(d, brph, slv, colConfig);
	world->setGravity(btVector3(0,-9.81,0));

	logData.open("log.csv", std::ios::out);
}



void render() {
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity();

	for (int i = 0; i < entities.size(); i++) {
		if (entities[i]->getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE) {
			
			renderSphere(entities[i]);
		}
		else {
			renderDaBaby(entities[i]);
		}
	}

	for (int i = 0; i < buckets.size(); i++) {
		double angle = renderBuckets(buckets[i]);
		if (i == 0) {
			logData << glfwGetTime() << "," << angle << "\n";
		}
	}

}

void physicsUpdate() {
	for (int i = 0; i < rateperframe; i++) {
		
		makeSphere(btVector3(-piperadius/2 + ((float)rand()/(float)RAND_MAX) * piperadius, 3, -10 - (piperadius / 2) + ((float)rand() / (float)RAND_MAX) * piperadius), 1, 0.01)->setLinearVelocity(btVector3(0,-dropVelocity,0));
	}
	

	for (int i = 0; i < entities.size(); i++) {
		btVector3 v;
		v = entities[i]->getWorldTransform().getOrigin();

		if (v.y() < -3) {
			world->removeCollisionObject(entities[i]);
			btMotionState* ms = entities[i]->getMotionState();
			btCollisionShape* cs = entities[i]->getCollisionShape();
			
			delete entities[i];
			entities.erase(entities.begin()+i);

			delete ms;
			
			delete cs;

			
		}
	}
}



int main() {

	setup();

	w = glfwCreateWindow(640, 640, "ChaosLab", NULL, NULL);
	glfwMakeContextCurrent(w);

	glViewport(0, 0, 640, 640);

	glMatrixMode(GL_PROJECTION); //Switch to setting the camera perspective

//Set the camera perspective

	glLoadIdentity(); //reset the camera

	gluPerspective(45.0f,                      //camera angle

		640 / 640, //The width to height ratio

		1.0f,                          //The near z clipping coordinate

		100.0f);

	quad = gluNewQuadric();

	glClearColor(0, 0, 0, 0);

	physicsInit();

	btRigidBody* r = makeWheel(btVector3(0, 0, -10), 1);

	
	
	for (int i = 0; i < 6; i++) {
		
		btRigidBody* r2 = makeBucket(btVector3(spokeDims.y() * sin(i * 60 * 180 / M_PI), spokeDims.y() * cos(i * 60 * 180 / M_PI), -10.5), 1);

		btTypedConstraint* tc = new btPoint2PointConstraint(*r2, *r, btVector3(0, bucketRad, -bucketRad), btVector3(spokeDims.y() * sin(i * M_PI/3), spokeDims.y() * cos(i * M_PI/3), -bucketRad));
		world->addConstraint(tc);

		btTypedConstraint* tc2 = new btPoint2PointConstraint(*r2, *r, btVector3(0, bucketRad, bucketRad), btVector3(spokeDims.y() * sin(i * M_PI/3), spokeDims.y() * cos(i * M_PI/3), bucketRad));
		world->addConstraint(tc2);
	}


	double lastTime = glfwGetTime();
	while (!glfwWindowShouldClose(w)) {

		physicsUpdate();

		render();

		

		world->stepSimulation(1);
		
		glfwPollEvents();
		glfwSwapBuffers(w);

		//std::cout << 1/(glfwGetTime()-lastTime) << std::endl;

		while (glfwGetTime() < lastTime + 1/30) {

		} 

		lastTime = glfwGetTime();
	}

	gluDeleteQuadric(quad);

	
	delete world;
	delete d;
	delete colConfig;
	delete slv;
	delete brph;

	glfwDestroyWindow(w);
	glfwTerminate();

	return 0;
}