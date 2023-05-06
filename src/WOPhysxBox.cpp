#include "WOPhysxBox.h"
#include "WO.h"

using namespace Aftr;


WOPhysxBox* WOPhysxBox::New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x, float y, float z)
{
	WOPhysxBox* glv = new WOPhysxBox();

	glv->onCreate(p, s, path, scale,x,y,z);
	return glv;
}

WOPhysxBox::~WOPhysxBox()
{

}

WOPhysxBox::WOPhysxBox() :WO(), IFace(this) {

}

void WOPhysxBox::onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale,float x, float y, float z)
{
	WO::onCreate(path, scale);
	physx::PxMaterial* material = p->createMaterial(0.5f, 0.5f, 0.6f);
	physx::PxShape* shape = p->createShape(PxBoxGeometry(scale.x * 2, scale.y * 2, scale.z * 2), *material, true);
	PxTransform t = PxTransform(PxVec3(x, y, z));
	actor = p->createRigidDynamic(t);
	actor->attachShape(*shape);

	actor->userData = this;
	s->addActor(*actor);
}


void WOPhysxBox::updatePoseFromPhysx()
{
	physx::PxMat44 m(this->actor->getGlobalPose().q);
	Aftr::Mat4 m2;
	for (int i = 0; i < 16; i++) {
		m2[i] = m(i % 4, i / 4);
	}
	this->setDisplayMatrix(m2);
	this->setPosition(this->actor->getGlobalPose().p.x, this->actor->getGlobalPose().p.y, this->actor->getGlobalPose().p.z);

}


void WOPhysxBox::physxSetPosition(float x, float y, float z) {
	WO::setPosition(x, y, z);
	physx::PxTransform t = this->actor->getGlobalPose();
	t.p = physx::PxVec3(x, y, z);
	this->actor->setGlobalPose(t);
}

void WOPhysxBox::addForces(float x, float y, float z) {
	this->actor->addForce(PxVec3(100*x, 100*y, 100*z), physx::PxForceMode::eIMPULSE);
}
