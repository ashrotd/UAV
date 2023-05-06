#include "WODronePhysx.h"
#include "WO.h"
#include <vehicle/PxVehicleUtil.h>
#include "extensions/PxRigidBodyExt.h"
#include "Model.h"
using namespace Aftr;


WODronePhysx* WODronePhysx::New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x)
{
	WODronePhysx* glv = new WODronePhysx();

	glv->onCreate(p, s, path, scale, x);
	return glv;
}

WODronePhysx::~WODronePhysx()
{

}

WODronePhysx::WODronePhysx() :WO(), IFace(this) {

}

void WODronePhysx::onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x)
{
	WO::onCreate(path, scale);
	
	
	physx::PxMaterial* material = p->createMaterial(0.5f, 0.5f, 0.6f);
	physx::PxShape* shape = p->createShape(PxBoxGeometry(4.0f, 2.0f, 0.5f), *material);
	physx::PxTransform t({ 0,0,x});
	actor = p->createRigidDynamic(t);
	actor->attachShape(*shape);
	
	actor->userData = this;
	s->addActor(*actor);
}

void WODronePhysx::updatePoseFromPhysx()

{

	physx::PxMat44 m(this->actor->getGlobalPose().q);
	Aftr::Mat4 m2;
	for (int i = 0; i < 16; i++) {
		m2[i] = m(i % 4, i / 4);
	}
	this->setDisplayMatrix(m2);
	this->setPosition(this->actor->getGlobalPose().p.x, this->actor->getGlobalPose().p.y, this->actor->getGlobalPose().p.z);

}


void WODronePhysx::physxSetPosition(float x, float y, float z) {
	WO::setPosition(x, y, z);
	physx::PxTransform t = this->actor->getGlobalPose();
	this->actor->setGlobalPose(t);
}

void WODronePhysx::addForces(float x, float y, float z) {
	actor->addForce(physx::PxVec3(100*z, 100*y, 100*z), physx::PxForceMode::eACCELERATION);
}