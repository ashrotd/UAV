#include "PhysxPlane.h"
#include "WO.h"

using namespace Aftr;


PhysxPlane* PhysxPlane::New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale)
{
	PhysxPlane* glv = new PhysxPlane();

	glv->onCreate(p, s, path, scale);
	return glv;
}

PhysxPlane::~PhysxPlane()
{

}

PhysxPlane::PhysxPlane() :WO(), IFace(this) {

}

void PhysxPlane::onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale)
{
	WO::onCreate(path, scale, MESH_SHADING_TYPE::mstFLAT);
	//Creating Physx Plane
	physx::PxMaterial* mMaterial = p->createMaterial(0.5f, 0.9f, 0.6f);
	this->actor = PxCreatePlane(*p, PxPlane(0, 0, 1, 0), *mMaterial);
	s->addActor(*actor);
}


void PhysxPlane::updatePoseFromPhysx()
{
	physx::PxMat44 m(this->actor->getGlobalPose().q);
	Aftr::Mat4 m2;
	for (int i = 0; i < 16; i++) {
		m2[i] = m(i % 4, i / 4);
	}
	this->setDisplayMatrix(m2);
	this->setPosition(this->actor->getGlobalPose().p.x, this->actor->getGlobalPose().p.y, this->actor->getGlobalPose().p.z);

}


void PhysxPlane::physxSetPosition(float x, float y, float z) {
	WO::setPosition(x, y, z);
	physx::PxTransform t = this->actor->getGlobalPose();
	t.p = physx::PxVec3(x, y, z);
	this->actor->setGlobalPose(t);
}


