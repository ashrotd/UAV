#pragma once
#include "PxPhysicsAPI.h"
#include "PxPhysics.h"
#include "extensions\PxDefaultErrorCallback.h"
#include "extensions\PxDefaultAllocator.h"
#include "GLView.h"
#include "WO.h"
#include "cooking/PxCooking.h"
#include <cstddef>

using namespace physx;
using namespace Aftr;

class WOCookingPhysx :public WO {
public:
	static WOCookingPhysx* New(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, const std::string& path, const Vector& scale);
	virtual ~WOCookingPhysx();


protected:
	WOCookingPhysx();
	virtual void onCreate(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, const std::string& path, const Vector& scale);
	PxRigidStatic* actor = nullptr;
	size_t vertexListSize;
	size_t indexListSize;
	float* vertexListCopy = nullptr;
	unsigned int* indicesCopy = nullptr;

};
