#pragma once
#include "WOGridECEFElevation.h"
#include "PxPhysicsAPI.h"
#include "Model.h"

using namespace Aftr;
using namespace physx;

class WOTerrainPhysx : public WOGridECEFElevation {
public:
    WOTerrainPhysx();
    virtual ~WOTerrainPhysx();

    static WOTerrainPhysx* New(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, VectorD upperLeft, VectorD lowerRight, Aftr::VectorD offset, VectorD scale, const std::string& path, const std::string& layer);

    virtual void onCreate(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, VectorD upperLeft, VectorD lowerRight, Aftr::VectorD offset, VectorD scale, const std::string& path, const std::string& layer);

    physx::PxRigidStatic* actor = nullptr;

private:
    float* vertexListCopy = nullptr;
    unsigned int* indiciesCopy = nullptr;
};
