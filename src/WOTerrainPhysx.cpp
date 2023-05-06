#include "WOTerrainPhysx.h"


using namespace Aftr;

WOTerrainPhysx* WOTerrainPhysx::New(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, VectorD upperLeft, VectorD lowerRight, VectorD offset, VectorD scale, const std::string& path, const std::string& layer)
{
    WOTerrainPhysx* grid = new WOTerrainPhysx();
    grid->onCreate(p, s, c, upperLeft, lowerRight, offset, scale, path, layer);
    return grid;
}

WOTerrainPhysx::~WOTerrainPhysx()
{

}


WOTerrainPhysx::WOTerrainPhysx() :IFace(this)
{

}



void WOTerrainPhysx::onCreate(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, VectorD upperLeft, VectorD lowerRight,VectorD offset, VectorD scale, const std::string& path, const std::string& layer)
{
    this->WOGridECEFElevation::onCreate(upperLeft, lowerRight, 0, offset, scale, path, 0, true, 0);
    VectorD ll = ((upperLeft + lowerRight) / 2.0);
       ll.z = 0.0;
       VectorD zdir = ll.toECEFfromWGS84().normalizeMe();
       VectorD northPoleECEF = VectorD(90.0, 0.0, 0.0).toECEFfromWGS84();
       VectorD xdir = northPoleECEF - ll.toECEFfromWGS84();
       xdir = xdir.vectorProjectOnToPlane(zdir);
       xdir.normalize();
       VectorD ydir = zdir.crossProduct(xdir);
       ydir.normalize();
       
       float localBodySpaceToLTP[16];

       localBodySpaceToLTP[0] = (float)xdir.x; localBodySpaceToLTP[4] = (float)ydir.x; localBodySpaceToLTP[8] = (float)zdir.x; localBodySpaceToLTP[12] = (float)0;
       localBodySpaceToLTP[1] = (float)xdir.y; localBodySpaceToLTP[5] = (float)ydir.y; localBodySpaceToLTP[9] = (float)zdir.y; localBodySpaceToLTP[13] = (float)0;
       localBodySpaceToLTP[2] = (float)xdir.z; localBodySpaceToLTP[6] = (float)ydir.z; localBodySpaceToLTP[10] = (float)zdir.z; localBodySpaceToLTP[14] = (float)0;
       localBodySpaceToLTP[3] = (float)0;      localBodySpaceToLTP[7] = (float)0;      localBodySpaceToLTP[11] = (float)0;      localBodySpaceToLTP[15] = (float)1.0f;


       this->getModel()->setDisplayMatrix(Mat4(localBodySpaceToLTP).transposeUpperLeft3x3());
         this->upon_async_model_loaded([this, p,s,c,layer] {

        for (size_t i = 0; i < this->getModel()->getModelDataShared()->getModelMeshes().size(); i++)
        {
            this->getModel()->getModelDataShared()->getModelMeshes().at(i)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(layer);
            this->getModel()->isUsingBlending(false);

        }


        size_t vertexListSize = this->getModel()->getModelDataShared()->getCompositeVertexList().size();
        size_t indexListSize = this->getModel()->getModelDataShared()->getCompositeIndexList().size();

        this->vertexListCopy = new float[vertexListSize * 3];
        this->indiciesCopy = new unsigned int[indexListSize];


        for (size_t i = 0; i < vertexListSize; i++) {
            this->vertexListCopy[i * 3 + 0] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).x;
            this->vertexListCopy[i * 3 + 1] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).y;
            this->vertexListCopy[i * 3 + 2] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).z;
        }
        for (size_t i = 0; i < indexListSize; i++) {
            indiciesCopy[i] = this->getModel()->getModelDataShared()->getCompositeIndexList().at(i);
        }

        physx::PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = vertexListSize;
        meshDesc.points.stride = sizeof(float) * 3;
        meshDesc.points.data = this->vertexListCopy;
        meshDesc.triangles.count = indexListSize / 3;
        meshDesc.triangles.stride = 3 * sizeof(unsigned int);
        meshDesc.triangles.data = this->indiciesCopy;


        physx::PxDefaultMemoryOutputStream writeBuffer;
        physx::PxTriangleMeshCookingResult::Enum result;
        bool status = c->cookTriangleMesh(meshDesc, writeBuffer, &result);
        if (!status) {
            std::cout << "Failed to create triangular mesh" << std::endl;
            std::cin.get();
        }

        physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        physx::PxTriangleMesh* mesh = p->createTriangleMesh(readBuffer);

        physx::PxMaterial* gMaterial = p->createMaterial(0.5f, 0.5f, 0.6f);
        physx::PxShape* shape = p->createShape(physx::PxTriangleMeshGeometry(mesh), *gMaterial, true);
        
        auto model_pos = this->getModel()->getPose();

        int k = 0;
        float mat[16];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat[k] = model_pos.at(i, j);
                k++;
            }
        }
        auto t = physx::PxTransform(physx::PxMat44(mat));

        auto rigid_actor = p->createRigidStatic(t);
        bool body = rigid_actor->attachShape(*shape);

        this->actor = rigid_actor;
        this->actor->userData = this;
        
        s->addActor(*actor);

        });
}