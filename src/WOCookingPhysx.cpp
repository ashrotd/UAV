
#include "WOCookingPhysx.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"

using namespace Aftr;


WOCookingPhysx* WOCookingPhysx::New(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, const std::string& path, const Vector& scale)
{
	WOCookingPhysx* glv = new WOCookingPhysx();

	glv->onCreate(p, s, c, path, scale);
	return glv;
}

WOCookingPhysx::~WOCookingPhysx()
{

}

WOCookingPhysx::WOCookingPhysx() :WO(), IFace(this) {

}

void WOCookingPhysx::onCreate(physx::PxPhysics* p, physx::PxScene* s, physx::PxCooking* c, const std::string& path, const Vector& scale)
{
	WO::onCreate(path, scale);


		this->upon_async_model_loaded([this, s, p, c]() {

			size_t vertexListSize = this->getModel()->getModelDataShared()->getCompositeVertexList().size();
			size_t indexListSize = this->getModel()->getModelDataShared()->getCompositeIndexList().size();

			this->vertexListCopy = new float[vertexListSize * 3];//might be a better way to do this without making a copy
			this->indicesCopy = new unsigned int[indexListSize];//assuming the composite lists are stored in contiguous memory

			for (size_t i = 0; i < vertexListSize; i++)
			{
				this->vertexListCopy[i * 3 + 0] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).x;
				this->vertexListCopy[i * 3 + 1] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).y;
				this->vertexListCopy[i * 3 + 2] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).z;
			}
			for (size_t i = 0; i < indexListSize; i++)
				this->indicesCopy[i] = this->getModel()->getModelDataShared()->getCompositeIndexList().at(i);

			PxTriangleMeshDesc meshDesc;
			meshDesc.points.count = vertexListSize;
			meshDesc.points.stride = sizeof(float) * 3;//tightly packaged
			meshDesc.points.data = vertexListCopy;

			meshDesc.triangles.count = indexListSize / 3;
			meshDesc.triangles.stride = 3 * sizeof(unsigned int);//aside about index lists here
			meshDesc.triangles.data = indicesCopy;
			PxDefaultMemoryOutputStream writeBuffer;
			PxTriangleMeshCookingResult::Enum result;
			bool status = c->cookTriangleMesh(meshDesc, writeBuffer, &result);
			if (!status)
			{
				std::cout << "Failed to create Triangular mesh" << std::endl;
				std::cin.get();
			}
			PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
			physx::PxTriangleMesh* mesh = p->createTriangleMesh(readBuffer);

			PxMaterial* gMaterial = p->createMaterial(0.5f, 0.5f, 0.6f);
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


			actor = p->createRigidStatic(t);
			bool b = actor->attachShape(*shape);

			actor->userData = this;
			s->addActor(*actor);


			});
	
}


