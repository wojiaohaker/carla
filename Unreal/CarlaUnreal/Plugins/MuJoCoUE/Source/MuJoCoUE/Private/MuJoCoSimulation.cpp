// Fill out your copyright notice in the Description page of Project Settings.

#include "MuJoCoSimulation.h"

#include "mujoco/mujoco.h"
#include <vector>
#include <string>

#include "KismetProceduralMeshLibrary.h"
#include "ProceduralMeshConversion.h"
#include "UObject/SoftObjectPath.h"

// MuJoCo (右手系, Z-up) → UE (左手系, Y-up) 坐标转换
// X: 米→厘米, Y: 翻转并米→厘米, Z: 米→厘米
static inline FVector MujocoToUE(mjtNum x, mjtNum y, mjtNum z)
{
	return FVector(x * 100.0f, -y * 100.0f, z * 100.0f);
}

// MuJoCo 四元数 [w,x,y,z] → UE 四元数 [x,y,z,w] + Y轴翻转修正
static inline FQuat MujocoQuatToUE(const mjtNum q[4])
{
	// MuJoCo: [w, x, y, z], UE: [x, y, z, w]
	// Y轴翻转 (右手→左手) 需要翻转 x 和 z 分量
	return FQuat(-q[1], q[2], -q[3], q[0]);
}

FVector CalculateWorldPosition(const FVector &BaseLocation, const FQuat &BaseRotation, const FVector &RelativeLocation)
{

	return BaseLocation + BaseRotation.RotateVector(RelativeLocation);
}

FQuat CalculateWorldRotation(const FQuat &BaseRotation, const FQuat &RelativeRotation)
{

	return RelativeRotation * BaseRotation;
}
ModelInfo ExtractModelInfo(const mjModel *m)
{

	ModelInfo modelInfo;

	// Extract body information
	for (int i = 0; i < m->nbody; ++i)
	{
		BodyInfo bodyInfo;
		bodyInfo.name = std::string(m->names + m->name_bodyadr[i]);
		std::copy(m->body_pos + 3 * i, m->body_pos + 3 * (i + 1), bodyInfo.pos);
		std::copy(m->body_quat + 4 * i, m->body_quat + 4 * (i + 1), bodyInfo.quat);
		bodyInfo.parent_id = m->body_parentid[i];
		bodyInfo.quat2 = MujocoQuatToUE(bodyInfo.quat);
		modelInfo.bodies.push_back(bodyInfo);
	}

	// Extract geom information
	for (int i = 0; i < m->ngeom; ++i)
	{
		GeomInfo geomInfo;
		geomInfo.name = std::string(m->names + m->name_geomadr[i]);
		geomInfo.body_id = m->geom_bodyid[i];
		geomInfo.type = m->geom_type[i];
		std::copy(m->geom_size + 3 * i, m->geom_size + 3 * (i + 1), geomInfo.size);
		std::copy(m->geom_pos + 3 * i, m->geom_pos + 3 * (i + 1), geomInfo.pos);
		std::copy(m->geom_quat + 4 * i, m->geom_quat + 4 * (i + 1), geomInfo.quat);
		geomInfo.quat2 = MujocoQuatToUE(geomInfo.quat);
		// Check if this geom has material or texture information
		if (m->geom_matid[i] >= 0)
		{
			int matid = m->geom_matid[i];
			geomInfo.color = FLinearColor(
				m->mat_rgba[matid * 4 + 0],
				m->mat_rgba[matid * 4 + 1],
				m->mat_rgba[matid * 4 + 2],
				m->mat_rgba[matid * 4 + 3]);
	
	      
			if (m->mat_texid[matid] >= 0)
			{
				int texid = m->mat_texid[matid];
				// Texture exists, but we'll use geom color as base
				geomInfo.color = FLinearColor(
					m->geom_rgba[i * 4 + 0],
					m->geom_rgba[i * 4 + 1],
					m->geom_rgba[i * 4 + 2],
					m->geom_rgba[i * 4 + 3]);
		
				//	geomInfo.texId = texid;
			}
			
		}
		// Otherwise use geom-specific RGBA color
		else
		{
			geomInfo.color = FLinearColor(
				m->geom_rgba[i * 4 + 0],
				m->geom_rgba[i * 4 + 1],
				m->geom_rgba[i * 4 + 2],
				m->geom_rgba[i * 4 + 3]);
		}
		
		// Adjust size to be used as scale
		//  assume all primitive meshes Have 1 meter size -> 100 cm in UE
		switch (geomInfo.type)
		{

		case mjGEOM_CYLINDER:
			geomInfo.size[0] *= 2;
			geomInfo.size[2] = geomInfo.size[1] * 2;
			geomInfo.size[1] = geomInfo.size[0];
			break;
		case mjGEOM_CAPSULE:

			geomInfo.size[2] = geomInfo.size[1] + geomInfo.size[0];
			geomInfo.size[0] *= 2;
			geomInfo.size[1] = geomInfo.size[0];
			break;
		case mjGEOM_SPHERE:
			geomInfo.size[0] *= 2;
			geomInfo.size[1] = geomInfo.size[0];
			geomInfo.size[2] = geomInfo.size[0];
			break;
		case mjGEOM_BOX:
			geomInfo.size[0] *= 2;
			geomInfo.size[1] *= 2;
			geomInfo.size[2] *= 2;
			break;
		case mjGEOM_ELLIPSOID:
			geomInfo.size[0] *= 2;
			geomInfo.size[1] *= 2;
			geomInfo.size[2] *= 2;
			break;
			break;
		}
		modelInfo.geoms.push_back(geomInfo);
	}

	return modelInfo;
}
void AMuJoCoSimulation::GenerateMeshes(ModelInfo &modelInfo)
{

	BodyMap.Empty();
	GeomMap1.Empty();

	// Generate body componenets
	int BodyId = 0;
	for (const BodyInfo &bodyInfo : modelInfo.bodies)
	{

		USceneComponent *sceneComponent = NewObject<USceneComponent>(this, FName(*(FString(bodyInfo.name.c_str()) + *FString::Printf(TEXT("_Body%d"), BodyId))));

		BodyMap.Add(BodyId++, sceneComponent);
		sceneComponent->RegisterComponent();
		sceneComponent->SetRelativeLocation(MujocoToUE(bodyInfo.pos[0], bodyInfo.pos[1], bodyInfo.pos[2]));
		sceneComponent->SetRelativeRotation(bodyInfo.quat2);
		if (bodyInfo.parent_id == 0)
			sceneComponent->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		else
		{
			USceneComponent *parentComponent = BodyMap[bodyInfo.parent_id];
			sceneComponent->AttachToComponent(parentComponent, FAttachmentTransformRules::KeepRelativeTransform);
		}
	}

	// Generate geom meshes
	int GeomId = 0;
	for (GeomInfo &geomInfo : modelInfo.geoms)
	{
		// Create a new mesh component
		UStaticMeshComponent *staticMeshComponent = NewObject<UStaticMeshComponent>(this);//, FName(*(FString(geomInfo.name.c_str()) + *FString::Printf(TEXT("_Geom%d"), BodyId))));
		staticMeshComponent->RegisterComponent();
		geomInfo.posAdjust[2] = geomInfo.size[2] * -50;
		staticMeshComponent->SetRelativeLocation(MujocoToUE(geomInfo.pos[0], geomInfo.pos[1], geomInfo.pos[2])); //+geomInfo.posAdjust[2]
		staticMeshComponent->SetRelativeRotation(geomInfo.quat2);
		staticMeshComponent->AttachToComponent(this->BodyMap[geomInfo.body_id], FAttachmentTransformRules::KeepRelativeTransform);
		;
		// Get mesh for this geometry
		auto *mesh = MeshAssets.Find(geomInfo.type) ? MeshAssets[geomInfo.type] : nullptr;
		// Generate Procedural Mesh if type = mesh
		if (!mesh)
		{
			if (geomInfo.type == mjGEOM_MESH && mModel->geom_dataid[GeomId] != -1)
			{
				int meshId = mModel->geom_dataid[GeomId];
				if (meshId >= 0 && meshId < ProceduralMeshes.Num())
				{
					UProceduralMeshComponent *procMesh = ProceduralMeshes[meshId];

					const FMeshDescription description = BuildMeshDescription(procMesh);
					TArray<const FMeshDescription *> descs;
					descs.Add(&description);

					UStaticMesh *NewStaticMesh = NewObject<UStaticMesh>(staticMeshComponent);
					NewStaticMesh->AddMaterial(procMesh->GetMaterial(0));
					NewStaticMesh->BuildFromMeshDescriptions(descs);
					staticMeshComponent->SetStaticMesh(NewStaticMesh);
					mesh = NewStaticMesh;
					if (mesh)
					{
						geomInfo.size[0] = 1;
						geomInfo.size[1] = 1;
						geomInfo.size[2] = 1;
					}
				}
			}
			if (!mesh)
				mesh = defaultMesh;
		}

		staticMeshComponent->SetStaticMesh(mesh);
		SetMeshColor(staticMeshComponent, geomInfo.color);
		staticMeshComponent->SetSimulatePhysics(false);
		staticMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		staticMeshComponent->SetWorldScale3D(FVector(geomInfo.size[0], geomInfo.size[1], geomInfo.size[2]));

		this->GeomMap1.Add(GeomId++, staticMeshComponent);
	}
}

void AMuJoCoSimulation::ExtractCurrentState(ModelInfo &info)
{

	for (int i = 0; i < mModel->nbody; ++i)
	{
		// Get positional data from global coordinates (xpos and xquat)
		BodyInfo bodyInfo;
		std::copy(mData->xpos + 3 * i, mData->xpos + 3 * (i + 1), info.bodies[i].pos);
		std::copy(mData->xquat + 4 * i, mData->xquat + 4 * (i + 1), info.bodies[i].quat);
		info.bodies[i].quat2 = MujocoQuatToUE(info.bodies[i].quat);
	}

	// Update geom states
	for (int i = 0; i < mModel->ngeom; ++i)
	{   GeomInfo& geomInfo=info.geoms[i];
		std::copy(mData->geom_xpos + 3 * i, mData->geom_xpos + 3 * (i + 1), info.geoms[i].pos);
		// Convert rotation matrix to quaternion
		const mjtNum *mat = mData->geom_xmat + 9 * i;
		mjtNum quat[4];
		mju_mat2Quat(quat, mat);

		geomInfo.quat2 = MujocoQuatToUE(quat);
	}
}

AMuJoCoSimulation::AMuJoCoSimulation()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bCanEverTick = true;
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}
	mData = nullptr;
	mModel = nullptr;
	bSimulationRunning = false;
}

void AMuJoCoSimulation::BeginPlay()
{
	Super::BeginPlay();
	mData = nullptr;
	mModel = nullptr;
	LoadModel(XmlSourcePath);
	if (mModel)
	{
		// Set initial FLAT pose (Matrix Image 1: belly down, legs splayed outward)
		if (mModel->nq >= 19) // 7 (freejoint) + 12 joints
		{
			mData->qpos[2] = 0.15; // body z: flat on ground

			// Joint order: FAR(ABAD,HIP,KNEE), FBL, RAR, RBL
			// ABAD sign: right legs (FAR=0, RAR=2) negative, left legs (FBL=1, RBL=3) positive
			const float AbadInit[4] = { -0.4f, 0.4f, -0.4f, 0.4f };
			for (int j = 0; j < 4; j++)
			{
				mData->qpos[7 + j*3 + 0] = AbadInit[j]; // ABAD splayed
				mData->qpos[7 + j*3 + 1] = 0.2f;        // HIP slightly bent
				mData->qpos[7 + j*3 + 2] = -0.4f;       // KNEE slightly bent
			}
		}
		mj_forward(mModel, mData);

		_info = ExtractModelInfo(mModel);
		ConvertMuJoCoModelToProceduralMeshes(mModel, this);
		GenerateMeshes(_info);

		// Start simulation immediately (passive mode, robot stays in crouch)
		StartSimulation();

		UE_LOG(LogTemp, Warning, TEXT("MuJoCo: Started in crouch pose. Press U=Stand, Space=LieDown."));
	}
}

void AMuJoCoSimulation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{

	if (mData)
		mj_deleteData(mData);

	if (mModel)
		mj_deleteModel(mModel);
	mData = nullptr;
	mModel = nullptr;
	Super::EndPlay(EndPlayReason);
}

void AMuJoCoSimulation::UpdateSimulationView(const ModelInfo &Info)
{
	FVector BaseLocation = BodyMap[0]->GetComponentLocation();
	FQuat BaseRotation = BodyMap[0]->GetComponentRotation().Quaternion(); // GetActorRotation().Quaternion();

	int BodyId = 0;
	for (const BodyInfo &bodyInfo : Info.bodies)
	{

		USceneComponent *sceneComponent = BodyMap[BodyId];
		if (!sceneComponent)
			continue;
		FVector WorldLoc = CalculateWorldPosition(BaseLocation, BaseRotation, MujocoToUE(bodyInfo.pos[0], bodyInfo.pos[1], bodyInfo.pos[2]));
		sceneComponent->SetWorldLocation(WorldLoc);
		FQuat worldRot = CalculateWorldRotation(BaseRotation, bodyInfo.quat2);
		sceneComponent->SetWorldRotation(bodyInfo.quat2 /*worldRot*/);

		//	sceneComponent->SetRelativeLocation(FVector(bodyInfo.pos[0]*100, bodyInfo.pos[1]*100, bodyInfo.pos[2]*100));
		//		sceneComponent->SetRelativeRotation(bodyInfo.quat2);

		//	UE_LOG(LogTemp, Warning, TEXT("Body %d [%hs][%f]: %f %f %f"), BodyId,bodyInfo.name.c_str(), mData->time,bodyInfo.pos[0], bodyInfo.pos[1], bodyInfo.pos[2]);
		//	UE_LOG(LogTemp, Warning, TEXT("Body %d [%hs][%f]: %f %f %f %f"), BodyId,bodyInfo.name.c_str(), mData->time,bodyInfo.quat[1], bodyInfo.quat[2], bodyInfo.quat[3], bodyInfo.quat[0]);
		BodyId++;
	}

	int GeomId = 0;
	for (const GeomInfo &geomInfo : Info.geoms)
	{

		UStaticMeshComponent *staticMeshComponent = GeomMap1[GeomId];
		if (!staticMeshComponent)
			continue;

		//	staticMeshComponent->SetRelativeLocation(FVector(geomInfo.pos[0]*100, geomInfo.pos[1]*100, geomInfo.pos[2]*100));
		//	staticMeshComponent->SetRelativeRotation(geomInfo.quat2);

		FVector WorldLoc = CalculateWorldPosition(BaseLocation, BaseRotation, MujocoToUE(geomInfo.pos[0], geomInfo.pos[1], geomInfo.pos[2]));
		staticMeshComponent->SetWorldLocation(WorldLoc);
		FQuat worldRot = CalculateWorldRotation(BaseRotation, geomInfo.quat2);
		//	staticMeshComponent->SetWorldRotation(geomInfo.quat2/*worldRot*/);

		// UE_LOG(LogTemp, Warning, TEXT("Geom %d[%hs][%f]: %f %f %f"), GeomId,geomInfo.name.c_str() ,mData->time,geomInfo.pos[0], geomInfo.pos[1], geomInfo.pos[2]);
		// UE_LOG(LogTemp, Warning, TEXT("Geom %d[%hs][%f]: %f %f %f %f"), GeomId,geomInfo.name.c_str() ,mData->time,geomInfo.quat[1], geomInfo.quat[2], geomInfo.quat[3], geomInfo.quat[0]);
		GeomId++;
	}
}

void AMuJoCoSimulation::SimulateMuJoCo(float DeltaTime)
{
	if (mData == nullptr || mModel == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Model or data is null"));
		return;
	}
	double startTime = mData->time;
	while (mData->time - startTime < DeltaTime)
	{
		// Apply PD stand-up control before each physics step
		if (bStandUpActive)
			ApplyStandUpControl();
		mj_step(mModel, mData);
	}

	ModelInfo info;
	if (!_info.bodies.size())
		return;
	ExtractCurrentState(_info);

	UpdateSimulationView(_info);
}

bool AMuJoCoSimulation::LoadModel(FString Xml)
{
	FString FullPath = FPaths::Combine(FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir()), Xml);
	if (!FPaths::FileExists(FullPath))
	{
		UE_LOG(LogTemp, Error, TEXT("File does not exist: %s"), *FullPath);
		return false;
	}
	mModel = mj_loadXML(TCHAR_TO_ANSI(*FullPath), NULL, NULL, 0);
	if (!mModel)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to load model from %s"), *Xml);
		return false;
	}
	mData = mj_makeData(mModel);
	if (!mData)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to make data for model"));
		return false;
	}
	return true;
}

// Called every frame
void AMuJoCoSimulation::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (bSimulationRunning)
		SimulateMuJoCo(DeltaTime);
}

void AMuJoCoSimulation::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Stand / Lie down
	PlayerInputComponent->BindAction("StandUp", IE_Pressed, this, &AMuJoCoSimulation::RequestStandUp);
	PlayerInputComponent->BindAction("LieDown", IE_Pressed, this, &AMuJoCoSimulation::RequestLieDown);

	// Walk directions (pressed/released)
	PlayerInputComponent->BindAction("MoveForward", IE_Pressed, this, &AMuJoCoSimulation::OnMoveForwardPressed);
	PlayerInputComponent->BindAction("MoveForward", IE_Released, this, &AMuJoCoSimulation::OnMoveForwardReleased);
	PlayerInputComponent->BindAction("MoveBackward", IE_Pressed, this, &AMuJoCoSimulation::OnMoveBackwardPressed);
	PlayerInputComponent->BindAction("MoveBackward", IE_Released, this, &AMuJoCoSimulation::OnMoveBackwardReleased);
	PlayerInputComponent->BindAction("StrafeRight", IE_Pressed, this, &AMuJoCoSimulation::OnStrafeRightPressed);
	PlayerInputComponent->BindAction("StrafeRight", IE_Released, this, &AMuJoCoSimulation::OnStrafeRightReleased);
	PlayerInputComponent->BindAction("StrafeLeft", IE_Pressed, this, &AMuJoCoSimulation::OnStrafeLeftPressed);
	PlayerInputComponent->BindAction("StrafeLeft", IE_Released, this, &AMuJoCoSimulation::OnStrafeLeftReleased);
	PlayerInputComponent->BindAction("TurnRight", IE_Pressed, this, &AMuJoCoSimulation::OnTurnRightPressed);
	PlayerInputComponent->BindAction("TurnRight", IE_Released, this, &AMuJoCoSimulation::OnTurnRightReleased);
	PlayerInputComponent->BindAction("TurnLeft", IE_Pressed, this, &AMuJoCoSimulation::OnTurnLeftPressed);
	PlayerInputComponent->BindAction("TurnLeft", IE_Released, this, &AMuJoCoSimulation::OnTurnLeftReleased);

	UE_LOG(LogTemp, Warning, TEXT("[MuJoCo] Input bindings registered: U=Stand, Space=Lie, WASD=Move, QE=Turn"));
}

void AMuJoCoSimulation::SetControl(int Id, float Value)
{
	if (!mData || !mModel || mModel->nu <= Id)
		return;
	mData->ctrl[Id] = Value;
}
void AMuJoCoSimulation::StartSimulation()
{
	bSimulationRunning = true;
}

void AMuJoCoSimulation::PauseSimulation()
{
	bSimulationRunning = false;
}

void AMuJoCoSimulation::ResetSimulation()
{
	bSimulationRunning = false;
	if (mData)
		mj_deleteData(mData);
	mData = mj_makeData(mModel);
	ExtractCurrentState(_info);
	UpdateSimulationView(_info);
}

void AMuJoCoSimulation::StepSimulation()
{
	LogInfo();
	mj_step(mModel, mData);
	ExtractCurrentState(_info);
	UpdateSimulationView(_info);
	LogInfo();
}

void AMuJoCoSimulation::LogInfo()
{
	// Log body information
	int BodyId = 0;
	for (const auto &bodyInfo : _info.bodies)
	{
		if (USceneComponent *bodyComponent = BodyMap[BodyId])
		{
			FVector worldLoc = bodyComponent->GetComponentLocation();
			FRotator worldRot = bodyComponent->GetComponentRotation();
			UE_LOG(LogTemp, Warning, TEXT("Body[%d] %hs - WorldLocation: (%f, %f, %f), WorldRotation: (%f, %f, %f)"),
				   BodyId,
				   bodyInfo.name.c_str(),
				   worldLoc.X, worldLoc.Y, worldLoc.Z,
				   worldRot.Pitch, worldRot.Yaw, worldRot.Roll);
		}
		BodyId++;
	}

	// Log geom information
	int GeomId = 0;
	for (const auto &geomInfo : _info.geoms)
	{
		if (UStaticMeshComponent *geomComponent = GeomMap1[GeomId])
		{
			FVector worldLoc = geomComponent->GetComponentLocation();
			FRotator worldRot = geomComponent->GetComponentRotation();
			UE_LOG(LogTemp, Warning, TEXT("Geom[%d] %hs - WorldLocation: (%f, %f, %f), WorldRotation: (%f, %f, %f)"),
				   GeomId,
				   geomInfo.name.c_str(),
				   worldLoc.X, worldLoc.Y, worldLoc.Z,
				   worldRot.Pitch, worldRot.Yaw, worldRot.Roll);
		}
		GeomId++;
	}
}

void AMuJoCoSimulation::ConvertMuJoCoModelToProceduralMeshes(const mjModel *mjModel, UObject *Outer)
{

	if (!mjModel || !Outer || mjModel->nmesh == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid input parameters or no meshes in model"));
		return;
	}

	// Iterate through all meshes in the MuJoCo model
	for (int mesh_id = 0; mesh_id < mjModel->nmesh; mesh_id++)
	{
		// Extract mesh data from MuJoCo
		const int vert_start = mjModel->mesh_vertadr[mesh_id];
		const int nvert = mjModel->mesh_vertnum[mesh_id];
		const float *mj_vertices = &mjModel->mesh_vert[vert_start * 3];

		const int face_start = mjModel->mesh_faceadr[mesh_id];
		const int nface = mjModel->mesh_facenum[mesh_id];
		const int *mj_faces = &mjModel->mesh_face[face_start * 3];

		// Skip empty meshes
		if (nvert == 0 || nface == 0)
			continue;

		// Convert vertices to Unreal coordinates
		TArray<FVector> UnrealVertices;
		for (int i = 0; i < nvert; i++)
		{
			const float *v = &mj_vertices[i * 3];
			UnrealVertices.Add(FVector(
				v[0] * 100.0f,	// X: meters to cm
				-v[1] * 100.0f, // Y: flip axis for left-handed
				v[2] * 100.0f	// Z: meters to cm
				));
		}

		// Convert faces to Unreal winding order (CW instead of MuJoCo's CCW)
		TArray<int32> UnrealTriangles;
		for (int i = 0; i < nface; i++)
		{
			UnrealTriangles.Add(mj_faces[i * 3 + 0]);
			UnrealTriangles.Add(mj_faces[i * 3 + 2]); // Swap order
			UnrealTriangles.Add(mj_faces[i * 3 + 1]);
		}

		// Create procedural mesh component
		UProceduralMeshComponent *ProcMesh = NewObject<UProceduralMeshComponent>(Outer);
		ProcMesh->RegisterComponent();

		// Generate normals/tangents (using placeholder UVs if none exist)
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FProcMeshTangent> Tangents;

		// Generate default UVs if needed (flat projection)
		UVs.SetNum(UnrealVertices.Num());
		for (FVector2D &uv : UVs)
		{
			uv = FVector2D(0.5f, 0.5f); // Simple default
		}

		UKismetProceduralMeshLibrary::CalculateTangentsForMesh(
			UnrealVertices,
			UnrealTriangles,
			UVs,
			Normals,
			Tangents);

		// Create mesh section
		ProcMesh->CreateMeshSection(
			0,
			UnrealVertices,
			UnrealTriangles,
			Normals,
			UVs,
			TArray<FColor>(), // Vertex colors
			Tangents,
			true // Enable collision
		);

		ProcMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// 为 ProceduralMesh 设置基础材质，确保后续 SetMeshColor 能创建动态材质实例
		static UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/MuJoCo/M_BaseColor.M_BaseColor"));
		if (BaseMat)
		{
			ProcMesh->SetMaterial(0, BaseMat);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("M_BaseColor material not found at /Game/MuJoCo/M_BaseColor"));
		}

		// Add to output array

		ProceduralMeshes.Add(ProcMesh);
		ProcMesh->SetVisibility(false);
	}

	return;
}

void AMuJoCoSimulation::SetMeshColor(UStaticMeshComponent *StaticMeshComponent, FLinearColor Color)
{
	if (!StaticMeshComponent)
		return;

	// 确保 mesh 有基础材质，如果没有则尝试加载 M_BaseColor
	UMaterialInterface *BaseMaterial = StaticMeshComponent->GetMaterial(0);
	if (!BaseMaterial)
	{
		BaseMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/MuJoCo/M_BaseColor.M_BaseColor"));
		if (BaseMaterial)
		{
			StaticMeshComponent->SetMaterial(0, BaseMaterial);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("SetMeshColor: M_BaseColor material not found, cannot apply color"));
			return;
		}
	}

	// Create dynamic material instance
	UMaterialInstanceDynamic *DynamicMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, StaticMeshComponent);
	if (!DynamicMaterial)
		return;

	DynamicMaterial->SetVectorParameterValue(FName("BaseColor"), Color);

	StaticMeshComponent->SetMaterial(0, DynamicMaterial);
}

void AMuJoCoSimulation::RequestStandUp()
{
	if (!mModel || !mData)
	{
		UE_LOG(LogTemp, Error, TEXT("RequestStandUp: Model or data not loaded"));
		return;
	}

	// Reset gait state
	GaitPhase = 0.0f;
	CmdVelX = CmdVelY = CmdVelYaw = 0.0f;

	// Capture CURRENT joint angles as starting point
	// (continuous trajectory handles any starting pose smoothly)
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		int jntIdx = i + 1; // skip freejoint
		float currentPos = mData->qpos[mModel->jnt_qposadr[jntIdx]];
		StandUpStartAngles[i] = currentPos;
		StandUpTargetAngles[i] = currentPos; // target = current → zero error on first frame
	}

	bStandUpActive = true;
	StandUpRampTime = 0.0f;

	UE_LOG(LogTemp, Warning, TEXT("[MuJoCo] STAND UP - Unified gain: Kp=%.0f, Kd=%.1f. WASD=Move, QE=Turn, Space=LieDown"), PD_Kp, PD_Kd);
}

void AMuJoCoSimulation::RequestLieDown()
{
	if (!mModel || !mData)
		return;

	bStandUpActive = false;
	CmdVelX = CmdVelY = CmdVelYaw = 0.0f;
	for (int i = 0; i < mModel->nu; i++)
		mData->ctrl[i] = 0.0f;

	UE_LOG(LogTemp, Warning, TEXT("[MuJoCo] LIE DOWN - passive mode"));
}

void AMuJoCoSimulation::SetWalkVelocity(float X, float Y, float Yaw)
{
	CmdVelX = FMath::Clamp(X, -1.0f, 1.0f);
	CmdVelY = FMath::Clamp(Y, -1.0f, 1.0f);
	CmdVelYaw = FMath::Clamp(Yaw, -1.0f, 1.0f);
}

void AMuJoCoSimulation::UpdateGaitTargets(float dt)
{
	// Advance gait phase
	GaitPhase += dt / GAIT_PERIOD;
	if (GaitPhase >= 1.0f) GaitPhase -= 1.0f;

	bool bHasCommand = (FMath::Abs(CmdVelX) > 0.01f || FMath::Abs(CmdVelYaw) > 0.01f || FMath::Abs(CmdVelY) > 0.01f);

	// Throttled debug log
	if (bHasCommand)
	{
		if (++GaitLogCounter >= 250)
		{
			GaitLogCounter = 0;
			UE_LOG(LogTemp, Warning, TEXT("[Gait] cmd=(%.2f, %.2f, %.2f) phase=%.2f"),
				CmdVelX, CmdVelY, CmdVelYaw, GaitPhase);
		}
	}
	else
	{
		GaitLogCounter = 0;
	}

	// Stand-up ramp factor (overall progress 0→1 over STAND_UP_RAMP_DURATION)
	float standRamp = FMath::Clamp(StandUpRampTime / STAND_UP_RAMP_DURATION, 0.0f, 1.0f);

	// === Matrix THREE-phase stand-up: JOINT-SPACE continuous trajectory ===
	// Continuous smooth path: Start → Crouch (at t=2s) → Standing (at t=3s)
	// Phase A (0~1s): First half of path to crouch, LOW gain (gentle)
	// Phase B (1~2s): Second half to crouch, MEDIUM gain
	// Phase C (2~3s): Crouch to standing, HIGH gain
	//
	// NO discontinuities in target - smooth single trajectory!
	const float PhaseBEnd = STAND_PHASE_A_DURATION + STAND_PHASE_B_DURATION;

	// Keyframe: crouch pose (reached at t=2s)
	const float HipCrouch = 0.6f, KneeCrouch = -1.2f;
	// Keyframe: standing pose (reached at t=3s)
	const float HipStand = 0.8f, KneeStand = -1.5f;

	for (int leg = 0; leg < 4; leg++)
	{
		int base = leg * 3;

		// ABAD: gather over first 1s, then stay 0
		float abadT = FMath::Clamp(StandUpRampTime / STAND_PHASE_A_DURATION, 0.0f, 1.0f);
		abadT = abadT * abadT * (3.0f - 2.0f * abadT);
		StandUpTargetAngles[base + 0] = FMath::Lerp(StandUpStartAngles[base + 0], 0.0f, abadT);

		// HIP/KNEE: continuous two-segment path
		if (StandUpRampTime < PhaseBEnd)
		{
			// Segment 1 (0~2s): Start → Crouch (smoothstep over 2s)
			float t = FMath::Clamp(StandUpRampTime / PhaseBEnd, 0.0f, 1.0f);
			t = t * t * (3.0f - 2.0f * t);
			StandUpTargetAngles[base + 1] = FMath::Lerp(StandUpStartAngles[base + 1], HipCrouch, t);
			StandUpTargetAngles[base + 2] = FMath::Lerp(StandUpStartAngles[base + 2], KneeCrouch, t);
		}
		else
		{
			// Segment 2 (2~3s): Crouch → Standing
			float t = FMath::Clamp((StandUpRampTime - PhaseBEnd) / STAND_PHASE_C_DURATION, 0.0f, 1.0f);
			t = t * t * (3.0f - 2.0f * t);
			StandUpTargetAngles[base + 1] = FMath::Lerp(HipCrouch, HipStand, t);
			StandUpTargetAngles[base + 2] = FMath::Lerp(KneeCrouch, KneeStand, t);
		}
	}

	// === PHASE 2: Walking (IK-based foot trajectory, only after stand-up complete) ===
	if (!bHasCommand || standRamp < 1.0f)
		return;

	// Trot gait: diagonal pairs
	const float PairOffset[4] = { 0.0f, 0.5f, 0.5f, 0.0f };

	for (int leg = 0; leg < 4; leg++)
	{
		float legPhase = GaitPhase + PairOffset[leg];
		if (legPhase >= 1.0f) legPhase -= 1.0f;

		// Desired foot position in HIP frame (x=forward, z=down-negative)
		float fx = FOOT_X_NOMINAL;
		float fz = -BODY_HEIGHT;
		float abadTarget = 0.0f;

		// Compute stride for this leg
		float strideX = CmdVelX * STRIDE_LENGTH;
		// Yaw: differential between left/right legs
		float yawSign = (leg == 0 || leg == 2) ? -1.0f : 1.0f;
		strideX += CmdVelYaw * YAW_STRIDE * yawSign;

		if (legPhase < 0.5f)
		{
			// === STANCE PHASE: foot on ground, sweep backward ===
			float t = legPhase / 0.5f;
			fx += strideX * (0.5f - t);
			abadTarget = CmdVelY * LAT_STRIDE * (0.5f - t);
		}
		else
		{
			// === SWING PHASE: foot in air, return forward with lift ===
			float t = (legPhase - 0.5f) / 0.5f;
			fx += strideX * (-0.5f + t);
			fz += SWING_HEIGHT * FMath::Sin(t * PI);
			abadTarget = CmdVelY * LAT_STRIDE * (-0.5f + t);
		}

		// === 2-link IK: compute HIP and KNEE from foot position ===
		float d2 = fx * fx + fz * fz;
		float cosKnee = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
		cosKnee = FMath::Clamp(cosKnee, -1.0f, 1.0f);
		float kneeAngle = -FMath::Acos(cosKnee);

		float beta = FMath::Atan2(-fz, fx);
		float alpha = FMath::Atan2(L2 * FMath::Sin(kneeAngle), L1 + L2 * FMath::Cos(kneeAngle));
		float hipAngle = beta - alpha;

		int base = leg * 3;
		StandUpTargetAngles[base + 0] = abadTarget;  // ABAD
		StandUpTargetAngles[base + 1] = hipAngle - HALF_PI;    // HIP (IK is from +x axis; model HIP=0 is straight down -> subtract 90 deg)
		StandUpTargetAngles[base + 2] = kneeAngle;   // KNEE
	}
}

void AMuJoCoSimulation::ApplyStandUpControl()
{
	if (!mData || !mModel)
		return;

	float dt = mModel->opt.timestep;
	StandUpRampTime += dt;

	// Update gait targets (modulates StandUpTargetAngles)
	UpdateGaitTargets(dt);

	// Unified high gain: smoothstep trajectory provides slow-fast-slow profile
	// No phase-dependent gain switching (eliminates gain-jump transients)
	float kp = PD_Kp;
	float kd = PD_Kd;

	// PD control: torque = Kp * error - Kd * velocity
	for (int i = 0; i < NUM_JOINTS && i < mModel->nu; i++)
	{
		int jntIdx = i + 1; // skip freejoint
		float currentPos = mData->qpos[mModel->jnt_qposadr[jntIdx]];
		float currentVel = mData->qvel[mModel->jnt_dofadr[jntIdx]];
		float error = StandUpTargetAngles[i] - currentPos;
		mData->ctrl[i] = kp * error - kd * currentVel;
	}
}
