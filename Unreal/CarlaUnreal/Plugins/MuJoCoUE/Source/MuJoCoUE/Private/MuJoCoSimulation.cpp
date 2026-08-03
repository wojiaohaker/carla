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

	// Create UDP components for mc_ctrl communication
	UdpReceiver = CreateDefaultSubobject<UUdpReceiverComponent>(TEXT("UdpReceiver"));
	UdpSender = CreateDefaultSubobject<UUdpSenderComponent>(TEXT("UdpSender"));

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
		// ---- External Physics Mode: load model for mesh only, no internal physics ----
		if (bExternalPhysicsMode)
		{
			// Still need initial FK to generate meshes at a valid pose
			mj_forward(mModel, mData);

			_info = ExtractModelInfo(mModel);
			ConvertMuJoCoModelToProceduralMeshes(mModel, this);
			GenerateMeshes(_info);

			// Disable internal physics loop
			bSimulationRunning = false;

			// Disable mc_ctrl UDP control (mujoco_sim handles that)
			bUdpControlEnabled = false;
			if (UdpSender) UdpSender->bAutoSend = false;

			// Switch UdpReceiver to RobotState mode (port 25001, parse mujoco_sim state)
			if (UdpReceiver)
			{
				UdpReceiver->StopListening();
				UdpReceiver->SetParseMode(EUdpParseMode::ParseRobotState);
				UdpReceiver->bAutoStart = false;
				UdpReceiver->StartListening();
			}

			UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] External physics mode ENABLED. UdpReceiver listening on port %d for RobotState."),
				UdpReceiver ? UdpReceiver->ListenPort : -1);
			UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] Internal mj_step DISABLED. Rendering driven by UDP RobotState."));
			return;
		}

		// ---- Normal (Internal Physics) Mode ----
		// Set initial LIEDOWN pose so mc_ctrl's FK sees low body_height (~0.05).
		// Joint angles from xg-user-parameters.yaml: hip_liedown=1.4, knee_liedown=-2.4
		// IMPORTANT: mc_ctrl computes body_height via FK(joint_angles), NOT from base xpos.
		// Simply letting the robot fall (zero torque) drops the base but joints stay
		// straight → FK height remains ~0.37 → STANDUP FSM stuck.
		if (mModel->nq >= 19) // 7 (freejoint) + 12 joints
		{
			mData->qpos[2] = 0.10; // base z: low, near ground
			for (int j = 0; j < 4; j++)
			{
				mData->qpos[7 + j*3 + 0] = 0.0f;   // ABAD = 0
				mData->qpos[7 + j*3 + 1] = 1.4f;   // HIP  = 1.4 (liedown)
				mData->qpos[7 + j*3 + 2] = -2.4f;  // KNEE = -2.4 (liedown)
			}
		}
		mj_forward(mModel, mData);

		_info = ExtractModelInfo(mModel);
		ConvertMuJoCoModelToProceduralMeshes(mModel, this);
		GenerateMeshes(_info);

		// Start simulation immediately (passive mode, robot collapses under gravity)
		SimStartWallTime = FPlatformTime::Seconds();
		StartSimulation();

		UE_LOG(LogTemp, Warning, TEXT("MuJoCo: Started in LIEDOWN pose (HIP=1.4 KNEE=-2.4). %.1fs before mc_ctrl."), UDP_STARTUP_DELAY);
		UE_LOG(LogTemp, Warning, TEXT("[INIT-DIAG] UdpSender=%s, UdpReceiver=%s, bUdpControlEnabled=%d, bSimulationRunning=%d"),
			UdpSender ? TEXT("valid") : TEXT("NULL"),
			UdpReceiver ? TEXT("valid") : TEXT("NULL"),
			bUdpControlEnabled ? 1 : 0,
			bSimulationRunning ? 1 : 0);
		if (UdpSender)
		{
			UE_LOG(LogTemp, Warning, TEXT("[INIT-DIAG] UdpSender: bSocketReady=%d, TargetIP=%s, TargetPort=%d"),
				UdpSender->bSocketReady ? 1 : 0, *UdpSender->TargetIP, UdpSender->TargetPort);
		}
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

	// Throttled UDP diagnostic (every ~2s)
	static int32 DiagCounter = 0;
	if (++DiagCounter >= 120)
	{
		DiagCounter = 0;
		int64 Pkts = UdpReceiver ? UdpReceiver->TotalPacketsReceived : 0;
		int64 Cmds = UdpReceiver ? UdpReceiver->TotalCommandsParsed : 0;
		bool bFresh = false;
		float diagMaxKp = 0.0f;
		if (UdpReceiver && UdpReceiver->LastCommand.bValid)
		{
			double age = FPlatformTime::Seconds() - UdpReceiver->LastCommand.ReceiveTime;
			bFresh = (age < UDP_CMD_TIMEOUT);
			for (int32 k = 0; k < UdpReceiver->LastCommand.KpValues.Num(); k++)
			{
				if (UdpReceiver->LastCommand.KpValues[k] > diagMaxKp)
					diagMaxKp = UdpReceiver->LastCommand.KpValues[k];
			}
		}
		UE_LOG(LogTemp, Warning, TEXT("[UDP-Diag] pkts=%lld cmds=%lld fresh=%d maxKp=%.1f active=%d"),
			Pkts, Cmds, bFresh ? 1 : 0, diagMaxKp, (bFresh && diagMaxKp > 10.0f) ? 1 : 0);
	}

	double startTime = mData->time;
	while (mData->time - startTime < DeltaTime)
	{
		// Control priority: UDP (mc_ctrl) > StandUp/Gait > Passive
		// Gate: only apply UDP when mc_ctrl is in ACTIVE mode (Kp > threshold)
		// mc_ctrl sends low/zero Kp in damping/idle mode before user presses U
		bool bUdpActive = false;

		// Startup grace period: let robot collapse under gravity before mc_ctrl PD takes over.
		// Without this, mc_ctrl sends Kp=80 immediately and holds the robot standing,
		// so the STANDUP FSM never sees the collapsed pose it requires.
		bool bInStartupDelay = (FPlatformTime::Seconds() - SimStartWallTime) < UDP_STARTUP_DELAY;

		if (bInStartupDelay)
		{
			// Passive: zero torque, let gravity collapse the robot
			for (int i = 0; i < mModel->nu && i < NUM_JOINTS; i++)
			{
				mData->ctrl[i] = 0.0;
			}
			// Log body height at end of startup delay for diagnostics
			static bool bLoggedCollapse = false;
			if (!bLoggedCollapse && (FPlatformTime::Seconds() - SimStartWallTime) >= UDP_STARTUP_DELAY - 0.05)
			{
				bLoggedCollapse = true;
				int baseBody = 1;
				UE_LOG(LogTemp, Warning, TEXT("[Startup] Delay ended. body z=%.4f (should be ~0.05 collapsed)"),
					static_cast<float>(mData->xpos[baseBody * 3 + 2]));
			}
		}
		else if (bUdpControlEnabled && UdpReceiver && UdpReceiver->LastCommand.bValid)
		{
			double age = FPlatformTime::Seconds() - UdpReceiver->LastCommand.ReceiveTime;
			if (age < UDP_CMD_TIMEOUT)
			{
				// Check if mc_ctrl is in active control (any Kp > 10)
				const FUdpCommandData& Cmd = UdpReceiver->LastCommand;
				float maxKp = 0.0f;
				for (int32 k = 0; k < Cmd.KpValues.Num(); k++)
				{
					if (Cmd.KpValues[k] > maxKp) maxKp = Cmd.KpValues[k];
				}
				if (maxKp > 10.0f)
				{
					// Rising edge: reset gain ramp timer and capture current joint angles
					if (!bUdpWasActive)
					{
						UdpGainRampTime = 0.0f;
						// Capture current joint positions for target blending
						for (int j = 0; j < NUM_JOINTS && j < mModel->nu; j++)
						{
							int jntIdx = j + 1; // skip freejoint
							UdpActivationAngles[j] = static_cast<float>(mData->qpos[mModel->jnt_qposadr[jntIdx]]);
						}
						UE_LOG(LogTemp, Warning, TEXT("[UDP] mc_ctrl active (maxKp=%.0f), gain ramp %.1fs + target blend"), maxKp, UDP_GAIN_RAMP_DURATION);
					}
					UdpGainRampTime += mModel->opt.timestep;
					float gainScale = FMath::Clamp(UdpGainRampTime / UDP_GAIN_RAMP_DURATION, 0.0f, 1.0f);
					ApplyUdpControl(gainScale);
					bUdpActive = true;
				}
				else
				{
					// mc_ctrl in damping/idle mode (Kp<=10): apply gentle damping to prevent uncontrolled fall
					for (int i = 0; i < mModel->nu && i < NUM_JOINTS; i++)
					{
						int jntIdx = i + 1;
						float vel = static_cast<float>(mData->qvel[mModel->jnt_dofadr[jntIdx]]);
						mData->ctrl[i] = FMath::Clamp(-2.0f * vel, -10.0f, 10.0f);
					}
				}
			}
			else if (bStandUpActive)
			{
				ApplyStandUpControl();
			}
		}
		else if (bStandUpActive)
		{
			ApplyStandUpControl();
		}
		bUdpWasActive = bUdpActive;

		mj_step(mModel, mData);

		// Send state feedback to mc_ctrl after each physics step.
		// IMPORTANT: suppress during startup delay so mc_ctrl never sees the
		// initial standing pose.  mc_ctrl's STANDUP FSM checks body_height at
		// first reception; if it sees 0.37 (standing) it gets stuck.
		// After the delay the robot is collapsed (~0.05) — matching Matrix UE.
		if (bUdpControlEnabled && UdpSender && !bInStartupDelay)
		{
			SendStateToMcCtrl();

			// Throttled send-path diagnostic (every ~500 calls ≈ 1s at 500Hz)
			static int32 SendDiagCounter = 0;
			if (++SendDiagCounter >= 500)
			{
				SendDiagCounter = 0;
				UE_LOG(LogTemp, Warning, TEXT("[SEND-DIAG] SendStateToMcCtrl called. UdpSender->TotalPacketsSent=%lld, bSocketReady=%d"),
					UdpSender->TotalPacketsSent, UdpSender->bSocketReady ? 1 : 0);
			}
		}
		else
		{
			// Throttled diagnostic for WHY we're not sending (every ~500 steps)
			static int32 NoSendDiagCounter = 0;
			if (++NoSendDiagCounter >= 500)
			{
				NoSendDiagCounter = 0;
				UE_LOG(LogTemp, Warning, TEXT("[SEND-DIAG] NOT sending: bUdpControlEnabled=%d, UdpSender=%s, bInStartupDelay=%d (elapsed=%.2fs)"),
					bUdpControlEnabled ? 1 : 0,
					UdpSender ? TEXT("valid") : TEXT("NULL"),
					bInStartupDelay ? 1 : 0,
					FPlatformTime::Seconds() - SimStartWallTime);
			}
		}
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

	// External physics mode: render from UDP state, no internal mj_step
	if (bExternalPhysicsMode)
	{
		TickExternalPhysics(DeltaTime);
		return;
	}

	// Throttled tick diagnostic (every ~300 frames ≈ 5s at 60fps)
	static int32 TickDiagCounter = 0;
	if (++TickDiagCounter >= 300)
	{
		TickDiagCounter = 0;
		UE_LOG(LogTemp, Warning, TEXT("[TICK-DIAG] bSimulationRunning=%d, mData=%s, mModel=%s, DeltaTime=%.4f"),
			bSimulationRunning ? 1 : 0,
			mData ? TEXT("valid") : TEXT("NULL"),
			mModel ? TEXT("valid") : TEXT("NULL"),
			DeltaTime);
	}

	if (bSimulationRunning)
		SimulateMuJoCo(DeltaTime);
}

void AMuJoCoSimulation::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Stand / Lie down
	/*PlayerInputComponent->BindAction("StandUp", IE_Pressed, this, &AMuJoCoSimulation::RequestStandUp);
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
	PlayerInputComponent->BindAction("TurnLeft", IE_Released, this, &AMuJoCoSimulation::OnTurnLeftReleased);*/

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

	// Integrate yaw heading for drift diagnostics (world-frame z angular velocity)
	if (mData)
		YawHeading += mData->qvel[5] * dt;

	bool bHasCommand = (FMath::Abs(CmdVelX) > 0.01f || FMath::Abs(CmdVelYaw) > 0.01f || FMath::Abs(CmdVelY) > 0.01f);

	// Throttled debug log
	if (bHasCommand)
	{
		if (++GaitLogCounter >= 250)
		{
			GaitLogCounter = 0;
			UE_LOG(LogTemp, Warning, TEXT("[Gait] cmd=(%.2f, %.2f, %.2f) phase=%.2f yawRate=%.3f rad/s yawDeg=%.1f"),
				CmdVelX, CmdVelY, CmdVelYaw, GaitPhase,
				mData->qvel[5], FMath::RadiansToDegrees(YawHeading));
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

	// Yaw stabilization: measure body yaw rate and feed back to cancel drift.
	// Open-loop gait accumulates yaw error (especially during backward walking),
	// so we actively damp any unwanted rotation.
	// Sign: yawCmd>0 makes left legs stride further -> clockwise torque (wz<0),
	// so the plant is dwz/dt = -k*yawCmd. To damp wz we need yawCmd = +GAIN*wz.
	// IMPORTANT: low-pass filter the yaw rate first. The raw signal contains large
	// gait-frequency yaw wiggle (±0.8 rad/s); feeding that back directly hijacks the
	// gait and makes the robot spin in place. The filter keeps only slow heading drift.
	float yawRate = mData->qvel[5]; // freejoint world-frame z angular velocity
	float lpfAlpha = FMath::Clamp(dt / YAW_FILTER_TAU, 0.0f, 1.0f);
	YawRateLPF += (yawRate - YawRateLPF) * lpfAlpha;
	// Yaw control: heading-hold during lateral, rate-damper during forward.
	float latAbs = FMath::Clamp(FMath::Abs(CmdVelY), 0.0f, 1.0f);
	const bool bLateral = (latAbs > 0.3f);
	float yawCmd;
	if (bLateral)
	{
		// HEADING HOLD: capture reference heading at lateral start, then use
		// P+D controller to keep heading fixed. strideX differential creates yaw
		// torque with ZERO net forward/backward force (left/right cancel).
		if (!bHeadingRefValid)
		{
			HeadingRef = YawHeading;
			bHeadingRefValid = true;
		}
		float headingErr = YawHeading - HeadingRef;
		// Normalize to [-PI, PI]
		while (headingErr > PI) headingErr -= 2.0f * PI;
		while (headingErr < -PI) headingErr += 2.0f * PI;
		const float HEADING_KP = 5.0f;  // proportional heading correction
		const float HEADING_KD = 0.5f;  // rate damping
		yawCmd = -HEADING_KP * headingErr - HEADING_KD * YawRateLPF;
		yawCmd = FMath::Clamp(yawCmd, -2.0f, 2.0f);
	}
	else
	{
		bHeadingRefValid = false;
		yawCmd = CmdVelYaw + YawRateLPF * YAW_DAMP_GAIN;
	}

	// Gait selection:
	// - Forward/backward/yaw: Trot (diagonal pairs, 50% duty)
	// - Lateral: SEQUENTIAL walk (each leg offset by 0.25). At any time 3 legs
	//   stance + 1 leg swing. Stance legs pull body toward offset feet (strong,
	//   grounded). Swing leg repositions (weak, light). Net ratcheting motion.
	const float SWING_FRAC_LAT = 0.25f; // 25% swing per leg (75% stance = 3 legs support)
	const float TrotOffset[4] = { 0.0f, 0.5f, 0.5f, 0.0f };
	const float LatSeqOffset[4] = { 0.0f, 0.25f, 0.5f, 0.75f }; // sequential walk

	for (int leg = 0; leg < 4; leg++)
	{
		float legPhase = GaitPhase + (bLateral ? LatSeqOffset[leg] : TrotOffset[leg]);
		if (legPhase >= 1.0f) legPhase -= 1.0f;

		// Desired foot position in HIP frame (x=forward, z=down-negative)
		float fx = FOOT_X_NOMINAL;
		float fz = -BODY_HEIGHT;
		float abadTarget = 0.0f;

		// Compute forward stride for this leg
		float strideX = CmdVelX * STRIDE_LENGTH;
		float yawSign = (leg == 0 || leg == 2) ? -1.0f : 1.0f;
		strideX += yawCmd * YAW_STRIDE * yawSign;

		if (!bLateral)
		{
			// === NORMAL TROT: forward/backward/yaw ===
			if (legPhase < 0.5f)
			{
				float t = legPhase / 0.5f;
				fx += strideX * (0.5f - t);
			}
			else
			{
				float t = (legPhase - 0.5f) / 0.5f;
				fx += strideX * (-0.5f + t);
				fz += SWING_HEIGHT * FMath::Sin(t * PI);
			}
		}
		else
		{
			// === LATERAL SEQUENCE WALK ===
			// Negate CmdVelY: +ABAD = foot LEFT -> body pulled LEFT during stance.
			// D (CmdVelY=+1) should go RIGHT -> need -ABAD -> latCmd = -CmdVelY.
			float latCmd = -CmdVelY;
			float stanceFrac = 1.0f - SWING_FRAC_LAT; // 0.75
			if (legPhase < stanceFrac)
			{
				// STANCE: foot grounded at offset, PD pulls body toward foot.
				// ABAD sweeps from offset to 0 (body shifts toward offset direction).
				float t = legPhase / stanceFrac;
				abadTarget = latCmd * LAT_STRIDE * (1.0f - t);
			}
			else
			{
				// SWING: single leg lifts and returns to offset for next step.
				float t = (legPhase - stanceFrac) / SWING_FRAC_LAT;
				fz += SWING_HEIGHT * FMath::Sin(t * PI);
				abadTarget = latCmd * LAT_STRIDE * t;
			}
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

void AMuJoCoSimulation::ApplyUdpControl(float GainScale)
{
	if (!mData || !mModel || !UdpReceiver)
		return;

	const FUdpCommandData& Cmd = UdpReceiver->LastCommand;

	if (Cmd.JointTargets.Num() < NUM_JOINTS)
		return;

	// PD control using targets and gains from mc_ctrl
	// Protocol order in JointTargets: [abad×4(FR,FL,RR,RL), hip×4, knee×4]
	// MuJoCo joint order: leg0(ABAD,HIP,KNEE), leg1(ABAD,HIP,KNEE), ...
	// Mapping: MuJoCo(leg*3+jointType) ← Protocol(jointType*4+leg)
	for (int leg = 0; leg < 4; leg++)
	{
		for (int jointType = 0; jointType < 3; jointType++)
		{
			int mjIdx = leg * 3 + jointType; // MuJoCo actuator index
			int protoIdx = jointType * 4 + leg; // Protocol array index

			if (mjIdx >= mModel->nu)
				break;

			int jntIdx = mjIdx + 1; // skip freejoint (qpos[0..6])
			float currentPos = mData->qpos[mModel->jnt_qposadr[jntIdx]];
			float currentVel = mData->qvel[mModel->jnt_dofadr[jntIdx]];

			float target = Cmd.JointTargets[protoIdx];
			float kp = (Cmd.KpValues.Num() > protoIdx && Cmd.KpValues[protoIdx] > 0.0f) ? Cmd.KpValues[protoIdx] : 20.0f;
			float kd = (Cmd.KdValues.Num() > protoIdx && Cmd.KdValues[protoIdx] > 0.0f) ? Cmd.KdValues[protoIdx] : 0.7f;

			// Target blending: lerp from activation pose to mc_ctrl target
			// This prevents large position errors from generating explosive torques
			float blendedTarget = FMath::Lerp(UdpActivationAngles[mjIdx], target, GainScale);

			// Apply gain ramp to prevent launch impulse on activation
			kp *= GainScale;
			kd *= GainScale;

			float torque = kp * (blendedTarget - currentPos) - kd * currentVel;

			// Add feedforward torque if provided (also scaled)
			if (Cmd.TauFF.Num() > protoIdx)
			{
				torque += Cmd.TauFF[protoIdx] * GainScale;
			}

			// Clamp to actuator limits (±28 Nm for xgb)
			mData->ctrl[mjIdx] = FMath::Clamp(torque, -28.0f, 28.0f);
		}
	}
}

void AMuJoCoSimulation::SendStateToMcCtrl()
{
	if (!mData || !mModel || !UdpSender)
		return;

	// Build joint arrays: [abad×4, hip×4, knee×4]
	// MuJoCo joint order: FAR(ABAD,HIP,KNEE), FBL, RAR, RBL
	// Protocol order:     abad[FR,FL,RR,RL], hip[FR,FL,RR,RL], knee[FR,FL,RR,RL]
	TArray<float> JointPos, JointVel, JointTau;
	JointPos.SetNum(12);
	JointVel.SetNum(12);
	JointTau.SetNum(12);

	for (int leg = 0; leg < 4; leg++)
	{
		for (int jointType = 0; jointType < 3; jointType++)
		{
			// MuJoCo index: leg*3 + jointType (skip freejoint)
			int mjIdx = leg * 3 + jointType;
			int jntIdx = mjIdx + 1; // +1 to skip freejoint

			// Protocol index: jointType*4 + leg
			int protoIdx = jointType * 4 + leg;

			JointPos[protoIdx] = static_cast<float>(mData->qpos[mModel->jnt_qposadr[jntIdx]]);
			JointVel[protoIdx] = static_cast<float>(mData->qvel[mModel->jnt_dofadr[jntIdx]]);
			JointTau[protoIdx] = static_cast<float>(mData->qfrc_actuator[mModel->dof_jntid[mModel->jnt_dofadr[jntIdx]]]);
		}
	}

	// IMU data from base body (body 1 = torso)
	// Quaternion: MuJoCo stores [w,x,y,z] in body_xquat
	TArray<float> Quat, Gyro, Acc, RPY;
	Quat.SetNum(4);
	Gyro.SetNum(3);
	Acc.SetNum(3);
	RPY.SetNum(3);

	// Base body quaternion (world frame)
	int baseBody = 1; // torso
	Quat[0] = static_cast<float>(mData->xquat[baseBody * 4 + 0]); // w
	Quat[1] = static_cast<float>(mData->xquat[baseBody * 4 + 1]); // x
	Quat[2] = static_cast<float>(mData->xquat[baseBody * 4 + 2]); // y
	Quat[3] = static_cast<float>(mData->xquat[baseBody * 4 + 3]); // z

	// Angular velocity in BODY frame (gyroscope)
	// MuJoCo cvel is [angular(3), linear(3)] in global frame
	// Transform to body frame using rotation matrix from xmat
	float wx = static_cast<float>(mData->cvel[baseBody * 6 + 0]);
	float wy = static_cast<float>(mData->cvel[baseBody * 6 + 1]);
	float wz = static_cast<float>(mData->cvel[baseBody * 6 + 2]);
	// xmat is row-major 3x3 rotation (world←body)
	const double* R = &mData->xmat[baseBody * 9];
	// Body angular velocity = R^T * world angular velocity
	Gyro[0] = static_cast<float>(R[0]*wx + R[3]*wy + R[6]*wz);
	Gyro[1] = static_cast<float>(R[1]*wx + R[4]*wy + R[7]*wz);
	Gyro[2] = static_cast<float>(R[2]*wx + R[5]*wy + R[8]*wz);

	// Accelerometer in BODY frame
	// A real accelerometer measures specific force = acceleration - gravity
	// When stationary and upright: acc = [0, 0, +9.81] (pointing up in body Z)
	// Compute: acc_body = R^T * (linear_accel_world - gravity_world)
	// For simplicity, use velocity derivative approximation + gravity projection
	// gravity_world = [0, 0, -9.81], specific_force = -gravity in body frame when static
	float gx = 0.0f, gy = 0.0f, gz = -9.81f;
	// acc_body = R^T * (-gravity) = R^T * [0, 0, 9.81]
	Acc[0] = static_cast<float>(R[6] * 9.81);  // R^T row0 · [0,0,9.81]
	Acc[1] = static_cast<float>(R[7] * 9.81);  // R^T row1 · [0,0,9.81]
	Acc[2] = static_cast<float>(R[8] * 9.81);  // R^T row2 · [0,0,9.81]

	// RPY from quaternion
	float w = Quat[0], x = Quat[1], y = Quat[2], z = Quat[3];
	RPY[0] = FMath::Atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y)); // roll
	RPY[1] = FMath::Asin(FMath::Clamp(2.0f * (w*y - z*x), -1.0f, 1.0f));   // pitch
	RPY[2] = FMath::Atan2(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z)); // yaw

	// World position and velocity of base body
	TArray<float> Position, VWorld;
	Position.SetNum(3);
	VWorld.SetNum(3);
	Position[0] = static_cast<float>(mData->xpos[baseBody * 3 + 0]);
	Position[1] = static_cast<float>(mData->xpos[baseBody * 3 + 1]);
	Position[2] = static_cast<float>(mData->xpos[baseBody * 3 + 2]);
	// cvel is [angular(3), linear(3)] in global frame
	VWorld[0] = static_cast<float>(mData->cvel[baseBody * 6 + 3]);
	VWorld[1] = static_cast<float>(mData->cvel[baseBody * 6 + 4]);
	VWorld[2] = static_cast<float>(mData->cvel[baseBody * 6 + 5]);

	UdpSender->UpdateState(JointPos, JointVel, JointTau, Quat, Gyro, Acc, RPY, Position, VWorld);
	bool bSent = UdpSender->SendState();

	// Throttled detailed data diagnostic (every ~500 calls)
	static int32 DataDiagCounter = 0;
	if (++DataDiagCounter >= 500)
	{
		DataDiagCounter = 0;
		UE_LOG(LogTemp, Warning, TEXT("[DATA-DIAG] bSent=%d hip[0]=%.3f knee[0]=%.3f pos_z=%.4f quat_w=%.3f"),
			bSent ? 1 : 0, JointPos[4], JointPos[8], Position[2], Quat[0]);
	}
}

void AMuJoCoSimulation::TickExternalPhysics(float DeltaTime)
{
	if (!mData || !mModel || !UdpReceiver)
		return;

	// Get latest state from mujoco_sim (thread-safe)
	FUdpStateData State = UdpReceiver->GetLatestState();

	if (!State.bValid)
	{
		// No data received yet — wait silently
		return;
	}

	// Check staleness
	double age = FPlatformTime::Seconds() - State.ReceiveTime;
	if (age > EXT_STATE_TIMEOUT)
	{
		// Throttled stale warning
		if (++ExtDiagCounter >= 300)
		{
			ExtDiagCounter = 0;
			UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] State STALE (age=%.2fs > %.1fs). mujoco_sim may have stopped."),
				age, EXT_STATE_TIMEOUT);
		}
		return;
	}

	// ---- Write received state into mData->qpos for FK computation ----
	// qpos layout: [x, y, z, qw, qx, qy, qz, abad0, hip0, knee0, abad1, ...]

	// Base position [x, y, z]
	if (State.Position.Num() >= 3)
	{
		mData->qpos[0] = State.Position[0];
		mData->qpos[1] = State.Position[1];
		mData->qpos[2] = State.Position[2];
	}

	// Base quaternion [w, x, y, z] → qpos[3..6]
	if (State.Quat.Num() >= 4)
	{
		mData->qpos[3] = State.Quat[0]; // w
		mData->qpos[4] = State.Quat[1]; // x
		mData->qpos[5] = State.Quat[2]; // y
		mData->qpos[6] = State.Quat[3]; // z
	}

	// Joint angles: RobotState has [abad×4, hip×4, knee×4] per type
	// MuJoCo qpos[7..18] layout: leg0(abad,hip,knee), leg1(abad,hip,knee), ...
	// Leg order: FAR=0, FBL=1, RAR=2, RBL=3
	if (State.QAbad.Num() >= 4 && State.QHip.Num() >= 4 && State.QKnee.Num() >= 4)
	{
		for (int leg = 0; leg < 4; leg++)
		{
			int qposBase = 7 + leg * 3;
			mData->qpos[qposBase + 0] = State.QAbad[leg];
			mData->qpos[qposBase + 1] = State.QHip[leg];
			mData->qpos[qposBase + 2] = State.QKnee[leg];
		}
	}

	// ---- Forward Kinematics only (no dynamics) ----
	mj_kinematics(mModel, mData);
	mj_comPos(mModel, mData);

	// ---- Update mesh transforms ----
	if (_info.bodies.size() > 0)
	{
		ExtractCurrentState(_info);
		UpdateSimulationView(_info);
	}

	// Throttled diagnostic (every ~5s at 60fps)
	if (++ExtDiagCounter >= 300)
	{
		ExtDiagCounter = 0;
		UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] Rendering OK. pkts=%lld states=%lld pos=(%.3f,%.3f,%.3f) hip[0]=%.3f"),
			UdpReceiver->TotalPacketsReceived,
			UdpReceiver->TotalStatesParsed,
			State.Position.Num() >= 3 ? State.Position[0] : 0.0f,
			State.Position.Num() >= 3 ? State.Position[1] : 0.0f,
			State.Position.Num() >= 3 ? State.Position[2] : 0.0f,
			State.QHip.Num() >= 1 ? State.QHip[0] : 0.0f);
	}
}
