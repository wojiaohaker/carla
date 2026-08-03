// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "mujoco/mujoco.h"

#include "CoreMinimal.h"

#include "GameFramework/Pawn.h"
#include "ProceduralMeshComponent.h"
// #include "Components/InstancedStaticMeshComponent.h"
#include "UdpReceiverComponent.h"
#include "UdpSenderComponent.h"
#include "MuJoCoSimulation.generated.h"

/**
 * @struct BodyInfo
 * @brief Contains information about a body extracted form the MuJoCo Model and Data.
 *
 * @property std::string name - The name of the body.
 * @property int parent_id - The ID of the parent body.
 * @property mjtNum[3] pos - The position of the body as a 3D vector.
 * @property mjtNum[4] quat - The orientation of the body as a quaternion in MuJoCo format.
 * @property FQuat quat2 - The orientation of the body as an Unreal Engine quaternion.
 */
struct BodyInfo
{
	std::string name;
	int parent_id;
	mjtNum pos[3];
	mjtNum quat[4];
	FQuat quat2;
};

/**
 * @struct GeomInfo
 * @brief Contains information about a geometry extracted form the MuJoCo Model and Data.
 *
 * This structure stores various properties of a geometry object such as its name,
 * body ID, type, size, position, orientation, and color. It provides a way to manage
 * and adjust geometries within the MuJoCo simulation environment.
 *
 * @var std::string name
 * The name identifier of the geometry.
 *
 * @var int body_id
 * The ID of the body to which this geometry is attached.
 *
 * @var int type
 * The type of the geometry (e.g., box, sphere, etc.).
 *
 * @var mjtNum size[3]
 * The dimensions of the geometry along each axis.
 *
 * @var mjtNum pos[3]
 * The position of the geometry in 3D space.
 *
 * @var mjtNum posAdjust[3]
 * Additional position adjustment values, initialized to zero.
 *
 * @var mjtNum quat[4]
 * Quaternion representing the orientation of the geometry in MuJoCo format.
 *
 * @var FQuat quat2
 * Quaternion representing the orientation in Unreal Engine format.
 *
 * @var FLinearColor color
 * The color of the geometry in Unreal Engine format.
 */
struct GeomInfo
{
	std::string name;
	int body_id;
	int type;
	mjtNum size[3];
	mjtNum pos[3];
	mjtNum posAdjust[3];
	mjtNum quat[4];
	FQuat quat2;
	FLinearColor color;
	GeomInfo()
	{
		posAdjust[0] = 0;
		posAdjust[1] = 0;
		posAdjust[2] = 0;
	}
};

/**
 * @struct ModelInfo
 * @brief Represents information about a MuJoCo model.
 *
 * This structure contains collections of body and geometry MuJoCo information
 * that we might need inside the Unreal Engine.
 *
 * @member bodies A vector of BodyInfo structures representing the physical bodies in the model.
 * @member geoms A vector of GeomInfo structures representing the geometric shapes in the model.
 */
struct ModelInfo
{
	std::vector<BodyInfo> bodies;
	std::vector<GeomInfo> geoms;
};

/**
 * @brief Actor class that interfaces with MuJoCo physics simulation in Unreal Engine.
 *
 * This class provides functionality to load, simulate, and visualize MuJoCo physics models
 * within Unreal Engine. It handles the conversion between MuJoCo physics representation and
 * Unreal Engine's visual components, allowing for real-time physics simulation and visualization.
 *
 * The simulation can be controlled via Blueprint functions to start, pause, reset, and step through
 * the physics simulation. It also maps MuJoCo bodies and geometries to Unreal Engine components.
 *
 * @note Requires the MuJoCo physics library to be properly integrated with the project.
 */
/**
 * @class AMuJoCoSimulation
 * @brief Actor for MuJoCo physics simulation integration within Unreal Engine
 *
 * This class provides functionality to load, visualize, and simulate MuJoCo physics models.
 * It handles the conversion between MuJoCo's physics representation and Unreal Engine's
 * visual components, allowing for real-time physics simulation and visualization.
 */

// Protected variables
/** @brief MuJoCo simulation data */
// mjData* mData;

/** @brief MuJoCo model */
// mjModel* mModel;

/** @brief Current model information */
// ModelInfo _info;

/** @brief Initial model information */
// ModelInfo _infoStart;

/** @brief Flag indicating if simulation is currently running */
// bool bSimulationRunning=false;

// Public properties
/** @brief Maps MuJoCo body IDs to scene components */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// TMap<int,USceneComponent*> BodyMap;

/** @brief Maps MuJoCo geometry IDs to static mesh components */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// TMap<int,UStaticMeshComponent*> GeomMap1;

/** @brief Collection of all procedural meshes created for the simulation */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// TArray<UProceduralMeshComponent*> ProceduralMeshes;

/** @brief Path to the MuJoCo XML model file */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// FString XmlSourcePath;

/** @brief Collection of available static meshes for geometry visualization */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// TMap<int, UStaticMesh*> MeshAssets;

/** @brief Default mesh to use when specific meshes are not provided */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
// UStaticMesh* defaultMesh;
UCLASS()
class MUJOCOUE_API AMuJoCoSimulation : public APawn
{
	GENERATED_BODY()

public:
	AMuJoCoSimulation();

protected:
	mjData *mData;
	mjModel *mModel;
	ModelInfo _info;
	ModelInfo _infoStart;
	bool bSimulationRunning = false;

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	TMap<int, USceneComponent *> BodyMap;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	TMap<int, UStaticMeshComponent *> GeomMap1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	TMap<int, UProceduralMeshComponent *> GeomMap2;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	TArray<UProceduralMeshComponent *> ProceduralMeshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	FString XmlSourcePath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	TMap<int, UStaticMesh *> MeshAssets;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	UStaticMesh *defaultMesh;

	/** @brief Unified PD gains (smoothstep trajectory provides speed profile, no gain switching) */
	static constexpr float PD_Kp = 150.0f;
	static constexpr float PD_Kd = 2.0f;

	/** @brief Number of actuated joints */
	static constexpr int32 NUM_JOINTS = 12;

	// ---- External Physics Mode (mujoco_sim drives, UE renders only) ----

	/** @brief External physics mode: no internal mj_step, receive RobotState from mujoco_sim via UDP and render */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "External Physics")
	bool bExternalPhysicsMode = true;

	/** @brief Timeout (s) before external state is considered stale */
	static constexpr float EXT_STATE_TIMEOUT = 0.5f;

	/** @brief Counter for throttled external mode diagnostics */
	int32 ExtDiagCounter = 0;

	// ---- UDP mc_ctrl integration ----

	/** @brief UDP receiver for mc_ctrl commands (port 25002) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UDP Control")
	UUdpReceiverComponent* UdpReceiver;

	/** @brief UDP sender for state feedback to mc_ctrl (port 25001) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UDP Control")
	UUdpSenderComponent* UdpSender;

	/** @brief Enable UDP control mode (mc_ctrl drives joints, local gait disabled) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Control")
	bool bUdpControlEnabled = true;

	/** @brief Timeout (s) before UDP commands are considered stale */
	static constexpr float UDP_CMD_TIMEOUT = 0.1f;

	/** @brief Startup grace period (s): ignore mc_ctrl PD so robot collapses under gravity first.
	 *  Matrix UE behaves the same — robot is already collapsed before mc_ctrl takes over. */
	static constexpr float UDP_STARTUP_DELAY = 3.0f;

	/** @brief Wall-clock time when simulation started (for startup delay) */
	double SimStartWallTime = 0.0;

	/** @brief Gain ramp duration (s) when UDP control first activates. Prevents launch impulse. */
	static constexpr float UDP_GAIN_RAMP_DURATION = 1.0f;

	/** @brief Whether UDP control was active last physics step (for edge detection) */
	bool bUdpWasActive = false;

	/** @brief Elapsed time since UDP control activated (for gain ramp) */
	float UdpGainRampTime = 0.0f;

	/** @brief Joint angles captured at UDP activation moment (for target blending) */
	float UdpActivationAngles[NUM_JOINTS];

	/** @brief Whether stand-up PD control is active */
	bool bStandUpActive = false;

	/** @brief Elapsed time since stand-up was activated */
	float StandUpRampTime = 0.0f;

	/** @brief Matrix stand-up sequence timing (total 3s, DO NOT CHANGE) */
	static constexpr float STAND_PHASE_A_DURATION = 1.0f;  // legs forward (Image 1→2)
	static constexpr float STAND_PHASE_B_DURATION = 1.0f;  // legs backward (Image 2→3)
	static constexpr float STAND_PHASE_C_DURATION = 1.0f;  // legs extend (Image 3→4)
	static constexpr float STAND_UP_RAMP_DURATION = STAND_PHASE_A_DURATION + STAND_PHASE_B_DURATION + STAND_PHASE_C_DURATION; // 3s total

	/** @brief Target joint angles (updated by gait generator each step) */
	float StandUpTargetAngles[NUM_JOINTS];

	/** @brief Captured joint angles at the moment stand-up is requested */
	float StandUpStartAngles[NUM_JOINTS];

	// ---- Gait controller (IK-based trot, following Matrix architecture) ----

	/** @brief Gait period in seconds (Matrix: 0.2s, we use 0.3 for stability) */
	static constexpr float GAIT_PERIOD = 0.3f;

	/** @brief Stride length in meters (forward foot displacement per half-cycle) */
	static constexpr float STRIDE_LENGTH = 0.08f;

	/** @brief Swing foot lift height in meters (Matrix: leg_height=0.1) */
	static constexpr float SWING_HEIGHT = 0.06f;

	/** @brief Body height (foot z below hip) in meters. Matches standing pose (HIP=0.8, KNEE=-1.5): L1*cos(0.8)+L2*cos(-0.7)=0.3028 */
	static constexpr float BODY_HEIGHT = 0.3028f;

	/** @brief Nominal foot x offset from hip (forward) in meters. 0 = feet directly under hips (matches standing pose) */
	static constexpr float FOOT_X_NOMINAL = 0.0f;

	/** @brief Thigh length (hip to knee) in meters */
	static constexpr float L1 = 0.2f;

	/** @brief Shank length (knee to foot) in meters */
	static constexpr float L2 = 0.21366f;

	/** @brief Yaw stride differential factor */
	static constexpr float YAW_STRIDE = 0.04f;

	/** @brief Yaw rate feedback gain for heading stabilization (s). Cancels accumulated yaw drift in open-loop gait. */
	static constexpr float YAW_DAMP_GAIN = 1.0f;

	/** @brief Yaw rate low-pass filter time constant (s). Rejects gait-frequency yaw wiggle, keeps slow drift. */
	static constexpr float YAW_FILTER_TAU = 0.5f;

	/** @brief Lateral stride factor (rad). Large enough to overcome ground friction. */
	static constexpr float LAT_STRIDE = 0.10f;

	/** @brief Current gait phase [0, 1) */
	float GaitPhase = 0.0f;

	/** @brief Counter for throttled gait debug logging */
	int32 GaitLogCounter = 0;

	/** @brief Integrated yaw heading (rad), for drift diagnostics */
	float YawHeading = 0.0f;

	/** @brief Low-pass filtered yaw rate (rad/s), for heading stabilization feedback */
	float YawRateLPF = 0.0f;

	/** @brief Reference heading when lateral movement started (rad) */
	float HeadingRef = 0.0f;
	bool bHeadingRefValid = false;

	/** @brief Velocity commands (set by WASD/QE keys via Blueprint) */
	float CmdVelX = 0.0f;   // W=+1 (forward), S=-1 (backward)
	float CmdVelY = 0.0f;   // A=-1 (left), D=+1 (right)
	float CmdVelYaw = 0.0f; // Q=-1 (turn left), E=+1 (turn right)

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/**
	 * Simulate MuJoCo physics for a given time step.
	 * Advances the MuJoCo physics simulation by the specified delta time.
	 *
	 * @param DeltaTime The time step in seconds to advance the simulation
	 */
	void SimulateMuJoCo(float DeltaTime);

	/**
	 * @brief Extracts current state information from the MuJoCo simulation
	 *
	 * This function populates the provided ModelInfo structure with the current
	 * body and geometry data from the active MuJoCo simulation, including positions,
	 * orientations, and other relevant properties.
	 *
	 * @param modelInfo Reference to a ModelInfo structure to be filled with current simulation state
	 */
	void ExtractCurrentState(ModelInfo &modelInfo);

	/**
	 * @brief Updates the visual representation to match the current simulation state
	 *
	 * This function takes the current model info containing body and geometry data from
	 * the MuJoCo simulation and updates the corresponding Unreal Engine components to
	 * reflect the current state. It updates positions, rotations, and other visual properties
	 * of bodies and geometries in the scene.
	 *
	 * @param Info Reference to a ModelInfo structure containing the current simulation state
	 */
	void UpdateSimulationView(const ModelInfo &Info);

	/**
	 * @brief Generates mesh components for all geometries in the model
	 *
	 * This function creates and initializes appropriate mesh components (either static or procedural)
	 * for each geometry in the provided ModelInfo. It handles different geometry types from MuJoCo
	 * (spheres, capsules, boxes, etc.) and creates corresponding visual representations in Unreal Engine.
	 *
	 * @param modelInfo Reference to the ModelInfo structure containing geometry information
	 */
	void GenerateMeshes(ModelInfo &modelInfo);

	/**
	 * @brief Converts custom MuJoCo mesh geometries to procedural meshes in Unreal Engine
	 *
	 * This function extracts mesh data from the MuJoCo model for geometries that are not basic shapes
	 * (like spheres, boxes, etc.) and creates corresponding procedural mesh components in Unreal Engine.
	 * It reads vertex positions, normals, face indices, and other mesh properties from the MuJoCo model
	 * and creates equivalent mesh representations that can be rendered in the Unreal Engine scene.
	 *
	 * @param mjModel Pointer to the MuJoCo model containing the mesh data to extract
	 * @param  UObject* Outer Reference to owner of the procedural mesh components
	 */
	void ConvertMuJoCoModelToProceduralMeshes(const mjModel *mjModel, UObject *Outer);

	/**
	 * @brief Request the robot to stand up using PD joint control
	 * Call this from Blueprint (e.g. on 'U' key press)
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void RequestStandUp();

	/**
	 * @brief Request the robot to lie down (passive mode, zero torque)
	 * Call this from Blueprint (e.g. on 'Space' key press)
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void RequestLieDown();

	/**
	 * @brief Set walking velocity command (call from Blueprint on WASD/QE)
	 * @param X  Forward(+1)/Backward(-1)/Stop(0)
	 * @param Y  Right(+1)/Left(-1)/Stop(0)
	 * @param Yaw TurnRight(+1)/TurnLeft(-1)/Stop(0)
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void SetWalkVelocity(float X, float Y, float Yaw);

	/**
	 * @brief Sets the color of a static mesh component.
	 *
	 * @param StaticMeshComponent The static mesh component to update
	 * @param Color The new linear color to apply to the mesh
	 */
	void SetMeshColor(UStaticMeshComponent *StaticMeshComponent, FLinearColor Color);

	/**
	 * @brief Applies PD control with gait-generated target angles
	 */
	void ApplyStandUpControl();

	/**
	 * @brief External physics mode tick: receive state → FK → render (no mj_step)
	 * @param DeltaTime Frame delta time
	 */
	void TickExternalPhysics(float DeltaTime);

	/**
	 * @brief Applies PD control using targets received from mc_ctrl via UDP
	 * @param GainScale Gain multiplier [0,1] for activation ramp (prevents launch impulse)
	 */
	void ApplyUdpControl(float GainScale = 1.0f);

	/**
	 * @brief Sends current robot state to mc_ctrl via UDP
	 */
	void SendStateToMcCtrl();

	/**
	 * @brief Updates StandUpTargetAngles using trot gait generator
	 * @param dt Physics timestep
	 */
	void UpdateGaitTargets(float dt);

	// Input handlers
	void OnMoveForwardPressed() { SetWalkVelocity(1.0f, CmdVelY, CmdVelYaw); }
	void OnMoveForwardReleased() { SetWalkVelocity(0.0f, CmdVelY, CmdVelYaw); }
	void OnMoveBackwardPressed() { SetWalkVelocity(-1.0f, CmdVelY, CmdVelYaw); }
	void OnMoveBackwardReleased() { SetWalkVelocity(0.0f, CmdVelY, CmdVelYaw); }
	void OnStrafeRightPressed() { SetWalkVelocity(CmdVelX, 1.0f, CmdVelYaw); }
	void OnStrafeRightReleased() { SetWalkVelocity(CmdVelX, 0.0f, CmdVelYaw); }
	void OnStrafeLeftPressed() { SetWalkVelocity(CmdVelX, -1.0f, CmdVelYaw); }
	void OnStrafeLeftReleased() { SetWalkVelocity(CmdVelX, 0.0f, CmdVelYaw); }
	void OnTurnRightPressed() { SetWalkVelocity(CmdVelX, CmdVelY, 1.0f); }
	void OnTurnRightReleased() { SetWalkVelocity(CmdVelX, CmdVelY, 0.0f); }
	void OnTurnLeftPressed() { SetWalkVelocity(CmdVelX, CmdVelY, -1.0f); }
	void OnTurnLeftReleased() { SetWalkVelocity(CmdVelX, CmdVelY, 0.0f); }

	/**
	 * used for Debugging toprintout some properties
	 **/

	void LogInfo();

public:
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	bool LoadModel(FString Xml);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void StartSimulation();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void PauseSimulation();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void ResetSimulation();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void StepSimulation();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo")
	void SetControl(int Id, float Value);
};
