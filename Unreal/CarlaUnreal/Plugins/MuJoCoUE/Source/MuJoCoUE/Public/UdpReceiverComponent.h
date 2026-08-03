#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/CriticalSection.h"
#include "UdpReceiverComponent.generated.h"

class FSocket;
class FUdpSocketReceiver;

// ==================== Parse Mode ====================

/**
 * @brief Determines what protobuf message type to parse from incoming UDP packets.
 */
UENUM(BlueprintType)
enum class EUdpParseMode : uint8
{
	/** Internal physics mode: parse mc_ctrl RobotCmd (joint targets, PD gains) */
	ParseRobotCmd   UMETA(DisplayName = "RobotCmd (mc_ctrl)"),

	/** External physics mode: parse mujoco_sim RobotState (joint positions, IMU, pose) */
	ParseRobotState UMETA(DisplayName = "RobotState (mujoco_sim)")
};

// ==================== RobotCmd Data ====================

/**
 * @brief Parsed command data from mc_ctrl RobotCmd protobuf packet.
 */
USTRUCT(BlueprintType)
struct FUdpCommandData
{
	GENERATED_BODY()

	/** 12 target joint angles (rad): [abad×4, hip×4, knee×4] */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	TArray<float> JointTargets;

	/** 12 target joint velocities (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	TArray<float> JointVelocities;

	/** 12 Kp values per joint */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	TArray<float> KpValues;

	/** 12 Kd values per joint */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	TArray<float> KdValues;

	/** 12 feedforward torques (Nm) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	TArray<float> TauFF;

	/** Timestamp when this packet was received */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	double ReceiveTime = 0.0;

	/** Whether this packet contains valid parsed data */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Command")
	bool bValid = false;
};

// ==================== RobotState Data ====================

/**
 * @brief Parsed RobotState data from mujoco_sim UDP packet.
 *
 * Contains joint positions, velocities, IMU data, and base pose
 * received from the external MuJoCo physics simulation.
 */
USTRUCT(BlueprintType)
struct FUdpStateData
{
	GENERATED_BODY()

	/** 4 ABAD joint angles (rad) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QAbad;

	/** 4 HIP joint angles (rad) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QHip;

	/** 4 KNEE joint angles (rad) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QKnee;

	/** 4 ABAD joint velocities (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QdAbad;

	/** 4 HIP joint velocities (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QdHip;

	/** 4 KNEE joint velocities (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> QdKnee;

	/** Body quaternion [w, x, y, z] */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> Quat;

	/** Body world position [x, y, z] (m) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> Position;

	/** Gyroscope (body frame) [wx, wy, wz] (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> Gyro;

	/** Accelerometer (body frame) [ax, ay, az] (m/s^2) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	TArray<float> Acc;

	/** Timestamp when this packet was received (FPlatformTime) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	double ReceiveTime = 0.0;

	/** Whether this packet contains valid parsed data */
	UPROPERTY(BlueprintReadOnly, Category = "UDP State")
	bool bValid = false;
};

// ==================== Delegates ====================

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnUdpCommandReceived, const FUdpCommandData&, CommandData);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnUdpStateReceived, const FUdpStateData&, StateData);

// ==================== Component ====================

/**
 * @brief Unified UDP Receiver Component supporting both RobotCmd and RobotState parsing.
 *
 * - ParseRobotCmd mode: Listens on port 25002 for mc_ctrl RobotCmd (internal physics).
 * - ParseRobotState mode: Listens on port 25001 for mujoco_sim RobotState (external physics).
 */
UCLASS(ClassGroup = "MuJoCo", meta = (BlueprintSpawnableComponent))
class MUJOCOUE_API UUdpReceiverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UUdpReceiverComponent();

	// ---- Configuration ----

	/** Parse mode: RobotCmd (mc_ctrl) or RobotState (mujoco_sim) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	EUdpParseMode ParseMode = EUdpParseMode::ParseRobotCmd;

	/** UDP port to listen on. Default: 25002 (RobotCmd) or 25001 (RobotState). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	int32 ListenPort = 25002;

	/** Listen IP. 0.0.0.0 = all interfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	FString ListenIP = TEXT("0.0.0.0");

	/** Whether to auto-start listening on BeginPlay */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	bool bAutoStart = true;

	/** Whether to process data on game thread (only for RobotCmd mode) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	bool bProcessOnGameThread = true;

	// ---- Events ----

	/** Broadcast when a valid RobotCmd is received (ParseRobotCmd mode) */
	UPROPERTY(BlueprintAssignable, Category = "UDP Receiver|Events")
	FOnUdpCommandReceived OnCommandReceived;

	/** Broadcast when a valid RobotState is received (ParseRobotState mode) */
	UPROPERTY(BlueprintAssignable, Category = "UDP Receiver|Events")
	FOnUdpStateReceived OnStateReceived;

	// ---- Runtime State ----

	/** Whether the receiver is currently listening */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	bool bIsListening = false;

	/** Last received command data (ParseRobotCmd mode) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	FUdpCommandData LastCommand;

	/** Last received state data (ParseRobotState mode) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	FUdpStateData LastState;

	/** Total packets received since start */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	int64 TotalPacketsReceived = 0;

	/** Total valid commands parsed (ParseRobotCmd mode) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	int64 TotalCommandsParsed = 0;

	/** Total valid states parsed (ParseRobotState mode) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	int64 TotalStatesParsed = 0;

	// ---- Functions ----

	UFUNCTION(BlueprintCallable, Category = "UDP Receiver")
	bool StartListening();

	UFUNCTION(BlueprintCallable, Category = "UDP Receiver")
	void StopListening();

	UFUNCTION(BlueprintPure, Category = "UDP Receiver")
	FUdpCommandData GetLastCommand() const { return LastCommand; }

	UFUNCTION(BlueprintPure, Category = "UDP Receiver")
	FUdpStateData GetLastState() const { return LastState; }

	/** Thread-safe: get latest state (safe to call from game thread) */
	FUdpStateData GetLatestState();

	/** Set parse mode and adjust default port accordingly */
	void SetParseMode(EUdpParseMode Mode);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void HandleDataReceived(const TArray<uint8>& Data, const FString& SenderIP, int32 SenderPort);
	bool ParseProtobufCommand(const TArray<uint8>& Data, FUdpCommandData& OutData);
	bool ParseProtobufState(const TArray<uint8>& Data, FUdpStateData& OutData);

	FSocket* ReceiverSocket = nullptr;
	FUdpSocketReceiver* UdpReceiver = nullptr;

	/** Mutex for thread-safe state access (ParseRobotState mode) */
	FCriticalSection StateMutex;
};
