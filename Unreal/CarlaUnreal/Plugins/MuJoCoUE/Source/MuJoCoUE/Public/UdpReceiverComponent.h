#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UdpReceiverComponent.generated.h"

class FSocket;
class FUdpSocketReceiver;

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

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnUdpCommandReceived, const FUdpCommandData&, CommandData);

/**
 * @brief UDP Receiver Component for Matrix mc_ctrl Protobuf protocol.
 *
 * Listens on port 25002 (cmd_port) for Protobuf-serialized RobotCmd packets.
 * Parses joint targets, PD gains, and feedforward torques.
 */
UCLASS(ClassGroup = "MuJoCo", meta = (BlueprintSpawnableComponent))
class MUJOCOUE_API UUdpReceiverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UUdpReceiverComponent();

	// ---- Configuration ----

	/** UDP port to listen on. Matrix cmd_port = 25002. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	int32 ListenPort = 25002;

	/** Listen IP. 0.0.0.0 = all interfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	FString ListenIP = TEXT("0.0.0.0");

	/** Whether to auto-start listening on BeginPlay */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	bool bAutoStart = true;

	/** Whether to process data on game thread */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Receiver")
	bool bProcessOnGameThread = true;

	// ---- Events ----

	/** Broadcast when a valid RobotCmd is received and parsed */
	UPROPERTY(BlueprintAssignable, Category = "UDP Receiver|Events")
	FOnUdpCommandReceived OnCommandReceived;

	// ---- Runtime State ----

	/** Whether the receiver is currently listening */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	bool bIsListening = false;

	/** Last received command data (cached for polling) */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	FUdpCommandData LastCommand;

	/** Total packets received since start */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	int64 TotalPacketsReceived = 0;

	/** Total valid commands parsed */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Receiver|State")
	int64 TotalCommandsParsed = 0;

	// ---- Functions ----

	UFUNCTION(BlueprintCallable, Category = "UDP Receiver")
	bool StartListening();

	UFUNCTION(BlueprintCallable, Category = "UDP Receiver")
	void StopListening();

	UFUNCTION(BlueprintPure, Category = "UDP Receiver")
	FUdpCommandData GetLastCommand() const { return LastCommand; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void HandleDataReceived(const TArray<uint8>& Data, const FString& SenderIP, int32 SenderPort);
	bool ParseProtobufCommand(const TArray<uint8>& Data, FUdpCommandData& OutData);

	FSocket* ReceiverSocket = nullptr;
	FUdpSocketReceiver* UdpReceiver = nullptr;
};
