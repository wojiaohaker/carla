#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UdpSenderComponent.generated.h"

class FSocket;
class FInternetAddr;

namespace robot_sdk { namespace pb { class RobotState; } }

/**
 * @brief UDP Sender Component for Matrix mc_ctrl state feedback.
 *
 * Sends Protobuf-serialized RobotState to mc_ctrl on port 25001.
 * Call UpdateState() with current joint data, then SendState() at desired rate.
 * Or use bAutoSend with Tick to send every frame.
 */
UCLASS(ClassGroup = "MuJoCo", meta = (BlueprintSpawnableComponent))
class MUJOCOUE_API UUdpSenderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UUdpSenderComponent();
	virtual ~UUdpSenderComponent();

	// ---- Configuration ----

	/** Target IP for mc_ctrl. Default 127.0.0.1 (localhost). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Sender")
	FString TargetIP = TEXT("127.0.0.1");

	/** Target port for mc_ctrl state_port. Default 25001. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Sender")
	int32 TargetPort = 25001;

	/** Whether to auto-send state every Tick */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Sender")
	bool bAutoSend = true;

	// ---- Functions ----

	/** Initialize the send socket. Called automatically on BeginPlay. */
	UFUNCTION(BlueprintCallable, Category = "UDP Sender")
	bool InitSocket();

	/** Close the send socket. */
	UFUNCTION(BlueprintCallable, Category = "UDP Sender")
	void CloseSocket();

	/**
	 * Update the full robot state before sending.
	 * @param JointPos    12 joint angles [abad×4, hip×4, knee×4] (rad)
	 * @param JointVel    12 joint velocities (rad/s)
	 * @param JointTau    12 joint torques (Nm)
	 * @param Quat        Body quaternion [w, x, y, z]
	 * @param Gyro        Body angular velocity [wx, wy, wz] (rad/s)
	 * @param Acc         Body linear acceleration [ax, ay, az] (m/s^2)
	 * @param RPY         Body euler angles [roll, pitch, yaw] (rad)
	 */
	UFUNCTION(BlueprintCallable, Category = "UDP Sender")
	void UpdateState(
		const TArray<float>& JointPos,
		const TArray<float>& JointVel,
		const TArray<float>& JointTau,
		const TArray<float>& Quat,
		const TArray<float>& Gyro,
		const TArray<float>& Acc,
		const TArray<float>& RPY);

	/** Send the current state packet to mc_ctrl. */
	UFUNCTION(BlueprintCallable, Category = "UDP Sender")
	bool SendState();

	/** Total packets sent */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Sender|State")
	int64 TotalPacketsSent = 0;

	/** Whether socket is initialized */
	UPROPERTY(BlueprintReadOnly, Category = "UDP Sender|State")
	bool bSocketReady = false;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	FSocket* SenderSocket = nullptr;
	TSharedPtr<FInternetAddr> TargetAddr;
	robot_sdk::pb::RobotState* StateMsg = nullptr;
};
