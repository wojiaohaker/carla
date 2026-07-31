#include "UdpSenderComponent.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Common/UdpSocketBuilder.h"
#include "robot_sdk.pb.h"
#include <time.h>

DEFINE_LOG_CATEGORY_STATIC(LogUdpSender, Log, All);

UUdpSenderComponent::UUdpSenderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickInterval = 0.002f; // 500Hz target
	bWantsInitializeComponent = false;
	bAutoActivate = true;

	StateMsg = new robot_sdk::pb::RobotState();
}

UUdpSenderComponent::~UUdpSenderComponent()
{
	delete StateMsg;
	StateMsg = nullptr;
}

void UUdpSenderComponent::BeginPlay()
{
	Super::BeginPlay();
	// Disable auto-send: MuJoCoSimulation calls UpdateState()+SendState() explicitly
	// after each physics step. Auto-send would send empty/stale packets from TickComponent.
	bAutoSend = false;
	InitSocket();
}

void UUdpSenderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	CloseSocket();
	Super::EndPlay(EndPlayReason);
}

void UUdpSenderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bAutoSend && bSocketReady)
	{
		SendState();
	}
}

bool UUdpSenderComponent::InitSocket()
{
	if (bSocketReady)
	{
		return true;
	}

	SenderSocket = FUdpSocketBuilder(TEXT("McCtrlStateSender"))
		.AsNonBlocking()
		.AsReusable();

	if (!SenderSocket)
	{
		UE_LOG(LogUdpSender, Error, TEXT("UdpSender: Failed to create send socket."));
		return false;
	}

	ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	TargetAddr = SocketSubsystem->CreateInternetAddr();

	FIPv4Address Addr;
	if (!FIPv4Address::Parse(TargetIP, Addr))
	{
		UE_LOG(LogUdpSender, Error, TEXT("UdpSender: Invalid target IP '%s'"), *TargetIP);
		CloseSocket();
		return false;
	}

	TargetAddr->SetIp(Addr.Value);
	TargetAddr->SetPort(TargetPort);

	bSocketReady = true;

	UE_LOG(LogUdpSender, Warning,
		TEXT("UdpSender: Socket ready, sending Protobuf RobotState to %s:%d."),
		*TargetIP, TargetPort);

	return true;
}

void UUdpSenderComponent::CloseSocket()
{
	if (SenderSocket)
	{
		SenderSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SenderSocket);
		SenderSocket = nullptr;
	}
	bSocketReady = false;
}

void UUdpSenderComponent::UpdateState(
	const TArray<float>& JointPos,
	const TArray<float>& JointVel,
	const TArray<float>& JointTau,
	const TArray<float>& Quat,
	const TArray<float>& Gyro,
	const TArray<float>& Acc,
	const TArray<float>& RPY,
	const TArray<float>& Position,
	const TArray<float>& VWorld)
{
	// Joint positions: [abad×4, hip×4, knee×4]
	if (JointPos.Num() >= 12)
	{
		StateMsg->clear_q_abad();
		StateMsg->clear_q_hip();
		StateMsg->clear_q_knee();
		StateMsg->clear_q_foot();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_q_abad(JointPos[i]);
			StateMsg->add_q_hip(JointPos[4 + i]);
			StateMsg->add_q_knee(JointPos[8 + i]);
			StateMsg->add_q_foot(0.0f); // No foot joint
		}
	}

	// Joint velocities
	if (JointVel.Num() >= 12)
	{
		StateMsg->clear_qd_abad();
		StateMsg->clear_qd_hip();
		StateMsg->clear_qd_knee();
		StateMsg->clear_qd_foot();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_qd_abad(JointVel[i]);
			StateMsg->add_qd_hip(JointVel[4 + i]);
			StateMsg->add_qd_knee(JointVel[8 + i]);
			StateMsg->add_qd_foot(0.0f); // No foot joint
		}
	}

	// Joint torques
	if (JointTau.Num() >= 12)
	{
		StateMsg->clear_tau_abad_fb();
		StateMsg->clear_tau_hip_fb();
		StateMsg->clear_tau_knee_fb();
		StateMsg->clear_tau_foot_fb();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_tau_abad_fb(JointTau[i]);
			StateMsg->add_tau_hip_fb(JointTau[4 + i]);
			StateMsg->add_tau_knee_fb(JointTau[8 + i]);
			StateMsg->add_tau_foot_fb(0.0f); // No foot torque
		}
	}

	// IMU quaternion [w, x, y, z]
	if (Quat.Num() >= 4)
	{
		StateMsg->clear_quat();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_quat(Quat[i]);
		}
	}

	// Gyroscope (body frame)
	if (Gyro.Num() >= 3)
	{
		StateMsg->clear_gyro();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_gyro(Gyro[i]);
		}
	}

	// Accelerometer (body frame)
	if (Acc.Num() >= 3)
	{
		StateMsg->clear_acc();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_acc(Acc[i]);
		}
	}

	// RPY euler angles — NOT sent by Matrix UE (field 16 absent in 303-byte packet).
	// Omit to match exact wire format mc_ctrl expects.
	// if (RPY.Num() >= 3) { StateMsg->clear_rpy(); ... }

	// Timestamp in nanoseconds — MUST use CLOCK_REALTIME to match mc_ctrl's time base.
	// Matrix UE sends wall-clock time (since epoch), mc_ctrl compares with CLOCK_REALTIME.
	// CLOCK_MONOTONIC (since boot) differs by system uptime → data rejected as stale.
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	uint64 NowNs = static_cast<uint64>(ts.tv_sec) * 1000000000ULL + static_cast<uint64>(ts.tv_nsec);
	StateMsg->set_time_stamp(NowNs);

	// World position [x, y, z]
	if (Position.Num() >= 3)
	{
		StateMsg->clear_position();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_position(Position[i]);
		}
	}

	// World velocity [vx, vy, vz]
	if (VWorld.Num() >= 3)
	{
		StateMsg->clear_v_world();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_v_world(VWorld[i]);
		}
	}
}

bool UUdpSenderComponent::SendState()
{
	if (!bSocketReady || !SenderSocket || !TargetAddr.IsValid())
	{
		// Throttled failure log
		static int32 FailDiagCounter = 0;
		if (++FailDiagCounter >= 500)
		{
			FailDiagCounter = 0;
			UE_LOG(LogUdpSender, Error, TEXT("[UDP-SEND-FAIL] bSocketReady=%d, SenderSocket=%s, TargetAddr=%s"),
				bSocketReady ? 1 : 0,
				SenderSocket ? TEXT("valid") : TEXT("NULL"),
				TargetAddr.IsValid() ? TEXT("valid") : TEXT("invalid"));
		}
		return false;
	}

	// Serialize protobuf message to byte array (avoid std::string ABI mismatch)
	const int32 MsgSize = StateMsg->ByteSizeLong();

	// Guard: don't send empty messages (no data populated yet)
	if (MsgSize <= 0)
	{
		return false;
	}

	TArray<uint8> Buffer;
	Buffer.AddUninitialized(MsgSize);

	if (!StateMsg->SerializeToArray(Buffer.GetData(), MsgSize))
	{
		UE_LOG(LogUdpSender, Error, TEXT("UdpSender: Protobuf serialization failed."));
		return false;
	}

	// Send
	int32 BytesSent = 0;
	bool bSuccess = SenderSocket->SendTo(
		Buffer.GetData(),
		MsgSize,
		BytesSent,
		*TargetAddr);

	if (bSuccess && BytesSent == MsgSize)
	{
		TotalPacketsSent++;

		// Throttled success diagnostic (every ~1000 packets)
		static int32 SuccessDiagCounter = 0;
		if (++SuccessDiagCounter >= 1000)
		{
			SuccessDiagCounter = 0;
			UE_LOG(LogUdpSender, Warning, TEXT("[UDP-SEND-OK] MsgSize=%d, BytesSent=%d, TotalSent=%lld, target=%s:%d"),
				MsgSize, BytesSent, TotalPacketsSent, *TargetIP, TargetPort);
		}
		return true;
	}

	// Throttled send-failure log
	static int32 SendFailCounter = 0;
	if (++SendFailCounter >= 500)
	{
		SendFailCounter = 0;
		UE_LOG(LogUdpSender, Error, TEXT("[UDP-SEND-FAIL] SendTo failed: bSuccess=%d, BytesSent=%d, MsgSize=%d"),
			bSuccess ? 1 : 0, BytesSent, MsgSize);
	}
	return false;
}
