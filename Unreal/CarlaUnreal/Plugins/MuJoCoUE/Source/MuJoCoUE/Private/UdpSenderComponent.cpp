#include "UdpSenderComponent.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Common/UdpSocketBuilder.h"
#include "robot_sdk.pb.h"

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

	UE_LOG(LogUdpSender, Log,
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
	const TArray<float>& RPY)
{
	// Joint positions: [abad×4, hip×4, knee×4]
	if (JointPos.Num() >= 12)
	{
		StateMsg->clear_q_abad();
		StateMsg->clear_q_hip();
		StateMsg->clear_q_knee();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_q_abad(JointPos[i]);
			StateMsg->add_q_hip(JointPos[4 + i]);
			StateMsg->add_q_knee(JointPos[8 + i]);
		}
	}

	// Joint velocities
	if (JointVel.Num() >= 12)
	{
		StateMsg->clear_qd_abad();
		StateMsg->clear_qd_hip();
		StateMsg->clear_qd_knee();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_qd_abad(JointVel[i]);
			StateMsg->add_qd_hip(JointVel[4 + i]);
			StateMsg->add_qd_knee(JointVel[8 + i]);
		}
	}

	// Joint torques
	if (JointTau.Num() >= 12)
	{
		StateMsg->clear_tau_abad_fb();
		StateMsg->clear_tau_hip_fb();
		StateMsg->clear_tau_knee_fb();
		for (int32 i = 0; i < 4; i++)
		{
			StateMsg->add_tau_abad_fb(JointTau[i]);
			StateMsg->add_tau_hip_fb(JointTau[4 + i]);
			StateMsg->add_tau_knee_fb(JointTau[8 + i]);
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

	// Gyroscope
	if (Gyro.Num() >= 3)
	{
		StateMsg->clear_gyro();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_gyro(Gyro[i]);
		}
	}

	// Accelerometer
	if (Acc.Num() >= 3)
	{
		StateMsg->clear_acc();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_acc(Acc[i]);
		}
	}

	// RPY euler angles
	if (RPY.Num() >= 3)
	{
		StateMsg->clear_rpy();
		for (int32 i = 0; i < 3; i++)
		{
			StateMsg->add_rpy(RPY[i]);
		}
	}

	// Timestamp in nanoseconds
	uint64 NowNs = static_cast<uint64>(FPlatformTime::Seconds() * 1e9);
	StateMsg->set_time_stamp(NowNs);
}

bool UUdpSenderComponent::SendState()
{
	if (!bSocketReady || !SenderSocket || !TargetAddr.IsValid())
	{
		return false;
	}

	// Serialize protobuf message to byte array (avoid std::string ABI mismatch)
	const int32 MsgSize = StateMsg->ByteSizeLong();
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
		return true;
	}

	return false;
}
