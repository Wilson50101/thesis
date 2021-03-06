#include "ns3/vlc-channel-model.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("VlcChannel");

NS_OBJECT_ENSURE_REGISTERED (VlcChannel);

ns3::TypeId VlcChannel::GetTypeId(void)	// returns meta-information about VlcErrorModel class
		{ 	// including parent class, group name, constructor, and attributes
	static ns3::TypeId tid =
			ns3::TypeId("VlcChannel").SetParent<ns3::Channel>().AddConstructor<
					VlcChannel>().AddAttribute("AveragePower", "Average Power",
					DoubleValue(0), MakeDoubleAccessor(&VlcChannel::m_AvgPower),
					MakeDoubleChecker<double>());
	return tid;
}

VlcChannel::VlcChannel() :
	m_distanceBWTXandRX(0) {
	NS_LOG_FUNCTION(this);
	m_loss = CreateObject<VlcPropagationLossModel>();
	m_AvgPower = 0;

	m_SNR = CreateObject<VlcSnr>();
	m_nDevices = 0;
}

void ns3::VlcChannel::Attach(Ptr<VlcNetDevice> device) {
	m_link[m_nDevices++].m_src = device;
	if (m_nDevices == 2) {
		m_link[0].m_dst = m_link[1].m_src;
		m_link[1].m_dst = m_link[0].m_src;
	}
}

uint32_t VlcChannel::GetNDevices(void) const {
	return 2;
}

Ptr<NetDevice> VlcChannel::GetDevice(uint32_t i) const {
	return m_link[i].m_src;
}

void VlcChannel::SetPropagationLossModel(
		ns3::Ptr<ns3::PropagationLossModel> loss) {
	NS_LOG_FUNCTION(this<<loss);
	this->m_loss = loss;
}

ns3::Ptr<ns3::PropagationLossModel> VlcChannel::GetPropagationLossModel() {
	NS_LOG_FUNCTION(this);
	return this->m_loss;
}

void VlcChannel::SetPropagationDelayModel(
		ns3::Ptr<ns3::PropagationDelayModel> delay) {
	NS_LOG_FUNCTION(this<<delay);
	this->m_delay = delay;
}
ns3::Ptr<ns3::PropagationDelayModel> VlcChannel::GetPropagationDelayModel() {
	NS_LOG_FUNCTION(this);
	return this->m_delay;
}

double VlcChannel::GetDistance() {
	NS_LOG_FUNCTION(this);
	return this->m_distanceBWTXandRX;
}

void VlcChannel::SetDistance() {
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcNetDevice > first = ns3::DynamicCast < VlcNetDevice
			> (this->GetDevice(0));
	ns3::Ptr < VlcNetDevice > second = ns3::DynamicCast < VlcNetDevice
			> (this->GetDevice(1));

	ns3::Ptr < VlcPropagationLossModel > l = ns3::DynamicCast
			< VlcPropagationLossModel > (this->m_loss);
	this->m_distanceBWTXandRX = l->GetDistance(first->GetMobilityModel(),
			second->GetMobilityModel());
}

void VlcChannel::DoCalcPropagationLoss() {
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcTxNetDevice > first = ns3::DynamicCast < VlcTxNetDevice
			> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > second = ns3::DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	double loss = 0;

	for (unsigned int i = 0; i < first->GetTXOpticalPowerSignal().size(); i++) {
		loss = m_loss->CalcRxPower(first->GetTXOpticalPowerSignal().at(i),first->GetMobilityModel(), second->GetMobilityModel());
		second->GetRXOpticalPowerSignal().at(i) = loss;
	}

}

void VlcChannel::SetPropagationDelay(double delay) {
	NS_LOG_FUNCTION(this<<delay);
	//this->m_delay->set
}

double VlcChannel::DoCalcPropagationLossForSignal(int timeInstant) {

	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcTxNetDevice > first = ns3::DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > second = ns3::DynamicCast < VlcRxNetDevice> (this->GetDevice(1));
//	double pMax = *std::max_element(first->GetTXOpticalPowerSignal().begin(),first->GetTXOpticalPowerSignal().end());
//	double pMin = *std::min_element(first->GetTXOpticalPowerSignal().begin(),first->GetTXOpticalPowerSignal().end());
//	double dutyCycle = second->GetAlpha();
//	double pAvg = second->GetErrorModel()->GetAveragePower(pMax,pMin,dutyCycle);
//	double loss = m_loss->CalcRxPower(pAvg,first->GetMobilityModel(), second->GetMobilityModel());
    double chGain = first->GetTXGain();
    double pwr = first->GetTXpower();
	m_SNR->SetReceivedPower(std::pow(chGain,2) * pwr);

	return std::pow(chGain,2) * pwr;
}

double VlcChannel::GetDistance(ns3::Ptr<ns3::MobilityModel> aTX,
		ns3::Ptr<ns3::MobilityModel> bRX) const {
	double dist = 0;
	Vector tx = aTX->GetPosition();
	Vector rx = bRX->GetPosition();
	dist = std::pow((tx.x - rx.x), 2) + std::pow((tx.y - rx.y), 2)
			+ std::pow((tx.z - rx.z), 2);
	dist = std::sqrt(dist);
	return dist;
}

void VlcChannel::SetWavelength(int lower, int upper) {// sets upper and lower bound wavelength [nm]
	NS_LOG_FUNCTION(this);
	this->m_SNR->SetWavelength(lower, upper);
}

void VlcChannel::SetTemperature(double t) {	// sets the blackbody temperature of LED
	NS_LOG_FUNCTION(this<<t);
	this->m_SNR->SetTemperature(t);
}

double VlcChannel::GetTemperature() {
	NS_LOG_FUNCTION(this);
	return this->m_SNR->GetTemperature();
}

void VlcChannel::SetReceivedPower(double p) {// sets the average received optical signal power
	NS_LOG_FUNCTION(this<<p);
	this->m_SNR->SetReceivedPower(p);
}

void VlcChannel::CalculateNoiseVar() {	//calculates the noise variance
	NS_LOG_FUNCTION(this );
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	this->m_SNR->CalculateNoiseVar(rx->GetPhotoDetectorArea());

}
void VlcChannel::CalculateSNR() {		// caluclates the SNR value
	NS_LOG_FUNCTION(this);
	this->m_SNR->CalculateSNR();
}

double VlcChannel::GetSNR() const {	// returns the signal-to-noise ratio (SNR)
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	this->m_SNR->CalculateNoiseVar(rx->GetPhotoDetectorArea());
//	m_SNR->CalculateSNR();
//	return this->m_SNR->GetSNR();
    return this->m_snr_val;
}

void VlcChannel::SetSNR(double snr){
    this->m_snr_val = snr;
}

void VlcChannel::SetAveragePower(double power) {
	NS_LOG_FUNCTION(this<<power);
	m_AvgPower = power;
}

double VlcChannel::GetAveragePower() {
	NS_LOG_FUNCTION(this);
	return m_AvgPower;
}

void VlcChannel::SetElectricNoiseBandWidth(double b) {// sets the noise bandwidth
	NS_LOG_FUNCTION(this<<b);
	this->m_SNR->SetElectricNoiseBandWidth(b);
}

double VlcChannel::GetNoiseBandwidth() {			//return the noise bandwidth
	NS_LOG_FUNCTION(this);
	return m_SNR->GetNoiseBandwidth();
}

bool VlcChannel::TransmitStart(Ptr<Packet> p, Ptr<VlcNetDevice> src,
		Time txTime) {
	NS_LOG_FUNCTION(this<<p<<src);
	//static int i = 0;
	this->DoCalcPropagationLossForSignal(0);
	this->CalculateNoiseVar();

	ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> (this->GetDevice(1));

	rx->GetErrorModel()->SetSNR(this->GetSNR()); //for each transmission event calculate the SNR and set it in error model.

	uint32_t id = src == m_link[0].m_src ? 0 : 1;

	m_delay = CreateObject<ConstantSpeedPropagationDelayModel>();

	Time m_delayTime = m_delay->GetDelay(tx->GetMobilityModel(), rx->GetMobilityModel());

//	Simulator::ScheduleWithContext(m_link[id].m_dst->GetNode()->GetId(),
//	txTime + m_delayTime, &VlcNetDevice::Receive, m_link[id].m_dst, p);

	// Call the tx anim callback on the net device
	m_txrxVlcChannel(p, src, m_link[id].m_dst, txTime, txTime + m_delayTime);

	return true;
}

void VlcChannel::TransmitDataPacket(Ptr<Packet> p) {
	NS_LOG_FUNCTION(this<<p);
	//this is the point where we decide to keep the packet or corrupt it
	this->DoCalcPropagationLossForSignal(0);
	this->CalculateNoiseVar();

	ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> (this->GetDevice(1));

	rx->GetErrorModel()->SetSNR(this->GetSNR()); //for each transmission event calculate the SNR and set it in error model.
//	double distance = this->GetDistance(tx->GetMobilityModel(),	rx->GetMobilityModel());
	ns3::Ptr < ns3::VlcErrorModel > rxErrorModel = rx->GetErrorModel();

//	double erRate = rxErrorModel->CalculateErrorRate();
    double erRate = 0.00001;
	int size = p->GetSize();
//	double packetErrorRate = 1.0 - std::pow((1 - erRate), size);

//	std::ofstream pers;
//	pers.open("packetErrorRateFile.txt", std::ios_base::app);
//	pers << "\t" << packetErrorRate << std::endl;
//	std::cout << "\t" << packetErrorRate << std::endl;
	//bool isCorrupt = rxErrorModel->CorruptPacket(p, erRate);
	bool isCorrupt = rxErrorModel->CorruptPacket(p, erRate);
	//std::cout << packetErrorRate<< std::endl;
	rx->EnqueueDataPacketAfterCorruption(p, isCorrupt);
//	pers.close();

//	TransmitStart(p, tx, Seconds(0.1));
//    std::cout<<"AP "<<tx->GetNode()->GetId()<<" sending pkt "<<p->GetUid()<<" size " << p->GetSize()<<" at "<<Simulator::Now().GetSeconds()<<'\n';
    TransmitStart(p, tx, Simulator::Now());
}

VlcChannel::~VlcChannel() {

}

} /* namespace vlc */
