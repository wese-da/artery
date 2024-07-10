//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "SlotService.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/DenmObject.h"
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/btp/ports.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <artery/application/DenService.h>
#include <omnetpp/checkandcast.h>

using namespace omnetpp;

namespace artery
{
static const auto microdegree = vanetza::units::degree * boost::units::si::micro;
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
static const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalDenmReceived = cComponent::registerSignal("DenmReceived");
static const simsignal_t scSignalDenmSent = cComponent::registerSignal("DenmSent");

Define_Module(SlotService)

SlotService::SlotService()
{
}

//SlotService::~SlotService()
//{
//	cancelAndDelete(m_self_msg);
//}

void SlotService::indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>, const NetworkInterface&)
{
	Enter_Method("indicate");

	//if (packet->getByteLength() == 42) {
	//	EV_INFO << "packet indication on channel " << net.channel << "\n";
	//}

	//delete(packet);
}

void SlotService::initialize()
{
	ItsG5BaseService::initialize();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mService = omnetpp::check_and_cast<DenService*>(getParentModule());
	//subscribe(scSignalCamReceived);
	//subscribe(scSignalDenmReceived);

	//scheduleAt(simTime() + 3.0, m_self_msg);
}

void SlotService::finish()
{
	// you could record some scalars at this point
	ItsG5BaseService::finish();
}

void SlotService::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "self message\n";
	}
}

void SlotService::trigger()
{
	Enter_Method("trigger");

	sendDenm();


}

void SlotService::sendDenm()
{
	auto message = createDecentralizedEnvironmentalNotificationMessage();

    using vanetza::units::si::meter;

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::DENM;
	request.gn.its_aid = aid::DEN;
	request.gn.transport_type = geonet::TransportType::GBC;
	//request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(1);

    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = 1000.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude = mVehicleDataProvider->latitude();
    destination.position.longitude = mVehicleDataProvider->longitude();
    request.gn.destination = destination;

	message.size();
	DenmObject obj(std::move(message));
	emit(scSignalDenmSent, &obj);

	using DenmConvertible = vanetza::convertible::byte_buffer_impl<vanetza::asn1::Denm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket };
    std::unique_ptr<vanetza::convertible::byte_buffer> denm { new DenmConvertible { obj.shared_ptr() } };
    payload->layer(OsiLayer::Application) = vanetza::ByteBufferConvertible { std::move(denm) };
    this->request(request, std::move(payload));

}

void SlotService::addManagementContainer(ManagementContainer_t& management)
{

	management.actionID.sequenceNumber = mSequenceNumber;
	mSequenceNumber++;

}


void SlotService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
	Enter_Method("receiveSignal");

	/*if (signal == scSignalCamReceived) {
		auto& vehicle = getFacilities().get_const<traci::VehicleController>();
		EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
	}*/
}

vanetza::asn1::Denm SlotService::createDecentralizedEnvironmentalNotificationMessage()
{

	using namespace den;

	vanetza::asn1::Denm message;


	// fill header
	ItsPduHeader_t header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_denm;
	//header.stationID = mVehicleDataProvider.station_id();

	auto action_id = mService->requestActionID();
    message->denm.management.actionID.originatingStationID = action_id.originatingStationID;
    message->denm.management.actionID.sequenceNumber = action_id.sequenceNumber;

	// fill message
	DecentralizedEnvironmentalNotificationMessage_t denm = (*message).denm;
	ManagementContainer_t& management = denm.management;
	//LocationContainer* location = denm.location;
	//SituationContainer* situation = denm.situation;
	//AlacarteContainer* alacarte = denm.alacarte;

	// management
	//management.referenceTime = asn_long2INTEGER(0L);
	// management.detectionTime = TimestampIts_t{};
	// management.actionID.originatingStationID = vdp.station_id();

    message->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *message->denm.management.validityDuration = 20;


	return message;

}

} // namespace artery
