/*
 * Artery V2X Simulation Framework
 * Copyright 2016-2017 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/DenmObject.h"
#include "SlotService.h"
#include "artery/application/Timer.h"
#include "artery/application/StoryboardSignal.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/FilterRules.h"
#include <omnetpp/checkandcast.h>
#include <omnetpp/ccomponenttype.h>
#include <omnetpp/cxmlelement.h>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/btp/ports.hpp>

using namespace omnetpp;

namespace artery
{

Define_Module(SlotService)

static const simsignal_t denmReceivedSignal = cComponent::registerSignal("DenmReceived");
static const simsignal_t denmSentSignal = cComponent::registerSignal("DenmSent");
static const simsignal_t storyboardSignal = cComponent::registerSignal("StoryboardSignal");

SlotService::SlotService() :
    mTimer(nullptr), mSequenceNumber(0)
{
}

void SlotService::initialize()
{
    ItsG5BaseService::initialize();
    mTimer = &getFacilities().get_const<Timer>();
    mMemory.reset(new artery::den::Memory(*mTimer));

    subscribe(storyboardSignal);
    initUseCases();
}

void SlotService::initUseCases()
{
    omnetpp::cXMLElement* useCases = par("useCases").xmlValue();
    for (omnetpp::cXMLElement* useCaseElement : useCases->getChildrenByTagName("usecase")) {
        omnetpp::cModuleType* useCaseType = omnetpp::cModuleType::get(useCaseElement->getAttribute("type"));
        omnetpp::cXMLElement* filter = useCaseElement->getFirstChildWithTag("filters");

        bool useCaseApplicable = true;
        if (filter) {
            artery::FilterRules rules(getRNG(0), getFacilities().get_const<artery::Identity>());
            useCaseApplicable = rules.applyFilterConfig(*filter);
        }

        if (useCaseApplicable) {
            const char* useCaseName = useCaseElement->getAttribute("name") ?
                useCaseElement->getAttribute("name") : useCaseType->getName();
            omnetpp::cModule* module = useCaseType->create(useCaseName, this);
            // do not call initialize here! omnetpp::cModule initializes submodules on its own!
            module->buildInside();
            den::UseCase* useCase = dynamic_cast<den::UseCase*>(module);
            if (useCase) {
                mUseCases.push_front(useCase);
            }
        }
    }
}

void SlotService::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == storyboardSignal) {
        StoryboardSignal* storyboardSignalObj = check_and_cast<StoryboardSignal*>(obj);
        for (auto use_case : mUseCases) {
            use_case->handleStoryboardTrigger(*storyboardSignalObj);
        }
    }
}

void SlotService::indicate(const vanetza::btp::DataIndication& indication, std::unique_ptr<vanetza::UpPacket> packet)
{
    Asn1PacketVisitor<vanetza::asn1::Denm> visitor;
    const vanetza::asn1::Denm* denm = boost::apply_visitor(visitor, *packet);
    const auto egoStationID = getFacilities().get_const<VehicleDataProvider>().station_id();

    if (denm && (*denm)->header.stationID != egoStationID) {
        DenmObject obj = visitor.shared_wrapper;
        mMemory->received(obj);
        emit(denmReceivedSignal, &obj);

        for (auto use_case : mUseCases) {
            use_case->indicate(obj);
        }
    }
}

void SlotService::trigger()
{
    mMemory->drop();

    for (auto use_case : mUseCases) {
        use_case->check();
    }
}

ActionID_t SlotService::requestActionID()
{
    ActionID_t id;
    id.originatingStationID = getFacilities().get_const<VehicleDataProvider>().station_id();
    id.sequenceNumber = ++mSequenceNumber;
    return id;
}

const Timer* SlotService::getTimer() const
{
    return mTimer;
}

std::shared_ptr<const den::Memory> SlotService::getMemory() const
{
    return mMemory;
}

void SlotService::sendDenm(vanetza::asn1::Denm&& message, vanetza::btp::DataRequestB& request)
{
    fillRequest(request);
    DenmObject obj { std::move(message) };
    emit(denmSentSignal, &obj);

    using namespace vanetza;
    using DenmConvertible = vanetza::convertible::byte_buffer_impl<vanetza::asn1::Denm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket };
    std::unique_ptr<vanetza::convertible::byte_buffer> denm { new DenmConvertible { obj.shared_ptr() } };
    payload->layer(OsiLayer::Application) = vanetza::ByteBufferConvertible { std::move(denm) };
    this->request(request, std::move(payload));
}

void SlotService::fillRequest(vanetza::btp::DataRequestB& request)
{
    using namespace vanetza;

    request.destination_port = btp::ports::DENM;
    request.gn.its_aid = aid::DEN;
    request.gn.transport_type = geonet::TransportType::GBC;
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
}

} // namespace artery
