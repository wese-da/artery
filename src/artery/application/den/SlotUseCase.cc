/*
 * Artery V2X Simulation Framework
 * Copyright 2016-2018 Raphael Riebl, Christina Obermaier
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/den/SlotUseCase.h"
#include "artery/application/SampleBufferAlgorithm.h"
#include "artery/application/VehicleMiddleware.h"
#include <boost/units/base_units/metric/hour.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/csimulation.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/time.hpp>
#include <vanetza/units/velocity.hpp>
#include <algorithm>
#include <numeric>

static const auto hour = 3600.0 * boost::units::si::seconds;
static const auto km_per_hour = boost::units::si::kilo * boost::units::si::meter / hour;

using omnetpp::SIMTIME_S;
using omnetpp::SIMTIME_MS;

Define_Module(artery::den::SlotUseCase)

namespace artery
{
namespace den
{

SlotUseCase::SlotUseCase()
{
    mVm = &mService->getFacilities().get_const<VehicleMiddleware>();
    //mVc = &mService->getFacilities().get_const<VehicleController>();
    //*mVdp->setStationType(StationType_passengerCar);
}

void SlotUseCase::initialize(int stage)
{
    UseCase::initialize(stage);
    if (stage == 0)
    {
        mNonUrbanEnvironment = par("nonUrbanEnvironment").boolValue();
        mDenmMemory = mService->getMemory();
        mVelocitySampler.setDuration(par("sampleDuration"));
        mVelocitySampler.setInterval(par("sampleInterval"));
    }

    mVdp->getStationType();
}

void SlotUseCase::check()
{
    mVelocitySampler.feed(mVdp->speed(), mVdp->updated());
    if (!isDetectionBlocked() && checkPreconditions() && checkConditions())
    {
        blockDetection();
        auto message = createMessage();
        auto request = createRequest();
        mService->sendDenm(std::move(message), request);
    }
}

bool SlotUseCase::checkPreconditions()
{
    return mNonUrbanEnvironment;
}

bool SlotUseCase::checkConditions()
{
    const TriggeringCondition tc0 = std::bind(&SlotUseCase::checkEgoDeceleration, this);
    const bool tc1 = false; // there are no passengers in ego vehicle enabling hazard lights, assume false
    const bool tc2 = false; // so far no simulated vehicle enables hazard lights, assume false
    const TriggeringCondition tc3 = std::bind(&SlotUseCase::checkEndOfQueueReceived, this);
    const TriggeringCondition tc4 = std::bind(&SlotUseCase::checkJamAheadReceived, this);
    const bool tc5 = false; // so far there are no emergency vehicles in simulation, assume false
    const bool tc6 = false; // simulated vehicle has no on-board sensors for end of queue detection, assume false

    return (tc1 && tc2) || (tc0() && (tc2 || tc3() || tc4() || tc5 || tc6));
}

bool SlotUseCase::checkEgoDeceleration() const
 {
     using vanetza::units::Acceleration;
     using vanetza::units::Duration;
     using vanetza::units::Velocity;
     using boost::units::si::seconds;
     using boost::units::si::meter_per_second_squared;

     static const Velocity targetVelocityThreshold { 30.0 * km_per_hour };
     static const Velocity initialVelocityThreshold { 80.0 * km_per_hour };
     static const Acceleration initialDecelThreshold { -0.1 * meter_per_second_squared };
     static const Duration instantDecelDuration { 10.0 * seconds };
     static const Acceleration instantDecelThreshold { -3.5 * meter_per_second_squared };

     bool fulfilled = false;
     const auto& velocitySamples = mVelocitySampler.buffer();

     // current velocity shall not exceed target velocity
     if (!velocitySamples.empty() && velocitySamples.latest().value <= targetVelocityThreshold) {
         // find the newest sample above initial velocity threshold
         auto initialVelocity = std::find_if(std::next(velocitySamples.begin()), velocitySamples.end(),
                 [](const Sample<Velocity>& s) { return s.value >= initialVelocityThreshold; });
         if (initialVelocity != velocitySamples.end()) {
             // should never fail because only 10s are buffered at all
             assert(duration(*initialVelocity, velocitySamples.latest()) < instantDecelDuration);
             fulfilled = differentiate(*initialVelocity, velocitySamples.latest()) < instantDecelThreshold;
         }
     }

     return fulfilled;
 }

 bool SlotUseCase::checkEndOfQueueReceived() const
 {
     // TODO relevance check for ego vehicle is missing
     return mDenmMemory->count(CauseCode::DangerousEndOfQueue) >= 1;
 }

 bool SlotUseCase::checkJamAheadReceived() const
 {
     // TODO relevance check for ego vehicle is missing
     return mDenmMemory->count(CauseCode::TrafficCondition) >= 5;
 }

vanetza::asn1::Denm SlotUseCase::createMessage()
{
    auto msg = createMessageSkeleton();
    msg->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *msg->denm.management.relevanceDistance = RelevanceDistance_lessThan1000m;
    msg->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *msg->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    msg->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *msg->denm.management.validityDuration = 20;
    msg->denm.management.stationType = StationType_passengerCar; // TODO retrieve type from SUMO

    msg->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    msg->denm.situation->informationQuality = 1;
    msg->denm.situation->eventType.causeCode = CauseCodeType_dangerousEndOfQueue;
    msg->denm.situation->eventType.subCauseCode = 0;

    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    return msg;
}

vanetza::btp::DataRequestB SlotUseCase::createRequest()
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;

    vanetza::btp::DataRequestB request;
    request.gn.traffic_class.tc_id(1);

    geonet::DataRequest::Repetition repetition;
    repetition.interval = 0.5 * seconds;
    repetition.maximum = 20.0 * seconds;
    request.gn.repetition = repetition;

    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = 1000.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude = mVdp->latitude();
    destination.position.longitude = mVdp->longitude();
    request.gn.destination = destination;

    return request;
}

} // namespace den
} // namespace artery
