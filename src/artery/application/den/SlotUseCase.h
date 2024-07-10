/*
 * Artery V2X Simulation Framework
 * Copyright 2016-2018 Raphael Riebl, Christina Obermaier
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_SLOTUSECASE_H
#define ARTERY_SLOTUSECASE_H

#include "artery/application/den/Memory.h"
#include "artery/application/den/SuspendableUseCase.h"
#include "artery/application/Sampling.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/units/velocity.hpp>

namespace artery
{

// forward declaration
class LocalDynamicMap;
class VehicleController;

namespace den
{

/**
 * Check triggering conditions for "Dangerous End Of Queue" use case.
 * See release 1.1.0 of C2C-CC Triggering Conditions "Traffic Jam" (Version 3.3.0)
 */
class SlotUseCase : public SuspendableUseCase
{
public:
    /**
     * Switch if on-board map or camera sensor shall report a "non-urban environment".
     * A non-urban environment is the precondition for this use case, i.e. DENMs are only generated when set to true.
     * \param flag true if vehicle is assumed to be in an non-urban environment
     */
    SlotUseCase();

    void setNonUrbanEnvironment(bool flag) { mNonUrbanEnvironment = flag; }

    vanetza::asn1::Denm createMessage();
    vanetza::btp::DataRequestB createRequest();

    void check() override;
    void indicate(const artery::DenmObject&) override {};
    void handleStoryboardTrigger(const StoryboardSignal&) override {};

protected:
    void initialize(int) override;

    bool checkPreconditions();
    bool checkConditions();
    bool checkEgoDeceleration() const;
    bool checkEndOfQueueReceived() const;
    bool checkJamAheadReceived() const;

private:
    std::shared_ptr<const Memory> mDenmMemory;
    bool mNonUrbanEnvironment;
    SkipEarlySampler<vanetza::units::Velocity> mVelocitySampler;

    const VehicleController* mVc = nullptr;
    //const StationType_t mStationType;
};


} // namespace den
} // namespace artery

#endif // ARTERY_SLOTUSECASE_H

