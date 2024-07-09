package artery.application;

import artery.application.ItsG5Service;

simple SlotService like ItsG5Service
{
    parameters:
        @class(SlotService);
        @signal[DenmSent](type=artery::DenmObject);
        @signal[DenmReceived](type=artery::DenmObject);

        @statistic[reception](source=DenmReceived; record=count,vector(denmActionId)?,vector(denmCauseCode)?);
        @statistic[transmission](source=DenmSent; record=count,vector(denmActionId)?,vector(denmCauseCode)?);
}
