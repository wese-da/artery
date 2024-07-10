package artery.application.den;

simple SlotUseCase like SuspendableUseCase
{
    @class(den::SlotUseCase);
    double sampleDuration @unit(s) = default(10s);
    double sampleInterval @unit(s) = default(100ms);
    bool nonUrbanEnvironment = default(true);
    double detectionBlockingInterval @unit(s) = default(60s);
}
