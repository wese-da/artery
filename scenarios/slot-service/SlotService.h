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

#ifndef SLOTSERVICE_H_
#define SLOTSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/NetworkInterface.h"
#include <vanetza/asn1/denm.hpp>
//#include "artery/application/VehicleDataProvider.h"

namespace artery
{

class VehicleDataProvider;

class SlotService : public ItsG5BaseService
{
    public:
        SlotService();
        ~SlotService();

		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        void sendDenm();
        void addManagementContainer(ManagementContainer_t& management);
        omnetpp::cMessage* m_self_msg;
		const Timer* mTimer = nullptr;

		const VehicleDataProvider* mVehicleDataProvider = nullptr;

        long mSequenceNumber = 1L;
};

vanetza::asn1::Denm createDecentralizedEnvironmentalNotificationMessage(const VehicleDataProvider&);

} // namespace artery

#endif /* SLOTSERVICE_H_ */
