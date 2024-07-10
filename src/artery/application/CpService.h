/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef CASERVICE_H_
#define CASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include <vanetza/asn1/cpm.hpp>

namespace artery
{

class CpService : public ItsG5BaseService
{

    public:
        CpService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication & , std::unique_ptr<vanetza::UpPacket> ) override;
        void trigger() override;

    private:
        void sendCpm(vanetza::asn1::Cpm&);

};

}

#endif
