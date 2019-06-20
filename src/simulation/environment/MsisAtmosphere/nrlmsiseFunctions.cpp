//
// Created by andrew on 3/8/19.
//

#include "atmosphere.h"

void Atmosphere::updateInputParams()
{
    this->msisInput.ap = this->ap;
    //std::cout<<this->aph.a[0]<<std::endl;
    //std::cout<<this->msisInput.ap_a->a[0]<<std::endl;
    this->msisInput.ap_a->a[0] = this->aph.a[0];
    this->msisInput.ap_a->a[1] = this->aph.a[1];
    this->msisInput.ap_a->a[2] = this->aph.a[2];
    this->msisInput.ap_a->a[3] = this->aph.a[3];
    this->msisInput.ap_a->a[4] = this->aph.a[4];
    this->msisInput.ap_a->a[5] = this->aph.a[5];
    this->msisInput.ap_a->a[6] = this->aph.a[6];

    this->msisInput.f107A = this->f107A;
    this->msisInput.f107 = this->f107;

}

void Atmosphere::updateSwIndices()
{
    this->ap = this->swDataList[0].dataValue;
    this->aph.a[0] = this->swDataList[0].dataValue;
    this->aph.a[1] = this->swDataList[1].dataValue;
    this->aph.a[2] = this->swDataList[2].dataValue;
    this->aph.a[3] = this->swDataList[3].dataValue;
    this->aph.a[4] = this->swDataList[4].dataValue;

    uint64_t mth = 5;
    double tmp_avg = 0.0;
    for(mth; mth < 13; mth++){
        tmp_avg = tmp_avg + this->swDataList[mth].dataValue;
    }
    this->aph.a[5] = tmp_avg/8.0;

    uint64_t nth = 13;
    tmp_avg = 0.0;
    for(nth; nth < 21; nth++){
        tmp_avg = tmp_avg + this->swDataList[nth].dataValue;
    }
    this->aph.a[6] = tmp_avg/8.0;

    this->f107A = this->swDataList[21].dataValue;
    this->f107 = this->swDataList[22].dataValue;

}