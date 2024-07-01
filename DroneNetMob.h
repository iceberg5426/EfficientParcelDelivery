/*
 * DroneNetMob.h
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#ifndef __INET_DRONENETMOB_H
#define __INET_DRONENETMOB_H

#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"
#include <vector>
#include <algorithm>
#include <iostream>

///////////////(24/06/26)
#include <stdio.h>
#include <math.h>


namespace inet {

/**
 * @brief Random mobility model for a mobile host with a mass.
 * See NED file for more info.
 *
 * @author Emin Ilker Cetinbas, Andras Varga
 */
struct parcel{
    int parcelID;
    double weight;
    int priority;    // is similar to Penalty
    double exp_time;
    Coord parceldest;
};
void parcelsDefinition (int nparcels);


//###(240615)###### 0.CDPF  1.CNPF_greedy  2. TSP_BnB  3.EPDS  4.RSPF 5.HPF  ##########
enum parcelSelection{
    CDPF = 0,        //Closest-Deadline-Parcel-First
    CNPF = 1,       //Closest-Neighbor-Parcel-First, greedy
    TSP_BnB = 2,     //TSP_BnB
    EPDS = 3,      //Efficient Parcel Delivery Service distance/weight
    RSPF = 4,     //Randomly-Selected-Parcel-First
    HPF  = 5     //Heaviest Parcel First
};

//enum parcelSelection{
//    RSPF = 0,     //Randomly-Selected-Parcel-First
//    CNPF = 1,       //Closest-Neighbor-Parcel-First
//    HPF  = 2,     //Heaviest Parcel First
//    CDPF = 3,        //Closest-Deadline-Parcel-First
//    EPDS = 4,      //Efficient Parcel Delivery Service distance/weight
//    TSP_BnB = 5     //TSP_BnB
//};

//enum parcelSelection{
//    CDPF = 0,        //Closest-Deadline-Parcel-First
//    CNPF = 1,       //Closest-Neighbor-Parcel-First
//    EPDS = 2,      //Efficient Parcel Delivery Service distance/weight
//    RSPF = 3,     //Randomly-Selected-Parcel-First
//    HPF  = 4,     //Heaviest Parcel First
//    TSP_BnB = 5     //TSP_BnB
//};





class INET_API DroneNetMob : public LineSegmentsMobilityBase
{
  protected:
    // config (see NED file for explanation)
    cPar *changeIntervalParameter = nullptr;
    cPar *angleDeltaParameter = nullptr;
    cPar *rotationAxisAngleParameter = nullptr;
    cPar *speedParameter = nullptr;
    cPar *numdst = nullptr;
    cPar *numParcel = nullptr;

    cPar *ox = nullptr;
    cPar *oy = nullptr;
    cPar *oz = nullptr;

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
//    bool flagmovedtodst;
    double droneweightcapacity;
    double droneremainingbattery;
    int selectionMethod;
    std::vector<parcel> MissionParcels;
    double deliveryStartTime = 0;
    double deliveryEndTime =   0;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Move the host according to the current simulation time. */
    virtual void move() override;
    void orient() override;

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition() override;
//    virtual void handleMessage(cMessage *msg) override;


  public:
    DroneNetMob();
    virtual double getMaxSpeed() const override;
    void destGen(int ndst);
    void parcelsDefinition (int nparcels);
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord destAssignment();

//    double dist(Coord a, Coord b);
//    double tsp(std::vector<parcel>& parcels, int pos, int visited, std::vector<std::vector<double>>& dp, std::vector<std::vector<double>>& distance);
//    void solveTSP(std::vector<parcel>& parcels);
};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H

