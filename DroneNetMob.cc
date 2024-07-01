/*
 * DroneNetMob.cc
 *
 *  Created on: June 14, 2021  >  24/6/15
 *      Author: iotlab_aime
 */


#include "inet/common/INETMath.h"
#include "DroneNetMob.h"

using namespace std;
namespace inet {

bool flag_original = false;
Coord originPos;
std::vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 500;
bool flagArrangedDepot = false;
bool OngoingMission = false;
// int selectionMethod = 0;


// Performance parameters (24/6/30)
double total_delay_time = 0.0;   // completion time
double total_flight_distance = 0.0;
double total_battery_consumption = 0.0;
int total_launching_drones = 0;
double avr_delay_time = 0.0;
double avr_battery_consumption = 0.0;
double avr_flight_distance = 0.0;


std::vector<parcel> parcel_depot;

Define_Module(DroneNetMob);
bool sortDepotByDeadline (parcel i, parcel j) {
    return (i.exp_time < j.exp_time);
}
bool sortDepotByDestination (parcel i, parcel j) {
    return ((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
                 +pow(i.parceldest.z - originPos.z, 2))) < (sqrt(pow(j.parceldest.x - originPos.x, 2)
                    + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2))));
}
bool greedySortDepot (parcel i, parcel j) {
    return (((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
            +pow(i.parceldest.z - originPos.z, 2)))/i.weight) < ((sqrt(pow(j.parceldest.x - originPos.x, 2)
               + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2)))/j.weight));
}
bool SortDepotByWeight (parcel i, parcel j) {
    return (i.weight > j.weight);
}


//double dist(Coord a, Coord b);
//double tsp(vector<parcel>& parcels, int pos, int visited, vector<vector<double>>& dp, vector<vector<double>>& distance);
//void solveTSP(vector<parcel>& parcels);

//double DroneNetMob::dist(Coord a, Coord b) {
//    double dx = a.x - b.x;
//    double dy = a.y - b.y;
//    return sqrt(dx*dx + dy*dy);
//}

///////////////(24/6/15)
double dist(Coord a, Coord b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

//greedy algorithm for TSP
vector<parcel> greedyTSP(vector<parcel>& parcels){
    vector<parcel> sortedParcels;
    vector<bool> visited(parcels.size(), false);
    Coord currPos = originPos;
    double totalDist = 0;
    while(sortedParcels.size() < parcels.size()) {
        double minDist = 1e9;
        int nextParcel = -1;
        for(int i=0; i<parcels.size(); i++) {
            if(!visited[i]) {
                double d = dist(currPos, parcels[i].parceldest);
                if(d < minDist) {
                    minDist = d;
                    nextParcel = i;
                }
            }
        }
        totalDist += minDist;
        visited[nextParcel] = true;
        sortedParcels.push_back(parcels[nextParcel]);
        currPos = parcels[nextParcel].parceldest;
    }
    totalDist += dist(currPos, originPos);
    cout << "Greedy TSP distance: " << totalDist << endl;
    return sortedParcels;
}

//optimal path가 수행되는지 확인
double tsp(vector<parcel>& parcels, int pos, int visited, vector<vector<double>>& dp, vector<vector<double>>& distance, double currDist, double& ans, vector<int>& optimalPath, vector<int>& path) {
    path.push_back(pos); // add the current position to the path
    if(visited == ((1<<parcels.size()) - 1)) {
        ans = min(ans, currDist + distance[pos][0]); // update the minimum distance
        if(currDist + distance[pos][0] == ans) {
            optimalPath = path; // update the optimal path
        }
    }
    if(dp[pos][visited] != -1 && currDist >= dp[pos][visited]) {
        path.pop_back(); // remove the current position from the path
        return ans; // prune the branch if the current distance is already greater than the previously calculated distance
    }

    for(int i=0; i<parcels.size(); i++) {
        if((visited & (1<<i)) == 0) { // if not visited
            double newDist = currDist + distance[pos][i];
            if(newDist < ans) { // only explore the branch if the new distance is less than the current minimum distance
                tsp(parcels, i, visited | (1<<i), dp, distance, newDist, ans, optimalPath, path);
            }
        }
    }
    dp[pos][visited] = currDist; // update the previously calculated distance
    path.pop_back(); // remove the current position from the path
    return ans;
}

vector<parcel> solveTSP(vector<parcel>& parcels) {
    cout << "Solving TSP" << endl;
    //print the parcels
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve: " << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    int n = parcels.size();
    vector<vector<double>> distance(n, vector<double>(n));
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            distance[i][j] = dist(parcels[i].parceldest, parcels[j].parceldest);
        }
    }

    vector<vector<double>> dp(n, vector<double>(1<<n, -1));
    double ans = 1e9;
    vector<int> optimalPath;
    vector<int> path;
    tsp(parcels, 0, 1, dp, distance, 0, ans, optimalPath, path);

    // Sort parcels based on the optimal path
    vector<parcel> sortedParcels;
    for(int i=optimalPath.size()-1; i>=0; i--) {
        sortedParcels.push_back(parcels[optimalPath[i]]);
    }

    //print the optimal path
    cout << "Optimal path: ";
    for(int i=0; i<optimalPath.size(); i++) {
        cout << optimalPath[i] << " ";
    }
    cout << endl;

    cout << "Minimum distance: " << ans << endl;
    return sortedParcels;
}
///////////////(24/6/15)



//double DroneNetMob::tsp(std::vector<parcel>& parcels, int pos, int visited, std::vector<std::vector<double>>& dp, std::vector<std::vector<double>>& distance) {
//
//    if(visited == ((1<<parcels.size()) - 1)) {
//        return distance[pos][0]; // return to the start
//    }
//    if(dp[pos][visited] != -1) {
//        return dp[pos][visited]; // already calculated
//    }
//    double ans = 1e9;
//    for(int i=0; i<parcels.size(); i++) {
//        if((visited & (1<<i)) == 0) { // if not visited
//            double newAns = distance[pos][i] + tsp(parcels, i, visited | (1<<i), dp, distance);
//            ans = std::min(ans, newAns);
//        }
//    }
//    return dp[pos][visited] = ans;
//}
//
//void DroneNetMob::solveTSP(std::vector<parcel>& parcels) {
//    std::cout << "Solving TSP" << std::endl;
//    int n = parcels.size();
//    std::vector<std::vector<double>> distance(n, std::vector<double>(n));
//    for(int i=0; i<n; i++) {
//        for(int j=0; j<n; j++) {
//            distance[i][j] = dist(parcels[i].parceldest, parcels[j].parceldest);
//        }
//    }
//    std::vector<std::vector<double>> dp(n, std::vector<double>(1<<n, -1));
//    double ans = tsp(parcels, 0, 1, dp, distance);
//    std::cout << "Minimum distance: " << ans << std::endl;
//}


DroneNetMob::DroneNetMob()
{
}

void DroneNetMob::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);
//    std::cout << "initializing DroneMobility stage " << stage << endl;

///////////////(24/6/15)
//   EV_INFO << "initialize: 현재 시뮬레이션 시작 시간: " << simTime() << endl;
//    cout << "INITIALIZE: 현재 시뮬레이션 시작 시간: " << simTime() << endl;
///////////////(24/6/30)
    simtime_t tmp = simTime();
    cout << "INITIALIZE: 현재 시뮬레이션 시작 시간: " << tmp << endl;

    if (stage == INITSTAGE_LOCAL) {
        rad heading = deg(par("initialMovementHeading"));
        rad elevation = deg(par("initialMovementElevation"));
        changeIntervalParameter = &par("changeInterval");
        angleDeltaParameter = &par("angleDelta");
        rotationAxisAngleParameter = &par("rotationAxisAngle");
        speedParameter = &par("speed");
        quaternion = Quaternion(EulerAngles(heading, -elevation, rad(0)));
        WATCH(quaternion);
        numdst = &par("ndst");
        numParcel = &par("npar");
        ox = &par("initialX");
        oy = &par("initialY");
        oz = &par("initialZ");
        originPos.x = ox->doubleValue();
        originPos.y = oy->doubleValue();
        originPos.z = oz->doubleValue();
        droneweightcapacity =  (&par("weightCapacity"))->doubleValue();
        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
        selectionMethod = (&par("parcelSelectionMethod"))->intValue();


        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            std::cout << "Name -----> " << getParentModule()->getFullName() <<std::endl;
            //print selectionMethod
            std::cout << " Selection Method -----> " << selectionMethod <<std::endl;
             if (numdst->intValue() > 0){
                destGen (numdst->intValue());
            }
//             int numParcel = 500;
//            parcelsDefinition(numParcel);
            parcelsDefinition(numParcel->intValue());

            /*           for (auto i:parcel_depot){
               std::cout << i.parcelID <<" ; W =" <<i.weight <<"; P = " << i.priority << "; Exp = " <<i.exp_time
                       <<" Pos = (" <<i.parceldest.x <<"; " << i.parceldest.y <<"; " <<i.parceldest.z <<")" << std::endl;
           }*/
        }
/*        destination = missionPathNextDest(originPos);
        std::cout << " Name -----> " << getParentModule()->getFullName() <<
                    " And Speed = " << speedParameter->doubleValue() << std::endl;*/
    }
}

void DroneNetMob::orient()
{
    if (faceForward)
        lastOrientation = quaternion;
}

void DroneNetMob::setTargetPosition()
{

    if (flag_original){
        ///////////////(24/6/30)
        cout << "SETTARGETPOSITION_flag_original=T?: "<< flag_original  << ", simtime: "<< simTime() << endl;

        rad angleDelta = deg(angleDeltaParameter->doubleValue());
        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
        quaternion = quaternion * dQ;
        quaternion.normalize();
        Coord direction = quaternion.rotate(Coord::X_AXIS);

        simtime_t nextChangeInterval = *changeIntervalParameter;
        EV_DEBUG << "interval: " << nextChangeInterval << endl;
        sourcePosition = lastPosition;
        targetPosition = lastPosition + direction * (*speedParameter) * nextChangeInterval.dbl();
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
    else{
        ///////////////(24/6/30)
        cout << "SETTARGETPOSITION_flag_original=F?: "<< flag_original  << ", simtime: "<< simTime() << endl;

        EV_INFO << "target setting " << endl;
        simtime_t nextChangeInterval = *changeIntervalParameter;
        sourcePosition = lastPosition;
        //print the source position
        EV_INFO << "source position: " << sourcePosition << endl;
//            destination = missionPathNextDest(originPos);
        targetPosition = missionPathNextDest(lastPosition);
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
}

void DroneNetMob::move()
{


    if (flag_original){
        ///////////////(24/6/30)
        cout << "MOVE_flag_original=T?: "<< flag_original  << ", simtime: "<< simTime() << endl;

        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
    }
    else{
        ///////////////(24/6/30)
        cout << "MOVE_flag_original=F?: "<< flag_original  << ", simtime: "<< simTime() << endl;

        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);

            if (std::strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
                std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
            }

/*           std::cout << " Name -----> " << getParentModule()->getFullName() << " , Velocity = ("
                            << getCurrentVelocity().x << "; " << getCurrentVelocity().y << "; "<< getCurrentVelocity().z <<"), Position = ("
                            << getCurrentPosition().x<< "; " << getCurrentPosition().y <<"; "<< getCurrentPosition().z <<")"<< std::endl;*/
        }
    }
}

double DroneNetMob::getMaxSpeed() const
{

    ///////////////(24/6/15)
    cout << "GETMAXSPEED: " << simTime() << endl;

    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}


///////////////(24/6/15)
//void DroneNetMob::destGen(int ndst){
//    for (unsigned int i = 0; i < numdst->intValue(); i++){
//        Coord nextdst;
//        nextdst.x = rand() % 600;
//        nextdst.y = rand() % 400;
//        nextdst.z = 0;
//        dst.push_back(nextdst);
//    }
/*    std::cout <<std::endl <<std::endl;
    for(auto i:dst){
        std::cout <<"(" <<i.x <<"; " << i.y <<"; " <<i.z <<") | ";
    }
    std::cout <<std::endl <<std::endl;*/
//}


///////////////(24/6/15)
void DroneNetMob::destGen(int ndst){

Coord nextdst;

///////////////(24/6/15)
cout << "DESTGEN: " << simTime() << endl;


nextdst.x = 100;
nextdst.y = 100;
nextdst.z = 0;
dst.push_back(nextdst);

nextdst.x = 100;
nextdst.y = 200;
nextdst.z = 0;
dst.push_back(nextdst);

nextdst.x = 200;
nextdst.y = 200;
nextdst.z = 0;
dst.push_back(nextdst);

nextdst.x = 200;
nextdst.y = 400;
nextdst.z = 0;
dst.push_back(nextdst);

nextdst.x = 400;
nextdst.y = 400;
nextdst.z = 0;
dst.push_back(nextdst);

nextdst.x = 500;
nextdst.y = 200;
nextdst.z = 0;
dst.push_back(nextdst);

std::cout << "numdst: " << numdst->intValue() << std::endl;
    for(auto i:dst){
        std::cout <<"(" <<i.x <<";" << i.y <<";" <<i.z <<") ";
    }
    cout << endl<< endl;

}


void DroneNetMob::parcelsDefinition (int nparcels){

    ///////////////(24/6/15)
    cout << "PARCELSDEFINITION : " << simTime() << "; " << "(Parcel ID, Weight,priority, exp_time, dindex)" << endl;

    for (unsigned int i = 0; i < nparcels; i++){
        parcel tmpparcel;
        parcel *p;
        tmpparcel.parcelID = i;
        tmpparcel.weight =  rand() % 10 + 1;   //weight for each parcel is under 10
        tmpparcel.priority = 1;
        tmpparcel.exp_time = rand() % 300;
        int n = numdst->intValue();
        int dindex = rand() % n;
        tmpparcel.parceldest = dst[dindex];
        parcel_depot.push_back(tmpparcel);

        ///////////////(24/6/15)
        cout << "(" <<tmpparcel.parcelID <<":"<< tmpparcel.weight <<":"<< tmpparcel.priority <<":"<< tmpparcel.exp_time <<":"<< dindex << ")" <<endl;

    }
    std::cout <<std::endl<<std::endl;

}

std::vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    std::vector<parcel> selectedParcels;
    double packedweight = 0;

///////////////(24/6/15)
    std::cout << "DRONEPARCELSSELECTIONFROMSOURCE: " << simTime() << ",  "  << "Selection ===>  " << parcelSel << std::endl;
    std::cout << " Drone -----> " << getParentModule()->getFullName() <<
                    " with speed = " << speedParameter->doubleValue() <<" Defines its mission!"<< std::endl;

//###(240615)###### 0.CDPF  1.CNPF  2. TSP_BnB  3.EPDS  4.RSPF 5.HPF  ##########

    switch(parcelSel){
        /* Closest-Deadline-Parcel-First
         * Depot sorted by Deadline
         * First deliver the ones with small deadline*/


    case CDPF:{
        std::cout <<"CDPF -- parcel_depot size = " <<parcel_depot.size() << std::endl;
        if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);
/*                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    std::cout <<"P" <<parcel_depot[i].parcelID << ":: " << "W = " <<parcel_depot[i].weight <<" dst = (" <<parcel_depot[i].parceldest.x <<"; "<<parcel_depot[i].parceldest.y <<"; "
                            <<parcel_depot[i].parceldest.z <<")"<<" Deadline = "<< parcel_depot[i].exp_time  <<std::endl;
                }*/
                cout << "--------------sorted destination list----------------" << endl;
                cout << "destination of parcels in depot: " << endl;

                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    cout << "(" <<parcel_depot[i].parceldest.x <<"," << parcel_depot[i].parceldest.y <<")--" << endl;
                }
                cout << endl;

                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                std::cout <<"11111 Number of removed Parcels = " <<k<< "|| Depot = " <<parcel_depot.size() <<std::endl;
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                std::cout <<"11111 Number of removed Parcels = " <<k<< "|| Depot = " <<parcel_depot.size() <<std::endl;
        }
        else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
        }
        break;
    }



    /* Closest-Neighbor-Parcel-First
     * Depot sorted by Positions
     * First deliver the ones closer to source*/

///////////////(24/6/15)
          case CNPF:{
//              cout <<" CNPF ----- > working------------"  << endl;
              std::cout <<"CNPF -- parcel_depot size = " <<parcel_depot.size() << std::endl;
              if (!flagArrangedDepot){
                  cout << "--------------sorted destination list----------------" << endl;
                  cout << "destination of parcels in depot: " << endl;

                  // std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      cout << "(" <<parcel_depot[i].parceldest.x <<"," << parcel_depot[i].parceldest.y << ")--";
                  }
                  cout << endl;

                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              else{
                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              break;
          }


//    case CNPF:{
//        std::cout <<" CNPF ----- >    Parcel Selection Method! parcel_depot size = " <<parcel_depot.size() << std::endl;
//        if (!flagArrangedDepot){
//            std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
//            int k=0;
//            for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                packedweight+=parcel_depot[i].weight;
//                if (packedweight < droneweightcapacity){
//                    selectedParcels.push_back(parcel_depot[i]);
//                    k++;
//                }
//                else{
//                    break;
//                }
//            }
//            parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//        }
//        else{
//            int k=0;
//            for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                packedweight+=parcel_depot[i].weight;
//                if (packedweight < droneweightcapacity){
//                    selectedParcels.push_back(parcel_depot[i]);
//                    k++;
//                }
//                else{
//                    break;
//                }
//            }
//            parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//        }
//        break;
//    }


          /* TSP_BnB  */
          case TSP_BnB:{
              std::cout <<"TSP_BnB -- parcel size = " <<parcel_depot.size() << std::endl;

              if (!flagArrangedDepot){
 //                std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);
 //                std::vector<parcel> temp_parcel;
 //                auto local_parcel_depot = parcel_depot;
 //                solveTSP(local_parcel_depot);     // sort by TSP_BnB

 ///////////////(24/6/17)****** sort is correct for BnB?
                  std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);

                  cout << "--------------sorted destination list----------------" << endl;
                  cout << "destination of parcels in depot: " << endl;

                  // std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      cout << "(" <<parcel_depot[i].parceldest.x <<"," << parcel_depot[i].parceldest.y <<")--" << endl;
                  }



                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              else{
                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              break;
          }




          /* Efficient Parcel Delivery Service
           * Depot sorted in a greedy way ()
           * First deliver the ones with small ratio distance/weight*/
          case EPDS:{
              std::cout <<"EDPS -- parcel_depot size = " <<parcel_depot.size() << std::endl;

              if (!flagArrangedDepot){
                  std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);

                  cout << "--------------sorted destination list----------------" << endl;
                  cout << "destination of parcels in depot: " << endl;

                  // std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      cout << "(" <<parcel_depot[i].parceldest.x <<"," << parcel_depot[i].parceldest.y <<")--" << endl;
                  }


                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              else{
                  int k=0;
                  for (unsigned int i = 0; i < parcel_depot.size(); i++){
                      packedweight+=parcel_depot[i].weight;
                      if (packedweight < droneweightcapacity){
                          selectedParcels.push_back(parcel_depot[i]);
                          k++;
                      }
                      else{
                          break;
                      }
                  }
                  parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
              }
              break;
          }


         /* Randomly-Selected-Parcel-First
           * Randomly select Parcels to be delivered first*/
          case RSPF:{

              std::cout <<"RSPF -- parcel_depot size = " <<parcel_depot.size() << std::endl;


              if (!flagArrangedDepot){
              }
              else{
              }

      //        int k=0;
      //        for (unsigned int i = 0; i < parcel_depot.size(); i++){
      //            packedweight+=parcel_depot[i].weight;
      //            if (packedweight < droneweightcapacity){
      //                selectedParcels.push_back(parcel_depot[i]);
      //                k++;
      //            }
      //            else{
      //                break;
      //            }
      //        }
      //        parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);

              break;
          }


    /* Heaviest Parcel First
     * Depot is sorted based on parcel weight
     * First deliver the heaviest ones to the lightest*/
    case HPF:{
        if (!flagArrangedDepot){
            std::sort (parcel_depot.begin(), parcel_depot.end(),SortDepotByWeight);
            int k=0;
            for (unsigned int i = 0; i < parcel_depot.size(); i++){
                packedweight+=parcel_depot[i].weight;
                if (packedweight < droneweightcapacity){
                    selectedParcels.push_back(parcel_depot[i]);
                    k++;
                }
                else{
                    break;
                }
            }
            parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
        }
        else{
            int k=0;
            for (unsigned int i = 0; i < parcel_depot.size(); i++){
                packedweight+=parcel_depot[i].weight;
                if (packedweight < droneweightcapacity){
                    selectedParcels.push_back(parcel_depot[i]);
                    k++;
                }
                else{
                    break;
                }
            }
            parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
        }
        break;
    }





        default:{
            std::cout <<" Undefined Selection Method." <<std::endl;
        }
    }
    for (auto i:selectedParcels){
        std::cout <<"ID = " << i.parcelID << " Weight: " <<i.weight <<" deadline = " << i.exp_time <<
                "<<-->> par_depot = "<< parcel_depot.size() <<std::endl;
    }
    return selectedParcels;
}


Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest;

    if (!OngoingMission){
///////////////(24/6/30)
        std::cout <<"MISSIONPATHNEXTDEST[start_OngoingMission=F]:" << OngoingMission << "; Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
             <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< "), " << "greedy or TSP 선택 "  << std::endl;

        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);
        OngoingMission = true;


        switch (selectionMethod) {

        ///////////////(24/6/15)
        //        solveTSP(MissionParcels);   //240511
          std::cout <<"MISSIONPATHNEXTDEST/selectionMethod: Greedy(1) or BnB TSP(2): " << selectionMethod << std::endl;

            case 1:
                //Greedy
                MissionParcels = greedyTSP(MissionParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Greedy: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            case 2:
                // BnB TSP
                MissionParcels = solveTSP(MissionParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of TSP: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
        }
    }
    else{
///////////////(24/6/30)
         std::cout <<"MISSIONPATHNEXTDEST[start_OngoingMission=T]:" << OngoingMission << "; Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
              <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;

        if (MissionParcels.size() == 0){
            nextdest = originPos;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index

            for (unsigned int i = 0; i < MissionParcels.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
                else{
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionParcels[i].parceldest;
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionParcels.erase(MissionParcels.begin()+k);
        }
    }
    std::cout <<"MISSIONPATHNEXTDEST[end]:" << simTime() << ",  Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
            <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;
    return nextdest;
}

/*Algorithm for destination selection based on:
 * Shortest path and drone capacity*/
Coord DroneNetMob::destAssignment(){

    ///////////////(24/6/15)
    cout << "DESTASSIGNMENT: " << simTime() << endl;

    int sz = dst.size();
    Coord ds = dst[gen];
    gen++;
    return ds;
}



/*
///////////////(24/06/26) relation btw Payload and Range
//  입력과 출력은 각각 무엇으로 할것인가?     입력은 잔여밧데리량, 드론의 무게 /     출력은 비행거리 인가?
int main()
{
    // 드론의 배터리 관련 파라미터 설정
    double m2 = 1.5; // 배터리 무게 (kg)
    double sbatt = 200; // 배터리의 에너지 용량 (Wh/kg)
    double gamma = 0.8; // 배터리 방전 깊이
    double f = 1.1; // 안전 계수

    // 에너지 소비율 (Wh/m)
    double Epm_loaded = 100; // 페이로드 적재 시
    double Epm_unloaded = 50; // 빈 상태로 반환 시

    printf("Payload (kg) | Range (m)\n");
    printf("-------------------------\n");

    for (double payload = 0; payload <= 10; payload += 0.5) {
        // 페이로드에 따른 에너지 소비율 조정
        double Epm_current = Epm_loaded + 10 * exp(0.3 * payload);

        // 페이로드에 따른 비행 거리 계산
        double range = (m2 * sbatt * gamma) / ((Epm_current + Epm_unloaded) * f);

        // 결과 출력
        printf("%10.2f kg | %10.2f m\n", payload, range);
    }
    return 0;
}
*/


} // namespace inet

