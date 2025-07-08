#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;

#include "VLCMsgRREQ_m.h"
#include "VLCMsgRREP_m.h"

static const double EPSILON = 1e-6;

class VLCNode : public cSimpleModule {   
    private:
        double xpos = 0;
        double ypos = 0;

        double prevX = 0.0;
        double prevY = 0.0;
        simtime_t lastUpdate = 0.0;

        std::map<int, std::pair<int, double>> routingTable;
        std::map<int, int> node2gate;

        virtual void printInfo();
        virtual void findNodeGate(int node);

        virtual bool updateRoute(int dest, int nextHop, double reliability);
        virtual double updateReliability(double rel, double s_xpos, double s_ypos, int hops);
        
    protected:
        virtual VLCMsgRREQ *generateRREQ(int dest, int maxHops);
        virtual VLCMsgRREP *generateRREP(int dest, int maxHops);

        virtual void forwardRREQ(VLCMsgRREQ *msg);
        virtual void forwardRREP(VLCMsgRREP *msg);        

        virtual void handleRREQ(VLCMsgRREQ *msg);
        virtual void handleRREP(VLCMsgRREP *msg);
        virtual void handleRERR(VLCMsgRERR *msg);
        virtual void handleDATA(VLCMsgDATA *msg);

        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
};

Define_Module(VLCNode);

void VLCNode::initialize() {
    // Get own pos
    xpos = par("xpos").doubleValue();
    ypos = par("ypos").doubleValue();
    
    // Init prev
    prevX = xpos;
    prevY = ypos;
    lastUpdate = simTime();

    // Add self to the routing table
    routingTable[getIndex()] = {getIndex(), 1.0};

    // Node 0 sends message to node 3
    if (getIndex() == 0) {
        VLCMsgRREQ *msg = generateRREQ(5, 3);

        EV << "KICKSTART Node " << getIndex() << " sending RREQ to node " << msg->getDestination() << "\n";

        forwardRREQ(msg);
    }
}

void VLCNode::handleMessage(cMessage *msg) {
    // Check message type and handle accordingly
    if (auto rreq = dynamic_cast<VLCMsgRREQ *>(msg)) {
        handleRREQ(rreq);
    }
    else if (auto rrep = dynamic_cast<VLCMsgRREP *>(msg)) {
        handleRREP(rrep);
    }
    else if (auto data = dynamic_cast<VLCMsgDATA *>(msg)) {
        handleDATA(data);
    }
    else if (auto rerr = dynamic_cast<VLCMsgRERR *>(msg)) {
        handleRERR(rerr);
    }
    else {
        EV << "Unknown message received: " << msg->getName() << "\n";
        delete msg;
    }

    // Update prev after sending msg
    prevX = xpos;
    prevY = ypos;
    lastUpdate = simTime();
}

void VLCNode::handleRREQ(VLCMsgRREQ *msg) {
    // Update gate mapping
    // This is needed because node indices may not match gate indices
    findNodeGate(msg->getPrevious());

    double newRel = updateReliability(msg->getReliability(), msg->getXpos(), msg->getYpos(), msg->getHopCount());

    EV << "Updated reliability for RREQ from " << msg->getPrevious() << " to " << getIndex() << "; OLD: " << msg->getReliability() << ", NEW: " << newRel << "\n";

    msg->setReliability(newRel);

    // Update routing table
    bool updated = updateRoute(msg->getSource(), msg->getPrevious(), msg->getReliability());

    // Also add arrival node
    updateRoute(msg->getPrevious(), msg->getPrevious(), msg->getReliability());

    // If we are the destination, send RREP back to source
    if (msg->getDestination() == getIndex() && updated) {
        EV << "Destination reached! Sending RREP back to source node " << msg->getSource() << "\n";

        VLCMsgRREP *rrep = generateRREP(msg->getSource(), 3);

        // Delete the RREQ message
        delete msg;

        // Send RREP back to the source
        forwardRREP(rrep);
    } else {
        // Forward the RREQ
        forwardRREQ(msg);
    }
    
    // Info
    printInfo();
}

void VLCNode::handleRREP(VLCMsgRREP *msg) {
    // Update gate mapping
    // This is needed because node indices may not match gate indices
    findNodeGate(msg->getPrevious());

    double newRel = updateReliability(msg->getReliability(), msg->getXpos(), msg->getYpos(), msg->getHopCount());

    EV << "Updated reliability for RREQ from " << msg->getPrevious() << " to " << getIndex() << "; OLD: " << msg->getReliability() << ", NEW: " << newRel << "\n";
    
    msg->setReliability(newRel);

    // Update routing table
    updateRoute(msg->getSource(), msg->getPrevious(), msg->getReliability());

    // Check if we are the destination
    if (msg->getDestination() == getIndex()) {
        EV << "RREP received at destination node " << getIndex() << "\n";

        delete msg;
    } else {
        // Forward the RREP to the next hop
        forwardRREP(msg);
    }

    printInfo();
}

void VLCNode::handleRERR(VLCMsgRERR *msg) {

}

void VLCNode::handleDATA(VLCMsgDATA *msg) {

}

VLCMsgRREQ* VLCNode::generateRREQ(int dest, int maxHops) {
    VLCMsgRREQ *msg = new VLCMsgRREQ("RREQ");

    // Header
    msg->setHopCount(0);
    msg->setMaxHops(maxHops);
    msg->setReliability(1.0);
    
    // Body
    msg->setSource(getIndex());
    msg->setDestination(dest);

    return msg;
}

VLCMsgRREP* VLCNode::generateRREP(int dest, int maxHops) {
    VLCMsgRREP *msg = new VLCMsgRREP("RREP");

    // Header
    msg->setHopCount(0);
    msg->setMaxHops(maxHops);
    msg->setReliability(1.0);
    
    // Body
    msg->setSource(getIndex());
    msg->setDestination(dest);

    return msg;
}

void VLCNode::forwardRREQ(VLCMsgRREQ *msg) {
    // Check if the message has reached the maximum hop count
    if (msg->getHopCount() >= msg->getMaxHops()) {
        EV << "RREQ reached maximum hop count; dropping message\n";
        delete msg;
        return;
    }

    msg->setHopCount(msg->getHopCount() + 1);

    int dest = msg->getDestination();
    int prev = msg->getPrevious();
    int n = gateSize("gate$o");

    if (routingTable.find(dest) != routingTable.end()) {
        EV << "Route to destination already known\n";
        delete msg;
        return;
    }

    // Flood the RREQ to all output gates
    // We don't need to map to gates in this case, as we are flooding
    for (int i = 0; i < n; i++) {
        VLCMsgRREQ *forwardMsg = msg->dup();

        forwardMsg->setPrevious(getIndex());
        forwardMsg->setXpos(xpos);
        forwardMsg->setYpos(ypos);

        send(forwardMsg, "gate$o", i);
    }

    delete msg;
}

void VLCNode::forwardRREP(VLCMsgRREP *msg) {
    // Check if the message has reached the maximum hop count
    if (msg->getHopCount() >= msg->getMaxHops()) {
        EV << "RREQ reached maximum hop count; dropping message\n";
        delete msg;
        return;
    }

    msg->setHopCount(msg->getHopCount() + 1);

    // Find the next hop in the routing table
    int nextHop = routingTable[msg->getDestination()].first;

    if (nextHop == -1) {
        EV << "No route to destination " << msg->getDestination() << "\n";
        delete msg;
        return;
    }

    // Set the previous node to the current node
    msg->setPrevious(getIndex());
    msg->setXpos(xpos);
    msg->setYpos(ypos);

    // Send the RREP to the next hop
    send(msg, "gate$o", node2gate[nextHop]);
}

void VLCNode::printInfo() {
    EV << "Node " << getIndex() << " routing table:\n";
    for (const auto& entry : routingTable) {
        EV << " - Dest: " << entry.first << ", Next Hop: " << entry.second.first << ", Rel: " << entry.second.second <<  "\n";
    }

    // EV << "Node to Gate mapping:\n";
    // for (const auto& entry : node2gate) {
    //     EV << " - Node: " << entry.first << ", Gate: " << entry.second << "\n";
    // }
}

void VLCNode::findNodeGate(int node) {
    int n = gateSize("gate$o");

    for (int i = 0; i < n; i++) {
        cGate *outGate = gate("gate$o", i);
        cGate *inGate = outGate->getNextGate();

        if (inGate && inGate->getOwnerModule()->getIndex() == node) {
            node2gate[node] = i;
            return;
        }
    }
}

double gaussian_projection(double n, double k, double sigma = -1.0) {
    double mu = k / 2.0;

    if (sigma <= 0.0)
        sigma = k / 4.0; // default spread if not given

    double exponent = -std::pow(n - mu, 2) / (2.0 * std::pow(sigma, 2));
    double res = k * std::exp(exponent) + EPSILON; // Add small epsilon to avoid zero

    // Add shelf to favor lower hops
    if (n < k / 3.0)
        return std::max(res, 1.0);

    return res;
}

bool VLCNode::updateRoute(int dest, int nextHop, double reliability) {
    // Check if we already have a route to the destination
    if (routingTable.find(dest) != routingTable.end()) {
        // Check if the new route is better (higher reliability)
        if (routingTable[dest].second < reliability + EPSILON) {
            // Update the routing table with the new route
            routingTable[dest] = {nextHop, reliability};

            EV << "NEW! Updated route to destination " << dest << " with new reliability " << reliability << "\n";
            return true;
        }
    } else {
        // Add a new route to the destination
        routingTable[dest] = {nextHop, reliability};
        return true;
    }

    return false;
}

double VLCNode::updateReliability(double rel, double s_xpos, double s_ypos, int hops) {
    const double idealMaxDistance = 80.0;
    const double idealMaxHops = 3.0;
    const double futureDelta = 1.0;  // segundos
    const double distanceThreshold = 80.0;

    // Calculate distance to source
    double dist = std::sqrt(std::pow(xpos - s_xpos, 2) + std::pow(ypos - s_ypos, 2));
    double omega = std::max(dist / idealMaxDistance, 1.0);
    double phi = gaussian_projection(static_cast<double>(hops), idealMaxHops);

    // Predict vel vector
    double elapsed = simTime().dbl() - lastUpdate.dbl();
    if (elapsed < EPSILON) elapsed = 0.1; // avoid dividing by zero

    double vx = (xpos - prevX) / elapsed;
    double vy = (ypos - prevY) / elapsed;

    // Predict pos
    double futureX = xpos + futureDelta * vx;
    double futureY = ypos + futureDelta * vy;

    // Future distance
    double predictedDist = std::sqrt(std::pow(futureX - s_xpos, 2) + std::pow(futureY - s_ypos, 2));
    double gamma = std::max(predictedDist / distanceThreshold, 1.0);

    // Update reliability based on distance, hops and position prediction
    double updatedRel = rel * (1.0 / std::pow((omega / phi) * gamma, 2));

    EV << "RelCalc -> omega: " << omega << ", phi: " << phi << ", gamma: " << gamma << "; rel: " << updatedRel << "\n";

    return updatedRel;
}