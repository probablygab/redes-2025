
#include <omnetpp.h>
#include <cmath>

using namespace omnetpp;

class VLCNetwork : public cSimpleModule {
  private:
    cModule *movingNode = nullptr;
    double vx = 20.0;  // velocity in x (units/s)
    double vy = 0.0;   // velocity in y (units/s)
    simtime_t lastUpdate;

  protected:
    virtual void initialize() override {
        // Choose node by index (e.g., node[1])
        movingNode = getParentModule()->getSubmodule("node", 1);

        if (!movingNode) {
            EV << "Moving node not found!\n";
            return;
        }

        lastUpdate = simTime();

        scheduleAt(simTime() + 0.2, new cMessage("move"));
    }

    virtual void handleMessage(cMessage *msg) override {
        // Update time delta
        simtime_t now = simTime();
        double dt = now.dbl() - lastUpdate.dbl();
        lastUpdate = now;

        // Get current position
        double x = movingNode->par("xpos").doubleValue();
        double y = movingNode->par("ypos").doubleValue();

        // Update position
        x += vx * dt;
        y += vy * dt;

        // Optional: reverse direction at borders
        if (x > 300 || x < 0) vx *= -1;
        if (y > 300 || y < 0) vy *= -1;

        // Set new position
        movingNode->par("xpos").setDoubleValue(x);
        movingNode->par("ypos").setDoubleValue(y);

        EV << "Moved node " << movingNode->getFullName() << " to (" << x << ", " << y << ")\n";

        // Reschedule
        scheduleAt(simTime() + 0.2, msg);
    }
};

Define_Module(VLCNetwork);