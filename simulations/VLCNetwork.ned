network VLCTest {
    types:
        channel Channel extends ned.DelayChannel {
            delay = 10ms;
        }
    submodules:
        // node[6]: VLCNode {
        //     @display("i=block/routing");
        //     // Positioning nodes in a grid
        //     xpos = 100 * (int(index) % 3);
        //     ypos = 100 * (int(index) / 3);
        //     @display("p=${xpos},${ypos}");
        // }

        node[6]: VLCNode;

    connections:
        node[0].gate++ <--> Channel <--> node[5].gate++;
        node[0].gate++ <--> Channel <--> node[1].gate++;
        node[0].gate++ <--> Channel <--> node[2].gate++;

        node[1].gate++ <--> Channel <--> node[5].gate++;

        node[2].gate++ <--> Channel <--> node[3].gate++;

        node[3].gate++ <--> Channel <--> node[5].gate++;
        node[3].gate++ <--> Channel <--> node[4].gate++;

    controller: VLCNetwork;
}
