simple VLCNode {
    parameters:
        double xpos;
        double ypos;
        @display("i=block/routing;p=${xpos},${ypos}");
    gates:
        inout gate[];
}