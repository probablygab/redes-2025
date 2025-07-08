message VLCMsgRREQ {
    // Header
    int hopCount = 0;
    int maxHops = 0;
    double reliability = 1.0;

    // Body
    int source;
    int destination;
    int previous = -1;

    int xpos = 0;
    int ypos = 0;
}