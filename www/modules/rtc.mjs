//@ts-check
export class Connection {
    /**
     * @param {RTCPeerConnection} left
     * @param {RTCPeerConnection} right
     */
    constructor(left, right) {
        this.left = left;
        this.right = right;
    }

    async createOffers() {
        // no await want to happen in parallel.
        let left = createOffer(this.left);
        let right = createOffer(this.right);

        return {
            left: await left,
            right: await right,
        }
    }
}

/**
 * @param {{ left: MediaStream, right: MediaStream, }} cams
 */
export async function fromStreams(cams) {
    // no await want to happen in parallel.
    let left = fromStream(cams.left);
    let right = fromStream(cams.right);

    return new Connection(await left, await right)
}

/* ---- Private stuff --- */

/**
 * @param {RTCPeerConnection} peer
 */
async function createOffer(peer) {
    let offer = await peer.createOffer();
    peer.setLocalDescription(offer);
    return offer;
}

/**
 * @param {MediaStream} stream
 */
async function fromStream(stream) {
    let peer = new RTCPeerConnection();
    for (const track of stream.getTracks()) {
        peer.addTrack(track, stream);
    }
    return peer;
}