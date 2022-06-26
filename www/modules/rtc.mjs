//@ts-check
export class Connection {
    /**
     * @param {RTCPeerConnection} left
     * @param {RTCPeerConnection} right
     */
    constructor(left, right) {
        this.left = left;
        this.right = right;

        left.onconnectionstatechange = ev => {
            console.log("left.connectionState = ", left.connectionState);
        }
        right.onconnectionstatechange = ev => {
            console.log("right.connectionState = ", left.connectionState);
        }
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

    async createAnswers() {
        let left = createAnswer(this.left);
        let right = createAnswer(this.right);

        return {
            left: await left,
            right: await right,
        }
    }

    getStreams() {
        return {
            left: getStream(this.left),
            right: getStream(this.right),
        }
    }
    
    async setAnswers(answer) {
        let left = this.left.setRemoteDescription(answer.left);
        let right = this.right.setRemoteDescription(answer.right);

        await Promise.all([left, right]);
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

export async function fromOffers(offers) {
    // no await want to happen in parallel.
    let left = fromOffer(offers.left);
    let right = fromOffer(offers.right);

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
 * @param {RTCPeerConnection} peer
 */
 async function createAnswer(peer) {
    let offer = await peer.createAnswer();
    peer.setLocalDescription(offer);
    return offer;
}

/**
 * @param {RTCPeerConnection} peer
 */
function getStream(peer) {
    let stream = new MediaStream();
    for (const track of peer.getReceivers()) {
        stream.addTrack(track.track)
    }
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

async function fromOffer(offer) {
    let peer = new RTCPeerConnection();
    await peer.setRemoteDescription(offer);
    return peer
}