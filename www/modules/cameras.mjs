//@ts-check
const leftCam = "f0914100f10755b1b9414d0b890e88ca9e14caec37cbe17380b782254f87d162";
const leftHtmlId = "lefttag"
const rightCam = "2c2de863e9ae1abcc37d6cdecf5cdbfbd27b5d7ef916b7545ffef22cf72c8efa";
const rightHtmlId = "righttag"

/**
 * @return {Promise<{left:MediaStream, right:MediaStream}>}
 */
export async function loadCams(){
    const c = CamerasFromTags();
    c.loadFromUser();

    // log list of devices to console.
    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    // @ts-ignore
    return c.getStreams();
}

export function loadRtc(streams){
    const c = CamerasFromTags();
    c.setStreams(streams);
}

export default class Cameras{
    /**
     * @param {HTMLVideoElement} left
     * @param {HTMLVideoElement} right
     */
    constructor(left, right) {
        this.left = left;
        this.right = right;
    }

    async loadFromUser(leftId = leftCam, rightId = rightCam) {
        // Don't use await her as i want load cams in parallel.
        const left = navigator.mediaDevices.getUserMedia({ video: { deviceId: leftId, width: 1920, height: 1080 } });
        const right = navigator.mediaDevices.getUserMedia({ video: { deviceId: rightId, width: 1920, height: 1080 } });

        // Awaiting both cams
        let streams = {
            left: await left,
            right: await right,
        }

        this.setStreams(streams);
    }

    setStreams(streams) {
        this.left.srcObject = streams.left;
        this.right.srcObject = streams.right;
    }

    getStreams() {
        return {
            left: this.left.srcObject,
            right: this.right.srcObject
        }
    }
}

// Private

function CamerasFromTags() {
    return new Cameras(
        // @ts-ignore
        document.getElementById(leftHtmlId),
        document.getElementById(rightHtmlId)
    )
}