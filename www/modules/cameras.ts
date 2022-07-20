const leftCam = "bcde5eef8a8e23c6f8801e057d89a7fa4f93f37895b7172cc4da75defa0bece0";
const leftHtmlId = "lefttag"
const rightCam = "a1519691111f4b63905099ce037ac50be1f4da55e49eee61a592e741eae965d5";
const rightHtmlId = "righttag"


export async function loadCams(): Promise<{ left: MediaStream; right: MediaStream; }>{
    const c = CamerasFromTags();
    let streams = c.loadFromUser();

    // log list of devices to console.
    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    return streams;
}

export function loadRtc(streams){
    const c = CamerasFromTags();
    c.setStreams(streams);
}

export default class Cameras{
    constructor(public left: HTMLVideoElement, public right: HTMLVideoElement) {
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
        return streams;
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

function CamerasFromTags() {
    return new Cameras(
        // @ts-ignore
        document.getElementById(leftHtmlId),
        document.getElementById(rightHtmlId)
    )
}