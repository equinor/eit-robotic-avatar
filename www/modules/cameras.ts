const leftCam = "a8b68e4db341457caec73e5058c28c6f2278a21d5381171f40f515af7d649f87";
const rightCam = "1eaf34dd447b89454553a211b0e7adb038ab1bff3e2066a462e043d5074d0b4d";

export async function loadCams(leftId = leftCam, rightId = rightCam): Promise<{ left: MediaStream; right: MediaStream; }>{
    // log list of devices to console.
    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    const left = navigator.mediaDevices.getUserMedia({ video: { deviceId: leftId, width: 1920, height: 1080 } });
    const right = navigator.mediaDevices.getUserMedia({ video: { deviceId: rightId, width: 1920, height: 1080 } });

    // Awaiting both cams
    let streams = {
        left: await left,
        right: await right,
    }

    return streams;
}
