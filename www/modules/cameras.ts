const leftCam = "4819e649baeaee89af7db0ef264492290caaa0e484813cdd54bfb80fe7c14ab1";
const rightCam = "257ace1c32d21646fa57a49f73d50a8bb8f0c3f27499694d5f5e3523182a19d2";

export async function loadCams(leftId = leftCam, rightId = rightCam): Promise<{ left: MediaStream; right: MediaStream; }>{
    // log list of devices to console.
    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    const left = navigator.mediaDevices.getUserMedia({ video: { deviceId: leftId, width: 1280, height: 720 } });
    const right = navigator.mediaDevices.getUserMedia({ video: { deviceId: rightId, width: 1280, height: 720 } });

    // Awaiting both cams
    let streams = {
        left: await left,
        right: await right,
    }

    return streams;
}
