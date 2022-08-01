export async function loadCams(leftId: string, rightId: string): Promise<{ left: MediaStream; right: MediaStream; }>{
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
