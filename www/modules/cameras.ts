const leftCam = "171576bfc26c9ce37f0881de5c44ef2dc7d7b82d92a4e382f1d7b94957573e6f";
const rightCam = "c13e97362e3e5bc794c2f8d176fb475da32f32f27c7e09fdc2d9565187511b04";

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
