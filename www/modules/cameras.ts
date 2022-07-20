const leftCam = "bcde5eef8a8e23c6f8801e057d89a7fa4f93f37895b7172cc4da75defa0bece0";
const rightCam = "a1519691111f4b63905099ce037ac50be1f4da55e49eee61a592e741eae965d5";

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
